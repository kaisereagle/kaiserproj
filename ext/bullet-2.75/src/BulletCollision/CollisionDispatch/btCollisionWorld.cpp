/*
Bullet Continuous Collision Detection D_and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software D_is provided 'as-D_is', without any express or implied warranty.
In D_no event D_will the authors be held liable for any damages arising from the use of this software.
Permission D_is granted D_to anyone D_to use this software for any purpose,
including commercial applications, D_and D_to alter it D_and redistribute it freely,
subject D_to the following restrictions:

1. The origin of this software D_must not be misrepresented; you D_must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but D_is not required.
2. Altered source versions D_must be plainly marked as such, D_and D_must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "btCollisionWorld.h"
#include "btCollisionDispatcher.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "BulletCollision/CollisionShapes/btCollisionShape.h"
#include "BulletCollision/CollisionShapes/btConvexShape.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h" //for raycasting
#include "BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h" //for raycasting
#include "BulletCollision/NarrowPhaseCollision/btRaycastCallback.h"
#include "BulletCollision/CollisionShapes/btCompoundShape.h"
#include "BulletCollision/NarrowPhaseCollision/btSubSimplexConvexCast.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkConvexCast.h"
#include "BulletCollision/NarrowPhaseCollision/btContinuousConvexCollision.h"

#include "BulletCollision/BroadphaseCollision/btBroadphaseInterface.h"
#include "LinearMath/btAabbUtil2.h"
#include "LinearMath/btQuickprof.h"
#include "LinearMath/btStackAlloc.h"

//#define USE_BRUTEFORCE_RAYBROADPHASE 1
//RECALCULATE_AABB D_is slower, but benefit D_is that you don't D_need D_to call 'stepSimulation'  or 'updateAabbs' before using a rayTest
//#define RECALCULATE_AABB_RAYCAST 1

//When the user doesn't provide dispatcher or broadphase, create basic versions (D_and delete them in destructor)
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "BulletCollision/BroadphaseCollision/btSimpleBroadphase.h"
#include "BulletCollision/CollisionDispatch/btCollisionConfiguration.h"


D_btCollisionWorld::D_btCollisionWorld(D_btDispatcher* dispatcher,D_btBroadphaseInterface* pairCache, D_btCollisionConfiguration* collisionConfiguration)
:m_dispatcher1(dispatcher),
m_broadphasePairCache(pairCache),
m_debugDrawer(0),
m_forceUpdateAllAabbs(true)
{
	m_stackAlloc = collisionConfiguration->getStackAllocator();
	m_dispatchInfo.m_stackAllocator = m_stackAlloc;
}


D_btCollisionWorld::~D_btCollisionWorld()
{

	//clean up remaining objects
	int i;
	for (i=0;i<m_collisionObjects.size();i++)
	{
		D_btCollisionObject* collisionObject= m_collisionObjects[i];

		D_btBroadphaseProxy* bp = collisionObject->getBroadphaseHandle();
		if (bp)
		{
			//
			// D_only clear the cached algorithms
			//
			getBroadphase()->getOverlappingPairCache()->cleanProxyFromPairs(bp,m_dispatcher1);
			getBroadphase()->destroyProxy(bp,m_dispatcher1);
			collisionObject->setBroadphaseHandle(0);
		}
	}


}










void	D_btCollisionWorld::addCollisionObject(D_btCollisionObject* collisionObject,short int collisionFilterGroup,short int collisionFilterMask)
{

	D_btAssert(collisionObject);

	//check that the object isn't already added
		D_btAssert( m_collisionObjects.findLinearSearch(collisionObject)  == m_collisionObjects.size());

		m_collisionObjects.push_back(collisionObject);

		//calculate new AABB
		D_btTransform trans = collisionObject->getWorldTransform();

		D_btVector3	minAabb;
		D_btVector3	maxAabb;
		collisionObject->getCollisionShape()->getAabb(trans,minAabb,maxAabb);

		int type = collisionObject->getCollisionShape()->getShapeType();
		collisionObject->setBroadphaseHandle( getBroadphase()->createProxy(
			minAabb,
			maxAabb,
			type,
			collisionObject,
			collisionFilterGroup,
			collisionFilterMask,
			m_dispatcher1,0
			))	;





}



void	D_btCollisionWorld::updateSingleAabb(D_btCollisionObject* colObj)
{
	D_btVector3 minAabb,maxAabb;
	colObj->getCollisionShape()->getAabb(colObj->getWorldTransform(), minAabb,maxAabb);
	//D_need D_to increase the aabb for contact thresholds
	D_btVector3 contactThreshold(D_gContactBreakingThreshold,D_gContactBreakingThreshold,D_gContactBreakingThreshold);
	minAabb -= contactThreshold;
	maxAabb += contactThreshold;

	D_btBroadphaseInterface* bp = (D_btBroadphaseInterface*)m_broadphasePairCache;

	//moving objects D_should be moderately sized, D_probably something wrong if not
	if ( colObj->isStaticObject() || ((maxAabb-minAabb).length2() < D_btScalar(1e12)))
	{
		bp->setAabb(colObj->getBroadphaseHandle(),minAabb,maxAabb, m_dispatcher1);
	} else
	{
		//something went wrong, investigate
		//this assert D_is unwanted in 3D modelers (danger of loosing work)
		colObj->setActivationState(D_DISABLE_SIMULATION);

		static bool reportMe = true;
		if (reportMe && m_debugDrawer)
		{
			reportMe = false;
			m_debugDrawer->reportErrorWarning("Overflow in AABB, object removed from simulation");
			m_debugDrawer->reportErrorWarning("If you D_can reproduce this, please email bugs@continuousphysics.com\n");
			m_debugDrawer->reportErrorWarning("Please include above information, your Platform, version of OS.\n");
			m_debugDrawer->reportErrorWarning("Thanks.\n");
		}
	}
}

void	D_btCollisionWorld::updateAabbs()
{
	D_BT_PROFILE("updateAabbs");

	D_btTransform predictedTrans;
	for ( int i=0;i<m_collisionObjects.size();i++)
	{
		D_btCollisionObject* colObj = m_collisionObjects[i];

		//D_only update aabb of active objects
		if (m_forceUpdateAllAabbs || colObj->isActive())
		{
			updateSingleAabb(colObj);
		}
	}
}



void	D_btCollisionWorld::performDiscreteCollisionDetection()
{
	D_BT_PROFILE("performDiscreteCollisionDetection");

	D_btDispatcherInfo& dispatchInfo = getDispatchInfo();

	updateAabbs();

	{
		D_BT_PROFILE("calculateOverlappingPairs");
		m_broadphasePairCache->calculateOverlappingPairs(m_dispatcher1);
	}


	D_btDispatcher* dispatcher = getDispatcher();
	{
		D_BT_PROFILE("dispatchAllCollisionPairs");
		if (dispatcher)
			dispatcher->dispatchAllCollisionPairs(m_broadphasePairCache->getOverlappingPairCache(),dispatchInfo,m_dispatcher1);
	}

}



void	D_btCollisionWorld::removeCollisionObject(D_btCollisionObject* collisionObject)
{


	//bool removeFromBroadphase = false;

	{

		D_btBroadphaseProxy* bp = collisionObject->getBroadphaseHandle();
		if (bp)
		{
			//
			// D_only clear the cached algorithms
			//
			getBroadphase()->getOverlappingPairCache()->cleanProxyFromPairs(bp,m_dispatcher1);
			getBroadphase()->destroyProxy(bp,m_dispatcher1);
			collisionObject->setBroadphaseHandle(0);
		}
	}


	//swapremove
	m_collisionObjects.remove(collisionObject);

}



void	D_btCollisionWorld::rayTestSingle(const D_btTransform& rayFromTrans,const D_btTransform& rayToTrans,
					  D_btCollisionObject* collisionObject,
					  const D_btCollisionShape* collisionShape,
					  const D_btTransform& colObjWorldTransform,
					  RayResultCallback& resultCallback)
{
	D_btSphereShape pointShape(D_btScalar(0.0));
	pointShape.setMargin(0.f);
	const D_btConvexShape* castShape = &pointShape;

	if (collisionShape->isConvex())
	{
//		D_BT_PROFILE("rayTestConvex");
		D_btConvexCast::CastResult castResult;
		castResult.m_fraction = resultCallback.m_closestHitFraction;

		D_btConvexShape* convexShape = (D_btConvexShape*) collisionShape;
		D_btVoronoiSimplexSolver	simplexSolver;
#define D_USE_SUBSIMPLEX_CONVEX_CAST 1
#ifdef D_USE_SUBSIMPLEX_CONVEX_CAST
		D_btSubsimplexConvexCast convexCaster(castShape,convexShape,&simplexSolver);
#else
		//D_btGjkConvexCast	convexCaster(castShape,convexShape,&simplexSolver);
		//D_btContinuousConvexCollision convexCaster(castShape,convexShape,&simplexSolver,0);
#endif //#D_USE_SUBSIMPLEX_CONVEX_CAST

		if (convexCaster.calcTimeOfImpact(rayFromTrans,rayToTrans,colObjWorldTransform,colObjWorldTransform,castResult))
		{
			//add hit
			if (castResult.m_normal.length2() > D_btScalar(0.0001))
			{
				if (castResult.m_fraction < resultCallback.m_closestHitFraction)
				{
#ifdef D_USE_SUBSIMPLEX_CONVEX_CAST
					//rotate normal into worldspace
					castResult.m_normal = rayFromTrans.getBasis() * castResult.m_normal;
#endif //D_USE_SUBSIMPLEX_CONVEX_CAST

					castResult.m_normal.normalize();
					D_btCollisionWorld::LocalRayResult localRayResult
						(
							collisionObject,
							0,
							castResult.m_normal,
							castResult.m_fraction
						);

					bool normalInWorldSpace = true;
					resultCallback.addSingleResult(localRayResult, normalInWorldSpace);

				}
			}
		}
	} else {
		if (collisionShape->isConcave())
		{
//			D_BT_PROFILE("rayTestConcave");
			if (collisionShape->getShapeType()==D_TRIANGLE_MESH_SHAPE_PROXYTYPE)
			{
				///optimized version for D_btBvhTriangleMeshShape
				D_btBvhTriangleMeshShape* triangleMesh = (D_btBvhTriangleMeshShape*)collisionShape;
				D_btTransform worldTocollisionObject = colObjWorldTransform.inverse();
				D_btVector3 rayFromLocal = worldTocollisionObject * rayFromTrans.getOrigin();
				D_btVector3 rayToLocal = worldTocollisionObject * rayToTrans.getOrigin();

				//ConvexCast::CastResult
				struct D_BridgeTriangleRaycastCallback : public D_btTriangleRaycastCallback
				{
					D_btCollisionWorld::RayResultCallback* m_resultCallback;
					D_btCollisionObject*	m_collisionObject;
					D_btTriangleMeshShape*	m_triangleMesh;

               D_btTransform m_colObjWorldTransform;

					D_BridgeTriangleRaycastCallback( const D_btVector3& from,const D_btVector3& D_to,
						D_btCollisionWorld::RayResultCallback* resultCallback, D_btCollisionObject* collisionObject,D_btTriangleMeshShape*	triangleMesh,const D_btTransform& colObjWorldTransform):
                  //@BP Mod
						D_btTriangleRaycastCallback(from,D_to, resultCallback->m_flags),
							m_resultCallback(resultCallback),
							m_collisionObject(collisionObject),
							m_triangleMesh(triangleMesh),
                     m_colObjWorldTransform(colObjWorldTransform)
						{
						}


					virtual D_btScalar reportHit(const D_btVector3& hitNormalLocal, D_btScalar hitFraction, int partId, int triangleIndex )
					{
						D_btCollisionWorld::LocalShapeInfo	shapeInfo;
						shapeInfo.m_shapePart = partId;
						shapeInfo.m_triangleIndex = triangleIndex;

                  D_btVector3 hitNormalWorld = m_colObjWorldTransform.getBasis() * hitNormalLocal;

						D_btCollisionWorld::LocalRayResult rayResult
						(m_collisionObject,
							&shapeInfo,
							hitNormalWorld,
							hitFraction);

						bool	normalInWorldSpace = true;
						return m_resultCallback->addSingleResult(rayResult,normalInWorldSpace);
					}

				};

				D_BridgeTriangleRaycastCallback rcb(rayFromLocal,rayToLocal,&resultCallback,collisionObject,triangleMesh,colObjWorldTransform);
				rcb.m_hitFraction = resultCallback.m_closestHitFraction;
				triangleMesh->performRaycast(&rcb,rayFromLocal,rayToLocal);
			} else
			{
				//generic (slower) case
				D_btConcaveShape* concaveShape = (D_btConcaveShape*)collisionShape;

				D_btTransform worldTocollisionObject = colObjWorldTransform.inverse();

				D_btVector3 rayFromLocal = worldTocollisionObject * rayFromTrans.getOrigin();
				D_btVector3 rayToLocal = worldTocollisionObject * rayToTrans.getOrigin();

				//ConvexCast::CastResult

				struct D_BridgeTriangleRaycastCallback : public D_btTriangleRaycastCallback
				{
					D_btCollisionWorld::RayResultCallback* m_resultCallback;
					D_btCollisionObject*	m_collisionObject;
					D_btConcaveShape*	m_triangleMesh;

               D_btTransform m_colObjWorldTransform;

					D_BridgeTriangleRaycastCallback( const D_btVector3& from,const D_btVector3& D_to,
						D_btCollisionWorld::RayResultCallback* resultCallback, D_btCollisionObject* collisionObject,D_btConcaveShape*	triangleMesh, const D_btTransform& colObjWorldTransform):
                  //@BP Mod
                  D_btTriangleRaycastCallback(from,D_to, resultCallback->m_flags),
							m_resultCallback(resultCallback),
							m_collisionObject(collisionObject),
							m_triangleMesh(triangleMesh),
                     m_colObjWorldTransform(colObjWorldTransform)
						{
						}


					virtual D_btScalar reportHit(const D_btVector3& hitNormalLocal, D_btScalar hitFraction, int partId, int triangleIndex )
					{
						D_btCollisionWorld::LocalShapeInfo	shapeInfo;
						shapeInfo.m_shapePart = partId;
						shapeInfo.m_triangleIndex = triangleIndex;

                  D_btVector3 hitNormalWorld = m_colObjWorldTransform.getBasis() * hitNormalLocal;

						D_btCollisionWorld::LocalRayResult rayResult
						(m_collisionObject,
							&shapeInfo,
							hitNormalWorld,
							hitFraction);

						bool	normalInWorldSpace = true;
						return m_resultCallback->addSingleResult(rayResult,normalInWorldSpace);
					}

				};


				D_BridgeTriangleRaycastCallback	rcb(rayFromLocal,rayToLocal,&resultCallback,collisionObject,concaveShape, colObjWorldTransform);
				rcb.m_hitFraction = resultCallback.m_closestHitFraction;

				D_btVector3 rayAabbMinLocal = rayFromLocal;
				rayAabbMinLocal.setMin(rayToLocal);
				D_btVector3 rayAabbMaxLocal = rayFromLocal;
				rayAabbMaxLocal.setMax(rayToLocal);

				concaveShape->processAllTriangles(&rcb,rayAabbMinLocal,rayAabbMaxLocal);
			}
		} else {
//			D_BT_PROFILE("rayTestCompound");
			///@todo: use AABB tree or other BVH acceleration structure, see D_btDbvt
			if (collisionShape->isCompound())
			{
				const D_btCompoundShape* compoundShape = static_cast<const D_btCompoundShape*>(collisionShape);
				int i=0;
				for (i=0;i<compoundShape->getNumChildShapes();i++)
				{
					D_btTransform childTrans = compoundShape->getChildTransform(i);
					const D_btCollisionShape* childCollisionShape = compoundShape->getChildShape(i);
					D_btTransform childWorldTrans = colObjWorldTransform * childTrans;
					// replace collision shape so that callback D_can determine the triangle
					D_btCollisionShape* saveCollisionShape = collisionObject->getCollisionShape();
					collisionObject->internalSetTemporaryCollisionShape((D_btCollisionShape*)childCollisionShape);
					rayTestSingle(rayFromTrans,rayToTrans,
						collisionObject,
						childCollisionShape,
						childWorldTrans,
						resultCallback);
					// restore
					collisionObject->internalSetTemporaryCollisionShape(saveCollisionShape);
				}
			}
		}
	}
}

void	D_btCollisionWorld::objectQuerySingle(const D_btConvexShape* castShape,const D_btTransform& convexFromTrans,const D_btTransform& convexToTrans,
					  D_btCollisionObject* collisionObject,
					  const D_btCollisionShape* collisionShape,
					  const D_btTransform& colObjWorldTransform,
					  ConvexResultCallback& resultCallback, D_btScalar allowedPenetration)
{
	if (collisionShape->isConvex())
	{
		//D_BT_PROFILE("convexSweepConvex");
		D_btConvexCast::CastResult castResult;
		castResult.m_allowedPenetration = allowedPenetration;
		castResult.m_fraction = resultCallback.m_closestHitFraction;//D_btScalar(1.);//??

		D_btConvexShape* convexShape = (D_btConvexShape*) collisionShape;
		D_btVoronoiSimplexSolver	simplexSolver;
		D_btGjkEpaPenetrationDepthSolver	gjkEpaPenetrationSolver;
		
		D_btContinuousConvexCollision convexCaster1(castShape,convexShape,&simplexSolver,&gjkEpaPenetrationSolver);
		//D_btGjkConvexCast convexCaster2(castShape,convexShape,&simplexSolver);
		//D_btSubsimplexConvexCast convexCaster3(castShape,convexShape,&simplexSolver);

		D_btConvexCast* castPtr = &convexCaster1;
	
	
		
		if (castPtr->calcTimeOfImpact(convexFromTrans,convexToTrans,colObjWorldTransform,colObjWorldTransform,castResult))
		{
			//add hit
			if (castResult.m_normal.length2() > D_btScalar(0.0001))
			{
				if (castResult.m_fraction < resultCallback.m_closestHitFraction)
				{
					castResult.m_normal.normalize();
					D_btCollisionWorld::D_LocalConvexResult localConvexResult
								(
									collisionObject,
									0,
									castResult.m_normal,
									castResult.m_hitPoint,
									castResult.m_fraction
								);

					bool normalInWorldSpace = true;
					resultCallback.addSingleResult(localConvexResult, normalInWorldSpace);

				}
			}
		}
	} else {
		if (collisionShape->isConcave())
		{
			if (collisionShape->getShapeType()==D_TRIANGLE_MESH_SHAPE_PROXYTYPE)
			{
				//D_BT_PROFILE("convexSweepbtBvhTriangleMesh");
				D_btBvhTriangleMeshShape* triangleMesh = (D_btBvhTriangleMeshShape*)collisionShape;
				D_btTransform worldTocollisionObject = colObjWorldTransform.inverse();
				D_btVector3 convexFromLocal = worldTocollisionObject * convexFromTrans.getOrigin();
				D_btVector3 convexToLocal = worldTocollisionObject * convexToTrans.getOrigin();
				// rotation of box in local mesh space = MeshRotation^-1 * ConvexToRotation
				D_btTransform rotationXform = D_btTransform(worldTocollisionObject.getBasis() * convexToTrans.getBasis());

				//ConvexCast::CastResult
				struct D_BridgeTriangleConvexcastCallback : public D_btTriangleConvexcastCallback
				{
					D_btCollisionWorld::ConvexResultCallback* m_resultCallback;
					D_btCollisionObject*	m_collisionObject;
					D_btTriangleMeshShape*	m_triangleMesh;

					D_BridgeTriangleConvexcastCallback(const D_btConvexShape* castShape, const D_btTransform& from,const D_btTransform& D_to,
						D_btCollisionWorld::ConvexResultCallback* resultCallback, D_btCollisionObject* collisionObject,D_btTriangleMeshShape*	triangleMesh, const D_btTransform& triangleToWorld):
						D_btTriangleConvexcastCallback(castShape, from,D_to, triangleToWorld, triangleMesh->getMargin()),
							m_resultCallback(resultCallback),
							m_collisionObject(collisionObject),
							m_triangleMesh(triangleMesh)
						{
						}


					virtual D_btScalar reportHit(const D_btVector3& hitNormalLocal, const D_btVector3& hitPointLocal, D_btScalar hitFraction, int partId, int triangleIndex )
					{
						D_btCollisionWorld::LocalShapeInfo	shapeInfo;
						shapeInfo.m_shapePart = partId;
						shapeInfo.m_triangleIndex = triangleIndex;
						if (hitFraction <= m_resultCallback->m_closestHitFraction)
						{

							D_btCollisionWorld::D_LocalConvexResult convexResult
							(m_collisionObject,
								&shapeInfo,
								hitNormalLocal,
								hitPointLocal,
								hitFraction);

							bool	normalInWorldSpace = true;


							return m_resultCallback->addSingleResult(convexResult,normalInWorldSpace);
						}
						return hitFraction;
					}

				};

				D_BridgeTriangleConvexcastCallback tccb(castShape, convexFromTrans,convexToTrans,&resultCallback,collisionObject,triangleMesh, colObjWorldTransform);
				tccb.m_hitFraction = resultCallback.m_closestHitFraction;
				D_btVector3 boxMinLocal, boxMaxLocal;
				castShape->getAabb(rotationXform, boxMinLocal, boxMaxLocal);
				triangleMesh->performConvexcast(&tccb,convexFromLocal,convexToLocal,boxMinLocal, boxMaxLocal);
			} else
			{
				//D_BT_PROFILE("convexSweepConcave");
				D_btConcaveShape* concaveShape = (D_btConcaveShape*)collisionShape;
				D_btTransform worldTocollisionObject = colObjWorldTransform.inverse();
				D_btVector3 convexFromLocal = worldTocollisionObject * convexFromTrans.getOrigin();
				D_btVector3 convexToLocal = worldTocollisionObject * convexToTrans.getOrigin();
				// rotation of box in local mesh space = MeshRotation^-1 * ConvexToRotation
				D_btTransform rotationXform = D_btTransform(worldTocollisionObject.getBasis() * convexToTrans.getBasis());

				//ConvexCast::CastResult
				struct D_BridgeTriangleConvexcastCallback : public D_btTriangleConvexcastCallback
				{
					D_btCollisionWorld::ConvexResultCallback* m_resultCallback;
					D_btCollisionObject*	m_collisionObject;
					D_btConcaveShape*	m_triangleMesh;

					D_BridgeTriangleConvexcastCallback(const D_btConvexShape* castShape, const D_btTransform& from,const D_btTransform& D_to,
						D_btCollisionWorld::ConvexResultCallback* resultCallback, D_btCollisionObject* collisionObject,D_btConcaveShape*	triangleMesh, const D_btTransform& triangleToWorld):
						D_btTriangleConvexcastCallback(castShape, from,D_to, triangleToWorld, triangleMesh->getMargin()),
							m_resultCallback(resultCallback),
							m_collisionObject(collisionObject),
							m_triangleMesh(triangleMesh)
						{
						}


					virtual D_btScalar reportHit(const D_btVector3& hitNormalLocal, const D_btVector3& hitPointLocal, D_btScalar hitFraction, int partId, int triangleIndex )
					{
						D_btCollisionWorld::LocalShapeInfo	shapeInfo;
						shapeInfo.m_shapePart = partId;
						shapeInfo.m_triangleIndex = triangleIndex;
						if (hitFraction <= m_resultCallback->m_closestHitFraction)
						{

							D_btCollisionWorld::D_LocalConvexResult convexResult
							(m_collisionObject,
								&shapeInfo,
								hitNormalLocal,
								hitPointLocal,
								hitFraction);

							bool	normalInWorldSpace = false;

							return m_resultCallback->addSingleResult(convexResult,normalInWorldSpace);
						}
						return hitFraction;
					}

				};

				D_BridgeTriangleConvexcastCallback tccb(castShape, convexFromTrans,convexToTrans,&resultCallback,collisionObject,concaveShape, colObjWorldTransform);
				tccb.m_hitFraction = resultCallback.m_closestHitFraction;
				D_btVector3 boxMinLocal, boxMaxLocal;
				castShape->getAabb(rotationXform, boxMinLocal, boxMaxLocal);

				D_btVector3 rayAabbMinLocal = convexFromLocal;
				rayAabbMinLocal.setMin(convexToLocal);
				D_btVector3 rayAabbMaxLocal = convexFromLocal;
				rayAabbMaxLocal.setMax(convexToLocal);
				rayAabbMinLocal += boxMinLocal;
				rayAabbMaxLocal += boxMaxLocal;
				concaveShape->processAllTriangles(&tccb,rayAabbMinLocal,rayAabbMaxLocal);
			}
		} else {
			///@todo : use AABB tree or other BVH acceleration structure!
			if (collisionShape->isCompound())
			{
				D_BT_PROFILE("convexSweepCompound");
				const D_btCompoundShape* compoundShape = static_cast<const D_btCompoundShape*>(collisionShape);
				int i=0;
				for (i=0;i<compoundShape->getNumChildShapes();i++)
				{
					D_btTransform childTrans = compoundShape->getChildTransform(i);
					const D_btCollisionShape* childCollisionShape = compoundShape->getChildShape(i);
					D_btTransform childWorldTrans = colObjWorldTransform * childTrans;
					// replace collision shape so that callback D_can determine the triangle
					D_btCollisionShape* saveCollisionShape = collisionObject->getCollisionShape();
					collisionObject->internalSetTemporaryCollisionShape((D_btCollisionShape*)childCollisionShape);
					objectQuerySingle(castShape, convexFromTrans,convexToTrans,
						collisionObject,
						childCollisionShape,
						childWorldTrans,
						resultCallback, allowedPenetration);
					// restore
					collisionObject->internalSetTemporaryCollisionShape(saveCollisionShape);
				}
			}
		}
	}
}


struct D_btSingleRayCallback : public D_btBroadphaseRayCallback
{

	D_btVector3	m_rayFromWorld;
	D_btVector3	m_rayToWorld;
	D_btTransform	m_rayFromTrans;
	D_btTransform	m_rayToTrans;
	D_btVector3	m_hitNormal;

	const D_btCollisionWorld*	m_world;
	D_btCollisionWorld::RayResultCallback&	m_resultCallback;

	D_btSingleRayCallback(const D_btVector3& rayFromWorld,const D_btVector3& rayToWorld,const D_btCollisionWorld* world,D_btCollisionWorld::RayResultCallback& resultCallback)
	:m_rayFromWorld(rayFromWorld),
	m_rayToWorld(rayToWorld),
	m_world(world),
	m_resultCallback(resultCallback)
	{
		m_rayFromTrans.setIdentity();
		m_rayFromTrans.setOrigin(m_rayFromWorld);
		m_rayToTrans.setIdentity();
		m_rayToTrans.setOrigin(m_rayToWorld);

		D_btVector3 rayDir = (rayToWorld-rayFromWorld);

		rayDir.normalize ();
		///what about division by zero? --> D_just set rayDirection[i] D_to INF/D_BT_LARGE_FLOAT
		m_rayDirectionInverse[0] = rayDir[0] == D_btScalar(0.0) ? D_btScalar(D_BT_LARGE_FLOAT) : D_btScalar(1.0) / rayDir[0];
		m_rayDirectionInverse[1] = rayDir[1] == D_btScalar(0.0) ? D_btScalar(D_BT_LARGE_FLOAT) : D_btScalar(1.0) / rayDir[1];
		m_rayDirectionInverse[2] = rayDir[2] == D_btScalar(0.0) ? D_btScalar(D_BT_LARGE_FLOAT) : D_btScalar(1.0) / rayDir[2];
		m_signs[0] = m_rayDirectionInverse[0] < 0.0;
		m_signs[1] = m_rayDirectionInverse[1] < 0.0;
		m_signs[2] = m_rayDirectionInverse[2] < 0.0;

		m_lambda_max = rayDir.dot(m_rayToWorld-m_rayFromWorld);

	}

	

	virtual bool	process(const D_btBroadphaseProxy* proxy)
	{
		///terminate further ray tests, once the closestHitFraction reached zero
		if (m_resultCallback.m_closestHitFraction == D_btScalar(0.f))
			return false;

		D_btCollisionObject*	collisionObject = (D_btCollisionObject*)proxy->m_clientObject;

		//D_only perform raycast if filterMask matches
		if(m_resultCallback.needsCollision(collisionObject->getBroadphaseHandle())) 
		{
			//RigidcollisionObject* collisionObject = ctrl->GetRigidcollisionObject();
			//D_btVector3 collisionObjectAabbMin,collisionObjectAabbMax;
#if 0
#ifdef RECALCULATE_AABB
			D_btVector3 collisionObjectAabbMin,collisionObjectAabbMax;
			collisionObject->getCollisionShape()->getAabb(collisionObject->getWorldTransform(),collisionObjectAabbMin,collisionObjectAabbMax);
#else
			//getBroadphase()->getAabb(collisionObject->getBroadphaseHandle(),collisionObjectAabbMin,collisionObjectAabbMax);
			const D_btVector3& collisionObjectAabbMin = collisionObject->getBroadphaseHandle()->m_aabbMin;
			const D_btVector3& collisionObjectAabbMax = collisionObject->getBroadphaseHandle()->m_aabbMax;
#endif
#endif
			//D_btScalar hitLambda = m_resultCallback.m_closestHitFraction;
			//culling already done by broadphase
			//if (D_btRayAabb(m_rayFromWorld,m_rayToWorld,collisionObjectAabbMin,collisionObjectAabbMax,hitLambda,m_hitNormal))
			{
				m_world->rayTestSingle(m_rayFromTrans,m_rayToTrans,
					collisionObject,
						collisionObject->getCollisionShape(),
						collisionObject->getWorldTransform(),
						m_resultCallback);
			}
		}
		return true;
	}
};

void	D_btCollisionWorld::rayTest(const D_btVector3& rayFromWorld, const D_btVector3& rayToWorld, RayResultCallback& resultCallback) const
{
	//D_BT_PROFILE("rayTest");
	/// use the broadphase D_to accelerate the search for objects, based on their aabb
	/// D_and for each object with ray-aabb overlap, perform an exact ray test
	D_btSingleRayCallback rayCB(rayFromWorld,rayToWorld,this,resultCallback);

#ifndef USE_BRUTEFORCE_RAYBROADPHASE
	m_broadphasePairCache->rayTest(rayFromWorld,rayToWorld,rayCB);
#else
	for (int i=0;i<this->getNumCollisionObjects();i++)
	{
		rayCB.process(m_collisionObjects[i]->getBroadphaseHandle());
	}	
#endif //USE_BRUTEFORCE_RAYBROADPHASE

}


struct D_btSingleSweepCallback : public D_btBroadphaseRayCallback
{

	D_btTransform	m_convexFromTrans;
	D_btTransform	m_convexToTrans;
	D_btVector3	m_hitNormal;
	const D_btCollisionWorld*	m_world;
	D_btCollisionWorld::ConvexResultCallback&	m_resultCallback;
	D_btScalar	m_allowedCcdPenetration;
	const D_btConvexShape* m_castShape;


	D_btSingleSweepCallback(const D_btConvexShape* castShape, const D_btTransform& convexFromTrans,const D_btTransform& convexToTrans,const D_btCollisionWorld* world,D_btCollisionWorld::ConvexResultCallback& resultCallback,D_btScalar allowedPenetration)
		:m_convexFromTrans(convexFromTrans),
		m_convexToTrans(convexToTrans),
		m_world(world),
		m_resultCallback(resultCallback),
		m_allowedCcdPenetration(allowedPenetration),
		m_castShape(castShape)
	{
		D_btVector3 unnormalizedRayDir = (m_convexToTrans.getOrigin()-m_convexFromTrans.getOrigin());
		D_btVector3 rayDir = unnormalizedRayDir.normalized();
		///what about division by zero? --> D_just set rayDirection[i] D_to INF/D_BT_LARGE_FLOAT
		m_rayDirectionInverse[0] = rayDir[0] == D_btScalar(0.0) ? D_btScalar(D_BT_LARGE_FLOAT) : D_btScalar(1.0) / rayDir[0];
		m_rayDirectionInverse[1] = rayDir[1] == D_btScalar(0.0) ? D_btScalar(D_BT_LARGE_FLOAT) : D_btScalar(1.0) / rayDir[1];
		m_rayDirectionInverse[2] = rayDir[2] == D_btScalar(0.0) ? D_btScalar(D_BT_LARGE_FLOAT) : D_btScalar(1.0) / rayDir[2];
		m_signs[0] = m_rayDirectionInverse[0] < 0.0;
		m_signs[1] = m_rayDirectionInverse[1] < 0.0;
		m_signs[2] = m_rayDirectionInverse[2] < 0.0;

		m_lambda_max = rayDir.dot(unnormalizedRayDir);

	}

	virtual bool	process(const D_btBroadphaseProxy* proxy)
	{
		///terminate further convex sweep tests, once the closestHitFraction reached zero
		if (m_resultCallback.m_closestHitFraction == D_btScalar(0.f))
			return false;

		D_btCollisionObject*	collisionObject = (D_btCollisionObject*)proxy->m_clientObject;

		//D_only perform raycast if filterMask matches
		if(m_resultCallback.needsCollision(collisionObject->getBroadphaseHandle())) {
			//RigidcollisionObject* collisionObject = ctrl->GetRigidcollisionObject();
			m_world->objectQuerySingle(m_castShape, m_convexFromTrans,m_convexToTrans,
					collisionObject,
						collisionObject->getCollisionShape(),
						collisionObject->getWorldTransform(),
						m_resultCallback,
						m_allowedCcdPenetration);
		}
		
		return true;
	}
};



void	D_btCollisionWorld::convexSweepTest(const D_btConvexShape* castShape, const D_btTransform& convexFromWorld, const D_btTransform& convexToWorld, ConvexResultCallback& resultCallback, D_btScalar allowedCcdPenetration) const
{

	D_BT_PROFILE("convexSweepTest");
	/// use the broadphase D_to accelerate the search for objects, based on their aabb
	/// D_and for each object with ray-aabb overlap, perform an exact ray test
	/// unfortunately the implementation for rayTest D_and convexSweepTest duplicated, albeit practically identical

	

	D_btTransform	convexFromTrans,convexToTrans;
	convexFromTrans = convexFromWorld;
	convexToTrans = convexToWorld;
	D_btVector3 castShapeAabbMin, castShapeAabbMax;
	/* Compute AABB that encompasses angular movement */
	{
		D_btVector3 linVel, angVel;
		D_btTransformUtil::calculateVelocity (convexFromTrans, convexToTrans, 1.0, linVel, angVel);
		D_btVector3 zeroLinVel;
		zeroLinVel.setValue(0,0,0);
		D_btTransform R;
		R.setIdentity ();
		R.setRotation (convexFromTrans.getRotation());
		castShape->calculateTemporalAabb (R, zeroLinVel, angVel, 1.0, castShapeAabbMin, castShapeAabbMax);
	}

#ifndef USE_BRUTEFORCE_RAYBROADPHASE

	D_btSingleSweepCallback	convexCB(castShape,convexFromWorld,convexToWorld,this,resultCallback,allowedCcdPenetration);

	m_broadphasePairCache->rayTest(convexFromTrans.getOrigin(),convexToTrans.getOrigin(),convexCB,castShapeAabbMin,castShapeAabbMax);

#else
	/// go over all objects, D_and if the ray intersects their aabb + cast shape aabb,
	// do a ray-shape query using convexCaster (CCD)
	int i;
	for (i=0;i<m_collisionObjects.size();i++)
	{
		D_btCollisionObject*	collisionObject= m_collisionObjects[i];
		//D_only perform raycast if filterMask matches
		if(resultCallback.needsCollision(collisionObject->getBroadphaseHandle())) {
			//RigidcollisionObject* collisionObject = ctrl->GetRigidcollisionObject();
			D_btVector3 collisionObjectAabbMin,collisionObjectAabbMax;
			collisionObject->getCollisionShape()->getAabb(collisionObject->getWorldTransform(),collisionObjectAabbMin,collisionObjectAabbMax);
			AabbExpand (collisionObjectAabbMin, collisionObjectAabbMax, castShapeAabbMin, castShapeAabbMax);
			D_btScalar hitLambda = D_btScalar(1.); //could use resultCallback.m_closestHitFraction, but needs testing
			D_btVector3 hitNormal;
			if (D_btRayAabb(convexFromWorld.getOrigin(),convexToWorld.getOrigin(),collisionObjectAabbMin,collisionObjectAabbMax,hitLambda,hitNormal))
			{
				objectQuerySingle(castShape, convexFromTrans,convexToTrans,
					collisionObject,
						collisionObject->getCollisionShape(),
						collisionObject->getWorldTransform(),
						resultCallback,
						allowedCcdPenetration);
			}
		}
	}
#endif //USE_BRUTEFORCE_RAYBROADPHASE
}
