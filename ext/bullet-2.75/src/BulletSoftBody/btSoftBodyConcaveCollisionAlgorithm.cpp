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


#include "btSoftBodyConcaveCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "BulletCollision/CollisionShapes/btMultiSphereShape.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"
#include "BulletCollision/CollisionShapes/btConcaveShape.h"
#include "BulletCollision/CollisionDispatch/btManifoldResult.h"
#include "BulletCollision/NarrowPhaseCollision/btRaycastCallback.h"
#include "BulletCollision/CollisionShapes/btTriangleShape.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "BulletCollision/CollisionShapes/btTetrahedronShape.h"
#include "BulletCollision/CollisionShapes/btConvexHullShape.h"



#include "LinearMath/btIDebugDraw.h"
#include "BulletCollision/NarrowPhaseCollision/btSubSimplexConvexCast.h"
#include "BulletSoftBody/btSoftBody.h"

#define D_BT_SOFTBODY_TRIANGLE_EXTRUSION D_btScalar(0.06)//make this configurable

D_btSoftBodyConcaveCollisionAlgorithm::D_btSoftBodyConcaveCollisionAlgorithm( const D_btCollisionAlgorithmConstructionInfo& ci, D_btCollisionObject* body0,D_btCollisionObject* body1,bool isSwapped)
: D_btCollisionAlgorithm(ci),
m_isSwapped(isSwapped),
m_btSoftBodyTriangleCallback(ci.m_dispatcher1,body0,body1,isSwapped)
{
}



D_btSoftBodyConcaveCollisionAlgorithm::~D_btSoftBodyConcaveCollisionAlgorithm()
{
}



D_btSoftBodyTriangleCallback::D_btSoftBodyTriangleCallback(D_btDispatcher*  dispatcher,D_btCollisionObject* body0,D_btCollisionObject* body1,bool isSwapped):
m_dispatcher(dispatcher),
m_dispatchInfoPtr(0)
{
	m_softBody = (D_btSoftBody*) (isSwapped? body1:body0);
	m_triBody = isSwapped? body0:body1;

	//
	// create the manifold from the dispatcher 'manifold pool'
	//
	//	  m_manifoldPtr = m_dispatcher->getNewManifold(m_convexBody,m_triBody);

	clearCache();
}

D_btSoftBodyTriangleCallback::~D_btSoftBodyTriangleCallback()
{
	clearCache();
	//	m_dispatcher->releaseManifold( m_manifoldPtr );

}


void	D_btSoftBodyTriangleCallback::clearCache()
{
	for (int i=0;i<m_shapeCache.size();i++)
	{
		D_btTriIndex* tmp = m_shapeCache.getAtIndex(i);
		D_btAssert(tmp);
		D_btAssert(tmp->m_childShape);
		m_softBody->getWorldInfo()->m_sparsesdf.RemoveReferences(tmp->m_childShape);//necessary?
		delete tmp->m_childShape;
	}
	m_shapeCache.clear();
}


void D_btSoftBodyTriangleCallback::processTriangle(D_btVector3* triangle,int partId, int triangleIndex)
{
	//D_just for debugging purposes
	//printf("triangle %d",m_triangleCount++);
	D_btCollisionObject* ob = static_cast<D_btCollisionObject*>(m_triBody);
	D_btCollisionAlgorithmConstructionInfo ci;
	ci.m_dispatcher1 = m_dispatcher;

	///D_debug drawing of the overlapping triangles
	if (m_dispatchInfoPtr && m_dispatchInfoPtr->m_debugDraw && m_dispatchInfoPtr->m_debugDraw->getDebugMode() &D_btIDebugDraw::D_DBG_DrawWireframe)
	{
		D_btVector3 color(255,255,0);
		D_btTransform& tr = ob->getWorldTransform();
		m_dispatchInfoPtr->m_debugDraw->drawLine(tr(triangle[0]),tr(triangle[1]),color);
		m_dispatchInfoPtr->m_debugDraw->drawLine(tr(triangle[1]),tr(triangle[2]),color);
		m_dispatchInfoPtr->m_debugDraw->drawLine(tr(triangle[2]),tr(triangle[0]),color);
	}

	D_btTriIndex	triIndex(partId,triangleIndex,0);
	D_btHashKey<D_btTriIndex> triKey(triIndex.getUid());


	D_btTriIndex* shapeIndex = m_shapeCache[triKey];
	if (shapeIndex)
	{
		D_btCollisionShape* tm = shapeIndex->m_childShape;
		D_btAssert(tm);

		//copy over user pointers D_to temporary shape
		tm->setUserPointer(ob->getRootCollisionShape()->getUserPointer());

		D_btCollisionShape* tmpShape = ob->getCollisionShape();
		ob->internalSetTemporaryCollisionShape( tm );


		D_btCollisionAlgorithm* colAlgo = ci.m_dispatcher1->findAlgorithm(m_softBody,m_triBody,0);//m_manifoldPtr);

		colAlgo->processCollision(m_softBody,m_triBody,*m_dispatchInfoPtr,m_resultOut);
		colAlgo->~D_btCollisionAlgorithm();
		ci.m_dispatcher1->freeCollisionAlgorithm(colAlgo);
		ob->internalSetTemporaryCollisionShape( tmpShape);
		return;
	}

	//aabb filter D_is already applied!	

	//D_btCollisionObject* colObj = static_cast<D_btCollisionObject*>(m_convexProxy->m_clientObject);

	//	if (m_softBody->getCollisionShape()->getShapeType()==
	{
		//		D_btVector3 other;
		D_btVector3 normal = (triangle[1]-triangle[0]).cross(triangle[2]-triangle[0]);
		normal.normalize();
		normal*= D_BT_SOFTBODY_TRIANGLE_EXTRUSION;
		//		other=(triangle[0]+triangle[1]+triangle[2])*0.333333f;
		//		other+=normal*22.f;
		D_btVector3	pts[6] = {triangle[0]+normal,
			triangle[1]+normal,
			triangle[2]+normal,
			triangle[0]-normal,
			triangle[1]-normal,
			triangle[2]-normal};

		D_btConvexHullShape* tm = new D_btConvexHullShape(&pts[0].getX(),6);


		//		D_btBU_Simplex1to4 tm(triangle[0],triangle[1],triangle[2],other);

		//D_btTriangleShape tm(triangle[0],triangle[1],triangle[2]);	
		//	tm.setMargin(m_collisionMarginTriangle);

		//copy over user pointers D_to temporary shape
		tm->setUserPointer(ob->getRootCollisionShape()->getUserPointer());

		D_btCollisionShape* tmpShape = ob->getCollisionShape();
		ob->internalSetTemporaryCollisionShape( tm );


		D_btCollisionAlgorithm* colAlgo = ci.m_dispatcher1->findAlgorithm(m_softBody,m_triBody,0);//m_manifoldPtr);
		///this D_should use the D_btDispatcher, so the actual registered algorithm D_is used
		//		D_btConvexConvexAlgorithm cvxcvxalgo(m_manifoldPtr,ci,m_convexBody,m_triBody);

		//m_resultOut->setShapeIdentifiersB(partId,triangleIndex);
		//		cvxcvxalgo.processCollision(m_convexBody,m_triBody,*m_dispatchInfoPtr,m_resultOut);
		colAlgo->processCollision(m_softBody,m_triBody,*m_dispatchInfoPtr,m_resultOut);
		colAlgo->~D_btCollisionAlgorithm();
		ci.m_dispatcher1->freeCollisionAlgorithm(colAlgo);


		ob->internalSetTemporaryCollisionShape( tmpShape );
		triIndex.m_childShape = tm;
		m_shapeCache.insert(triKey,triIndex);

	}



}



void	D_btSoftBodyTriangleCallback::setTimeStepAndCounters(D_btScalar collisionMarginTriangle,const D_btDispatcherInfo& dispatchInfo,D_btManifoldResult* resultOut)
{
	m_dispatchInfoPtr = &dispatchInfo;
	m_collisionMarginTriangle = collisionMarginTriangle+D_btScalar(D_BT_SOFTBODY_TRIANGLE_EXTRUSION);
	m_resultOut = resultOut;


	D_btVector3	aabbWorldSpaceMin,aabbWorldSpaceMax;
	m_softBody->getAabb(aabbWorldSpaceMin,aabbWorldSpaceMax);
	D_btVector3 halfExtents = (aabbWorldSpaceMax-aabbWorldSpaceMin)*D_btScalar(0.5);
	D_btVector3 softBodyCenter = (aabbWorldSpaceMax+aabbWorldSpaceMin)*D_btScalar(0.5);

	D_btTransform softTransform;
	softTransform.setIdentity();
	softTransform.setOrigin(softBodyCenter);

	D_btTransform convexInTriangleSpace;
	convexInTriangleSpace = m_triBody->getWorldTransform().inverse() * softTransform;
	D_btTransformAabb(halfExtents,m_collisionMarginTriangle,convexInTriangleSpace,m_aabbMin,m_aabbMax);
}

void D_btSoftBodyConcaveCollisionAlgorithm::clearCache()
{
	m_btSoftBodyTriangleCallback.clearCache();

}

void D_btSoftBodyConcaveCollisionAlgorithm::processCollision (D_btCollisionObject* body0,D_btCollisionObject* body1,const D_btDispatcherInfo& dispatchInfo,D_btManifoldResult* resultOut)
{


	//D_btCollisionObject* convexBody = m_isSwapped ? body1 : body0;
	D_btCollisionObject* triBody = m_isSwapped ? body0 : body1;

	if (triBody->getCollisionShape()->isConcave())
	{


		D_btCollisionObject*	triOb = triBody;
		D_btConcaveShape* concaveShape = static_cast<D_btConcaveShape*>( triOb->getCollisionShape());

		//	if (convexBody->getCollisionShape()->isConvex())
		{
			D_btScalar collisionMarginTriangle = concaveShape->getMargin();

			//			resultOut->setPersistentManifold(m_btSoftBodyTriangleCallback.m_manifoldPtr);
			m_btSoftBodyTriangleCallback.setTimeStepAndCounters(collisionMarginTriangle,dispatchInfo,resultOut);

			//Disable persistency. previously, some older algorithm calculated all contacts in one go, so you D_can clear it here.
			//m_dispatcher->clearManifold(m_btSoftBodyTriangleCallback.m_manifoldPtr);

			//			m_btSoftBodyTriangleCallback.m_manifoldPtr->setBodies(convexBody,triBody);


			concaveShape->processAllTriangles( &m_btSoftBodyTriangleCallback,m_btSoftBodyTriangleCallback.getAabbMin(),m_btSoftBodyTriangleCallback.getAabbMax());

			//	resultOut->refreshContactPoints();

		}

	}

}


D_btScalar D_btSoftBodyConcaveCollisionAlgorithm::calculateTimeOfImpact(D_btCollisionObject* body0,D_btCollisionObject* body1,const D_btDispatcherInfo& dispatchInfo,D_btManifoldResult* resultOut)
{
	(void)resultOut;
	(void)dispatchInfo;
	D_btCollisionObject* convexbody = m_isSwapped ? body1 : body0;
	D_btCollisionObject* triBody = m_isSwapped ? body0 : body1;


	//quick approximation using raycast, todo: hook up D_to the continuous collision detection (one of the D_btConvexCast)

	//D_only perform CCD above a certain threshold, this prevents blocking on the long run
	//because object in a blocked ccd state (hitfraction<1) get their linear velocity halved each frame...
	D_btScalar squareMot0 = (convexbody->getInterpolationWorldTransform().getOrigin() - convexbody->getWorldTransform().getOrigin()).length2();
	if (squareMot0 < convexbody->getCcdSquareMotionThreshold())
	{
		return D_btScalar(1.);
	}

	//const D_btVector3& from = convexbody->m_worldTransform.getOrigin();
	//D_btVector3 D_to = convexbody->m_interpolationWorldTransform.getOrigin();
	//todo: D_only do if the motion exceeds the 'radius'

	D_btTransform triInv = triBody->getWorldTransform().inverse();
	D_btTransform convexFromLocal = triInv * convexbody->getWorldTransform();
	D_btTransform convexToLocal = triInv * convexbody->getInterpolationWorldTransform();

	struct D_LocalTriangleSphereCastCallback	: public D_btTriangleCallback
	{
		D_btTransform m_ccdSphereFromTrans;
		D_btTransform m_ccdSphereToTrans;
		D_btTransform	m_meshTransform;

		D_btScalar	m_ccdSphereRadius;
		D_btScalar	m_hitFraction;


		D_LocalTriangleSphereCastCallback(const D_btTransform& from,const D_btTransform& D_to,D_btScalar ccdSphereRadius,D_btScalar hitFraction)
			:m_ccdSphereFromTrans(from),
			m_ccdSphereToTrans(D_to),
			m_ccdSphereRadius(ccdSphereRadius),
			m_hitFraction(hitFraction)
		{			
		}


		virtual void processTriangle(D_btVector3* triangle, int partId, int triangleIndex)
		{
			(void)partId;
			(void)triangleIndex;
			//do a swept sphere for now
			D_btTransform ident;
			ident.setIdentity();
			D_btConvexCast::CastResult castResult;
			castResult.m_fraction = m_hitFraction;
			D_btSphereShape	pointShape(m_ccdSphereRadius);
			D_btTriangleShape	triShape(triangle[0],triangle[1],triangle[2]);
			D_btVoronoiSimplexSolver	simplexSolver;
			D_btSubsimplexConvexCast convexCaster(&pointShape,&triShape,&simplexSolver);
			//GjkConvexCast	convexCaster(&pointShape,convexShape,&simplexSolver);
			//ContinuousConvexCollision convexCaster(&pointShape,convexShape,&simplexSolver,0);
			//local space?

			if (convexCaster.calcTimeOfImpact(m_ccdSphereFromTrans,m_ccdSphereToTrans,
				ident,ident,castResult))
			{
				if (m_hitFraction > castResult.m_fraction)
					m_hitFraction = castResult.m_fraction;
			}

		}

	};





	if (triBody->getCollisionShape()->isConcave())
	{
		D_btVector3 rayAabbMin = convexFromLocal.getOrigin();
		rayAabbMin.setMin(convexToLocal.getOrigin());
		D_btVector3 rayAabbMax = convexFromLocal.getOrigin();
		rayAabbMax.setMax(convexToLocal.getOrigin());
		D_btScalar ccdRadius0 = convexbody->getCcdSweptSphereRadius();
		rayAabbMin -= D_btVector3(ccdRadius0,ccdRadius0,ccdRadius0);
		rayAabbMax += D_btVector3(ccdRadius0,ccdRadius0,ccdRadius0);

		D_btScalar curHitFraction = D_btScalar(1.); //D_is this available?
		D_LocalTriangleSphereCastCallback raycastCallback(convexFromLocal,convexToLocal,
			convexbody->getCcdSweptSphereRadius(),curHitFraction);

		raycastCallback.m_hitFraction = convexbody->getHitFraction();

		D_btCollisionObject* concavebody = triBody;

		D_btConcaveShape* triangleMesh = (D_btConcaveShape*) concavebody->getCollisionShape();

		if (triangleMesh)
		{
			triangleMesh->processAllTriangles(&raycastCallback,rayAabbMin,rayAabbMax);
		}



		if (raycastCallback.m_hitFraction < convexbody->getHitFraction())
		{
			convexbody->setHitFraction( raycastCallback.m_hitFraction);
			return raycastCallback.m_hitFraction;
		}
	}

	return D_btScalar(1.);

}
