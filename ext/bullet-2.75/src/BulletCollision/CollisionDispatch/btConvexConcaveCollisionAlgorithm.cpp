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


#include "btConvexConcaveCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "BulletCollision/CollisionShapes/btMultiSphereShape.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"
#include "BulletCollision/CollisionShapes/btConcaveShape.h"
#include "BulletCollision/CollisionDispatch/btManifoldResult.h"
#include "BulletCollision/NarrowPhaseCollision/btRaycastCallback.h"
#include "BulletCollision/CollisionShapes/btTriangleShape.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "LinearMath/btIDebugDraw.h"
#include "BulletCollision/NarrowPhaseCollision/btSubSimplexConvexCast.h"

D_btConvexConcaveCollisionAlgorithm::D_btConvexConcaveCollisionAlgorithm( const D_btCollisionAlgorithmConstructionInfo& ci, D_btCollisionObject* body0,D_btCollisionObject* body1,bool isSwapped)
: D_btActivatingCollisionAlgorithm(ci,body0,body1),
m_isSwapped(isSwapped),
m_btConvexTriangleCallback(ci.m_dispatcher1,body0,body1,isSwapped)
{
}

D_btConvexConcaveCollisionAlgorithm::~D_btConvexConcaveCollisionAlgorithm()
{
}

void	D_btConvexConcaveCollisionAlgorithm::getAllContactManifolds(D_btManifoldArray&	manifoldArray)
{
	if (m_btConvexTriangleCallback.m_manifoldPtr)
	{
		manifoldArray.push_back(m_btConvexTriangleCallback.m_manifoldPtr);
	}
}


D_btConvexTriangleCallback::D_btConvexTriangleCallback(D_btDispatcher*  dispatcher,D_btCollisionObject* body0,D_btCollisionObject* body1,bool isSwapped):
	  m_dispatcher(dispatcher),
	m_dispatchInfoPtr(0)
{
	m_convexBody = isSwapped? body1:body0;
	m_triBody = isSwapped? body0:body1;
	
	  //
	  // create the manifold from the dispatcher 'manifold pool'
	  //
	  m_manifoldPtr = m_dispatcher->getNewManifold(m_convexBody,m_triBody);

  	  clearCache();
}

D_btConvexTriangleCallback::~D_btConvexTriangleCallback()
{
	clearCache();
	m_dispatcher->releaseManifold( m_manifoldPtr );
  
}
  

void	D_btConvexTriangleCallback::clearCache()
{
	m_dispatcher->clearManifold(m_manifoldPtr);
}



void D_btConvexTriangleCallback::processTriangle(D_btVector3* triangle,int partId, int triangleIndex)
{
 
	//D_just for debugging purposes
	//printf("triangle %d",m_triangleCount++);


	//aabb filter D_is already applied!	

	D_btCollisionAlgorithmConstructionInfo ci;
	ci.m_dispatcher1 = m_dispatcher;

	D_btCollisionObject* ob = static_cast<D_btCollisionObject*>(m_triBody);


	
	///D_debug drawing of the overlapping triangles
	if (m_dispatchInfoPtr && m_dispatchInfoPtr->m_debugDraw && (m_dispatchInfoPtr->m_debugDraw->getDebugMode() &D_btIDebugDraw::D_DBG_DrawWireframe ))
	{
		D_btVector3 color(255,255,0);
		D_btTransform& tr = ob->getWorldTransform();
		m_dispatchInfoPtr->m_debugDraw->drawLine(tr(triangle[0]),tr(triangle[1]),color);
		m_dispatchInfoPtr->m_debugDraw->drawLine(tr(triangle[1]),tr(triangle[2]),color);
		m_dispatchInfoPtr->m_debugDraw->drawLine(tr(triangle[2]),tr(triangle[0]),color);

		//D_btVector3 center = triangle[0] + triangle[1]+triangle[2];
		//center *= D_btScalar(0.333333);
		//m_dispatchInfoPtr->m_debugDraw->drawLine(tr(triangle[0]),tr(center),color);
		//m_dispatchInfoPtr->m_debugDraw->drawLine(tr(triangle[1]),tr(center),color);
		//m_dispatchInfoPtr->m_debugDraw->drawLine(tr(triangle[2]),tr(center),color);

	}


	//D_btCollisionObject* colObj = static_cast<D_btCollisionObject*>(m_convexProxy->m_clientObject);
	
	if (m_convexBody->getCollisionShape()->isConvex())
	{
		D_btTriangleShape tm(triangle[0],triangle[1],triangle[2]);	
		tm.setMargin(m_collisionMarginTriangle);
		
		D_btCollisionShape* tmpShape = ob->getCollisionShape();
		ob->internalSetTemporaryCollisionShape( &tm );
		
		D_btCollisionAlgorithm* colAlgo = ci.m_dispatcher1->findAlgorithm(m_convexBody,m_triBody,m_manifoldPtr);
		///this D_should use the D_btDispatcher, so the actual registered algorithm D_is used
		//		D_btConvexConvexAlgorithm cvxcvxalgo(m_manifoldPtr,ci,m_convexBody,m_triBody);

		m_resultOut->setShapeIdentifiersB(partId,triangleIndex);
	
//		cvxcvxalgo.processCollision(m_convexBody,m_triBody,*m_dispatchInfoPtr,m_resultOut);
		colAlgo->processCollision(m_convexBody,m_triBody,*m_dispatchInfoPtr,m_resultOut);
		colAlgo->~D_btCollisionAlgorithm();
		ci.m_dispatcher1->freeCollisionAlgorithm(colAlgo);
		ob->internalSetTemporaryCollisionShape( tmpShape);
	}


}



void	D_btConvexTriangleCallback::setTimeStepAndCounters(D_btScalar collisionMarginTriangle,const D_btDispatcherInfo& dispatchInfo,D_btManifoldResult* resultOut)
{
	m_dispatchInfoPtr = &dispatchInfo;
	m_collisionMarginTriangle = collisionMarginTriangle;
	m_resultOut = resultOut;

	//recalc aabbs
	D_btTransform convexInTriangleSpace;
	convexInTriangleSpace = m_triBody->getWorldTransform().inverse() * m_convexBody->getWorldTransform();
	D_btCollisionShape* convexShape = static_cast<D_btCollisionShape*>(m_convexBody->getCollisionShape());
	//CollisionShape* triangleShape = static_cast<D_btCollisionShape*>(triBody->m_collisionShape);
	convexShape->getAabb(convexInTriangleSpace,m_aabbMin,m_aabbMax);
	D_btScalar extraMargin = collisionMarginTriangle;
	D_btVector3 extra(extraMargin,extraMargin,extraMargin);

	m_aabbMax += extra;
	m_aabbMin -= extra;
	
}

void D_btConvexConcaveCollisionAlgorithm::clearCache()
{
	m_btConvexTriangleCallback.clearCache();

}

void D_btConvexConcaveCollisionAlgorithm::processCollision (D_btCollisionObject* body0,D_btCollisionObject* body1,const D_btDispatcherInfo& dispatchInfo,D_btManifoldResult* resultOut)
{
	
	
	D_btCollisionObject* convexBody = m_isSwapped ? body1 : body0;
	D_btCollisionObject* triBody = m_isSwapped ? body0 : body1;

	if (triBody->getCollisionShape()->isConcave())
	{


		D_btCollisionObject*	triOb = triBody;
		D_btConcaveShape* concaveShape = static_cast<D_btConcaveShape*>( triOb->getCollisionShape());
		
		if (convexBody->getCollisionShape()->isConvex())
		{
			D_btScalar collisionMarginTriangle = concaveShape->getMargin();
					
			resultOut->setPersistentManifold(m_btConvexTriangleCallback.m_manifoldPtr);
			m_btConvexTriangleCallback.setTimeStepAndCounters(collisionMarginTriangle,dispatchInfo,resultOut);

			//Disable persistency. previously, some older algorithm calculated all contacts in one go, so you D_can clear it here.
			//m_dispatcher->clearManifold(m_btConvexTriangleCallback.m_manifoldPtr);

			m_btConvexTriangleCallback.m_manifoldPtr->setBodies(convexBody,triBody);

			concaveShape->processAllTriangles( &m_btConvexTriangleCallback,m_btConvexTriangleCallback.getAabbMin(),m_btConvexTriangleCallback.getAabbMax());
			
			resultOut->refreshContactPoints();
	
		}
	
	}

}


D_btScalar D_btConvexConcaveCollisionAlgorithm::calculateTimeOfImpact(D_btCollisionObject* body0,D_btCollisionObject* body1,const D_btDispatcherInfo& dispatchInfo,D_btManifoldResult* resultOut)
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
