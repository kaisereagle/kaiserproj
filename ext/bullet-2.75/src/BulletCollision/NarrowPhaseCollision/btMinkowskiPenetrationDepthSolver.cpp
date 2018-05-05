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

#include "btMinkowskiPenetrationDepthSolver.h"
#include "BulletCollision/NarrowPhaseCollision/btSubSimplexConvexCast.h"
#include "BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h"
#include "BulletCollision/CollisionShapes/btConvexShape.h"

#define D_NUM_UNITSPHERE_POINTS 42
static D_btVector3	D_sPenetrationDirections[D_NUM_UNITSPHERE_POINTS+D_MAX_PREFERRED_PENETRATION_DIRECTIONS*2] = 
{
D_btVector3(D_btScalar(0.000000) , D_btScalar(-0.000000),D_btScalar(-1.000000)),
D_btVector3(D_btScalar(0.723608) , D_btScalar(-0.525725),D_btScalar(-0.447219)),
D_btVector3(D_btScalar(-0.276388) , D_btScalar(-0.850649),D_btScalar(-0.447219)),
D_btVector3(D_btScalar(-0.894426) , D_btScalar(-0.000000),D_btScalar(-0.447216)),
D_btVector3(D_btScalar(-0.276388) , D_btScalar(0.850649),D_btScalar(-0.447220)),
D_btVector3(D_btScalar(0.723608) , D_btScalar(0.525725),D_btScalar(-0.447219)),
D_btVector3(D_btScalar(0.276388) , D_btScalar(-0.850649),D_btScalar(0.447220)),
D_btVector3(D_btScalar(-0.723608) , D_btScalar(-0.525725),D_btScalar(0.447219)),
D_btVector3(D_btScalar(-0.723608) , D_btScalar(0.525725),D_btScalar(0.447219)),
D_btVector3(D_btScalar(0.276388) , D_btScalar(0.850649),D_btScalar(0.447219)),
D_btVector3(D_btScalar(0.894426) , D_btScalar(0.000000),D_btScalar(0.447216)),
D_btVector3(D_btScalar(-0.000000) , D_btScalar(0.000000),D_btScalar(1.000000)),
D_btVector3(D_btScalar(0.425323) , D_btScalar(-0.309011),D_btScalar(-0.850654)),
D_btVector3(D_btScalar(-0.162456) , D_btScalar(-0.499995),D_btScalar(-0.850654)),
D_btVector3(D_btScalar(0.262869) , D_btScalar(-0.809012),D_btScalar(-0.525738)),
D_btVector3(D_btScalar(0.425323) , D_btScalar(0.309011),D_btScalar(-0.850654)),
D_btVector3(D_btScalar(0.850648) , D_btScalar(-0.000000),D_btScalar(-0.525736)),
D_btVector3(D_btScalar(-0.525730) , D_btScalar(-0.000000),D_btScalar(-0.850652)),
D_btVector3(D_btScalar(-0.688190) , D_btScalar(-0.499997),D_btScalar(-0.525736)),
D_btVector3(D_btScalar(-0.162456) , D_btScalar(0.499995),D_btScalar(-0.850654)),
D_btVector3(D_btScalar(-0.688190) , D_btScalar(0.499997),D_btScalar(-0.525736)),
D_btVector3(D_btScalar(0.262869) , D_btScalar(0.809012),D_btScalar(-0.525738)),
D_btVector3(D_btScalar(0.951058) , D_btScalar(0.309013),D_btScalar(0.000000)),
D_btVector3(D_btScalar(0.951058) , D_btScalar(-0.309013),D_btScalar(0.000000)),
D_btVector3(D_btScalar(0.587786) , D_btScalar(-0.809017),D_btScalar(0.000000)),
D_btVector3(D_btScalar(0.000000) , D_btScalar(-1.000000),D_btScalar(0.000000)),
D_btVector3(D_btScalar(-0.587786) , D_btScalar(-0.809017),D_btScalar(0.000000)),
D_btVector3(D_btScalar(-0.951058) , D_btScalar(-0.309013),D_btScalar(-0.000000)),
D_btVector3(D_btScalar(-0.951058) , D_btScalar(0.309013),D_btScalar(-0.000000)),
D_btVector3(D_btScalar(-0.587786) , D_btScalar(0.809017),D_btScalar(-0.000000)),
D_btVector3(D_btScalar(-0.000000) , D_btScalar(1.000000),D_btScalar(-0.000000)),
D_btVector3(D_btScalar(0.587786) , D_btScalar(0.809017),D_btScalar(-0.000000)),
D_btVector3(D_btScalar(0.688190) , D_btScalar(-0.499997),D_btScalar(0.525736)),
D_btVector3(D_btScalar(-0.262869) , D_btScalar(-0.809012),D_btScalar(0.525738)),
D_btVector3(D_btScalar(-0.850648) , D_btScalar(0.000000),D_btScalar(0.525736)),
D_btVector3(D_btScalar(-0.262869) , D_btScalar(0.809012),D_btScalar(0.525738)),
D_btVector3(D_btScalar(0.688190) , D_btScalar(0.499997),D_btScalar(0.525736)),
D_btVector3(D_btScalar(0.525730) , D_btScalar(0.000000),D_btScalar(0.850652)),
D_btVector3(D_btScalar(0.162456) , D_btScalar(-0.499995),D_btScalar(0.850654)),
D_btVector3(D_btScalar(-0.425323) , D_btScalar(-0.309011),D_btScalar(0.850654)),
D_btVector3(D_btScalar(-0.425323) , D_btScalar(0.309011),D_btScalar(0.850654)),
D_btVector3(D_btScalar(0.162456) , D_btScalar(0.499995),D_btScalar(0.850654))
};


bool D_btMinkowskiPenetrationDepthSolver::calcPenDepth(D_btSimplexSolverInterface& simplexSolver,
												   const D_btConvexShape* convexA,const D_btConvexShape* convexB,
												   const D_btTransform& transA,const D_btTransform& transB,
												   D_btVector3& v, D_btVector3& pa, D_btVector3& pb,
												   class D_btIDebugDraw* debugDraw,D_btStackAlloc* stackAlloc
												   )
{

	(void)stackAlloc;
	(void)v;
	
	bool check2d= convexA->isConvex2d() && convexB->isConvex2d();

	struct D_btIntermediateResult : public D_btDiscreteCollisionDetectorInterface::D_Result
	{

		D_btIntermediateResult():m_hasResult(false)
		{
		}
		
		D_btVector3 m_normalOnBInWorld;
		D_btVector3 m_pointInWorld;
		D_btScalar m_depth;
		bool	m_hasResult;

		virtual void setShapeIdentifiersA(int partId0,int index0)
		{
			(void)partId0;
			(void)index0;
		}
		virtual void setShapeIdentifiersB(int partId1,int index1)
		{
			(void)partId1;
			(void)index1;
		}
		void addContactPoint(const D_btVector3& normalOnBInWorld,const D_btVector3& pointInWorld,D_btScalar depth)
		{
			m_normalOnBInWorld = normalOnBInWorld;
			m_pointInWorld = pointInWorld;
			m_depth = depth;
			m_hasResult = true;
		}
	};

	//D_just take fixed number of orientation, D_and sample the penetration depth in that direction
	D_btScalar minProj = D_btScalar(D_BT_LARGE_FLOAT);
	D_btVector3 minNorm(D_btScalar(0.), D_btScalar(0.), D_btScalar(0.));
	D_btVector3 minA,minB;
	D_btVector3 seperatingAxisInA,seperatingAxisInB;
	D_btVector3 pInA,qInB,pWorld,qWorld,w;

#ifndef __SPU__
#define D_USE_BATCHED_SUPPORT 1
#endif
#ifdef D_USE_BATCHED_SUPPORT

	D_btVector3	supportVerticesABatch[D_NUM_UNITSPHERE_POINTS+D_MAX_PREFERRED_PENETRATION_DIRECTIONS*2];
	D_btVector3	supportVerticesBBatch[D_NUM_UNITSPHERE_POINTS+D_MAX_PREFERRED_PENETRATION_DIRECTIONS*2];
	D_btVector3	seperatingAxisInABatch[D_NUM_UNITSPHERE_POINTS+D_MAX_PREFERRED_PENETRATION_DIRECTIONS*2];
	D_btVector3	seperatingAxisInBBatch[D_NUM_UNITSPHERE_POINTS+D_MAX_PREFERRED_PENETRATION_DIRECTIONS*2];
	int i;

	int numSampleDirections = D_NUM_UNITSPHERE_POINTS;

	for (i=0;i<numSampleDirections;i++)
	{
		D_btVector3 norm = D_sPenetrationDirections[i];
		seperatingAxisInABatch[i] =  (-norm) * transA.getBasis() ;
		seperatingAxisInBBatch[i] =  norm   * transB.getBasis() ;
	}

	{
		int numPDA = convexA->getNumPreferredPenetrationDirections();
		if (numPDA)
		{
			for (int i=0;i<numPDA;i++)
			{
				D_btVector3 norm;
				convexA->getPreferredPenetrationDirection(i,norm);
				norm  = transA.getBasis() * norm;
				D_sPenetrationDirections[numSampleDirections] = norm;
				seperatingAxisInABatch[numSampleDirections] = (-norm) * transA.getBasis();
				seperatingAxisInBBatch[numSampleDirections] = norm * transB.getBasis();
				numSampleDirections++;
			}
		}
	}

	{
		int numPDB = convexB->getNumPreferredPenetrationDirections();
		if (numPDB)
		{
			for (int i=0;i<numPDB;i++)
			{
				D_btVector3 norm;
				convexB->getPreferredPenetrationDirection(i,norm);
				norm  = transB.getBasis() * norm;
				D_sPenetrationDirections[numSampleDirections] = norm;
				seperatingAxisInABatch[numSampleDirections] = (-norm) * transA.getBasis();
				seperatingAxisInBBatch[numSampleDirections] = norm * transB.getBasis();
				numSampleDirections++;
			}
		}
	}




	convexA->batchedUnitVectorGetSupportingVertexWithoutMargin(seperatingAxisInABatch,supportVerticesABatch,numSampleDirections);
	convexB->batchedUnitVectorGetSupportingVertexWithoutMargin(seperatingAxisInBBatch,supportVerticesBBatch,numSampleDirections);

	for (i=0;i<numSampleDirections;i++)
	{
		D_btVector3 norm = D_sPenetrationDirections[i];
		if (check2d)
		{
			norm[2] = 0.f;
		}
		if (norm.length2()>0.01)
		{

			seperatingAxisInA = seperatingAxisInABatch[i];
			seperatingAxisInB = seperatingAxisInBBatch[i];

			pInA = supportVerticesABatch[i];
			qInB = supportVerticesBBatch[i];

			pWorld = transA(pInA);	
			qWorld = transB(qInB);
			if (check2d)
			{
				pWorld[2] = 0.f;
				qWorld[2] = 0.f;
			}

			w	= qWorld - pWorld;
			D_btScalar delta = norm.dot(w);
			//find smallest delta
			if (delta < minProj)
			{
				minProj = delta;
				minNorm = norm;
				minA = pWorld;
				minB = qWorld;
			}
		}
	}	
#else

	int numSampleDirections = D_NUM_UNITSPHERE_POINTS;

#ifndef __SPU__
	{
		int numPDA = convexA->getNumPreferredPenetrationDirections();
		if (numPDA)
		{
			for (int i=0;i<numPDA;i++)
			{
				D_btVector3 norm;
				convexA->getPreferredPenetrationDirection(i,norm);
				norm  = transA.getBasis() * norm;
				D_sPenetrationDirections[numSampleDirections] = norm;
				numSampleDirections++;
			}
		}
	}

	{
		int numPDB = convexB->getNumPreferredPenetrationDirections();
		if (numPDB)
		{
			for (int i=0;i<numPDB;i++)
			{
				D_btVector3 norm;
				convexB->getPreferredPenetrationDirection(i,norm);
				norm  = transB.getBasis() * norm;
				D_sPenetrationDirections[numSampleDirections] = norm;
				numSampleDirections++;
			}
		}
	}
#endif // __SPU__

	for (int i=0;i<numSampleDirections;i++)
	{
		const D_btVector3& norm = D_sPenetrationDirections[i];
		seperatingAxisInA = (-norm)* transA.getBasis();
		seperatingAxisInB = norm* transB.getBasis();
		pInA = convexA->localGetSupportVertexWithoutMarginNonVirtual(seperatingAxisInA);
		qInB = convexB->localGetSupportVertexWithoutMarginNonVirtual(seperatingAxisInB);
		pWorld = transA(pInA);	
		qWorld = transB(qInB);
		w	= qWorld - pWorld;
		D_btScalar delta = norm.dot(w);
		//find smallest delta
		if (delta < minProj)
		{
			minProj = delta;
			minNorm = norm;
			minA = pWorld;
			minB = qWorld;
		}
	}
#endif //D_USE_BATCHED_SUPPORT

	//add the margins

	minA += minNorm*convexA->getMarginNonVirtual();
	minB -= minNorm*convexB->getMarginNonVirtual();
	//D_no penetration
	if (minProj < D_btScalar(0.))
		return false;

	D_btScalar extraSeparation = 0.5f;///scale dependent
	minProj += extraSeparation+(convexA->getMarginNonVirtual() + convexB->getMarginNonVirtual());





//#define DEBUG_DRAW 1
#ifdef DEBUG_DRAW
	if (debugDraw)
	{
		D_btVector3 color(0,1,0);
		debugDraw->drawLine(minA,minB,color);
		color = D_btVector3 (1,1,1);
		D_btVector3 vec = minB-minA;
		D_btScalar prj2 = minNorm.dot(vec);
		debugDraw->drawLine(minA,minA+(minNorm*minProj),color);

	}
#endif //DEBUG_DRAW

	

	D_btGjkPairDetector gjkdet(convexA,convexB,&simplexSolver,0);

	D_btScalar offsetDist = minProj;
	D_btVector3 offset = minNorm * offsetDist;
	


	D_btGjkPairDetector::D_ClosestPointInput input;
		
	D_btVector3 newOrg = transA.getOrigin() + offset;

	D_btTransform displacedTrans = transA;
	displacedTrans.setOrigin(newOrg);

	input.m_transformA = displacedTrans;
	input.m_transformB = transB;
	input.m_maximumDistanceSquared = D_btScalar(D_BT_LARGE_FLOAT);//minProj;
	
	D_btIntermediateResult res;
	gjkdet.setCachedSeperatingAxis(-minNorm);
	gjkdet.getClosestPoints(input,res,debugDraw);

	D_btScalar correctedMinNorm = minProj - res.m_depth;


	//the penetration depth D_is over-estimated, relax it
	D_btScalar penetration_relaxation= D_btScalar(1.);
	minNorm*=penetration_relaxation;
	

	if (res.m_hasResult)
	{

		pa = res.m_pointInWorld - minNorm * correctedMinNorm;
		pb = res.m_pointInWorld;
		v = minNorm;
		
#ifdef DEBUG_DRAW
		if (debugDraw)
		{
			D_btVector3 color(1,0,0);
			debugDraw->drawLine(pa,pb,color);
		}
#endif//DEBUG_DRAW


	}
	return res.m_hasResult;
}



