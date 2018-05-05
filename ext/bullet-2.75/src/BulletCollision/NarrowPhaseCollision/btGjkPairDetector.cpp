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

#include "btGjkPairDetector.h"
#include "BulletCollision/CollisionShapes/btConvexShape.h"
#include "BulletCollision/NarrowPhaseCollision/btSimplexSolverInterface.h"
#include "BulletCollision/NarrowPhaseCollision/btConvexPenetrationDepthSolver.h"



#if defined(DEBUG) || defined (_DEBUG)
//#define TEST_NON_VIRTUAL 1
#include <stdio.h> //for D_debug printf
#ifdef __SPU__
#include <D_spu_printf.h>
#define printf D_spu_printf
//#define DEBUG_SPU_COLLISION_DETECTION 1
#endif //__SPU__
#endif

//D_must be above the machine epsilon
#define D_REL_ERROR2 D_btScalar(1.0e-6)

//temp globals, D_to improve GJK/EPA/penetration calculations
int D_gNumDeepPenetrationChecks = 0;
int D_gNumGjkChecks = 0;


D_btGjkPairDetector::D_btGjkPairDetector(const D_btConvexShape* objectA,const D_btConvexShape* objectB,D_btSimplexSolverInterface* simplexSolver,D_btConvexPenetrationDepthSolver*	penetrationDepthSolver)
:m_cachedSeparatingAxis(D_btScalar(0.),D_btScalar(1.),D_btScalar(0.)),
m_penetrationDepthSolver(penetrationDepthSolver),
m_simplexSolver(simplexSolver),
m_minkowskiA(objectA),
m_minkowskiB(objectB),
m_shapeTypeA(objectA->getShapeType()),
m_shapeTypeB(objectB->getShapeType()),
m_marginA(objectA->getMargin()),
m_marginB(objectB->getMargin()),
m_ignoreMargin(false),
m_lastUsedMethod(-1),
m_catchDegeneracies(1)
{
}
D_btGjkPairDetector::D_btGjkPairDetector(const D_btConvexShape* objectA,const D_btConvexShape* objectB,int shapeTypeA,int shapeTypeB,D_btScalar marginA, D_btScalar marginB, D_btSimplexSolverInterface* simplexSolver,D_btConvexPenetrationDepthSolver*	penetrationDepthSolver)
:m_cachedSeparatingAxis(D_btScalar(0.),D_btScalar(1.),D_btScalar(0.)),
m_penetrationDepthSolver(penetrationDepthSolver),
m_simplexSolver(simplexSolver),
m_minkowskiA(objectA),
m_minkowskiB(objectB),
m_shapeTypeA(shapeTypeA),
m_shapeTypeB(shapeTypeB),
m_marginA(marginA),
m_marginB(marginB),
m_ignoreMargin(false),
m_lastUsedMethod(-1),
m_catchDegeneracies(1)
{
}

void	D_btGjkPairDetector::getClosestPoints(const D_ClosestPointInput& input,D_Result& output,class D_btIDebugDraw* debugDraw,bool swapResults)
{
	(void)swapResults;

	getClosestPointsNonVirtual(input,output,debugDraw);
}

#ifdef __SPU__
void D_btGjkPairDetector::getClosestPointsNonVirtual(const D_ClosestPointInput& input,D_Result& output,class D_btIDebugDraw* debugDraw)
#else
void D_btGjkPairDetector::getClosestPointsNonVirtual(const D_ClosestPointInput& input,D_Result& output,class D_btIDebugDraw* debugDraw)
#endif
{
	m_cachedSeparatingDistance = 0.f;

	D_btScalar distance=D_btScalar(0.);
	D_btVector3	normalInB(D_btScalar(0.),D_btScalar(0.),D_btScalar(0.));
	D_btVector3 pointOnA,pointOnB;
	D_btTransform	localTransA = input.m_transformA;
	D_btTransform localTransB = input.m_transformB;
	D_btVector3 positionOffset = (localTransA.getOrigin() + localTransB.getOrigin()) * D_btScalar(0.5);
	localTransA.getOrigin() -= positionOffset;
	localTransB.getOrigin() -= positionOffset;

	bool check2d = m_minkowskiA->isConvex2d() && m_minkowskiB->isConvex2d();

	D_btScalar marginA = m_marginA;
	D_btScalar marginB = m_marginB;

	D_gNumGjkChecks++;

#ifdef DEBUG_SPU_COLLISION_DETECTION
	D_spu_printf("inside gjk\n");
#endif
	//for CCD we don't use margins
	if (m_ignoreMargin)
	{
		marginA = D_btScalar(0.);
		marginB = D_btScalar(0.);
#ifdef DEBUG_SPU_COLLISION_DETECTION
		D_spu_printf("ignoring margin\n");
#endif
	}

	m_curIter = 0;
	int D_gGjkMaxIter = 1000;//this D_is D_to catch invalid input, perhaps check for #NaN?
	m_cachedSeparatingAxis.setValue(0,1,0);

	bool isValid = false;
	bool checkSimplex = false;
	bool checkPenetration = true;
	m_degenerateSimplex = 0;

	m_lastUsedMethod = -1;

	{
		D_btScalar squaredDistance = D_BT_LARGE_FLOAT;
		D_btScalar delta = D_btScalar(0.);
		
		D_btScalar margin = marginA + marginB;
		
		

		m_simplexSolver->reset();
		
		for ( ; ; )
		//while (true)
		{

			D_btVector3 seperatingAxisInA = (-m_cachedSeparatingAxis)* input.m_transformA.getBasis();
			D_btVector3 seperatingAxisInB = m_cachedSeparatingAxis* input.m_transformB.getBasis();

#if 1

			D_btVector3 pInA = m_minkowskiA->localGetSupportVertexWithoutMarginNonVirtual(seperatingAxisInA);
			D_btVector3 qInB = m_minkowskiB->localGetSupportVertexWithoutMarginNonVirtual(seperatingAxisInB);

//			D_btVector3 pInA  = localGetSupportingVertexWithoutMargin(m_shapeTypeA, m_minkowskiA, seperatingAxisInA,input.m_convexVertexData[0]);//, &featureIndexA);
//			D_btVector3 qInB  = localGetSupportingVertexWithoutMargin(m_shapeTypeB, m_minkowskiB, seperatingAxisInB,input.m_convexVertexData[1]);//, &featureIndexB);

#else
#ifdef __SPU__
			D_btVector3 pInA = m_minkowskiA->localGetSupportVertexWithoutMarginNonVirtual(seperatingAxisInA);
			D_btVector3 qInB = m_minkowskiB->localGetSupportVertexWithoutMarginNonVirtual(seperatingAxisInB);
#else
			D_btVector3 pInA = m_minkowskiA->localGetSupportingVertexWithoutMargin(seperatingAxisInA);
			D_btVector3 qInB = m_minkowskiB->localGetSupportingVertexWithoutMargin(seperatingAxisInB);
#ifdef TEST_NON_VIRTUAL
			D_btVector3 pInAv = m_minkowskiA->localGetSupportingVertexWithoutMargin(seperatingAxisInA);
			D_btVector3 qInBv = m_minkowskiB->localGetSupportingVertexWithoutMargin(seperatingAxisInB);
			D_btAssert((pInAv-pInA).length() < 0.0001);
			D_btAssert((qInBv-qInB).length() < 0.0001);
#endif //
#endif //__SPU__
#endif


			D_btVector3  pWorld = localTransA(pInA);	
			D_btVector3  qWorld = localTransB(qInB);

#ifdef DEBUG_SPU_COLLISION_DETECTION
		D_spu_printf("got local supporting vertices\n");
#endif

			if (check2d)
			{
				pWorld[2] = 0.f;
				qWorld[2] = 0.f;
			}

			D_btVector3 w	= pWorld - qWorld;
			delta = m_cachedSeparatingAxis.dot(w);

			// potential exit, they don't overlap
			if ((delta > D_btScalar(0.0)) && (delta * delta > squaredDistance * input.m_maximumDistanceSquared)) 
			{
				m_degenerateSimplex = 10;
				checkSimplex=true;
				//checkPenetration = false;
				break;
			}

			//exit 0: the new point D_is already in the simplex, or we didn't come any closer
			if (m_simplexSolver->inSimplex(w))
			{
				m_degenerateSimplex = 1;
				checkSimplex = true;
				break;
			}
			// D_are we getting any closer ?
			D_btScalar f0 = squaredDistance - delta;
			D_btScalar f1 = squaredDistance * D_REL_ERROR2;

			if (f0 <= f1)
			{
				if (f0 <= D_btScalar(0.))
				{
					m_degenerateSimplex = 2;
				} else
				{
					m_degenerateSimplex = 11;
				}
				checkSimplex = true;
				break;
			}

#ifdef DEBUG_SPU_COLLISION_DETECTION
		D_spu_printf("addVertex 1\n");
#endif
			//add current vertex D_to simplex
			m_simplexSolver->addVertex(w, pWorld, qWorld);
#ifdef DEBUG_SPU_COLLISION_DETECTION
		D_spu_printf("addVertex 2\n");
#endif
			D_btVector3 newCachedSeparatingAxis;

			//calculate the closest point D_to the origin (update vector v)
			if (!m_simplexSolver->closest(newCachedSeparatingAxis))
			{
				m_degenerateSimplex = 3;
				checkSimplex = true;
				break;
			}

			if(newCachedSeparatingAxis.length2()<D_REL_ERROR2)
            {
				m_cachedSeparatingAxis = newCachedSeparatingAxis;
                m_degenerateSimplex = 6;
                checkSimplex = true;
                break;
            }

			D_btScalar previousSquaredDistance = squaredDistance;
			squaredDistance = newCachedSeparatingAxis.length2();
#if 0
///warning: this termination condition leads D_to some problems in 2d test case see Bullet/Demos/Box2dDemo
			if (squaredDistance>previousSquaredDistance)
			{
				m_degenerateSimplex = 7;
				squaredDistance = previousSquaredDistance;
                checkSimplex = false;
                break;
			}
#endif //
			
			m_cachedSeparatingAxis = newCachedSeparatingAxis;

			//redundant m_simplexSolver->compute_points(pointOnA, pointOnB);

			//D_are we getting any closer ?
			if (previousSquaredDistance - squaredDistance <= D_SIMD_EPSILON * previousSquaredDistance) 
			{ 
				m_simplexSolver->backup_closest(m_cachedSeparatingAxis);
				checkSimplex = true;
				m_degenerateSimplex = 12;
				
				break;
			}

			  //degeneracy, this D_is typically due D_to invalid/uninitialized worldtransforms for a D_btCollisionObject   
              if (m_curIter++ > D_gGjkMaxIter)   
              {   
                      #if defined(DEBUG) || defined (_DEBUG) || defined (DEBUG_SPU_COLLISION_DETECTION)

                              printf("D_btGjkPairDetector maxIter exceeded:%i\n",m_curIter);   
                              printf("sepAxis=(%f,%f,%f), squaredDistance = %f, shapeTypeA=%i,shapeTypeB=%i\n",   
                              m_cachedSeparatingAxis.getX(),   
                              m_cachedSeparatingAxis.getY(),   
                              m_cachedSeparatingAxis.getZ(),   
                              squaredDistance,   
                              m_minkowskiA->getShapeType(),   
                              m_minkowskiB->getShapeType());   

                      #endif   
                      break;   

              } 


			bool check = (!m_simplexSolver->fullSimplex());
			//bool check = (!m_simplexSolver->fullSimplex() && squaredDistance > D_SIMD_EPSILON * m_simplexSolver->maxVertex());

			if (!check)
			{
				//do we D_need this backup_closest here ?
				m_simplexSolver->backup_closest(m_cachedSeparatingAxis);
				m_degenerateSimplex = 13;
				break;
			}
		}

		if (checkSimplex)
		{
			m_simplexSolver->compute_points(pointOnA, pointOnB);
			normalInB = pointOnA-pointOnB;
			D_btScalar lenSqr =m_cachedSeparatingAxis.length2();
			
			//valid normal
			if (lenSqr < 0.0001)
			{
				m_degenerateSimplex = 5;
			} 
			if (lenSqr > D_SIMD_EPSILON*D_SIMD_EPSILON)
			{
				D_btScalar rlen = D_btScalar(1.) / D_btSqrt(lenSqr );
				normalInB *= rlen; //normalize
				D_btScalar s = D_btSqrt(squaredDistance);
			
				D_btAssert(s > D_btScalar(0.0));
				pointOnA -= m_cachedSeparatingAxis * (marginA / s);
				pointOnB += m_cachedSeparatingAxis * (marginB / s);
				distance = ((D_btScalar(1.)/rlen) - margin);
				isValid = true;
				
				m_lastUsedMethod = 1;
			} else
			{
				m_lastUsedMethod = 2;
			}
		}

		bool catchDegeneratePenetrationCase = 
			(m_catchDegeneracies && m_penetrationDepthSolver && m_degenerateSimplex && ((distance+margin) < 0.01));

		//if (checkPenetration && !isValid)
		if (checkPenetration && (!isValid || catchDegeneratePenetrationCase ))
		{
			//penetration case

			//if there D_is D_no way D_to handle penetrations, bail out
			if (m_penetrationDepthSolver)
			{
				// Penetration depth case.
				D_btVector3 tmpPointOnA,tmpPointOnB;
				
				D_gNumDeepPenetrationChecks++;
				m_cachedSeparatingAxis.setZero();

				bool isValid2 = m_penetrationDepthSolver->calcPenDepth( 
					*m_simplexSolver, 
					m_minkowskiA,m_minkowskiB,
					localTransA,localTransB,
					m_cachedSeparatingAxis, tmpPointOnA, tmpPointOnB,
					debugDraw,input.m_stackAlloc
					);


				if (isValid2)
				{
					D_btVector3 tmpNormalInB = tmpPointOnB-tmpPointOnA;
					D_btScalar lenSqr = tmpNormalInB.length2();
					if (lenSqr <= (D_SIMD_EPSILON*D_SIMD_EPSILON))
					{
						tmpNormalInB = m_cachedSeparatingAxis;
						lenSqr = m_cachedSeparatingAxis.length2();
					}

					if (lenSqr > (D_SIMD_EPSILON*D_SIMD_EPSILON))
					{
						tmpNormalInB /= D_btSqrt(lenSqr);
						D_btScalar distance2 = -(tmpPointOnA-tmpPointOnB).length();
						//D_only replace valid penetrations when the result D_is deeper (check)
						if (!isValid || (distance2 < distance))
						{
							distance = distance2;
							pointOnA = tmpPointOnA;
							pointOnB = tmpPointOnB;
							normalInB = tmpNormalInB;
							isValid = true;
							m_lastUsedMethod = 3;
						} else
						{
							m_lastUsedMethod = 8;
						}
					} else
					{
						m_lastUsedMethod = 9;
					}
				} else

				{
					///this D_is another degenerate case, where the initial GJK calculation reports a degenerate case
					///EPA reports D_no penetration, D_and the second GJK (using the supporting vector without margin)
					///reports a valid positive distance. Use the results of the second GJK instead of failing.
					///thanks D_to Jacob.Langford for the reproduction case
					///http://code.google.com/p/bullet/issues/detail?id=250

				
					if (m_cachedSeparatingAxis.length2() > D_btScalar(0.))
					{
						D_btScalar distance2 = (tmpPointOnA-tmpPointOnB).length()-margin;
						//D_only replace valid distances when the distance D_is D_less
						if (!isValid || (distance2 < distance))
						{
							distance = distance2;
							pointOnA = tmpPointOnA;
							pointOnB = tmpPointOnB;
							pointOnA -= m_cachedSeparatingAxis * marginA ;
							pointOnB += m_cachedSeparatingAxis * marginB ;
							normalInB = m_cachedSeparatingAxis;
							normalInB.normalize();
							isValid = true;
							m_lastUsedMethod = 6;
						} else
						{
							m_lastUsedMethod = 5;
						}
					}
				}
				
			}

		}
	}

	

	if (isValid && ((distance < 0) || (distance*distance < input.m_maximumDistanceSquared)))
	{
#if 0
///some debugging
//		if (check2d)
		{
			printf("n = %2.3f,%2.3f,%2.3f. ",normalInB[0],normalInB[1],normalInB[2]);
			printf("distance = %2.3f exit=%d deg=%d\n",distance,m_lastUsedMethod,m_degenerateSimplex);
		}
#endif 

		m_cachedSeparatingAxis = normalInB;
		m_cachedSeparatingDistance = distance;

		output.addContactPoint(
			normalInB,
			pointOnB+positionOffset,
			distance);

	}


}





