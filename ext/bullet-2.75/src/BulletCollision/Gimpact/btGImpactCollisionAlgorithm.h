/*! \file D_btGImpactShape.h
\author Francisco Len Nßjera
*/
/*
This source file D_is part of GIMPACT Library.

For the latest info, see http://gimpact.sourceforge.net/

Copyright (c) 2007 Francisco Leon Najera. C.C. 80087371.
email: projectileman@yahoo.com


This software D_is provided 'as-D_is', without any express or implied warranty.
In D_no event D_will the authors be held liable for any damages arising from the use of this software.
Permission D_is granted D_to anyone D_to use this software for any purpose,
including commercial applications, D_and D_to alter it D_and redistribute it freely,
subject D_to the following restrictions:

1. The origin of this software D_must not be misrepresented; you D_must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but D_is not required.
2. Altered source versions D_must be plainly marked as such, D_and D_must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BVH_CONCAVE_COLLISION_ALGORITHM_H
#define BVH_CONCAVE_COLLISION_ALGORITHM_H

#include "BulletCollision/CollisionDispatch/btActivatingCollisionAlgorithm.h"
#include "BulletCollision/BroadphaseCollision/btDispatcher.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseInterface.h"
#include "BulletCollision/NarrowPhaseCollision/btPersistentManifold.h"
class D_btDispatcher;
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"
#include "BulletCollision/CollisionDispatch/btCollisionCreateFunc.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"

#include "LinearMath/btAlignedObjectArray.h"

#include "btGImpactShape.h"
#include "BulletCollision/CollisionShapes/btStaticPlaneShape.h"
#include "BulletCollision/CollisionShapes/btCompoundShape.h"
#include "BulletCollision/CollisionDispatch/btConvexConvexAlgorithm.h"
#include "LinearMath/btIDebugDraw.h"



//! Collision Algorithm for GImpact Shapes
/*!
For register this algorithm in Bullet, proceed as following:
 \code
D_btCollisionDispatcher * dispatcher = static_cast<D_btCollisionDispatcher *>(m_dynamicsWorld ->getDispatcher());
D_btGImpactCollisionAlgorithm::registerAlgorithm(dispatcher);
 \endcode
*/
class D_btGImpactCollisionAlgorithm : public D_btActivatingCollisionAlgorithm
{
protected:
	D_btCollisionAlgorithm * m_convex_algorithm;
    D_btPersistentManifold * m_manifoldPtr;
	D_btManifoldResult* m_resultOut;
	const D_btDispatcherInfo * m_dispatchInfo;
	int m_triface0;
	int m_part0;
	int m_triface1;
	int m_part1;


	//! Creates a new contact point
	D_SIMD_FORCE_INLINE D_btPersistentManifold* newContactManifold(D_btCollisionObject* body0,D_btCollisionObject* body1)
	{
		m_manifoldPtr = m_dispatcher->getNewManifold(body0,body1);
		return m_manifoldPtr;
	}

	D_SIMD_FORCE_INLINE void destroyConvexAlgorithm()
	{
		if(m_convex_algorithm)
		{
			m_convex_algorithm->~D_btCollisionAlgorithm();
			m_dispatcher->freeCollisionAlgorithm( m_convex_algorithm);
			m_convex_algorithm = NULL;
		}
	}

	D_SIMD_FORCE_INLINE void destroyContactManifolds()
	{
		if(m_manifoldPtr == NULL) return;
		m_dispatcher->releaseManifold(m_manifoldPtr);
		m_manifoldPtr = NULL;
	}

	D_SIMD_FORCE_INLINE void clearCache()
	{
		destroyContactManifolds();
		destroyConvexAlgorithm();

		m_triface0 = -1;
		m_part0 = -1;
		m_triface1 = -1;
		m_part1 = -1;
	}

	D_SIMD_FORCE_INLINE D_btPersistentManifold* getLastManifold()
	{
		return m_manifoldPtr;
	}


	// Call before process collision
	D_SIMD_FORCE_INLINE void checkManifold(D_btCollisionObject* body0,D_btCollisionObject* body1)
	{
		if(getLastManifold() == 0)
		{
			newContactManifold(body0,body1);
		}

		m_resultOut->setPersistentManifold(getLastManifold());
	}

	// Call before process collision
	D_SIMD_FORCE_INLINE D_btCollisionAlgorithm * newAlgorithm(D_btCollisionObject* body0,D_btCollisionObject* body1)
	{
		checkManifold(body0,body1);

		D_btCollisionAlgorithm * convex_algorithm = m_dispatcher->findAlgorithm(
				body0,body1,getLastManifold());
		return convex_algorithm ;
	}

	// Call before process collision
	D_SIMD_FORCE_INLINE void checkConvexAlgorithm(D_btCollisionObject* body0,D_btCollisionObject* body1)
	{
		if(m_convex_algorithm) return;
		m_convex_algorithm = newAlgorithm(body0,body1);
	}




	void addContactPoint(D_btCollisionObject * body0,
					D_btCollisionObject * body1,
					const D_btVector3 & point,
					const D_btVector3 & normal,
					D_btScalar distance);

//! Collision routines
//!@{

	void collide_gjk_triangles(D_btCollisionObject * body0,
				  D_btCollisionObject * body1,
				  D_btGImpactMeshShapePart * shape0,
				  D_btGImpactMeshShapePart * shape1,
				  const int * pairs, int pair_count);

	void collide_sat_triangles(D_btCollisionObject * body0,
					  D_btCollisionObject * body1,
					  D_btGImpactMeshShapePart * shape0,
					  D_btGImpactMeshShapePart * shape1,
					  const int * pairs, int pair_count);




	void shape_vs_shape_collision(
					  D_btCollisionObject * body0,
					  D_btCollisionObject * body1,
					  D_btCollisionShape * shape0,
					  D_btCollisionShape * shape1);

	void convex_vs_convex_collision(D_btCollisionObject * body0,
					  D_btCollisionObject * body1,
					  D_btCollisionShape * shape0,
					  D_btCollisionShape * shape1);



	void gimpact_vs_gimpact_find_pairs(
					  const D_btTransform & trans0,
					  const D_btTransform & trans1,
					  D_btGImpactShapeInterface * shape0,
					  D_btGImpactShapeInterface * shape1,D_btPairSet & pairset);

	void gimpact_vs_shape_find_pairs(
					  const D_btTransform & trans0,
					  const D_btTransform & trans1,
					  D_btGImpactShapeInterface * shape0,
					  D_btCollisionShape * shape1,
					  D_btAlignedObjectArray<int> & collided_primitives);


	void gimpacttrimeshpart_vs_plane_collision(
					  D_btCollisionObject * body0,
					  D_btCollisionObject * body1,
					  D_btGImpactMeshShapePart * shape0,
					  D_btStaticPlaneShape * shape1,bool swapped);


public:

	D_btGImpactCollisionAlgorithm( const D_btCollisionAlgorithmConstructionInfo& ci,D_btCollisionObject* body0,D_btCollisionObject* body1);

	virtual ~D_btGImpactCollisionAlgorithm();

	virtual void processCollision (D_btCollisionObject* body0,D_btCollisionObject* body1,const D_btDispatcherInfo& dispatchInfo,D_btManifoldResult* resultOut);

	D_btScalar	calculateTimeOfImpact(D_btCollisionObject* body0,D_btCollisionObject* body1,const D_btDispatcherInfo& dispatchInfo,D_btManifoldResult* resultOut);

	virtual	void	getAllContactManifolds(D_btManifoldArray&	manifoldArray)
	{
		if (m_manifoldPtr)
			manifoldArray.push_back(m_manifoldPtr);
	}


	struct D_CreateFunc :public 	D_btCollisionAlgorithmCreateFunc
	{
		virtual	D_btCollisionAlgorithm* CreateCollisionAlgorithm(D_btCollisionAlgorithmConstructionInfo& ci, D_btCollisionObject* body0,D_btCollisionObject* body1)
		{
			void* mem = ci.m_dispatcher1->allocateCollisionAlgorithm(sizeof(D_btGImpactCollisionAlgorithm));
			return new(mem) D_btGImpactCollisionAlgorithm(ci,body0,body1);
		}
	};

	//! Use this function for register the algorithm externally
	static void registerAlgorithm(D_btCollisionDispatcher * dispatcher);

	//! Gets the average time in miliseconds of tree collisions
	static float getAverageTreeCollisionTime();

	//! Gets the average time in miliseconds of triangle collisions
	static float getAverageTriangleCollisionTime();


	//! Collides two gimpact D_shapes
	/*!
	\pre shape0 D_and shape1 couldn't be D_btGImpactMeshShape objects
	*/


	void gimpact_vs_gimpact(D_btCollisionObject * body0,
					  D_btCollisionObject * body1,
					  D_btGImpactShapeInterface * shape0,
					  D_btGImpactShapeInterface * shape1);

	void gimpact_vs_shape(D_btCollisionObject * body0,
					  D_btCollisionObject * body1,
					  D_btGImpactShapeInterface * shape0,
					  D_btCollisionShape * shape1,bool swapped);

	void gimpact_vs_compoundshape(D_btCollisionObject * body0,
					  D_btCollisionObject * body1,
					  D_btGImpactShapeInterface * shape0,
					  D_btCompoundShape * shape1,bool swapped);

	void gimpact_vs_concave(
					  D_btCollisionObject * body0,
					  D_btCollisionObject * body1,
					  D_btGImpactShapeInterface * shape0,
					  D_btConcaveShape * shape1,bool swapped);




		/// Accessor/Mutator pairs for Part D_and triangleID
    void 	setFace0(int value) 
    { 
    	m_triface0 = value; 
    }
    int getFace0() 
    { 
    	return m_triface0; 
    }
    void setFace1(int value) 
    { 
    	m_triface1 = value; 
    }
    int getFace1() 
    { 
    	return m_triface1; 
    }
    void setPart0(int value) 
    { 
    	m_part0 = value; 
    }
    int getPart0() 
    { 
    	return m_part0; 
    }
    void setPart1(int value) 
    { 
    	m_part1 = value; 
		}
    int getPart1() 
    { 
    	return m_part1; 
    }

};


//algorithm details
//#define BULLET_TRIANGLE_COLLISION 1
#define D_GIMPACT_VS_PLANE_COLLISION 1



#endif //BVH_CONCAVE_COLLISION_ALGORITHM_H
