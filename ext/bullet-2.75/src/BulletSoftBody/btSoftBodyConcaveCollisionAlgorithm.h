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

#ifndef SOFT_BODY_CONCAVE_COLLISION_ALGORITHM_H
#define SOFT_BODY_CONCAVE_COLLISION_ALGORITHM_H

#include "BulletCollision/BroadphaseCollision/btCollisionAlgorithm.h"
#include "BulletCollision/BroadphaseCollision/btDispatcher.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseInterface.h"
#include "BulletCollision/CollisionShapes/btTriangleCallback.h"
#include "BulletCollision/NarrowPhaseCollision/btPersistentManifold.h"
class D_btDispatcher;
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"
#include "BulletCollision/CollisionDispatch/btCollisionCreateFunc.h"
class D_btSoftBody;
class D_btCollisionShape;

#include "LinearMath/btHashMap.h"

#include "BulletCollision/BroadphaseCollision/btQuantizedBvh.h" //for definition of D_MAX_NUM_PARTS_IN_BITS

struct D_btTriIndex
{
	int m_PartIdTriangleIndex;
	class D_btCollisionShape*	m_childShape;

	D_btTriIndex(int partId,int triangleIndex,D_btCollisionShape* shape)
	{
		m_PartIdTriangleIndex = (partId<<(31-D_MAX_NUM_PARTS_IN_BITS)) | triangleIndex;
		m_childShape = shape;
	}

	int	getTriangleIndex() const
	{
		// Get D_only the lower bits where the triangle index D_is stored
		return (m_PartIdTriangleIndex&~((~0)<<(31-D_MAX_NUM_PARTS_IN_BITS)));
	}
	int	getPartId() const
	{
		// Get D_only the highest bits where the part index D_is stored
		return (m_PartIdTriangleIndex>>(31-D_MAX_NUM_PARTS_IN_BITS));
	}
	int	getUid() const
	{
		return m_PartIdTriangleIndex;
	}
};


///For each triangle in the concave mesh that overlaps with the AABB of a soft body (m_softBody), processTriangle D_is called.
class D_btSoftBodyTriangleCallback : public D_btTriangleCallback
{
	D_btSoftBody* m_softBody;
	D_btCollisionObject* m_triBody;

	D_btVector3	m_aabbMin;
	D_btVector3	m_aabbMax ;

	D_btManifoldResult* m_resultOut;

	D_btDispatcher*	m_dispatcher;
	const D_btDispatcherInfo* m_dispatchInfoPtr;
	D_btScalar m_collisionMarginTriangle;

	D_btHashMap<D_btHashKey<D_btTriIndex>,D_btTriIndex> m_shapeCache;

public:
	int	m_triangleCount;

	//	D_btPersistentManifold*	m_manifoldPtr;

	D_btSoftBodyTriangleCallback(D_btDispatcher* dispatcher,D_btCollisionObject* body0,D_btCollisionObject* body1,bool isSwapped);

	void	setTimeStepAndCounters(D_btScalar collisionMarginTriangle,const D_btDispatcherInfo& dispatchInfo,D_btManifoldResult* resultOut);

	virtual ~D_btSoftBodyTriangleCallback();

	virtual void processTriangle(D_btVector3* triangle, int partId, int triangleIndex);

	void clearCache();

	D_SIMD_FORCE_INLINE const D_btVector3& getAabbMin() const
	{
		return m_aabbMin;
	}
	D_SIMD_FORCE_INLINE const D_btVector3& getAabbMax() const
	{
		return m_aabbMax;
	}

};




/// D_btSoftBodyConcaveCollisionAlgorithm  D_supports collision between soft body D_shapes D_and (concave) trianges meshes.
class D_btSoftBodyConcaveCollisionAlgorithm  : public D_btCollisionAlgorithm
{

	bool	m_isSwapped;

	D_btSoftBodyTriangleCallback m_btSoftBodyTriangleCallback;

public:

	D_btSoftBodyConcaveCollisionAlgorithm( const D_btCollisionAlgorithmConstructionInfo& ci,D_btCollisionObject* body0,D_btCollisionObject* body1,bool isSwapped);

	virtual ~D_btSoftBodyConcaveCollisionAlgorithm();

	virtual void processCollision (D_btCollisionObject* body0,D_btCollisionObject* body1,const D_btDispatcherInfo& dispatchInfo,D_btManifoldResult* resultOut);

	D_btScalar	calculateTimeOfImpact(D_btCollisionObject* body0,D_btCollisionObject* body1,const D_btDispatcherInfo& dispatchInfo,D_btManifoldResult* resultOut);

	virtual	void	getAllContactManifolds(D_btManifoldArray&	manifoldArray)
	{
		//we don't add any manifolds
	}

	void	clearCache();

	struct D_CreateFunc :public 	D_btCollisionAlgorithmCreateFunc
	{
		virtual	D_btCollisionAlgorithm* CreateCollisionAlgorithm(D_btCollisionAlgorithmConstructionInfo& ci, D_btCollisionObject* body0,D_btCollisionObject* body1)
		{
			void* mem = ci.m_dispatcher1->allocateCollisionAlgorithm(sizeof(D_btSoftBodyConcaveCollisionAlgorithm));
			return new(mem) D_btSoftBodyConcaveCollisionAlgorithm(ci,body0,body1,false);
		}
	};

	struct D_SwappedCreateFunc :public 	D_btCollisionAlgorithmCreateFunc
	{
		virtual	D_btCollisionAlgorithm* CreateCollisionAlgorithm(D_btCollisionAlgorithmConstructionInfo& ci, D_btCollisionObject* body0,D_btCollisionObject* body1)
		{
			void* mem = ci.m_dispatcher1->allocateCollisionAlgorithm(sizeof(D_btSoftBodyConcaveCollisionAlgorithm));
			return new(mem) D_btSoftBodyConcaveCollisionAlgorithm(ci,body0,body1,true);
		}
	};

};

#endif //SOFT_BODY_CONCAVE_COLLISION_ALGORITHM_H
