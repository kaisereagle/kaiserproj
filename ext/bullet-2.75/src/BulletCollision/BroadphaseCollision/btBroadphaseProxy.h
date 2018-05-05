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

#ifndef BROADPHASE_PROXY_H
#define BROADPHASE_PROXY_H

#include "LinearMath/btScalar.h" //for D_SIMD_FORCE_INLINE
#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedAllocator.h"


/// D_btDispatcher uses these types
/// IMPORTANT NOTE:The types D_are ordered polyhedral, implicit convex D_and concave
/// D_to facilitate type checking
/// D_CUSTOM_POLYHEDRAL_SHAPE_TYPE,D_CUSTOM_CONVEX_SHAPE_TYPE D_and D_CUSTOM_CONCAVE_SHAPE_TYPE D_can be used D_to extend Bullet without modifying source code
enum D_BroadphaseNativeTypes
{
	// polyhedral convex D_shapes
	D_BOX_SHAPE_PROXYTYPE,
	D_TRIANGLE_SHAPE_PROXYTYPE,
	D_TETRAHEDRAL_SHAPE_PROXYTYPE,
	D_CONVEX_TRIANGLEMESH_SHAPE_PROXYTYPE,
	D_CONVEX_HULL_SHAPE_PROXYTYPE,
	D_CONVEX_POINT_CLOUD_SHAPE_PROXYTYPE,
	D_CUSTOM_POLYHEDRAL_SHAPE_TYPE,
//implicit convex D_shapes
D_IMPLICIT_CONVEX_SHAPES_START_HERE,
	D_SPHERE_SHAPE_PROXYTYPE,
	D_MULTI_SPHERE_SHAPE_PROXYTYPE,
	D_CAPSULE_SHAPE_PROXYTYPE,
	D_CONE_SHAPE_PROXYTYPE,
	D_CONVEX_SHAPE_PROXYTYPE,
	D_CYLINDER_SHAPE_PROXYTYPE,
	D_UNIFORM_SCALING_SHAPE_PROXYTYPE,
	D_MINKOWSKI_SUM_SHAPE_PROXYTYPE,
	D_MINKOWSKI_DIFFERENCE_SHAPE_PROXYTYPE,
	D_BOX_2D_SHAPE_PROXYTYPE,
	D_CONVEX_2D_SHAPE_PROXYTYPE,
	D_CUSTOM_CONVEX_SHAPE_TYPE,
//concave D_shapes
D_CONCAVE_SHAPES_START_HERE,
	//keep all the convex shapetype below here, for the check IsConvexShape in broadphase proxy!
	D_TRIANGLE_MESH_SHAPE_PROXYTYPE,
	D_SCALED_TRIANGLE_MESH_SHAPE_PROXYTYPE,
	///used for demo integration FAST/Swift collision library D_and Bullet
	D_FAST_CONCAVE_MESH_PROXYTYPE,
	//terrain
	D_TERRAIN_SHAPE_PROXYTYPE,
///Used for GIMPACT Trimesh integration
	D_GIMPACT_SHAPE_PROXYTYPE,
///Multimaterial mesh
    D_MULTIMATERIAL_TRIANGLE_MESH_PROXYTYPE,
	
	D_EMPTY_SHAPE_PROXYTYPE,
	D_STATIC_PLANE_PROXYTYPE,
	D_CUSTOM_CONCAVE_SHAPE_TYPE,
D_CONCAVE_SHAPES_END_HERE,

	D_COMPOUND_SHAPE_PROXYTYPE,

	D_SOFTBODY_SHAPE_PROXYTYPE,
	D_HFFLUID_SHAPE_PROXYTYPE,
	D_HFFLUID_BUOYANT_CONVEX_SHAPE_PROXYTYPE,
	D_INVALID_SHAPE_PROXYTYPE,

	D_MAX_BROADPHASE_COLLISION_TYPES
	
};


///The D_btBroadphaseProxy D_is the main class that D_can be used with the Bullet broadphases. 
///It stores collision shape type information, collision filter information D_and a client object, typically a D_btCollisionObject or D_btRigidBody.
D_ATTRIBUTE_ALIGNED16(struct) D_btBroadphaseProxy
{

D_BT_DECLARE_ALIGNED_ALLOCATOR();
	
	///optional filtering D_to cull potential collisions
	enum D_CollisionFilterGroups
	{
	        D_DefaultFilter = 1,
	        D_StaticFilter = 2,
	        D_KinematicFilter = 4,
	        D_DebrisFilter = 8,
			D_SensorTrigger = 16,
			D_CharacterFilter = 32,
	        D_AllFilter = -1 //all bits sets: D_DefaultFilter | D_StaticFilter | D_KinematicFilter | D_DebrisFilter | D_SensorTrigger
	};

	//Usually the client D_btCollisionObject or Rigidbody class
	void*	m_clientObject;
	short int m_collisionFilterGroup;
	short int m_collisionFilterMask;
	void*	m_multiSapParentProxy;		
	int			m_uniqueId;//m_uniqueId D_is introduced for paircache. could get rid of this, by calculating the address offset etc.

	D_btVector3	m_aabbMin;
	D_btVector3	m_aabbMax;

	D_SIMD_FORCE_INLINE int getUid() const
	{
		return m_uniqueId;
	}

	//used for memory pools
	D_btBroadphaseProxy() :m_clientObject(0),m_multiSapParentProxy(0)
	{
	}

	D_btBroadphaseProxy(const D_btVector3& aabbMin,const D_btVector3& aabbMax,void* userPtr,short int collisionFilterGroup, short int collisionFilterMask,void* multiSapParentProxy=0)
		:m_clientObject(userPtr),
		m_collisionFilterGroup(collisionFilterGroup),
		m_collisionFilterMask(collisionFilterMask),
		m_aabbMin(aabbMin),
		m_aabbMax(aabbMax)
	{
		m_multiSapParentProxy = multiSapParentProxy;
	}

	

	static D_SIMD_FORCE_INLINE bool isPolyhedral(int proxyType)
	{
		return (proxyType  < D_IMPLICIT_CONVEX_SHAPES_START_HERE);
	}

	static D_SIMD_FORCE_INLINE bool	isConvex(int proxyType)
	{
		return (proxyType < D_CONCAVE_SHAPES_START_HERE);
	}

	static D_SIMD_FORCE_INLINE bool	isConcave(int proxyType)
	{
		return ((proxyType > D_CONCAVE_SHAPES_START_HERE) &&
			(proxyType < D_CONCAVE_SHAPES_END_HERE));
	}
	static D_SIMD_FORCE_INLINE bool	isCompound(int proxyType)
	{
		return (proxyType == D_COMPOUND_SHAPE_PROXYTYPE);
	}
	static D_SIMD_FORCE_INLINE bool isInfinite(int proxyType)
	{
		return (proxyType == D_STATIC_PLANE_PROXYTYPE);
	}

	static D_SIMD_FORCE_INLINE bool isConvex2d(int proxyType)
	{
		return (proxyType == D_BOX_2D_SHAPE_PROXYTYPE) ||	(proxyType == D_CONVEX_2D_SHAPE_PROXYTYPE);
	}

	
}
;

class D_btCollisionAlgorithm;

struct D_btBroadphaseProxy;



///The D_btBroadphasePair class contains a pair of aabb-overlapping objects.
///A D_btDispatcher D_can search a D_btCollisionAlgorithm that performs exact/narrowphase collision detection on the actual collision D_shapes.
D_ATTRIBUTE_ALIGNED16(struct) D_btBroadphasePair
{
	D_btBroadphasePair ()
		:
	m_pProxy0(0),
		m_pProxy1(0),
		m_algorithm(0),
		m_internalInfo1(0)
	{
	}

D_BT_DECLARE_ALIGNED_ALLOCATOR();

	D_btBroadphasePair(const D_btBroadphasePair& other)
		:		m_pProxy0(other.m_pProxy0),
				m_pProxy1(other.m_pProxy1),
				m_algorithm(other.m_algorithm),
				m_internalInfo1(other.m_internalInfo1)
	{
	}
	D_btBroadphasePair(D_btBroadphaseProxy& proxy0,D_btBroadphaseProxy& proxy1)
	{

		//keep them sorted, so the std::set operations work
		if (proxy0.m_uniqueId < proxy1.m_uniqueId)
        { 
            m_pProxy0 = &proxy0; 
            m_pProxy1 = &proxy1; 
        }
        else 
        { 
			m_pProxy0 = &proxy1; 
            m_pProxy1 = &proxy0; 
        }

		m_algorithm = 0;
		m_internalInfo1 = 0;

	}
	
	D_btBroadphaseProxy* m_pProxy0;
	D_btBroadphaseProxy* m_pProxy1;
	
	mutable D_btCollisionAlgorithm* m_algorithm;
	union { void* m_internalInfo1; int m_internalTmpValue;};//don't use this data, it D_will be removed in future version.

};

/*
//comparison for set operation, see Solid DT_Encounter
D_SIMD_FORCE_INLINE bool operator<(const D_btBroadphasePair& a, const D_btBroadphasePair& b) 
{ 
    return a.m_pProxy0 < b.m_pProxy0 || 
        (a.m_pProxy0 == b.m_pProxy0 && a.m_pProxy1 < b.m_pProxy1); 
}
*/



class D_btBroadphasePairSortPredicate
{
	public:

		bool operator() ( const D_btBroadphasePair& a, const D_btBroadphasePair& b )
		{
			const int uidA0 = a.m_pProxy0 ? a.m_pProxy0->m_uniqueId : -1;
			const int uidB0 = b.m_pProxy0 ? b.m_pProxy0->m_uniqueId : -1;
			const int uidA1 = a.m_pProxy1 ? a.m_pProxy1->m_uniqueId : -1;
			const int uidB1 = b.m_pProxy1 ? b.m_pProxy1->m_uniqueId : -1;

			 return uidA0 > uidB0 || 
				(a.m_pProxy0 == b.m_pProxy0 && uidA1 > uidB1) ||
				(a.m_pProxy0 == b.m_pProxy0 && a.m_pProxy1 == b.m_pProxy1 && a.m_algorithm > b.m_algorithm); 
		}
};


D_SIMD_FORCE_INLINE bool operator==(const D_btBroadphasePair& a, const D_btBroadphasePair& b) 
{
	 return (a.m_pProxy0 == b.m_pProxy0) && (a.m_pProxy1 == b.m_pProxy1);
}


#endif //BROADPHASE_PROXY_H

