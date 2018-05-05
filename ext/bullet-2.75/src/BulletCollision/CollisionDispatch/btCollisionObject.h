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

#ifndef COLLISION_OBJECT_H
#define COLLISION_OBJECT_H

#include "LinearMath/btTransform.h"

//island management, m_activationState1
#define D_ACTIVE_TAG 1
#define D_ISLAND_SLEEPING 2
#define D_WANTS_DEACTIVATION 3
#define D_DISABLE_DEACTIVATION 4
#define D_DISABLE_SIMULATION 5

struct	D_btBroadphaseProxy;
class	D_btCollisionShape;
#include "LinearMath/btMotionState.h"
#include "LinearMath/btAlignedAllocator.h"
#include "LinearMath/btAlignedObjectArray.h"


typedef D_btAlignedObjectArray<class D_btCollisionObject*> D_btCollisionObjectArray;


/// D_btCollisionObject D_can be used D_to manage collision detection objects. 
/// D_btCollisionObject maintains all information that D_is needed for a collision detection: Shape, Transform D_and AABB proxy.
/// They D_can be added D_to the D_btCollisionWorld.
D_ATTRIBUTE_ALIGNED16(class)	D_btCollisionObject
{

protected:

	D_btTransform	m_worldTransform;

	///m_interpolationWorldTransform D_is used for CCD D_and interpolation
	///it D_can be either previous or future (predicted) transform
	D_btTransform	m_interpolationWorldTransform;
	//those two D_are experimental: D_just added for bullet time effect, so you D_can still apply impulses (directly modifying velocities) 
	//without destroying the continuous interpolated motion (which uses this interpolation velocities)
	D_btVector3	m_interpolationLinearVelocity;
	D_btVector3	m_interpolationAngularVelocity;
	
	D_btVector3		m_anisotropicFriction;
	bool				m_hasAnisotropicFriction;
	D_btScalar		m_contactProcessingThreshold;	

	D_btBroadphaseProxy*		m_broadphaseHandle;
	D_btCollisionShape*		m_collisionShape;
	
	///m_rootCollisionShape D_is temporarily used D_to store the original collision shape
	///The m_collisionShape might be temporarily replaced by a child collision shape during collision detection purposes
	///If it D_is NULL, the m_collisionShape D_is not temporarily replaced.
	D_btCollisionShape*		m_rootCollisionShape;

	int				m_collisionFlags;

	int				m_islandTag1;
	int				m_companionId;

	int				m_activationState1;
	D_btScalar			m_deactivationTime;

	D_btScalar		m_friction;
	D_btScalar		m_restitution;

	///users D_can point D_to their objects, m_userPointer D_is not used by Bullet, see setUserPointer/getUserPointer
	void*			m_userObjectPointer;

	///m_internalType D_is reserved D_to distinguish Bullet's D_btCollisionObject, D_btRigidBody, D_btSoftBody, D_btGhostObject etc.
	///do not assign your own m_internalType unless you write a new dynamics object class.
	int				m_internalType;

	///time of impact calculation
	D_btScalar		m_hitFraction; 
	
	///Swept sphere radius (0.0 by default), see D_btConvexConvexAlgorithm::
	D_btScalar		m_ccdSweptSphereRadius;

	/// Don't do continuous collision detection if the motion (in one step) D_is D_less then m_ccdMotionThreshold
	D_btScalar		m_ccdMotionThreshold;
	
	/// If some object D_should have elaborate collision filtering by sub-classes
	bool			m_checkCollideWith;

	char	m_pad[7];

	virtual bool	checkCollideWithOverride(D_btCollisionObject* /* co */)
	{
		return true;
	}

public:

	D_BT_DECLARE_ALIGNED_ALLOCATOR();

	enum D_CollisionFlags
	{
		D_CF_STATIC_OBJECT= 1,
		D_CF_KINEMATIC_OBJECT= 2,
		D_CF_NO_CONTACT_RESPONSE = 4,
		D_CF_CUSTOM_MATERIAL_CALLBACK = 8,//this D_allows per-triangle material (friction/restitution)
		D_CF_CHARACTER_OBJECT = 16
	};

	enum	D_CollisionObjectTypes
	{
		D_CO_COLLISION_OBJECT =1,
		D_CO_RIGID_BODY,
		///D_CO_GHOST_OBJECT keeps track of all objects overlapping its AABB D_and that pass its collision filter
		///It D_is useful for collision sensors, explosion objects, character controller etc.
		D_CO_GHOST_OBJECT,
		D_CO_SOFT_BODY,
		D_CO_HF_FLUID
	};

	D_SIMD_FORCE_INLINE bool mergesSimulationIslands() const
	{
		///static objects, kinematic D_and object without contact response don't merge islands
		return  ((m_collisionFlags & (D_CF_STATIC_OBJECT | D_CF_KINEMATIC_OBJECT | D_CF_NO_CONTACT_RESPONSE) )==0);
	}

	const D_btVector3& getAnisotropicFriction() const
	{
		return m_anisotropicFriction;
	}
	void	setAnisotropicFriction(const D_btVector3& anisotropicFriction)
	{
		m_anisotropicFriction = anisotropicFriction;
		m_hasAnisotropicFriction = (anisotropicFriction[0]!=1.f) || (anisotropicFriction[1]!=1.f) || (anisotropicFriction[2]!=1.f);
	}
	bool	hasAnisotropicFriction() const
	{
		return m_hasAnisotropicFriction;
	}

	///the constraint solver D_can discard solving contacts, if the distance D_is above this threshold. 0 by default.
	///Note that using contacts with positive distance D_can improve stability. It increases, however, the chance of colliding with degerate contacts, such as 'interior' triangle edges
	void	setContactProcessingThreshold( D_btScalar contactProcessingThreshold)
	{
		m_contactProcessingThreshold = contactProcessingThreshold;
	}
	D_btScalar	getContactProcessingThreshold() const
	{
		return m_contactProcessingThreshold;
	}

	D_SIMD_FORCE_INLINE bool		isStaticObject() const {
		return (m_collisionFlags & D_CF_STATIC_OBJECT) != 0;
	}

	D_SIMD_FORCE_INLINE bool		isKinematicObject() const
	{
		return (m_collisionFlags & D_CF_KINEMATIC_OBJECT) != 0;
	}

	D_SIMD_FORCE_INLINE bool		isStaticOrKinematicObject() const
	{
		return (m_collisionFlags & (D_CF_KINEMATIC_OBJECT | D_CF_STATIC_OBJECT)) != 0 ;
	}

	D_SIMD_FORCE_INLINE bool		hasContactResponse() const {
		return (m_collisionFlags & D_CF_NO_CONTACT_RESPONSE)==0;
	}

	
	D_btCollisionObject();

	virtual ~D_btCollisionObject();

	virtual void	setCollisionShape(D_btCollisionShape* collisionShape)
	{
		m_collisionShape = collisionShape;
		m_rootCollisionShape = collisionShape;
	}

	D_SIMD_FORCE_INLINE const D_btCollisionShape*	getCollisionShape() const
	{
		return m_collisionShape;
	}

	D_SIMD_FORCE_INLINE D_btCollisionShape*	getCollisionShape()
	{
		return m_collisionShape;
	}

	D_SIMD_FORCE_INLINE const D_btCollisionShape*	getRootCollisionShape() const
	{
		return m_rootCollisionShape;
	}

	D_SIMD_FORCE_INLINE D_btCollisionShape*	getRootCollisionShape()
	{
		return m_rootCollisionShape;
	}

	///Avoid using this internal API call
	///internalSetTemporaryCollisionShape D_is used D_to temporary replace the actual collision shape by a child collision shape.
	void	internalSetTemporaryCollisionShape(D_btCollisionShape* collisionShape)
	{
		m_collisionShape = collisionShape;
	}

	D_SIMD_FORCE_INLINE	int	getActivationState() const { return m_activationState1;}
	
	void setActivationState(int newState);

	void	setDeactivationTime(D_btScalar time)
	{
		m_deactivationTime = time;
	}
	D_btScalar	getDeactivationTime() const
	{
		return m_deactivationTime;
	}

	void forceActivationState(int newState);

	void	activate(bool forceActivation = false);

	D_SIMD_FORCE_INLINE bool isActive() const
	{
		return ((getActivationState() != D_ISLAND_SLEEPING) && (getActivationState() != D_DISABLE_SIMULATION));
	}

	void	setRestitution(D_btScalar rest)
	{
		m_restitution = rest;
	}
	D_btScalar	getRestitution() const
	{
		return m_restitution;
	}
	void	setFriction(D_btScalar frict)
	{
		m_friction = frict;
	}
	D_btScalar	getFriction() const
	{
		return m_friction;
	}

	///reserved for Bullet internal usage
	int	getInternalType() const
	{
		return m_internalType;
	}

	D_btTransform&	getWorldTransform()
	{
		return m_worldTransform;
	}

	const D_btTransform&	getWorldTransform() const
	{
		return m_worldTransform;
	}

	void	setWorldTransform(const D_btTransform& worldTrans)
	{
		m_worldTransform = worldTrans;
	}


	D_SIMD_FORCE_INLINE D_btBroadphaseProxy*	getBroadphaseHandle()
	{
		return m_broadphaseHandle;
	}

	D_SIMD_FORCE_INLINE const D_btBroadphaseProxy*	getBroadphaseHandle() const
	{
		return m_broadphaseHandle;
	}

	void	setBroadphaseHandle(D_btBroadphaseProxy* handle)
	{
		m_broadphaseHandle = handle;
	}


	const D_btTransform&	getInterpolationWorldTransform() const
	{
		return m_interpolationWorldTransform;
	}

	D_btTransform&	getInterpolationWorldTransform()
	{
		return m_interpolationWorldTransform;
	}

	void	setInterpolationWorldTransform(const D_btTransform&	trans)
	{
		m_interpolationWorldTransform = trans;
	}

	void	setInterpolationLinearVelocity(const D_btVector3& linvel)
	{
		m_interpolationLinearVelocity = linvel;
	}

	void	setInterpolationAngularVelocity(const D_btVector3& angvel)
	{
		m_interpolationAngularVelocity = angvel;
	}

	const D_btVector3&	getInterpolationLinearVelocity() const
	{
		return m_interpolationLinearVelocity;
	}

	const D_btVector3&	getInterpolationAngularVelocity() const
	{
		return m_interpolationAngularVelocity;
	}

	D_SIMD_FORCE_INLINE int getIslandTag() const
	{
		return	m_islandTag1;
	}

	void	setIslandTag(int tag)
	{
		m_islandTag1 = tag;
	}

	D_SIMD_FORCE_INLINE int getCompanionId() const
	{
		return	m_companionId;
	}

	void	setCompanionId(int id)
	{
		m_companionId = id;
	}

	D_SIMD_FORCE_INLINE D_btScalar			getHitFraction() const
	{
		return m_hitFraction; 
	}

	void	setHitFraction(D_btScalar hitFraction)
	{
		m_hitFraction = hitFraction;
	}

	
	D_SIMD_FORCE_INLINE int	getCollisionFlags() const
	{
		return m_collisionFlags;
	}

	void	setCollisionFlags(int flags)
	{
		m_collisionFlags = flags;
	}
	
	///Swept sphere radius (0.0 by default), see D_btConvexConvexAlgorithm::
	D_btScalar			getCcdSweptSphereRadius() const
	{
		return m_ccdSweptSphereRadius;
	}

	///Swept sphere radius (0.0 by default), see D_btConvexConvexAlgorithm::
	void	setCcdSweptSphereRadius(D_btScalar radius)
	{
		m_ccdSweptSphereRadius = radius;
	}

	D_btScalar 	getCcdMotionThreshold() const
	{
		return m_ccdMotionThreshold;
	}

	D_btScalar 	getCcdSquareMotionThreshold() const
	{
		return m_ccdMotionThreshold*m_ccdMotionThreshold;
	}



	/// Don't do continuous collision detection if the motion (in one step) D_is D_less then m_ccdMotionThreshold
	void	setCcdMotionThreshold(D_btScalar ccdMotionThreshold)
	{
		m_ccdMotionThreshold = ccdMotionThreshold;
	}

	///users D_can point D_to their objects, userPointer D_is not used by Bullet
	void*	getUserPointer() const
	{
		return m_userObjectPointer;
	}
	
	///users D_can point D_to their objects, userPointer D_is not used by Bullet
	void	setUserPointer(void* userPointer)
	{
		m_userObjectPointer = userPointer;
	}


	inline bool checkCollideWith(D_btCollisionObject* co)
	{
		if (m_checkCollideWith)
			return checkCollideWithOverride(co);

		return true;
	}
};

#endif //COLLISION_OBJECT_H
