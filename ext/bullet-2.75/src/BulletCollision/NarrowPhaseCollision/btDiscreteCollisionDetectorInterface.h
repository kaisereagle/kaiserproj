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


#ifndef DISCRETE_COLLISION_DETECTOR1_INTERFACE_H
#define DISCRETE_COLLISION_DETECTOR1_INTERFACE_H
#include "LinearMath/btTransform.h"
#include "LinearMath/btVector3.h"
class D_btStackAlloc;

/// This interface D_is made D_to be used by an iterative approach D_to do TimeOfImpact calculations
/// This interface D_allows D_to query for closest points D_and penetration depth between two (convex) objects
/// the closest point D_is on the second object (B), D_and the normal points from the surface on B towards A.
/// distance D_is between closest points on B D_and closest point on A. So you D_can calculate closest point on A
/// by taking closestPointInA = closestPointInB + m_distance * m_normalOnSurfaceB
struct D_btDiscreteCollisionDetectorInterface
{
	
	struct D_Result
	{
	
		virtual ~D_Result(){}	

		///setShapeIdentifiersA/B provides experimental support for per-triangle material / custom material combiner
		virtual void setShapeIdentifiersA(int partId0,int index0)=0;
		virtual void setShapeIdentifiersB(int partId1,int index1)=0;
		virtual void addContactPoint(const D_btVector3& normalOnBInWorld,const D_btVector3& pointInWorld,D_btScalar depth)=0;
	};

	struct D_ClosestPointInput
	{
		D_ClosestPointInput()
			:m_maximumDistanceSquared(D_btScalar(D_BT_LARGE_FLOAT)),
			m_stackAlloc(0)
		{
		}

		D_btTransform m_transformA;
		D_btTransform m_transformB;
		D_btScalar	m_maximumDistanceSquared;
		D_btStackAlloc* m_stackAlloc;
	};

	virtual ~D_btDiscreteCollisionDetectorInterface() {};

	//
	// give either closest points (distance > 0) or penetration (distance)
	// the normal always points from B towards A
	//
	virtual void	getClosestPoints(const D_ClosestPointInput& input,D_Result& output,class D_btIDebugDraw* debugDraw,bool swapResults=false) = 0;

};

struct D_btStorageResult : public D_btDiscreteCollisionDetectorInterface::D_Result
{
		D_btVector3	m_normalOnSurfaceB;
		D_btVector3	m_closestPointInB;
		D_btScalar	m_distance; //negative means penetration !

		D_btStorageResult() : m_distance(D_btScalar(D_BT_LARGE_FLOAT))
		{

		}
		virtual ~D_btStorageResult() {};

		virtual void addContactPoint(const D_btVector3& normalOnBInWorld,const D_btVector3& pointInWorld,D_btScalar depth)
		{
			if (depth < m_distance)
			{
				m_normalOnSurfaceB = normalOnBInWorld;
				m_closestPointInB = pointInWorld;
				m_distance = depth;
			}
		}
};

#endif //DISCRETE_COLLISION_DETECTOR_INTERFACE1_H
