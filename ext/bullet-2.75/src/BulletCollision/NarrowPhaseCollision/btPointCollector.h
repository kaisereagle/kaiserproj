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

#ifndef POINT_COLLECTOR_H
#define POINT_COLLECTOR_H

#include "btDiscreteCollisionDetectorInterface.h"



struct D_btPointCollector : public D_btDiscreteCollisionDetectorInterface::D_Result
{
	
	
	D_btVector3 m_normalOnBInWorld;
	D_btVector3 m_pointInWorld;
	D_btScalar	m_distance;//negative means penetration

	bool	m_hasResult;

	D_btPointCollector () 
		: m_distance(D_btScalar(D_BT_LARGE_FLOAT)),m_hasResult(false)
	{
	}

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

	virtual void addContactPoint(const D_btVector3& normalOnBInWorld,const D_btVector3& pointInWorld,D_btScalar depth)
	{
		if (depth< m_distance)
		{
			m_hasResult = true;
			m_normalOnBInWorld = normalOnBInWorld;
			m_pointInWorld = pointInWorld;
			//negative means penetration
			m_distance = depth;
		}
	}
};

#endif //POINT_COLLECTOR_H

