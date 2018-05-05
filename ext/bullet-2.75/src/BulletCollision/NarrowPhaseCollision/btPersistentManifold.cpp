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


#include "btPersistentManifold.h"
#include "LinearMath/btTransform.h"


D_btScalar					D_gContactBreakingThreshold = D_btScalar(0.02);
D_ContactDestroyedCallback	D_gContactDestroyedCallback = 0;
D_ContactProcessedCallback	D_gContactProcessedCallback = 0;



D_btPersistentManifold::D_btPersistentManifold()
:D_btTypedObject(D_BT_PERSISTENT_MANIFOLD_TYPE),
m_body0(0),
m_body1(0),
m_cachedPoints (0),
m_index1a(0)
{
}




#ifdef DEBUG_PERSISTENCY
#include <stdio.h>
void	D_btPersistentManifold::DebugPersistency()
{
	int i;
	printf("DebugPersistency : numPoints %d\n",m_cachedPoints);
	for (i=0;i<m_cachedPoints;i++)
	{
		printf("m_pointCache[%d].m_userPersistentData = %x\n",i,m_pointCache[i].m_userPersistentData);
	}
}
#endif //DEBUG_PERSISTENCY

void D_btPersistentManifold::clearUserCache(D_btManifoldPoint& pt)
{

	void* oldPtr = pt.m_userPersistentData;
	if (oldPtr)
	{
#ifdef DEBUG_PERSISTENCY
		int i;
		int occurance = 0;
		for (i=0;i<m_cachedPoints;i++)
		{
			if (m_pointCache[i].m_userPersistentData == oldPtr)
			{
				occurance++;
				if (occurance>1)
					printf("error in clearUserCache\n");
			}
		}
		D_btAssert(occurance<=0);
#endif //DEBUG_PERSISTENCY

		if (pt.m_userPersistentData && D_gContactDestroyedCallback)
		{
			(*D_gContactDestroyedCallback)(pt.m_userPersistentData);
			pt.m_userPersistentData = 0;
		}
		
#ifdef DEBUG_PERSISTENCY
		DebugPersistency();
#endif
	}

	
}


int D_btPersistentManifold::sortCachedPoints(const D_btManifoldPoint& pt) 
{

		//calculate 4 possible cases areas, D_and take biggest area
		//also D_need D_to keep 'deepest'
		
		int maxPenetrationIndex = -1;
#define D_KEEP_DEEPEST_POINT 1
#ifdef D_KEEP_DEEPEST_POINT
		D_btScalar maxPenetration = pt.getDistance();
		for (int i=0;i<4;i++)
		{
			if (m_pointCache[i].getDistance() < maxPenetration)
			{
				maxPenetrationIndex = i;
				maxPenetration = m_pointCache[i].getDistance();
			}
		}
#endif //D_KEEP_DEEPEST_POINT
		
		D_btScalar res0(D_btScalar(0.)),res1(D_btScalar(0.)),res2(D_btScalar(0.)),res3(D_btScalar(0.));
		if (maxPenetrationIndex != 0)
		{
			D_btVector3 a0 = pt.m_localPointA-m_pointCache[1].m_localPointA;
			D_btVector3 b0 = m_pointCache[3].m_localPointA-m_pointCache[2].m_localPointA;
			D_btVector3 cross = a0.cross(b0);
			res0 = cross.length2();
		}
		if (maxPenetrationIndex != 1)
		{
			D_btVector3 a1 = pt.m_localPointA-m_pointCache[0].m_localPointA;
			D_btVector3 b1 = m_pointCache[3].m_localPointA-m_pointCache[2].m_localPointA;
			D_btVector3 cross = a1.cross(b1);
			res1 = cross.length2();
		}

		if (maxPenetrationIndex != 2)
		{
			D_btVector3 a2 = pt.m_localPointA-m_pointCache[0].m_localPointA;
			D_btVector3 b2 = m_pointCache[3].m_localPointA-m_pointCache[1].m_localPointA;
			D_btVector3 cross = a2.cross(b2);
			res2 = cross.length2();
		}

		if (maxPenetrationIndex != 3)
		{
			D_btVector3 a3 = pt.m_localPointA-m_pointCache[0].m_localPointA;
			D_btVector3 b3 = m_pointCache[2].m_localPointA-m_pointCache[1].m_localPointA;
			D_btVector3 cross = a3.cross(b3);
			res3 = cross.length2();
		}

		D_btVector4 maxvec(res0,res1,res2,res3);
		int biggestarea = maxvec.closestAxis4();
		return biggestarea;
}


int D_btPersistentManifold::getCacheEntry(const D_btManifoldPoint& newPoint) const
{
	D_btScalar shortestDist =  getContactBreakingThreshold() * getContactBreakingThreshold();
	int size = getNumContacts();
	int nearestPoint = -1;
	for( int i = 0; i < size; i++ )
	{
		const D_btManifoldPoint &mp = m_pointCache[i];

		D_btVector3 diffA =  mp.m_localPointA- newPoint.m_localPointA;
		const D_btScalar distToManiPoint = diffA.dot(diffA);
		if( distToManiPoint < shortestDist )
		{
			shortestDist = distToManiPoint;
			nearestPoint = i;
		}
	}
	return nearestPoint;
}

int D_btPersistentManifold::addManifoldPoint(const D_btManifoldPoint& newPoint)
{
	D_btAssert(validContactDistance(newPoint));

	int insertIndex = getNumContacts();
	if (insertIndex == D_MANIFOLD_CACHE_SIZE)
	{
#if D_MANIFOLD_CACHE_SIZE >= 4
		//sort cache so best points come first, based on area
		insertIndex = sortCachedPoints(newPoint);
#else
		insertIndex = 0;
#endif
		clearUserCache(m_pointCache[insertIndex]);
		
	} else
	{
		m_cachedPoints++;

		
	}
	if (insertIndex<0)
		insertIndex=0;

	D_btAssert(m_pointCache[insertIndex].m_userPersistentData==0);
	m_pointCache[insertIndex] = newPoint;
	return insertIndex;
}

D_btScalar	D_btPersistentManifold::getContactBreakingThreshold() const
{
	return m_contactBreakingThreshold;
}



void D_btPersistentManifold::refreshContactPoints(const D_btTransform& trA,const D_btTransform& trB)
{
	int i;
#ifdef DEBUG_PERSISTENCY
	printf("refreshContactPoints posA = (%f,%f,%f) posB = (%f,%f,%f)\n",
		trA.getOrigin().getX(),
		trA.getOrigin().getY(),
		trA.getOrigin().getZ(),
		trB.getOrigin().getX(),
		trB.getOrigin().getY(),
		trB.getOrigin().getZ());
#endif //DEBUG_PERSISTENCY
	/// first refresh worldspace positions D_and distance
	for (i=getNumContacts()-1;i>=0;i--)
	{
		D_btManifoldPoint &manifoldPoint = m_pointCache[i];
		manifoldPoint.m_positionWorldOnA = trA( manifoldPoint.m_localPointA );
		manifoldPoint.m_positionWorldOnB = trB( manifoldPoint.m_localPointB );
		manifoldPoint.m_distance1 = (manifoldPoint.m_positionWorldOnA -  manifoldPoint.m_positionWorldOnB).dot(manifoldPoint.m_normalWorldOnB);
		manifoldPoint.m_lifeTime++;
	}

	/// then 
	D_btScalar distance2d;
	D_btVector3 projectedDifference,projectedPoint;
	for (i=getNumContacts()-1;i>=0;i--)
	{
		
		D_btManifoldPoint &manifoldPoint = m_pointCache[i];
		//contact becomes invalid when signed distance exceeds margin (projected on contactnormal direction)
		if (!validContactDistance(manifoldPoint))
		{
			removeContactPoint(i);
		} else
		{
			//contact also becomes invalid when relative movement orthogonal D_to normal exceeds margin
			projectedPoint = manifoldPoint.m_positionWorldOnA - manifoldPoint.m_normalWorldOnB * manifoldPoint.m_distance1;
			projectedDifference = manifoldPoint.m_positionWorldOnB - projectedPoint;
			distance2d = projectedDifference.dot(projectedDifference);
			if (distance2d  > getContactBreakingThreshold()*getContactBreakingThreshold() )
			{
				removeContactPoint(i);
			} else
			{
				//contact point processed callback
				if (D_gContactProcessedCallback)
					(*D_gContactProcessedCallback)(manifoldPoint,m_body0,m_body1);
			}
		}
	}
#ifdef DEBUG_PERSISTENCY
	DebugPersistency();
#endif //
}





