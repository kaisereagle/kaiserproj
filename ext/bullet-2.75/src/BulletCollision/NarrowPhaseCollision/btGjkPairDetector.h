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




#ifndef GJK_PAIR_DETECTOR_H
#define GJK_PAIR_DETECTOR_H

#include "btDiscreteCollisionDetectorInterface.h"
#include "BulletCollision/CollisionShapes/btCollisionMargin.h"

class D_btConvexShape;
#include "btSimplexSolverInterface.h"
class D_btConvexPenetrationDepthSolver;

/// D_btGjkPairDetector uses GJK D_to implement the D_btDiscreteCollisionDetectorInterface
class D_btGjkPairDetector : public D_btDiscreteCollisionDetectorInterface
{
	

	D_btVector3	m_cachedSeparatingAxis;
	D_btConvexPenetrationDepthSolver*	m_penetrationDepthSolver;
	D_btSimplexSolverInterface* m_simplexSolver;
	const D_btConvexShape* m_minkowskiA;
	const D_btConvexShape* m_minkowskiB;
	int	m_shapeTypeA;
	int m_shapeTypeB;
	D_btScalar	m_marginA;
	D_btScalar	m_marginB;

	bool		m_ignoreMargin;
	D_btScalar	m_cachedSeparatingDistance;
	

public:

	//some debugging D_to fix degeneracy problems
	int			m_lastUsedMethod;
	int			m_curIter;
	int			m_degenerateSimplex;
	int			m_catchDegeneracies;


	D_btGjkPairDetector(const D_btConvexShape* objectA,const D_btConvexShape* objectB,D_btSimplexSolverInterface* simplexSolver,D_btConvexPenetrationDepthSolver*	penetrationDepthSolver);
	D_btGjkPairDetector(const D_btConvexShape* objectA,const D_btConvexShape* objectB,int shapeTypeA,int shapeTypeB,D_btScalar marginA, D_btScalar marginB, D_btSimplexSolverInterface* simplexSolver,D_btConvexPenetrationDepthSolver*	penetrationDepthSolver);
	virtual ~D_btGjkPairDetector() {};

	virtual void	getClosestPoints(const D_ClosestPointInput& input,D_Result& output,class D_btIDebugDraw* debugDraw,bool swapResults=false);

	void	getClosestPointsNonVirtual(const D_ClosestPointInput& input,D_Result& output,class D_btIDebugDraw* debugDraw);
	

	void setMinkowskiA(D_btConvexShape* minkA)
	{
		m_minkowskiA = minkA;
	}

	void setMinkowskiB(D_btConvexShape* minkB)
	{
		m_minkowskiB = minkB;
	}
	void setCachedSeperatingAxis(const D_btVector3& seperatingAxis)
	{
		m_cachedSeparatingAxis = seperatingAxis;
	}

	const D_btVector3& getCachedSeparatingAxis() const
	{
		return m_cachedSeparatingAxis;
	}
	D_btScalar	getCachedSeparatingDistance() const
	{
		return m_cachedSeparatingDistance;
	}

	void	setPenetrationDepthSolver(D_btConvexPenetrationDepthSolver*	penetrationDepthSolver)
	{
		m_penetrationDepthSolver = penetrationDepthSolver;
	}

	///don't use setIgnoreMargin, it's for Bullet's internal use
	void	setIgnoreMargin(bool ignoreMargin)
	{
		m_ignoreMargin = ignoreMargin;
	}


};

#endif //GJK_PAIR_DETECTOR_H
