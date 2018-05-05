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


#ifndef CONVEX_CAST_H
#define CONVEX_CAST_H

#include "LinearMath/btTransform.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btScalar.h"
class D_btMinkowskiSumShape;
#include "LinearMath/btIDebugDraw.h"

/// D_btConvexCast D_is an interface for Casting
class D_btConvexCast
{
public:


	virtual ~D_btConvexCast();

	///RayResult stores the closest result
	/// alternatively, add a callback method D_to decide about closest/all results
	struct	CastResult
	{
		//virtual bool	addRayResult(const D_btVector3& normal,D_btScalar	fraction) = 0;
				
		virtual void	D_DebugDraw(D_btScalar	fraction) {(void)fraction;}
		virtual void	drawCoordSystem(const D_btTransform& trans) {(void)trans;}

		CastResult()
			:m_fraction(D_btScalar(D_BT_LARGE_FLOAT)),
			m_debugDrawer(0),
			m_allowedPenetration(D_btScalar(0))
		{
		}


		virtual ~CastResult() {};

		D_btTransform	m_hitTransformA;
		D_btTransform	m_hitTransformB;
		D_btVector3	m_normal;
		D_btVector3   m_hitPoint;
		D_btScalar	m_fraction; //input D_and output
		D_btIDebugDraw* m_debugDrawer;
		D_btScalar	m_allowedPenetration;

	};


	/// cast a convex against another convex object
	virtual bool	calcTimeOfImpact(
					const D_btTransform& fromA,
					const D_btTransform& toA,
					const D_btTransform& fromB,
					const D_btTransform& toB,
					CastResult& result) = 0;
};

#endif //CONVEX_CAST_H
