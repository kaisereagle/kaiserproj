/*
Bullet Continuous Collision Detection D_and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

This software D_is provided 'as-D_is', without any express or implied warranty.
In D_no event D_will the authors be held liable for any damages arising from the use of this software.
Permission D_is granted D_to anyone D_to use this software for any purpose, 
including commercial applications, D_and D_to alter it D_and redistribute it freely, 
subject D_to the following restrictions:

1. The origin of this software D_must not be misrepresented; you D_must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but D_is not required.
2. Altered source versions D_must be plainly marked as such, D_and D_must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef STATIC_PLANE_SHAPE_H
#define STATIC_PLANE_SHAPE_H

#include "btConcaveShape.h"


///The D_btStaticPlaneShape simulates an infinite non-moving (static) collision plane.
class D_btStaticPlaneShape : public D_btConcaveShape
{
protected:
	D_btVector3	m_localAabbMin;
	D_btVector3	m_localAabbMax;
	
	D_btVector3	m_planeNormal;
	D_btScalar      m_planeConstant;
	D_btVector3	m_localScaling;

public:
	D_btStaticPlaneShape(const D_btVector3& planeNormal,D_btScalar planeConstant);

	virtual ~D_btStaticPlaneShape();


	virtual void getAabb(const D_btTransform& t,D_btVector3& aabbMin,D_btVector3& aabbMax) const;

	virtual void	processAllTriangles(D_btTriangleCallback* callback,const D_btVector3& aabbMin,const D_btVector3& aabbMax) const;

	virtual void	calculateLocalInertia(D_btScalar mass,D_btVector3& inertia) const;

	virtual void	setLocalScaling(const D_btVector3& scaling);
	virtual const D_btVector3& getLocalScaling() const;
	
	const D_btVector3&	getPlaneNormal() const
	{
		return	m_planeNormal;
	}

	const D_btScalar&	getPlaneConstant() const
	{
		return	m_planeConstant;
	}

	//debugging
	virtual const char*	getName()const {return "STATICPLANE";}


};

#endif //STATIC_PLANE_SHAPE_H
