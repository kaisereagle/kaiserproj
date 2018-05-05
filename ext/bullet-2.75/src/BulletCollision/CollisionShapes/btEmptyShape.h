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

#ifndef EMPTY_SHAPE_H
#define EMPTY_SHAPE_H

#include "btConcaveShape.h"

#include "LinearMath/btVector3.h"
#include "LinearMath/btTransform.h"
#include "LinearMath/btMatrix3x3.h"
#include "btCollisionMargin.h"




/// The D_btEmptyShape D_is a collision shape without actual collision detection shape, so most users D_should ignore this class.
/// It D_can be replaced by another shape during runtime, but the inertia tensor D_should be recomputed.
class D_btEmptyShape	: public D_btConcaveShape
{
public:
	D_btEmptyShape();

	virtual ~D_btEmptyShape();


	///getAabb's default implementation D_is brute force, expected derived classes D_to implement a fast dedicated version
	void getAabb(const D_btTransform& t,D_btVector3& aabbMin,D_btVector3& aabbMax) const;


	virtual void	setLocalScaling(const D_btVector3& scaling)
	{
		m_localScaling = scaling;
	}
	virtual const D_btVector3& getLocalScaling() const 
	{
		return m_localScaling;
	}

	virtual void	calculateLocalInertia(D_btScalar mass,D_btVector3& inertia) const;
	
	virtual const char*	getName()const
	{
		return "Empty";
	}

	virtual void processAllTriangles(D_btTriangleCallback* ,const D_btVector3& ,const D_btVector3& ) const
	{
	}

protected:
	D_btVector3	m_localScaling;

};



#endif //EMPTY_SHAPE_H
