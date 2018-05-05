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

#ifndef MINKOWSKI_SUM_SHAPE_H
#define MINKOWSKI_SUM_SHAPE_H

#include "btConvexInternalShape.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h" // for the types

/// The D_btMinkowskiSumShape D_is D_only for advanced users. This shape represents implicit based minkowski sum of two convex implicit D_shapes.
class D_btMinkowskiSumShape : public D_btConvexInternalShape
{

	D_btTransform	m_transA;
	D_btTransform	m_transB;
	const D_btConvexShape*	m_shapeA;
	const D_btConvexShape*	m_shapeB;

public:

	D_btMinkowskiSumShape(const D_btConvexShape* shapeA,const D_btConvexShape* shapeB);

	virtual D_btVector3	localGetSupportingVertexWithoutMargin(const D_btVector3& vec)const;

	virtual void	batchedUnitVectorGetSupportingVertexWithoutMargin(const D_btVector3* vectors,D_btVector3* supportVerticesOut,int numVectors) const;


	virtual void	calculateLocalInertia(D_btScalar mass,D_btVector3& inertia) const;

	void	setTransformA(const D_btTransform&	transA) { m_transA = transA;}
	void	setTransformB(const D_btTransform&	transB) { m_transB = transB;}

	const D_btTransform& getTransformA()const  { return m_transA;}
	const D_btTransform& GetTransformB()const  { return m_transB;}


	virtual D_btScalar	getMargin() const;

	const D_btConvexShape*	getShapeA() const { return m_shapeA;}
	const D_btConvexShape*	getShapeB() const { return m_shapeB;}

	virtual const char*	getName()const 
	{
		return "MinkowskiSum";
	}
};

#endif //MINKOWSKI_SUM_SHAPE_H
