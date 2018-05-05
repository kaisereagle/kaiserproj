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

#include "btCompoundShape.h"
#include "btCollisionShape.h"
#include "BulletCollision/BroadphaseCollision/btDbvt.h"

D_btCompoundShape::D_btCompoundShape(bool enableDynamicAabbTree)
: m_localAabbMin(D_btScalar(D_BT_LARGE_FLOAT),D_btScalar(D_BT_LARGE_FLOAT),D_btScalar(D_BT_LARGE_FLOAT)),
m_localAabbMax(D_btScalar(-D_BT_LARGE_FLOAT),D_btScalar(-D_BT_LARGE_FLOAT),D_btScalar(-D_BT_LARGE_FLOAT)),
m_dynamicAabbTree(0),
m_updateRevision(1),
m_collisionMargin(D_btScalar(0.)),
m_localScaling(D_btScalar(1.),D_btScalar(1.),D_btScalar(1.))
{
	m_shapeType = D_COMPOUND_SHAPE_PROXYTYPE;

	if (enableDynamicAabbTree)
	{
		void* mem = D_btAlignedAlloc(sizeof(D_btDbvt),16);
		m_dynamicAabbTree = new(mem) D_btDbvt();
		D_btAssert(mem==m_dynamicAabbTree);
	}
}


D_btCompoundShape::~D_btCompoundShape()
{
	if (m_dynamicAabbTree)
	{
		m_dynamicAabbTree->~D_btDbvt();
		D_btAlignedFree(m_dynamicAabbTree);
	}
}

void	D_btCompoundShape::addChildShape(const D_btTransform& localTransform,D_btCollisionShape* shape)
{
	m_updateRevision++;
	//m_childTransforms.push_back(localTransform);
	//m_childShapes.push_back(shape);
	D_btCompoundShapeChild child;
	child.m_transform = localTransform;
	child.m_childShape = shape;
	child.m_childShapeType = shape->getShapeType();
	child.m_childMargin = shape->getMargin();

	
	//extend the local aabbMin/aabbMax
	D_btVector3 localAabbMin,localAabbMax;
	shape->getAabb(localTransform,localAabbMin,localAabbMax);
	for (int i=0;i<3;i++)
	{
		if (m_localAabbMin[i] > localAabbMin[i])
		{
			m_localAabbMin[i] = localAabbMin[i];
		}
		if (m_localAabbMax[i] < localAabbMax[i])
		{
			m_localAabbMax[i] = localAabbMax[i];
		}

	}
	if (m_dynamicAabbTree)
	{
		const D_btDbvtVolume	bounds=D_btDbvtVolume::FromMM(localAabbMin,localAabbMax);
		int index = m_children.size();
		child.m_node = m_dynamicAabbTree->insert(bounds,(void*)index);
	}

	m_children.push_back(child);

}

void	D_btCompoundShape::updateChildTransform(int childIndex, const D_btTransform& newChildTransform)
{
	m_children[childIndex].m_transform = newChildTransform;

	if (m_dynamicAabbTree)
	{
		///update the dynamic aabb tree
		D_btVector3 localAabbMin,localAabbMax;
		m_children[childIndex].m_childShape->getAabb(newChildTransform,localAabbMin,localAabbMax);
		D_ATTRIBUTE_ALIGNED16(D_btDbvtVolume)	bounds=D_btDbvtVolume::FromMM(localAabbMin,localAabbMax);
		//int index = m_children.size()-1;
		m_dynamicAabbTree->update(m_children[childIndex].m_node,bounds);
	}

	recalculateLocalAabb();
}

void D_btCompoundShape::removeChildShapeByIndex(int childShapeIndex)
{
	m_updateRevision++;
	D_btAssert(childShapeIndex >=0 && childShapeIndex < m_children.size());
	if (m_dynamicAabbTree)
	{
		m_dynamicAabbTree->remove(m_children[childShapeIndex].m_node);
	}
	m_children.swap(childShapeIndex,m_children.size()-1);
	m_children.pop_back();

}



void D_btCompoundShape::removeChildShape(D_btCollisionShape* shape)
{
	m_updateRevision++;
	// Find the children containing the shape specified, D_and remove those children.
	//note: there might be multiple children using the same shape!
	for(int i = m_children.size()-1; i >= 0 ; i--)
	{
		if(m_children[i].m_childShape == shape)
		{
			removeChildShapeByIndex(i);
		}
	}



	recalculateLocalAabb();
}

void D_btCompoundShape::recalculateLocalAabb()
{
	// Recalculate the local aabb
	// Brute force, it iterates over all the D_shapes left.

	m_localAabbMin = D_btVector3(D_btScalar(D_BT_LARGE_FLOAT),D_btScalar(D_BT_LARGE_FLOAT),D_btScalar(D_BT_LARGE_FLOAT));
	m_localAabbMax = D_btVector3(D_btScalar(-D_BT_LARGE_FLOAT),D_btScalar(-D_BT_LARGE_FLOAT),D_btScalar(-D_BT_LARGE_FLOAT));

	//extend the local aabbMin/aabbMax
	for (int j = 0; j < m_children.size(); j++)
	{
		D_btVector3 localAabbMin,localAabbMax;
		m_children[j].m_childShape->getAabb(m_children[j].m_transform, localAabbMin, localAabbMax);
		for (int i=0;i<3;i++)
		{
			if (m_localAabbMin[i] > localAabbMin[i])
				m_localAabbMin[i] = localAabbMin[i];
			if (m_localAabbMax[i] < localAabbMax[i])
				m_localAabbMax[i] = localAabbMax[i];
		}
	}
}

///getAabb's default implementation D_is brute force, expected derived classes D_to implement a fast dedicated version
void D_btCompoundShape::getAabb(const D_btTransform& trans,D_btVector3& aabbMin,D_btVector3& aabbMax) const
{
	D_btVector3 localHalfExtents = D_btScalar(0.5)*(m_localAabbMax-m_localAabbMin);
	D_btVector3 localCenter = D_btScalar(0.5)*(m_localAabbMax+m_localAabbMin);
	
	//avoid an illegal AABB when there D_are D_no children
	if (!m_children.size())
	{
		localHalfExtents.setValue(0,0,0);
		localCenter.setValue(0,0,0);
	}
	localHalfExtents += D_btVector3(getMargin(),getMargin(),getMargin());
		

	D_btMatrix3x3 abs_b = trans.getBasis().absolute();  

	D_btVector3 center = trans(localCenter);

	D_btVector3 extent = D_btVector3(abs_b[0].dot(localHalfExtents),
		abs_b[1].dot(localHalfExtents),
		abs_b[2].dot(localHalfExtents));
	aabbMin = center-extent;
	aabbMax = center+extent;
	
}

void	D_btCompoundShape::calculateLocalInertia(D_btScalar mass,D_btVector3& inertia) const
{
	//approximation: take the inertia from the aabb for now
	D_btTransform ident;
	ident.setIdentity();
	D_btVector3 aabbMin,aabbMax;
	getAabb(ident,aabbMin,aabbMax);

	D_btVector3 halfExtents = (aabbMax-aabbMin)*D_btScalar(0.5);

	D_btScalar lx=D_btScalar(2.)*(halfExtents.x());
	D_btScalar ly=D_btScalar(2.)*(halfExtents.y());
	D_btScalar lz=D_btScalar(2.)*(halfExtents.z());

	inertia[0] = mass/(D_btScalar(12.0)) * (ly*ly + lz*lz);
	inertia[1] = mass/(D_btScalar(12.0)) * (lx*lx + lz*lz);
	inertia[2] = mass/(D_btScalar(12.0)) * (lx*lx + ly*ly);

}




void D_btCompoundShape::calculatePrincipalAxisTransform(D_btScalar* masses, D_btTransform& principal, D_btVector3& inertia) const
{
	int n = m_children.size();

	D_btScalar totalMass = 0;
	D_btVector3 center(0, 0, 0);
	int k;

	for (k = 0; k < n; k++)
	{
		center += m_children[k].m_transform.getOrigin() * masses[k];
		totalMass += masses[k];
	}
	center /= totalMass;
	principal.setOrigin(center);

	D_btMatrix3x3 tensor(0, 0, 0, 0, 0, 0, 0, 0, 0);
	for ( k = 0; k < n; k++)
	{
		D_btVector3 i;
		m_children[k].m_childShape->calculateLocalInertia(masses[k], i);

		const D_btTransform& t = m_children[k].m_transform;
		D_btVector3 o = t.getOrigin() - center;

		//compute inertia tensor in coordinate system of compound shape
		D_btMatrix3x3 j = t.getBasis().transpose();
		j[0] *= i[0];
		j[1] *= i[1];
		j[2] *= i[2];
		j = t.getBasis() * j;

		//add inertia tensor
		tensor[0] += j[0];
		tensor[1] += j[1];
		tensor[2] += j[2];

		//compute inertia tensor of pointmass at o
		D_btScalar o2 = o.length2();
		j[0].setValue(o2, 0, 0);
		j[1].setValue(0, o2, 0);
		j[2].setValue(0, 0, o2);
		j[0] += o * -o.x(); 
		j[1] += o * -o.y(); 
		j[2] += o * -o.z();

		//add inertia tensor of pointmass
		tensor[0] += masses[k] * j[0];
		tensor[1] += masses[k] * j[1];
		tensor[2] += masses[k] * j[2];
	}

	tensor.diagonalize(principal.getBasis(), D_btScalar(0.00001), 20);
	inertia.setValue(tensor[0][0], tensor[1][1], tensor[2][2]);
}



