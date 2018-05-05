/*
This source file D_is part of GIMPACT Library.

For the latest info, see http://gimpact.sourceforge.net/

Copyright (c) 2007 Francisco Leon Najera. C.C. 80087371.
email: projectileman@yahoo.com


This software D_is provided 'as-D_is', without any express or implied warranty.
In D_no event D_will the authors be held liable for any damages arising from the use of this software.
Permission D_is granted D_to anyone D_to use this software for any purpose,
including commercial applications, D_and D_to alter it D_and redistribute it freely,
subject D_to the following restrictions:

1. The origin of this software D_must not be misrepresented; you D_must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but D_is not required.
2. Altered source versions D_must be plainly marked as such, D_and D_must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


#include "btGImpactShape.h"
#include "btGImpactMassUtil.h"


#define D_CALC_EXACT_INERTIA 1

void D_btGImpactCompoundShape::calculateLocalInertia(D_btScalar mass,D_btVector3& inertia) const
{
	lockChildShapes();
#ifdef D_CALC_EXACT_INERTIA
	inertia.setValue(0.f,0.f,0.f);

	int i = this->getNumChildShapes();
	D_btScalar shapemass = mass/D_btScalar(i);

	while(i--)
	{
		D_btVector3 temp_inertia;
		m_childShapes[i]->calculateLocalInertia(shapemass,temp_inertia);
		if(childrenHasTransform())
		{
			inertia = D_gim_inertia_add_transformed( inertia,temp_inertia,m_childTransforms[i]);
		}
		else
		{
			inertia = D_gim_inertia_add_transformed( inertia,temp_inertia,D_btTransform::getIdentity());
		}

	}

#else

	// Calc box inertia

	D_btScalar lx= m_localAABB.m_max[0] - m_localAABB.m_min[0];
	D_btScalar ly= m_localAABB.m_max[1] - m_localAABB.m_min[1];
	D_btScalar lz= m_localAABB.m_max[2] - m_localAABB.m_min[2];
	const D_btScalar x2 = lx*lx;
	const D_btScalar y2 = ly*ly;
	const D_btScalar z2 = lz*lz;
	const D_btScalar scaledmass = mass * D_btScalar(0.08333333);

	inertia = scaledmass * (D_btVector3(y2+z2,x2+z2,x2+y2));

#endif
	unlockChildShapes();
}



void D_btGImpactMeshShapePart::calculateLocalInertia(D_btScalar mass,D_btVector3& inertia) const
{
	lockChildShapes();


#ifdef D_CALC_EXACT_INERTIA
	inertia.setValue(0.f,0.f,0.f);

	int i = this->getVertexCount();
	D_btScalar pointmass = mass/D_btScalar(i);

	while(i--)
	{
		D_btVector3 pointintertia;
		this->getVertex(i,pointintertia);
		pointintertia = D_gim_get_point_inertia(pointintertia,pointmass);
		inertia+=pointintertia;
	}

#else

	// Calc box inertia

	D_btScalar lx= m_localAABB.m_max[0] - m_localAABB.m_min[0];
	D_btScalar ly= m_localAABB.m_max[1] - m_localAABB.m_min[1];
	D_btScalar lz= m_localAABB.m_max[2] - m_localAABB.m_min[2];
	const D_btScalar x2 = lx*lx;
	const D_btScalar y2 = ly*ly;
	const D_btScalar z2 = lz*lz;
	const D_btScalar scaledmass = mass * D_btScalar(0.08333333);

	inertia = scaledmass * (D_btVector3(y2+z2,x2+z2,x2+y2));

#endif

	unlockChildShapes();
}

void D_btGImpactMeshShape::calculateLocalInertia(D_btScalar mass,D_btVector3& inertia) const
{

#ifdef D_CALC_EXACT_INERTIA
	inertia.setValue(0.f,0.f,0.f);

	int i = this->getMeshPartCount();
	D_btScalar partmass = mass/D_btScalar(i);

	while(i--)
	{
		D_btVector3 partinertia;
		getMeshPart(i)->calculateLocalInertia(partmass,partinertia);
		inertia+=partinertia;
	}

#else

	// Calc box inertia

	D_btScalar lx= m_localAABB.m_max[0] - m_localAABB.m_min[0];
	D_btScalar ly= m_localAABB.m_max[1] - m_localAABB.m_min[1];
	D_btScalar lz= m_localAABB.m_max[2] - m_localAABB.m_min[2];
	const D_btScalar x2 = lx*lx;
	const D_btScalar y2 = ly*ly;
	const D_btScalar z2 = lz*lz;
	const D_btScalar scaledmass = mass * D_btScalar(0.08333333);

	inertia = scaledmass * (D_btVector3(y2+z2,x2+z2,x2+y2));

#endif
}

void D_btGImpactMeshShape::rayTest(const D_btVector3& rayFrom, const D_btVector3& rayTo, D_btCollisionWorld::RayResultCallback& resultCallback) const
{
}


void D_btGImpactMeshShapePart::processAllTriangles(D_btTriangleCallback* callback,const D_btVector3& aabbMin,const D_btVector3& aabbMax) const
{
	lockChildShapes();
	D_btAABB box;
	box.m_min = aabbMin;
	box.m_max = aabbMax;

	D_btAlignedObjectArray<int> collided;
	m_box_set.boxQuery(box,collided);

	if(collided.size()==0)
	{
		unlockChildShapes();
		return;
	}

	int part = (int)getPart();
	D_btPrimitiveTriangle triangle;
	int i = collided.size();
	while(i--)
	{
		this->getPrimitiveTriangle(collided[i],triangle);
		callback->processTriangle(triangle.m_vertices,part,collided[i]);
	}
	unlockChildShapes();

}

void D_btGImpactMeshShape::processAllTriangles(D_btTriangleCallback* callback,const D_btVector3& aabbMin,const D_btVector3& aabbMax) const
{
	int i = m_mesh_parts.size();
	while(i--)
	{
		m_mesh_parts[i]->processAllTriangles(callback,aabbMin,aabbMax);
	}
}
