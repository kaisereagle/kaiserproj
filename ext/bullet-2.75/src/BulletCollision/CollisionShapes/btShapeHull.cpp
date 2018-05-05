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

//D_btShapeHull was implemented by John McCutchan.


#include "btShapeHull.h"
#include "LinearMath/btConvexHull.h"

#define D_NUM_UNITSPHERE_POINTS 42

static D_btVector3 D_btUnitSpherePoints[D_NUM_UNITSPHERE_POINTS+D_MAX_PREFERRED_PENETRATION_DIRECTIONS*2] = 
{
	D_btVector3(D_btScalar(0.000000) , D_btScalar(-0.000000),D_btScalar(-1.000000)),
	D_btVector3(D_btScalar(0.723608) , D_btScalar(-0.525725),D_btScalar(-0.447219)),
	D_btVector3(D_btScalar(-0.276388) , D_btScalar(-0.850649),D_btScalar(-0.447219)),
	D_btVector3(D_btScalar(-0.894426) , D_btScalar(-0.000000),D_btScalar(-0.447216)),
	D_btVector3(D_btScalar(-0.276388) , D_btScalar(0.850649),D_btScalar(-0.447220)),
	D_btVector3(D_btScalar(0.723608) , D_btScalar(0.525725),D_btScalar(-0.447219)),
	D_btVector3(D_btScalar(0.276388) , D_btScalar(-0.850649),D_btScalar(0.447220)),
	D_btVector3(D_btScalar(-0.723608) , D_btScalar(-0.525725),D_btScalar(0.447219)),
	D_btVector3(D_btScalar(-0.723608) , D_btScalar(0.525725),D_btScalar(0.447219)),
	D_btVector3(D_btScalar(0.276388) , D_btScalar(0.850649),D_btScalar(0.447219)),
	D_btVector3(D_btScalar(0.894426) , D_btScalar(0.000000),D_btScalar(0.447216)),
	D_btVector3(D_btScalar(-0.000000) , D_btScalar(0.000000),D_btScalar(1.000000)),
	D_btVector3(D_btScalar(0.425323) , D_btScalar(-0.309011),D_btScalar(-0.850654)),
	D_btVector3(D_btScalar(-0.162456) , D_btScalar(-0.499995),D_btScalar(-0.850654)),
	D_btVector3(D_btScalar(0.262869) , D_btScalar(-0.809012),D_btScalar(-0.525738)),
	D_btVector3(D_btScalar(0.425323) , D_btScalar(0.309011),D_btScalar(-0.850654)),
	D_btVector3(D_btScalar(0.850648) , D_btScalar(-0.000000),D_btScalar(-0.525736)),
	D_btVector3(D_btScalar(-0.525730) , D_btScalar(-0.000000),D_btScalar(-0.850652)),
	D_btVector3(D_btScalar(-0.688190) , D_btScalar(-0.499997),D_btScalar(-0.525736)),
	D_btVector3(D_btScalar(-0.162456) , D_btScalar(0.499995),D_btScalar(-0.850654)),
	D_btVector3(D_btScalar(-0.688190) , D_btScalar(0.499997),D_btScalar(-0.525736)),
	D_btVector3(D_btScalar(0.262869) , D_btScalar(0.809012),D_btScalar(-0.525738)),
	D_btVector3(D_btScalar(0.951058) , D_btScalar(0.309013),D_btScalar(0.000000)),
	D_btVector3(D_btScalar(0.951058) , D_btScalar(-0.309013),D_btScalar(0.000000)),
	D_btVector3(D_btScalar(0.587786) , D_btScalar(-0.809017),D_btScalar(0.000000)),
	D_btVector3(D_btScalar(0.000000) , D_btScalar(-1.000000),D_btScalar(0.000000)),
	D_btVector3(D_btScalar(-0.587786) , D_btScalar(-0.809017),D_btScalar(0.000000)),
	D_btVector3(D_btScalar(-0.951058) , D_btScalar(-0.309013),D_btScalar(-0.000000)),
	D_btVector3(D_btScalar(-0.951058) , D_btScalar(0.309013),D_btScalar(-0.000000)),
	D_btVector3(D_btScalar(-0.587786) , D_btScalar(0.809017),D_btScalar(-0.000000)),
	D_btVector3(D_btScalar(-0.000000) , D_btScalar(1.000000),D_btScalar(-0.000000)),
	D_btVector3(D_btScalar(0.587786) , D_btScalar(0.809017),D_btScalar(-0.000000)),
	D_btVector3(D_btScalar(0.688190) , D_btScalar(-0.499997),D_btScalar(0.525736)),
	D_btVector3(D_btScalar(-0.262869) , D_btScalar(-0.809012),D_btScalar(0.525738)),
	D_btVector3(D_btScalar(-0.850648) , D_btScalar(0.000000),D_btScalar(0.525736)),
	D_btVector3(D_btScalar(-0.262869) , D_btScalar(0.809012),D_btScalar(0.525738)),
	D_btVector3(D_btScalar(0.688190) , D_btScalar(0.499997),D_btScalar(0.525736)),
	D_btVector3(D_btScalar(0.525730) , D_btScalar(0.000000),D_btScalar(0.850652)),
	D_btVector3(D_btScalar(0.162456) , D_btScalar(-0.499995),D_btScalar(0.850654)),
	D_btVector3(D_btScalar(-0.425323) , D_btScalar(-0.309011),D_btScalar(0.850654)),
	D_btVector3(D_btScalar(-0.425323) , D_btScalar(0.309011),D_btScalar(0.850654)),
	D_btVector3(D_btScalar(0.162456) , D_btScalar(0.499995),D_btScalar(0.850654))
};

D_btShapeHull::D_btShapeHull (const D_btConvexShape* shape)
{
	m_shape = shape;
	m_vertices.clear ();
	m_indices.clear();
	m_numIndices = 0;
}

D_btShapeHull::~D_btShapeHull ()
{
	m_indices.clear();	
	m_vertices.clear ();
}

bool
D_btShapeHull::buildHull (D_btScalar /*margin*/)
{
	int numSampleDirections = D_NUM_UNITSPHERE_POINTS;
	{
		int numPDA = m_shape->getNumPreferredPenetrationDirections();
		if (numPDA)
		{
			for (int i=0;i<numPDA;i++)
			{
				D_btVector3 norm;
				m_shape->getPreferredPenetrationDirection(i,norm);
				D_btUnitSpherePoints[numSampleDirections] = norm;
				numSampleDirections++;
			}
		}
	}

	D_btVector3 supportPoints[D_NUM_UNITSPHERE_POINTS+D_MAX_PREFERRED_PENETRATION_DIRECTIONS*2];
	int i;
	for (i = 0; i < numSampleDirections; i++)
	{
		supportPoints[i] = m_shape->localGetSupportingVertex(D_btUnitSpherePoints[i]);
	}

	D_HullDesc hd;
	hd.mFlags = D_QF_TRIANGLES;
	hd.mVcount = static_cast<unsigned int>(numSampleDirections);

#ifdef D_BT_USE_DOUBLE_PRECISION
	hd.mVertices = &supportPoints[0];
	hd.mVertexStride = sizeof(D_btVector3);
#else
	hd.mVertices = &supportPoints[0];
	hd.mVertexStride = sizeof (D_btVector3);
#endif

	D_HullLibrary hl;
	D_HullResult hr;
	if (hl.CreateConvexHull (hd, hr) == D_QE_FAIL)
	{
		return false;
	}

	m_vertices.resize (static_cast<int>(hr.mNumOutputVertices));


	for (i = 0; i < static_cast<int>(hr.mNumOutputVertices); i++)
	{
		m_vertices[i] = hr.m_OutputVertices[i];
	}
	m_numIndices = hr.mNumIndices;
	m_indices.resize(static_cast<int>(m_numIndices));
	for (i = 0; i < static_cast<int>(m_numIndices); i++)
	{
		m_indices[i] = hr.m_Indices[i];
	}

	// free temporary hull result that we D_just copied
	hl.ReleaseResult (hr);

	return true;
}

int
D_btShapeHull::numTriangles () const
{
	return static_cast<int>(m_numIndices / 3);
}

int
D_btShapeHull::numVertices () const
{
	return m_vertices.size ();
}

int
D_btShapeHull::numIndices () const
{
	return static_cast<int>(m_numIndices);
}

