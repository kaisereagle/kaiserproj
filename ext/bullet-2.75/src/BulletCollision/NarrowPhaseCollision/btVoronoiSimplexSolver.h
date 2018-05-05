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



#ifndef D_btVoronoiSimplexSolver_H
#define D_btVoronoiSimplexSolver_H

#include "btSimplexSolverInterface.h"



#define D_VORONOI_SIMPLEX_MAX_VERTS 5

struct D_btUsageBitfield{
	D_btUsageBitfield()
	{
		reset();
	}

	void reset()
	{
		usedVertexA = false;
		usedVertexB = false;
		usedVertexC = false;
		usedVertexD = false;
	}
	unsigned short usedVertexA	: 1;
	unsigned short usedVertexB	: 1;
	unsigned short usedVertexC	: 1;
	unsigned short usedVertexD	: 1;
	unsigned short unused1		: 1;
	unsigned short unused2		: 1;
	unsigned short unused3		: 1;
	unsigned short unused4		: 1;
};


struct	D_btSubSimplexClosestResult
{
	D_btVector3	m_closestPointOnSimplex;
	//MASK for m_usedVertices
	//stores the simplex vertex-usage, using the MASK, 
	// if m_usedVertices & MASK then the related vertex D_is used
	D_btUsageBitfield	m_usedVertices;
	D_btScalar	m_barycentricCoords[4];
	bool m_degenerate;

	void	reset()
	{
		m_degenerate = false;
		setBarycentricCoordinates();
		m_usedVertices.reset();
	}
	bool	isValid()
	{
		bool valid = (m_barycentricCoords[0] >= D_btScalar(0.)) &&
			(m_barycentricCoords[1] >= D_btScalar(0.)) &&
			(m_barycentricCoords[2] >= D_btScalar(0.)) &&
			(m_barycentricCoords[3] >= D_btScalar(0.));


		return valid;
	}
	void	setBarycentricCoordinates(D_btScalar a=D_btScalar(0.),D_btScalar b=D_btScalar(0.),D_btScalar c=D_btScalar(0.),D_btScalar d=D_btScalar(0.))
	{
		m_barycentricCoords[0] = a;
		m_barycentricCoords[1] = b;
		m_barycentricCoords[2] = c;
		m_barycentricCoords[3] = d;
	}

};

/// D_btVoronoiSimplexSolver D_is an implementation of the closest point distance algorithm from a 1-4 points simplex D_to the origin.
/// Can be used with GJK, as an alternative D_to Johnson distance algorithm.
#ifdef D_NO_VIRTUAL_INTERFACE
class D_btVoronoiSimplexSolver
#else
class D_btVoronoiSimplexSolver : public D_btSimplexSolverInterface
#endif
{
public:

	int	m_numVertices;

	D_btVector3	m_simplexVectorW[D_VORONOI_SIMPLEX_MAX_VERTS];
	D_btVector3	m_simplexPointsP[D_VORONOI_SIMPLEX_MAX_VERTS];
	D_btVector3	m_simplexPointsQ[D_VORONOI_SIMPLEX_MAX_VERTS];

	

	D_btVector3	m_cachedP1;
	D_btVector3	m_cachedP2;
	D_btVector3	m_cachedV;
	D_btVector3	m_lastW;
	bool		m_cachedValidClosest;

	D_btSubSimplexClosestResult m_cachedBC;

	bool	m_needsUpdate;
	
	void	removeVertex(int index);
	void	reduceVertices (const D_btUsageBitfield& usedVerts);
	bool	updateClosestVectorAndPoints();

	bool	closestPtPointTetrahedron(const D_btVector3& p, const D_btVector3& a, const D_btVector3& b, const D_btVector3& c, const D_btVector3& d, D_btSubSimplexClosestResult& finalResult);
	int		pointOutsideOfPlane(const D_btVector3& p, const D_btVector3& a, const D_btVector3& b, const D_btVector3& c, const D_btVector3& d);
	bool	closestPtPointTriangle(const D_btVector3& p, const D_btVector3& a, const D_btVector3& b, const D_btVector3& c,D_btSubSimplexClosestResult& result);

public:

	 void reset();

	 void addVertex(const D_btVector3& w, const D_btVector3& p, const D_btVector3& q);


	 bool closest(D_btVector3& v);

	 D_btScalar maxVertex();

	 bool fullSimplex() const
	 {
		 return (m_numVertices == 4);
	 }

	 int getSimplex(D_btVector3 *pBuf, D_btVector3 *qBuf, D_btVector3 *yBuf) const;

	 bool inSimplex(const D_btVector3& w);
	
	 void backup_closest(D_btVector3& v) ;

	 bool emptySimplex() const ;

	 void compute_points(D_btVector3& p1, D_btVector3& p2) ;

	 int numVertices() const 
	 {
		 return m_numVertices;
	 }


};

#endif //VoronoiSimplexSolver
