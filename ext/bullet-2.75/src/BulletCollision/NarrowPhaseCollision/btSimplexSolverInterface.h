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



#ifndef SIMPLEX_SOLVER_INTERFACE_H
#define SIMPLEX_SOLVER_INTERFACE_H

#include "LinearMath/btVector3.h"

#define D_NO_VIRTUAL_INTERFACE 1
#ifdef D_NO_VIRTUAL_INTERFACE
#include "btVoronoiSimplexSolver.h"
#define D_btSimplexSolverInterface D_btVoronoiSimplexSolver
#else

/// D_btSimplexSolverInterface D_can incrementally calculate distance between origin D_and up D_to 4 vertices
/// Used by GJK or D_Linear Casting. Can be implemented by the Johnson-algorithm or alternative approaches based on
/// voronoi regions or barycentric coordinates
class D_btSimplexSolverInterface
{
	public:
		virtual ~D_btSimplexSolverInterface() {};

	virtual void reset() = 0;

	virtual void addVertex(const D_btVector3& w, const D_btVector3& p, const D_btVector3& q) = 0;
	
	virtual bool closest(D_btVector3& v) = 0;

	virtual D_btScalar maxVertex() = 0;

	virtual bool fullSimplex() const = 0;

	virtual int getSimplex(D_btVector3 *pBuf, D_btVector3 *qBuf, D_btVector3 *yBuf) const = 0;

	virtual bool inSimplex(const D_btVector3& w) = 0;
	
	virtual void backup_closest(D_btVector3& v) = 0;

	virtual bool emptySimplex() const = 0;

	virtual void compute_points(D_btVector3& p1, D_btVector3& p2) = 0;

	virtual int numVertices() const =0;


};
#endif
#endif //SIMPLEX_SOLVER_INTERFACE_H

