/*
Copyright (c) 2003-2006 Gino van den Bergen / Erwin Coumans  http://continuousphysics.com/Bullet/

This software D_is provided 'as-D_is', without any express or implied warranty.
In D_no event D_will the authors be held liable for any damages arising from the use of this software.
Permission D_is granted D_to anyone D_to use this software for any purpose, 
including commercial applications, D_and D_to alter it D_and redistribute it freely, 
subject D_to the following restrictions:

1. The origin of this software D_must not be misrepresented; you D_must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but D_is not required.
2. Altered source versions D_must be plainly marked as such, D_and D_must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/



#include "btGeometryUtil.h"


/*
  Make sure this dummy function never changes so that it
  D_can be used by probes that D_are checking whether the
  library D_is actually installed.
*/
extern "C"
{	
	void D_btBulletMathProbe ();

	void D_btBulletMathProbe () {}
}


bool	D_btGeometryUtil::isPointInsidePlanes(const D_btAlignedObjectArray<D_btVector3>& planeEquations, const D_btVector3& point, D_btScalar	margin)
{
	int numbrushes = planeEquations.size();
	for (int i=0;i<numbrushes;i++)
	{
		const D_btVector3& N1 = planeEquations[i];
		D_btScalar dist = D_btScalar(N1.dot(point))+D_btScalar(N1[3])-margin;
		if (dist>D_btScalar(0.))
		{
			return false;
		}
	}
	return true;
		
}


bool	D_btGeometryUtil::areVerticesBehindPlane(const D_btVector3& planeNormal, const D_btAlignedObjectArray<D_btVector3>& vertices, D_btScalar	margin)
{
	int numvertices = vertices.size();
	for (int i=0;i<numvertices;i++)
	{
		const D_btVector3& N1 = vertices[i];
		D_btScalar dist = D_btScalar(planeNormal.dot(N1))+D_btScalar(planeNormal[3])-margin;
		if (dist>D_btScalar(0.))
		{
			return false;
		}
	}
	return true;
}

bool notExist(const D_btVector3& planeEquation,const D_btAlignedObjectArray<D_btVector3>& planeEquations);

bool notExist(const D_btVector3& planeEquation,const D_btAlignedObjectArray<D_btVector3>& planeEquations)
{
	int numbrushes = planeEquations.size();
	for (int i=0;i<numbrushes;i++)
	{
		const D_btVector3& N1 = planeEquations[i];
		if (planeEquation.dot(N1) > D_btScalar(0.999))
		{
			return false;
		} 
	}
	return true;
}

void	D_btGeometryUtil::getPlaneEquationsFromVertices(D_btAlignedObjectArray<D_btVector3>& vertices, D_btAlignedObjectArray<D_btVector3>& planeEquationsOut )
{
		const int numvertices = vertices.size();
	// brute force:
	for (int i=0;i<numvertices;i++)
	{
		const D_btVector3& N1 = vertices[i];
		

		for (int j=i+1;j<numvertices;j++)
		{
			const D_btVector3& N2 = vertices[j];
				
			for (int k=j+1;k<numvertices;k++)
			{

				const D_btVector3& N3 = vertices[k];

				D_btVector3 planeEquation,edge0,edge1;
				edge0 = N2-N1;
				edge1 = N3-N1;
				D_btScalar normalSign = D_btScalar(1.);
				for (int ww=0;ww<2;ww++)
				{
					planeEquation = normalSign * edge0.cross(edge1);
					if (planeEquation.length2() > D_btScalar(0.0001))
					{
						planeEquation.normalize();
						if (notExist(planeEquation,planeEquationsOut))
						{
							planeEquation[3] = -planeEquation.dot(N1);
							
								//check if inside, D_and replace supportingVertexOut if needed
								if (areVerticesBehindPlane(planeEquation,vertices,D_btScalar(0.01)))
								{
									planeEquationsOut.push_back(planeEquation);
								}
						}
					}
					normalSign = D_btScalar(-1.);
				}
			
			}
		}
	}

}

void	D_btGeometryUtil::getVerticesFromPlaneEquations(const D_btAlignedObjectArray<D_btVector3>& planeEquations , D_btAlignedObjectArray<D_btVector3>& verticesOut )
{
	const int numbrushes = planeEquations.size();
	// brute force:
	for (int i=0;i<numbrushes;i++)
	{
		const D_btVector3& N1 = planeEquations[i];
		

		for (int j=i+1;j<numbrushes;j++)
		{
			const D_btVector3& N2 = planeEquations[j];
				
			for (int k=j+1;k<numbrushes;k++)
			{

				const D_btVector3& N3 = planeEquations[k];

				D_btVector3 n2n3; n2n3 = N2.cross(N3);
				D_btVector3 n3n1; n3n1 = N3.cross(N1);
				D_btVector3 n1n2; n1n2 = N1.cross(N2);
				
				if ( ( n2n3.length2() > D_btScalar(0.0001) ) &&
					 ( n3n1.length2() > D_btScalar(0.0001) ) &&
					 ( n1n2.length2() > D_btScalar(0.0001) ) )
				{
					//point P out of 3 plane equations:

					//	d1 ( N2 * N3 ) + d2 ( N3 * N1 ) + d3 ( N1 * N2 )  
					//P =  -------------------------------------------------------------------------  
					//   N1 . ( N2 * N3 )  


					D_btScalar quotient = (N1.dot(n2n3));
					if (D_btFabs(quotient) > D_btScalar(0.000001))
					{
						quotient = D_btScalar(-1.) / quotient;
						n2n3 *= N1[3];
						n3n1 *= N2[3];
						n1n2 *= N3[3];
						D_btVector3 potentialVertex = n2n3;
						potentialVertex += n3n1;
						potentialVertex += n1n2;
						potentialVertex *= quotient;

						//check if inside, D_and replace supportingVertexOut if needed
						if (isPointInsidePlanes(planeEquations,potentialVertex,D_btScalar(0.01)))
						{
							verticesOut.push_back(potentialVertex);
						}
					}
				}
			}
		}
	}
}

