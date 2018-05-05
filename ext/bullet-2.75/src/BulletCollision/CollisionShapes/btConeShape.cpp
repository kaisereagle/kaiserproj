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

#include "btConeShape.h"



D_btConeShape::D_btConeShape (D_btScalar radius,D_btScalar height): D_btConvexInternalShape (),
m_radius (radius),
m_height(height)
{
	m_shapeType = D_CONE_SHAPE_PROXYTYPE;
	setConeUpIndex(1);
	D_btVector3 halfExtents;
	m_sinAngle = (m_radius / D_btSqrt(m_radius * m_radius + m_height * m_height));
}

D_btConeShapeZ::D_btConeShapeZ (D_btScalar radius,D_btScalar height):
D_btConeShape(radius,height)
{
	setConeUpIndex(2);
}

D_btConeShapeX::D_btConeShapeX (D_btScalar radius,D_btScalar height):
D_btConeShape(radius,height)
{
	setConeUpIndex(0);
}

///choose upAxis index
void	D_btConeShape::setConeUpIndex(int upIndex)
{
	switch (upIndex)
	{
	case 0:
			m_coneIndices[0] = 1;
			m_coneIndices[1] = 0;
			m_coneIndices[2] = 2;
		break;
	case 1:
			m_coneIndices[0] = 0;
			m_coneIndices[1] = 1;
			m_coneIndices[2] = 2;
		break;
	case 2:
			m_coneIndices[0] = 0;
			m_coneIndices[1] = 2;
			m_coneIndices[2] = 1;
		break;
	default:
		D_btAssert(0);
	};
}

D_btVector3 D_btConeShape::coneLocalSupport(const D_btVector3& v) const
{
	
	D_btScalar halfHeight = m_height * D_btScalar(0.5);

 if (v[m_coneIndices[1]] > v.length() * m_sinAngle)
 {
	D_btVector3 tmp;

	tmp[m_coneIndices[0]] = D_btScalar(0.);
	tmp[m_coneIndices[1]] = halfHeight;
	tmp[m_coneIndices[2]] = D_btScalar(0.);
	return tmp;
 }
  else {
    D_btScalar s = D_btSqrt(v[m_coneIndices[0]] * v[m_coneIndices[0]] + v[m_coneIndices[2]] * v[m_coneIndices[2]]);
    if (s > D_SIMD_EPSILON) {
      D_btScalar d = m_radius / s;
	  D_btVector3 tmp;
	  tmp[m_coneIndices[0]] = v[m_coneIndices[0]] * d;
	  tmp[m_coneIndices[1]] = -halfHeight;
	  tmp[m_coneIndices[2]] = v[m_coneIndices[2]] * d;
	  return tmp;
    }
    else  {
		D_btVector3 tmp;
		tmp[m_coneIndices[0]] = D_btScalar(0.);
		tmp[m_coneIndices[1]] = -halfHeight;
		tmp[m_coneIndices[2]] = D_btScalar(0.);
		return tmp;
	}
  }

}

D_btVector3	D_btConeShape::localGetSupportingVertexWithoutMargin(const D_btVector3& vec) const
{
		return coneLocalSupport(vec);
}

void	D_btConeShape::batchedUnitVectorGetSupportingVertexWithoutMargin(const D_btVector3* vectors,D_btVector3* supportVerticesOut,int numVectors) const
{
	for (int i=0;i<numVectors;i++)
	{
		const D_btVector3& vec = vectors[i];
		supportVerticesOut[i] = coneLocalSupport(vec);
	}
}


D_btVector3	D_btConeShape::localGetSupportingVertex(const D_btVector3& vec)  const
{
	D_btVector3 supVertex = coneLocalSupport(vec);
	if ( getMargin()!=D_btScalar(0.) )
	{
		D_btVector3 vecnorm = vec;
		if (vecnorm .length2() < (D_SIMD_EPSILON*D_SIMD_EPSILON))
		{
			vecnorm.setValue(D_btScalar(-1.),D_btScalar(-1.),D_btScalar(-1.));
		} 
		vecnorm.normalize();
		supVertex+= getMargin() * vecnorm;
	}
	return supVertex;
}


