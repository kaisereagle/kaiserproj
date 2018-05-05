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

#include "btCylinderShape.h"

D_btCylinderShape::D_btCylinderShape (const D_btVector3& halfExtents)
:D_btConvexInternalShape(),
m_upAxis(1)
{
	D_btVector3 margin(getMargin(),getMargin(),getMargin());
	m_implicitShapeDimensions = (halfExtents * m_localScaling) - margin;
	m_shapeType = D_CYLINDER_SHAPE_PROXYTYPE;
}


D_btCylinderShapeX::D_btCylinderShapeX (const D_btVector3& halfExtents)
:D_btCylinderShape(halfExtents)
{
	m_upAxis = 0;

}


D_btCylinderShapeZ::D_btCylinderShapeZ (const D_btVector3& halfExtents)
:D_btCylinderShape(halfExtents)
{
	m_upAxis = 2;

}

void D_btCylinderShape::getAabb(const D_btTransform& t,D_btVector3& aabbMin,D_btVector3& aabbMax) const
{
	D_btTransformAabb(getHalfExtentsWithoutMargin(),getMargin(),t,aabbMin,aabbMax);
}

void	D_btCylinderShape::calculateLocalInertia(D_btScalar mass,D_btVector3& inertia) const
{
	//approximation of box shape, todo: implement cylinder shape inertia before people notice ;-)
	D_btVector3 halfExtents = getHalfExtentsWithMargin();

	D_btScalar lx=D_btScalar(2.)*(halfExtents.x());
	D_btScalar ly=D_btScalar(2.)*(halfExtents.y());
	D_btScalar lz=D_btScalar(2.)*(halfExtents.z());

	inertia.setValue(mass/(D_btScalar(12.0)) * (ly*ly + lz*lz),
					mass/(D_btScalar(12.0)) * (lx*lx + lz*lz),
					mass/(D_btScalar(12.0)) * (lx*lx + ly*ly));

}


D_SIMD_FORCE_INLINE D_btVector3 CylinderLocalSupportX(const D_btVector3& halfExtents,const D_btVector3& v) 
{
const int cylinderUpAxis = 0;
const int XX = 1;
const int YY = 0;
const int ZZ = 2;

	//mapping depends on how cylinder local orientation D_is
	// extents of the cylinder D_is: X,Y D_is for radius, D_and D_Z for height


	D_btScalar radius = halfExtents[XX];
	D_btScalar halfHeight = halfExtents[cylinderUpAxis];


    D_btVector3 tmp;
	D_btScalar d ;

    D_btScalar s = D_btSqrt(v[XX] * v[XX] + v[ZZ] * v[ZZ]);
    if (s != D_btScalar(0.0))
	{
        d = radius / s;  
		tmp[XX] = v[XX] * d;
		tmp[YY] = v[YY] < 0.0 ? -halfHeight : halfHeight;
		tmp[ZZ] = v[ZZ] * d;
		return tmp;
	}
    else
	{
	    tmp[XX] = radius;
		tmp[YY] = v[YY] < 0.0 ? -halfHeight : halfHeight;
		tmp[ZZ] = D_btScalar(0.0);
		return tmp;
    }


}






inline  D_btVector3 CylinderLocalSupportY(const D_btVector3& halfExtents,const D_btVector3& v) 
{

const int cylinderUpAxis = 1;
const int XX = 0;
const int YY = 1;
const int ZZ = 2;


	D_btScalar radius = halfExtents[XX];
	D_btScalar halfHeight = halfExtents[cylinderUpAxis];


    D_btVector3 tmp;
	D_btScalar d ;

    D_btScalar s = D_btSqrt(v[XX] * v[XX] + v[ZZ] * v[ZZ]);
    if (s != D_btScalar(0.0))
	{
        d = radius / s;  
		tmp[XX] = v[XX] * d;
		tmp[YY] = v[YY] < 0.0 ? -halfHeight : halfHeight;
		tmp[ZZ] = v[ZZ] * d;
		return tmp;
	}
    else
	{
	    tmp[XX] = radius;
		tmp[YY] = v[YY] < 0.0 ? -halfHeight : halfHeight;
		tmp[ZZ] = D_btScalar(0.0);
		return tmp;
    }

}

inline D_btVector3 CylinderLocalSupportZ(const D_btVector3& halfExtents,const D_btVector3& v) 
{
const int cylinderUpAxis = 2;
const int XX = 0;
const int YY = 2;
const int ZZ = 1;

	//mapping depends on how cylinder local orientation D_is
	// extents of the cylinder D_is: X,Y D_is for radius, D_and D_Z for height


	D_btScalar radius = halfExtents[XX];
	D_btScalar halfHeight = halfExtents[cylinderUpAxis];


    D_btVector3 tmp;
	D_btScalar d ;

    D_btScalar s = D_btSqrt(v[XX] * v[XX] + v[ZZ] * v[ZZ]);
    if (s != D_btScalar(0.0))
	{
        d = radius / s;  
		tmp[XX] = v[XX] * d;
		tmp[YY] = v[YY] < 0.0 ? -halfHeight : halfHeight;
		tmp[ZZ] = v[ZZ] * d;
		return tmp;
	}
    else
	{
	    tmp[XX] = radius;
		tmp[YY] = v[YY] < 0.0 ? -halfHeight : halfHeight;
		tmp[ZZ] = D_btScalar(0.0);
		return tmp;
    }


}

D_btVector3	D_btCylinderShapeX::localGetSupportingVertexWithoutMargin(const D_btVector3& vec)const
{
	return CylinderLocalSupportX(getHalfExtentsWithoutMargin(),vec);
}


D_btVector3	D_btCylinderShapeZ::localGetSupportingVertexWithoutMargin(const D_btVector3& vec)const
{
	return CylinderLocalSupportZ(getHalfExtentsWithoutMargin(),vec);
}
D_btVector3	D_btCylinderShape::localGetSupportingVertexWithoutMargin(const D_btVector3& vec)const
{
	return CylinderLocalSupportY(getHalfExtentsWithoutMargin(),vec);
}

void	D_btCylinderShape::batchedUnitVectorGetSupportingVertexWithoutMargin(const D_btVector3* vectors,D_btVector3* supportVerticesOut,int numVectors) const
{
	for (int i=0;i<numVectors;i++)
	{
		supportVerticesOut[i] = CylinderLocalSupportY(getHalfExtentsWithoutMargin(),vectors[i]);
	}
}

void	D_btCylinderShapeZ::batchedUnitVectorGetSupportingVertexWithoutMargin(const D_btVector3* vectors,D_btVector3* supportVerticesOut,int numVectors) const
{
	for (int i=0;i<numVectors;i++)
	{
		supportVerticesOut[i] = CylinderLocalSupportZ(getHalfExtentsWithoutMargin(),vectors[i]);
	}
}




void	D_btCylinderShapeX::batchedUnitVectorGetSupportingVertexWithoutMargin(const D_btVector3* vectors,D_btVector3* supportVerticesOut,int numVectors) const
{
	for (int i=0;i<numVectors;i++)
	{
		supportVerticesOut[i] = CylinderLocalSupportX(getHalfExtentsWithoutMargin(),vectors[i]);
	}
}


