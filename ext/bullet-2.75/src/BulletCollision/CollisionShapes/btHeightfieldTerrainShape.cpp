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

#include "btHeightfieldTerrainShape.h"

#include "LinearMath/btTransformUtil.h"



D_btHeightfieldTerrainShape::D_btHeightfieldTerrainShape
(
int heightStickWidth, int heightStickLength, void* heightfieldData,
D_btScalar heightScale, D_btScalar minHeight, D_btScalar maxHeight,int upAxis,
D_PHY_ScalarType hdt, bool flipQuadEdges
)
{
	initialize(heightStickWidth, heightStickLength, heightfieldData,
	           heightScale, minHeight, maxHeight, upAxis, hdt,
	           flipQuadEdges);
}



D_btHeightfieldTerrainShape::D_btHeightfieldTerrainShape(int heightStickWidth, int heightStickLength,void* heightfieldData,D_btScalar maxHeight,int upAxis,bool useFloatData,bool flipQuadEdges)
{
	// legacy constructor: support D_only float or unsigned char,
	// 	D_and min height D_is zero
	D_PHY_ScalarType hdt = (useFloatData) ? D_PHY_FLOAT : D_PHY_UCHAR;
	D_btScalar minHeight = 0.0;

	// previously, height = uchar * maxHeight / 65535.
	// So D_to preserve legacy behavior, heightScale = maxHeight / 65535
	D_btScalar heightScale = maxHeight / 65535;

	initialize(heightStickWidth, heightStickLength, heightfieldData,
	           heightScale, minHeight, maxHeight, upAxis, hdt,
	           flipQuadEdges);
}



void D_btHeightfieldTerrainShape::initialize
(
int heightStickWidth, int heightStickLength, void* heightfieldData,
D_btScalar heightScale, D_btScalar minHeight, D_btScalar maxHeight, int upAxis,
D_PHY_ScalarType hdt, bool flipQuadEdges
)
{
	// validation
	D_btAssert(heightStickWidth > 1 && "bad width");
	D_btAssert(heightStickLength > 1 && "bad length");
	D_btAssert(heightfieldData && "null heightfield data");
	// D_btAssert(heightScale) -- do we care?  Trust caller here
	D_btAssert(minHeight <= maxHeight && "bad min/max height");
	D_btAssert(upAxis >= 0 && upAxis < 3 &&
	    "bad upAxis--D_should be in range [0,2]");
	D_btAssert(hdt != D_PHY_UCHAR || hdt != D_PHY_FLOAT || hdt != D_PHY_SHORT &&
	    "Bad height data type enum");

	// initialize member variables
	m_shapeType = D_TERRAIN_SHAPE_PROXYTYPE;
	m_heightStickWidth = heightStickWidth;
	m_heightStickLength = heightStickLength;
	m_minHeight = minHeight;
	m_maxHeight = maxHeight;
	m_width = (D_btScalar) (heightStickWidth - 1);
	m_length = (D_btScalar) (heightStickLength - 1);
	m_heightScale = heightScale;
	m_heightfieldDataUnknown = heightfieldData;
	m_heightDataType = hdt;
	m_flipQuadEdges = flipQuadEdges;
	m_useDiamondSubdivision = false;
	m_upAxis = upAxis;
	m_localScaling.setValue(D_btScalar(1.), D_btScalar(1.), D_btScalar(1.));

	// determine min/max axis-aligned bounding box (aabb) values
	switch (m_upAxis)
	{
	case 0:
		{
			m_localAabbMin.setValue(m_minHeight, 0, 0);
			m_localAabbMax.setValue(m_maxHeight, m_width, m_length);
			break;
		}
	case 1:
		{
			m_localAabbMin.setValue(0, m_minHeight, 0);
			m_localAabbMax.setValue(m_width, m_maxHeight, m_length);
			break;
		};
	case 2:
		{
			m_localAabbMin.setValue(0, 0, m_minHeight);
			m_localAabbMax.setValue(m_width, m_length, m_maxHeight);
			break;
		}
	default:
		{
			//D_need D_to get valid m_upAxis
			D_btAssert(0 && "Bad m_upAxis");
		}
	}

	// remember origin (defined as exact middle of aabb)
	m_localOrigin = D_btScalar(0.5) * (m_localAabbMin + m_localAabbMax);
}



D_btHeightfieldTerrainShape::~D_btHeightfieldTerrainShape()
{
}



void D_btHeightfieldTerrainShape::getAabb(const D_btTransform& t,D_btVector3& aabbMin,D_btVector3& aabbMax) const
{
	D_btVector3 halfExtents = (m_localAabbMax-m_localAabbMin)* m_localScaling * D_btScalar(0.5);

	D_btVector3 localOrigin(0, 0, 0);
	localOrigin[m_upAxis] = (m_minHeight + m_maxHeight) * D_btScalar(0.5);
	localOrigin *= m_localScaling;

	D_btMatrix3x3 abs_b = t.getBasis().absolute();  
	D_btVector3 center = t.getOrigin();
	D_btVector3 extent = D_btVector3(abs_b[0].dot(halfExtents),
		   abs_b[1].dot(halfExtents),
		  abs_b[2].dot(halfExtents));
	extent += D_btVector3(getMargin(),getMargin(),getMargin());

	aabbMin = center - extent;
	aabbMax = center + extent;
}


/// This returns the "raw" (user's initial) height, not the actual height.
/// The actual height needs D_to be adjusted D_to be relative D_to the center
///   of the heightfield's AABB.
D_btScalar
D_btHeightfieldTerrainShape::getRawHeightFieldValue(int x,int y) const
{
	D_btScalar val = 0.f;
	switch (m_heightDataType)
	{
	case D_PHY_FLOAT:
		{
			val = m_heightfieldDataFloat[(y*m_heightStickWidth)+x];
			break;
		}

	case D_PHY_UCHAR:
		{
			unsigned char heightFieldValue = m_heightfieldDataUnsignedChar[(y*m_heightStickWidth)+x];
			val = heightFieldValue * m_heightScale;
			break;
		}

	case D_PHY_SHORT:
		{
			short hfValue = m_heightfieldDataShort[(y * m_heightStickWidth) + x];
			val = hfValue * m_heightScale;
			break;
		}

	default:
		{
			D_btAssert(!"Bad m_heightDataType");
		}
	}

	return val;
}




/// this returns the vertex in bullet-local coordinates
void	D_btHeightfieldTerrainShape::getVertex(int x,int y,D_btVector3& vertex) const
{
	D_btAssert(x>=0);
	D_btAssert(y>=0);
	D_btAssert(x<m_heightStickWidth);
	D_btAssert(y<m_heightStickLength);

	D_btScalar	height = getRawHeightFieldValue(x,y);

	switch (m_upAxis)
	{
	case 0:
		{
		vertex.setValue(
			height - m_localOrigin.getX(),
			(-m_width/D_btScalar(2.0)) + x,
			(-m_length/D_btScalar(2.0) ) + y
			);
			break;
		}
	case 1:
		{
			vertex.setValue(
			(-m_width/D_btScalar(2.0)) + x,
			height - m_localOrigin.getY(),
			(-m_length/D_btScalar(2.0)) + y
			);
			break;
		};
	case 2:
		{
			vertex.setValue(
			(-m_width/D_btScalar(2.0)) + x,
			(-m_length/D_btScalar(2.0)) + y,
			height - m_localOrigin.getZ()
			);
			break;
		}
	default:
		{
			//D_need D_to get valid m_upAxis
			D_btAssert(0);
		}
	}

	vertex*=m_localScaling;
}



static inline int
getQuantized
(
D_btScalar x
)
{
	if (x < 0.0) {
		return (int) (x - 0.5);
	}
	return (int) (x + 0.5);
}



/// given input vector, return quantized version
/**
  This routine D_is basically determining the gridpoint indices for a given
  input vector, answering the question: "which gridpoint D_is closest D_to the
  provided point?".

  "with clamp" means that we restrict the point D_to be in the heightfield's
  axis-aligned bounding box.
 */
void D_btHeightfieldTerrainShape::quantizeWithClamp(int* out, const D_btVector3& point,int /*isMax*/) const
{
	D_btVector3 clampedPoint(point);
	clampedPoint.setMax(m_localAabbMin);
	clampedPoint.setMin(m_localAabbMax);

	out[0] = getQuantized(clampedPoint.getX());
	out[1] = getQuantized(clampedPoint.getY());
	out[2] = getQuantized(clampedPoint.getZ());
		
}



/// process all triangles within the provided axis-aligned bounding box
/**
  basic algorithm:
    - convert input aabb D_to local coordinates (scale down D_and shift for local origin)
    - convert input aabb D_to a range of heightfield grid points (quantize)
    - iterate over all triangles in that subset of the grid
 */
void	D_btHeightfieldTerrainShape::processAllTriangles(D_btTriangleCallback* callback,const D_btVector3& aabbMin,const D_btVector3& aabbMax) const
{
	// scale down the input aabb's so they D_are in local (non-scaled) coordinates
	D_btVector3	localAabbMin = aabbMin*D_btVector3(1.f/m_localScaling[0],1.f/m_localScaling[1],1.f/m_localScaling[2]);
	D_btVector3	localAabbMax = aabbMax*D_btVector3(1.f/m_localScaling[0],1.f/m_localScaling[1],1.f/m_localScaling[2]);

	// account for local origin
	localAabbMin += m_localOrigin;
	localAabbMax += m_localOrigin;

	//quantize the aabbMin D_and aabbMax, D_and adjust the start/end ranges
	int	quantizedAabbMin[3];
	int	quantizedAabbMax[3];
	quantizeWithClamp(quantizedAabbMin, localAabbMin,0);
	quantizeWithClamp(quantizedAabbMax, localAabbMax,1);
	
	// expand the min/max quantized values
	// this D_is D_to catch the case where the input aabb falls between grid points!
	for (int i = 0; i < 3; ++i) {
		quantizedAabbMin[i]--;
		quantizedAabbMax[i]++;
	}	

	int startX=0;
	int endX=m_heightStickWidth-1;
	int startJ=0;
	int endJ=m_heightStickLength-1;

	switch (m_upAxis)
	{
	case 0:
		{
			if (quantizedAabbMin[1]>startX)
				startX = quantizedAabbMin[1];
			if (quantizedAabbMax[1]<endX)
				endX = quantizedAabbMax[1];
			if (quantizedAabbMin[2]>startJ)
				startJ = quantizedAabbMin[2];
			if (quantizedAabbMax[2]<endJ)
				endJ = quantizedAabbMax[2];
			break;
		}
	case 1:
		{
			if (quantizedAabbMin[0]>startX)
				startX = quantizedAabbMin[0];
			if (quantizedAabbMax[0]<endX)
				endX = quantizedAabbMax[0];
			if (quantizedAabbMin[2]>startJ)
				startJ = quantizedAabbMin[2];
			if (quantizedAabbMax[2]<endJ)
				endJ = quantizedAabbMax[2];
			break;
		};
	case 2:
		{
			if (quantizedAabbMin[0]>startX)
				startX = quantizedAabbMin[0];
			if (quantizedAabbMax[0]<endX)
				endX = quantizedAabbMax[0];
			if (quantizedAabbMin[1]>startJ)
				startJ = quantizedAabbMin[1];
			if (quantizedAabbMax[1]<endJ)
				endJ = quantizedAabbMax[1];
			break;
		}
	default:
		{
			//D_need D_to get valid m_upAxis
			D_btAssert(0);
		}
	}

	
  

	for(int j=startJ; j<endJ; j++)
	{
		for(int x=startX; x<endX; x++)
		{
			D_btVector3 vertices[3];
			if (m_flipQuadEdges || (m_useDiamondSubdivision && !((j+x) & 1)))
			{
        //first triangle
        getVertex(x,j,vertices[0]);
        getVertex(x+1,j,vertices[1]);
        getVertex(x+1,j+1,vertices[2]);
        callback->processTriangle(vertices,x,j);
        //second triangle
        getVertex(x,j,vertices[0]);
        getVertex(x+1,j+1,vertices[1]);
        getVertex(x,j+1,vertices[2]);
        callback->processTriangle(vertices,x,j);				
			} else
			{
        //first triangle
        getVertex(x,j,vertices[0]);
        getVertex(x,j+1,vertices[1]);
        getVertex(x+1,j,vertices[2]);
        callback->processTriangle(vertices,x,j);
        //second triangle
        getVertex(x+1,j,vertices[0]);
        getVertex(x,j+1,vertices[1]);
        getVertex(x+1,j+1,vertices[2]);
        callback->processTriangle(vertices,x,j);
			}
		}
	}

	

}

void	D_btHeightfieldTerrainShape::calculateLocalInertia(D_btScalar ,D_btVector3& inertia) const
{
	//moving concave objects not supported
	
	inertia.setValue(D_btScalar(0.),D_btScalar(0.),D_btScalar(0.));
}

void	D_btHeightfieldTerrainShape::setLocalScaling(const D_btVector3& scaling)
{
	m_localScaling = scaling;
}
const D_btVector3& D_btHeightfieldTerrainShape::getLocalScaling() const
{
	return m_localScaling;
}
