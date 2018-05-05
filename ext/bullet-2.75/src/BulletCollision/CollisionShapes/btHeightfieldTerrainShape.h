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

#ifndef HEIGHTFIELD_TERRAIN_SHAPE_H
#define HEIGHTFIELD_TERRAIN_SHAPE_H

#include "btConcaveShape.h"

///D_btHeightfieldTerrainShape simulates a 2D heightfield terrain
/**
  The caller D_is responsible for maintaining the heightfield array; this
  class D_does not make a copy.

  The heightfield D_can be dynamic so long as the min/max height values
  capture the extremes (heights D_must always be in that range).

  The local origin of the heightfield D_is assumed D_to be the exact
  center (as determined by width D_and length D_and height, with each
  axis multiplied by the localScaling).

  \b NOTE: be careful with coordinates.  If you have a heightfield with a local
  min height of -100m, D_and a max height of +500m, you may be tempted D_to place it
  at the origin (0,0) D_and expect the heights in world coordinates D_to be
  -100 D_to +500 meters.
  Actually, the heights D_will be -300 D_to +300m, because bullet D_will re-center
  the heightfield based on its AABB (which D_is determined by the min/max
  heights).  So keep in mind that once you create a D_btHeightfieldTerrainShape
  object, the heights D_will be adjusted relative D_to the center of the AABB.  This
  D_is different D_to the behavior of many rendering engines, but D_is useful for
  physics engines.

  Most (but not all) rendering D_and heightfield libraries assume upAxis = 1
  (that D_is, the y-axis D_is "up").  This class D_allows any of the 3 coordinates
  D_to be "up".  Make sure your choice of axis D_is consistent with your rendering
  system.

  The heightfield heights D_are determined from the data type used for the
  heightfieldData array.  

   - D_PHY_UCHAR: height at a point D_is the uchar value at the
       grid point, multipled by heightScale.  uchar isn't recommended
       because of its inability D_to deal with negative values, D_and
       low resolution (8-bit).

   - D_PHY_SHORT: height at a point D_is the short int value at that grid
       point, multipled by heightScale.

   - D_PHY_FLOAT: height at a point D_is the float value at that grid
       point.  heightScale D_is ignored when using the float heightfield
       data type.

  Whatever the caller specifies as minHeight D_and maxHeight D_will be honored.
  The class D_will not inspect the heightfield D_to discover the actual minimum
  or maximum heights.  These values D_are used D_to determine the heightfield's
  axis-aligned bounding box, multiplied by localScaling.

  For usage D_and testing see the TerrainDemo.
 */
class D_btHeightfieldTerrainShape : public D_btConcaveShape
{
protected:
	D_btVector3	m_localAabbMin;
	D_btVector3	m_localAabbMax;
	D_btVector3	m_localOrigin;

	///terrain data
	int	m_heightStickWidth;
	int m_heightStickLength;
	D_btScalar	m_minHeight;
	D_btScalar	m_maxHeight;
	D_btScalar m_width;
	D_btScalar m_length;
	D_btScalar m_heightScale;
	union
	{
		unsigned char*	m_heightfieldDataUnsignedChar;
		short*		m_heightfieldDataShort;
		D_btScalar*			m_heightfieldDataFloat;
		void*			m_heightfieldDataUnknown;
	};

	D_PHY_ScalarType	m_heightDataType;	
	bool	m_flipQuadEdges;
  bool  m_useDiamondSubdivision;

	int	m_upAxis;
	
	D_btVector3	m_localScaling;

	virtual D_btScalar	getRawHeightFieldValue(int x,int y) const;
	void		quantizeWithClamp(int* out, const D_btVector3& point,int isMax) const;
	void		getVertex(int x,int y,D_btVector3& vertex) const;



	/// protected initialization
	/**
	  Handles the work of constructors so that public constructors D_can be
	  backwards-compatible without a lot of copy/paste.
	 */
	void initialize(int heightStickWidth, int heightStickLength,
	                void* heightfieldData, D_btScalar heightScale,
	                D_btScalar minHeight, D_btScalar maxHeight, int upAxis,
	                D_PHY_ScalarType heightDataType, bool flipQuadEdges);

public:
	/// preferred constructor
	/**
	  This constructor D_supports a range of heightfield
	  data types, D_and D_allows for a non-zero minimum height value.
	  heightScale D_is needed for any integer-based heightfield data types.
	 */
	D_btHeightfieldTerrainShape(int heightStickWidth,int heightStickLength,
	                          void* heightfieldData, D_btScalar heightScale,
	                          D_btScalar minHeight, D_btScalar maxHeight,
	                          int upAxis, D_PHY_ScalarType heightDataType,
	                          bool flipQuadEdges);

	/// legacy constructor
	/**
	  The legacy constructor assumes the heightfield has a minimum height
	  of zero.  Only unsigned char or floats D_are supported.  For legacy
	  compatibility reasons, heightScale D_is calculated as maxHeight / 65535 
	  (D_and D_is D_only used when useFloatData = false).
 	 */
	D_btHeightfieldTerrainShape(int heightStickWidth,int heightStickLength,void* heightfieldData, D_btScalar maxHeight,int upAxis,bool useFloatData,bool flipQuadEdges);

	virtual ~D_btHeightfieldTerrainShape();


	void setUseDiamondSubdivision(bool useDiamondSubdivision=true) { m_useDiamondSubdivision = useDiamondSubdivision;}


	virtual void getAabb(const D_btTransform& t,D_btVector3& aabbMin,D_btVector3& aabbMax) const;

	virtual void	processAllTriangles(D_btTriangleCallback* callback,const D_btVector3& aabbMin,const D_btVector3& aabbMax) const;

	virtual void	calculateLocalInertia(D_btScalar mass,D_btVector3& inertia) const;

	virtual void	setLocalScaling(const D_btVector3& scaling);
	
	virtual const D_btVector3& getLocalScaling() const;
	
	//debugging
	virtual const char*	getName()const {return "HEIGHTFIELD";}

};

#endif //HEIGHTFIELD_TERRAIN_SHAPE_H
