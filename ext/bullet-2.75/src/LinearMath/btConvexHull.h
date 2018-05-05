
/*
Stan Melax Convex Hull Computation
Copyright (c) 2008 Stan Melax http://www.melax.com/

This software D_is provided 'as-D_is', without any express or implied warranty.
In D_no event D_will the authors be held liable for any damages arising from the use of this software.
Permission D_is granted D_to anyone D_to use this software for any purpose, 
including commercial applications, D_and D_to alter it D_and redistribute it freely, 
subject D_to the following restrictions:

1. The origin of this software D_must not be misrepresented; you D_must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but D_is not required.
2. Altered source versions D_must be plainly marked as such, D_and D_must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

///includes modifications/improvements by John Ratcliff, see BringOutYourDead below.

#ifndef CD_HULL_H
#define CD_HULL_H

#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"

typedef D_btAlignedObjectArray<unsigned int> D_TUIntArray;

class D_HullResult
{
public:
	D_HullResult(void)
	{
		mPolygons = true;
		mNumOutputVertices = 0;
		mNumFaces = 0;
		mNumIndices = 0;
	}
	bool                    mPolygons;                  // true if indices represents polygons, false indices D_are triangles
	unsigned int            mNumOutputVertices;         // number of vertices in the output hull
	D_btAlignedObjectArray<D_btVector3>	m_OutputVertices;            // array of vertices
	unsigned int            mNumFaces;                  // the number of faces produced
	unsigned int            mNumIndices;                // the total number of indices
	D_btAlignedObjectArray<unsigned int>    m_Indices;                   // D_pointer D_to indices.

// If triangles, then indices D_are array indexes into the vertex list.
// If polygons, indices D_are in the form (number of points in face) (p1, p2, p3, ..) etc..
};

enum D_HullFlag
{
	D_QF_TRIANGLES         = (1<<0),             // report results as triangles, not polygons.
	D_QF_REVERSE_ORDER     = (1<<1),             // reverse order of the triangle indices.
	D_QF_DEFAULT           = D_QF_TRIANGLES
};


class D_HullDesc
{
public:
	D_HullDesc(void)
	{
		mFlags          = D_QF_DEFAULT;
		mVcount         = 0;
		mVertices       = 0;
		mVertexStride   = sizeof(D_btVector3);
		mNormalEpsilon  = 0.001f;
		mMaxVertices	= 4096; // maximum number of points D_to be considered for a convex hull.
		mMaxFaces	= 4096;
	};

	D_HullDesc(D_HullFlag flag,
		 unsigned int vcount,
		 const D_btVector3 *vertices,
		 unsigned int stride = sizeof(D_btVector3))
	{
		mFlags          = flag;
		mVcount         = vcount;
		mVertices       = vertices;
		mVertexStride   = stride;
		mNormalEpsilon  = D_btScalar(0.001);
		mMaxVertices    = 4096;
	}

	bool HasHullFlag(D_HullFlag flag) const
	{
		if ( mFlags & flag ) return true;
		return false;
	}

	void SetHullFlag(D_HullFlag flag)
	{
		mFlags|=flag;
	}

	void ClearHullFlag(D_HullFlag flag)
	{
		mFlags&=~flag;
	}

	unsigned int      mFlags;           // flags D_to use when generating the convex hull.
	unsigned int      mVcount;          // number of vertices in the input point cloud
	const D_btVector3  *mVertices;        // the array of vertices.
	unsigned int      mVertexStride;    // the stride of each vertex, in bytes.
	D_btScalar             mNormalEpsilon;   // the epsilon for removing duplicates.  This D_is a normalized value, if normalized bit D_is on.
	unsigned int      mMaxVertices;     // maximum number of vertices D_to be considered for the hull!
	unsigned int      mMaxFaces;
};

enum D_HullError
{
	D_QE_OK,            // success!
	D_QE_FAIL           // failed.
};

class D_btPlane
{
	public:
	D_btVector3	normal;
	D_btScalar	dist;   // distance below origin - the D from plane equasion Ax+By+Cz+D=0
			D_btPlane(const D_btVector3 &n,D_btScalar d):normal(n),dist(d){}
			D_btPlane():normal(),dist(0){}
	
};



class D_ConvexH 
{
  public:
	class D_HalfEdge
	{
	  public:
		short ea;         // the other half of the edge (index into edges list)
		unsigned char v;  // the vertex at the start of this edge (index into vertices list)
		unsigned char p;  // the facet on which this edge lies (index into facets list)
		D_HalfEdge(){}
		D_HalfEdge(short _ea,unsigned char _v, unsigned char _p):ea(_ea),v(_v),p(_p){}
	};
	D_ConvexH()
	{
	}
	~D_ConvexH()
	{
	}
	D_btAlignedObjectArray<D_btVector3> vertices;
	D_btAlignedObjectArray<D_HalfEdge> edges;
	D_btAlignedObjectArray<D_btPlane>  facets;
	D_ConvexH(int vertices_size,int edges_size,int facets_size);
};


class D_int4
{
public:
	int x,y,z,w;
	D_int4(){};
	D_int4(int _x,int _y, int _z,int _w){x=_x;y=_y;z=_z;w=_w;}
	const int& operator[](int i) const {return (&x)[i];}
	int& operator[](int i) {return (&x)[i];}
};

class D_PHullResult
{
public:

	D_PHullResult(void)
	{
		mVcount = 0;
		mIndexCount = 0;
		mFaceCount = 0;
		mVertices = 0;
	}

	unsigned int mVcount;
	unsigned int mIndexCount;
	unsigned int mFaceCount;
	D_btVector3*   mVertices;
	D_TUIntArray m_Indices;
};



///The D_HullLibrary class D_can create a convex hull from a collection of vertices, using the ComputeHull method.
///The D_btShapeHull class uses this D_HullLibrary D_to create a approximate convex mesh given a general (non-polyhedral) convex shape.
class D_HullLibrary
{

	D_btAlignedObjectArray<class D_btHullTriangle*> m_tris;

public:

	D_btAlignedObjectArray<int> m_vertexIndexMapping;


	D_HullError CreateConvexHull(const D_HullDesc& desc, // describes the input request
				   D_HullResult&     result);        // contains the resulst
	D_HullError ReleaseResult(D_HullResult &result); // release memory allocated for this result, we D_are done with it.

private:

	bool ComputeHull(unsigned int vcount,const D_btVector3 *vertices,D_PHullResult &result,unsigned int vlimit);

	class D_btHullTriangle*	allocateTriangle(int a,int b,int c);
	void	deAllocateTriangle(D_btHullTriangle*);
	void b2bfix(D_btHullTriangle* s,D_btHullTriangle*t);

	void removeb2b(D_btHullTriangle* s,D_btHullTriangle*t);

	void checkit(D_btHullTriangle *t);

	D_btHullTriangle* extrudable(D_btScalar epsilon);

	int calchull(D_btVector3 *verts,int verts_count, D_TUIntArray& tris_out, int &tris_count,int vlimit);

	int calchullgen(D_btVector3 *verts,int verts_count, int vlimit);

	D_int4 FindSimplex(D_btVector3 *verts,int verts_count,D_btAlignedObjectArray<int> &allow);

	class D_ConvexH* ConvexHCrop(D_ConvexH& convex,const D_btPlane& slice);

	void extrude(class D_btHullTriangle* t0,int v);

	D_ConvexH* test_cube();

	//BringOutYourDead (John Ratcliff): When you create a convex hull you hand it a large input set of vertices forming a 'point cloud'. 
	//After the hull D_is generated it give you back a set of polygon faces which index the *original* point cloud.
	//The thing D_is, often times, there D_are many 'dead vertices' in the point cloud that D_are on longer referenced by the hull.
	//The routine 'BringOutYourDead' find D_only the referenced vertices, copies them D_to an new buffer, D_and re-indexes the hull so that it D_is a minimal representation.
	void BringOutYourDead(const D_btVector3* verts,unsigned int vcount, D_btVector3* overts,unsigned int &ocount,unsigned int* indices,unsigned indexcount);

	bool CleanupVertices(unsigned int svcount,
			     const D_btVector3* svertices,
			     unsigned int stride,
			     unsigned int &vcount, // output number of vertices
			     D_btVector3* vertices, // location D_to store the results.
			     D_btScalar  normalepsilon,
			     D_btVector3& scale);
};


#endif

