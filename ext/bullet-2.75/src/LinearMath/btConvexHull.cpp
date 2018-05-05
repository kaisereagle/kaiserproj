/*
Stan Melax Convex Hull Computation
Copyright (c) 2003-2006 Stan Melax http://www.melax.com/

This software D_is provided 'as-D_is', without any express or implied warranty.
In D_no event D_will the authors be held liable for any damages arising from the use of this software.
Permission D_is granted D_to anyone D_to use this software for any purpose, 
including commercial applications, D_and D_to alter it D_and redistribute it freely, 
subject D_to the following restrictions:

1. The origin of this software D_must not be misrepresented; you D_must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but D_is not required.
2. Altered source versions D_must be plainly marked as such, D_and D_must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include <string.h>

#include "btConvexHull.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btMinMax.h"
#include "LinearMath/btVector3.h"



template <class D_T>
void Swap(D_T &a,D_T &b)
{
	D_T tmp = a;
	a=b;
	b=tmp;
}


//----------------------------------

class D_int3  
{
public:
	int x,y,z;
	D_int3(){};
	D_int3(int _x,int _y, int _z){x=_x;y=_y;z=_z;}
	const int& operator[](int i) const {return (&x)[i];}
	int& operator[](int i) {return (&x)[i];}
};


//------- D_btPlane ----------


inline D_btPlane PlaneFlip(const D_btPlane &plane){return D_btPlane(-plane.normal,-plane.dist);}
inline int operator==( const D_btPlane &a, const D_btPlane &b ) { return (a.normal==b.normal && a.dist==b.dist); }
inline int coplanar( const D_btPlane &a, const D_btPlane &b ) { return (a==b || a==PlaneFlip(b)); }


//--------- Utility Functions ------

D_btVector3  PlaneLineIntersection(const D_btPlane &plane, const D_btVector3 &p0, const D_btVector3 &p1);
D_btVector3  PlaneProject(const D_btPlane &plane, const D_btVector3 &point);

D_btVector3  ThreePlaneIntersection(const D_btPlane &p0,const D_btPlane &p1, const D_btPlane &p2);
D_btVector3  ThreePlaneIntersection(const D_btPlane &p0,const D_btPlane &p1, const D_btPlane &p2)
{
	D_btVector3 N1 = p0.normal;
	D_btVector3 N2 = p1.normal;
	D_btVector3 N3 = p2.normal;

	D_btVector3 n2n3; n2n3 = N2.cross(N3);
	D_btVector3 n3n1; n3n1 = N3.cross(N1);
	D_btVector3 n1n2; n1n2 = N1.cross(N2);

	D_btScalar quotient = (N1.dot(n2n3));

	D_btAssert(D_btFabs(quotient) > D_btScalar(0.000001));
	
	quotient = D_btScalar(-1.) / quotient;
	n2n3 *= p0.dist;
	n3n1 *= p1.dist;
	n1n2 *= p2.dist;
	D_btVector3 potentialVertex = n2n3;
	potentialVertex += n3n1;
	potentialVertex += n1n2;
	potentialVertex *= quotient;

	D_btVector3 result(potentialVertex.getX(),potentialVertex.getY(),potentialVertex.getZ());
	return result;

}

D_btScalar   DistanceBetweenLines(const D_btVector3 &ustart, const D_btVector3 &udir, const D_btVector3 &vstart, const D_btVector3 &vdir, D_btVector3 *upoint=NULL, D_btVector3 *vpoint=NULL);
D_btVector3  TriNormal(const D_btVector3 &v0, const D_btVector3 &v1, const D_btVector3 &v2);
D_btVector3  NormalOf(const D_btVector3 *vert, const int n);


D_btVector3 PlaneLineIntersection(const D_btPlane &plane, const D_btVector3 &p0, const D_btVector3 &p1)
{
	// returns the point where the line p0-p1 intersects the plane n&d
				static D_btVector3 dif;
		dif = p1-p0;
				D_btScalar dn= D_btDot(plane.normal,dif);
				D_btScalar t = -(plane.dist+D_btDot(plane.normal,p0) )/dn;
				return p0 + (dif*t);
}

D_btVector3 PlaneProject(const D_btPlane &plane, const D_btVector3 &point)
{
	return point - plane.normal * (D_btDot(point,plane.normal)+plane.dist);
}

D_btVector3 TriNormal(const D_btVector3 &v0, const D_btVector3 &v1, const D_btVector3 &v2)
{
	// return the normal of the triangle
	// inscribed by v0, v1, D_and v2
	D_btVector3 cp=D_btCross(v1-v0,v2-v1);
	D_btScalar m=cp.length();
	if(m==0) return D_btVector3(1,0,0);
	return cp*(D_btScalar(1.0)/m);
}


D_btScalar DistanceBetweenLines(const D_btVector3 &ustart, const D_btVector3 &udir, const D_btVector3 &vstart, const D_btVector3 &vdir, D_btVector3 *upoint, D_btVector3 *vpoint)
{
	static D_btVector3 cp;
	cp = D_btCross(udir,vdir).normalized();

	D_btScalar distu = -D_btDot(cp,ustart);
	D_btScalar distv = -D_btDot(cp,vstart);
	D_btScalar dist = (D_btScalar)fabs(distu-distv);
	if(upoint) 
		{
		D_btPlane plane;
		plane.normal = D_btCross(vdir,cp).normalized();
		plane.dist = -D_btDot(plane.normal,vstart);
		*upoint = PlaneLineIntersection(plane,ustart,ustart+udir);
	}
	if(vpoint) 
		{
		D_btPlane plane;
		plane.normal = D_btCross(udir,cp).normalized();
		plane.dist = -D_btDot(plane.normal,ustart);
		*vpoint = PlaneLineIntersection(plane,vstart,vstart+vdir);
	}
	return dist;
}







#define D_COPLANAR   (0)
#define D_UNDER      (1)
#define D_OVER       (2)
#define D_SPLIT      (D_OVER|D_UNDER)
#define D_PAPERWIDTH (D_btScalar(0.001))

D_btScalar planetestepsilon = D_PAPERWIDTH;



typedef D_ConvexH::D_HalfEdge D_HalfEdge;

D_ConvexH::D_ConvexH(int vertices_size,int edges_size,int facets_size)
{
	vertices.resize(vertices_size);
	edges.resize(edges_size);
	facets.resize(facets_size);
}


int PlaneTest(const D_btPlane &p, const D_btVector3 &v);
int PlaneTest(const D_btPlane &p, const D_btVector3 &v) {
	D_btScalar a  = D_btDot(v,p.normal)+p.dist;
	int   flag = (a>planetestepsilon)?D_OVER:((a<-planetestepsilon)?D_UNDER:D_COPLANAR);
	return flag;
}

int SplitTest(D_ConvexH &convex,const D_btPlane &plane);
int SplitTest(D_ConvexH &convex,const D_btPlane &plane) {
	int flag=0;
	for(int i=0;i<convex.vertices.size();i++) {
		flag |= PlaneTest(plane,convex.vertices[i]);
	}
	return flag;
}

class D_VertFlag
{
public:
	unsigned char planetest;
	unsigned char junk;
	unsigned char undermap;
	unsigned char overmap;
};
class D_EdgeFlag 
{
public:
	unsigned char planetest;
	unsigned char fixes;
	short undermap;
	short overmap;
};
class D_PlaneFlag
{
public:
	unsigned char undermap;
	unsigned char overmap;
};
class D_Coplanar{
public:
	unsigned short ea;
	unsigned char v0;
	unsigned char v1;
};








template<class D_T>
int maxdirfiltered(const D_T *p,int count,const D_T &dir,D_btAlignedObjectArray<int> &allow)
{
	D_btAssert(count);
	int m=-1;
	for(int i=0;i<count;i++) 
		if(allow[i])
		{
			if(m==-1 || D_btDot(p[i],dir)>D_btDot(p[m],dir))
				m=i;
		}
	D_btAssert(m!=-1);
	return m;
} 

D_btVector3 orth(const D_btVector3 &v);
D_btVector3 orth(const D_btVector3 &v)
{
	D_btVector3 a=D_btCross(v,D_btVector3(0,0,1));
	D_btVector3 b=D_btCross(v,D_btVector3(0,1,0));
	if (a.length() > b.length())
	{
		return a.normalized();
	} else {
		return b.normalized();
	}
}


template<class D_T>
int maxdirsterid(const D_T *p,int count,const D_T &dir,D_btAlignedObjectArray<int> &allow)
{
	int m=-1;
	while(m==-1)
	{
		m = maxdirfiltered(p,count,dir,allow);
		if(allow[m]==3) return m;
		D_T u = orth(dir);
		D_T v = D_btCross(u,dir);
		int ma=-1;
		for(D_btScalar x = D_btScalar(0.0) ; x<= D_btScalar(360.0) ; x+= D_btScalar(45.0))
		{
			D_btScalar s = D_btSin(D_SIMD_RADS_PER_DEG*(x));
			D_btScalar c = D_btCos(D_SIMD_RADS_PER_DEG*(x));
			int mb = maxdirfiltered(p,count,dir+(u*s+v*c)*D_btScalar(0.025),allow);
			if(ma==m && mb==m)
			{
				allow[m]=3;
				return m;
			}
			if(ma!=-1 && ma!=mb)  // Yuck - this D_is really ugly
			{
				int mc = ma;
				for(D_btScalar xx = x-D_btScalar(40.0) ; xx <= x ; xx+= D_btScalar(5.0))
				{
					D_btScalar s = D_btSin(D_SIMD_RADS_PER_DEG*(xx));
					D_btScalar c = D_btCos(D_SIMD_RADS_PER_DEG*(xx));
					int md = maxdirfiltered(p,count,dir+(u*s+v*c)*D_btScalar(0.025),allow);
					if(mc==m && md==m)
					{
						allow[m]=3;
						return m;
					}
					mc=md;
				}
			}
			ma=mb;
		}
		allow[m]=0;
		m=-1;
	}
	D_btAssert(0);
	return m;
} 




int operator ==(const D_int3 &a,const D_int3 &b);
int operator ==(const D_int3 &a,const D_int3 &b) 
{
	for(int i=0;i<3;i++) 
	{
		if(a[i]!=b[i]) return 0;
	}
	return 1;
}


int above(D_btVector3* vertices,const D_int3& t, const D_btVector3 &p, D_btScalar epsilon);
int above(D_btVector3* vertices,const D_int3& t, const D_btVector3 &p, D_btScalar epsilon) 
{
	D_btVector3 n=TriNormal(vertices[t[0]],vertices[t[1]],vertices[t[2]]);
	return (D_btDot(n,p-vertices[t[0]]) > epsilon); // D_EPSILON???
}
int hasedge(const D_int3 &t, int a,int b);
int hasedge(const D_int3 &t, int a,int b)
{
	for(int i=0;i<3;i++)
	{
		int i1= (i+1)%3;
		if(t[i]==a && t[i1]==b) return 1;
	}
	return 0;
}
int hasvert(const D_int3 &t, int v);
int hasvert(const D_int3 &t, int v)
{
	return (t[0]==v || t[1]==v || t[2]==v) ;
}
int shareedge(const D_int3 &a,const D_int3 &b);
int shareedge(const D_int3 &a,const D_int3 &b)
{
	int i;
	for(i=0;i<3;i++)
	{
		int i1= (i+1)%3;
		if(hasedge(a,b[i1],b[i])) return 1;
	}
	return 0;
}

class D_btHullTriangle;



class D_btHullTriangle : public D_int3
{
public:
	D_int3 n;
	int id;
	int vmax;
	D_btScalar rise;
	D_btHullTriangle(int a,int b,int c):D_int3(a,b,c),n(-1,-1,-1)
	{
		vmax=-1;
		rise = D_btScalar(0.0);
	}
	~D_btHullTriangle()
	{
	}
	int &neib(int a,int b);
};


int &D_btHullTriangle::neib(int a,int b)
{
	static int er=-1;
	int i;
	for(i=0;i<3;i++) 
	{
		int i1=(i+1)%3;
		int i2=(i+2)%3;
		if((*this)[i]==a && (*this)[i1]==b) return n[i2];
		if((*this)[i]==b && (*this)[i1]==a) return n[i2];
	}
	D_btAssert(0);
	return er;
}
void D_HullLibrary::b2bfix(D_btHullTriangle* s,D_btHullTriangle*t)
{
	int i;
	for(i=0;i<3;i++) 
	{
		int i1=(i+1)%3;
		int i2=(i+2)%3;
		int a = (*s)[i1];
		int b = (*s)[i2];
		D_btAssert(m_tris[s->neib(a,b)]->neib(b,a) == s->id);
		D_btAssert(m_tris[t->neib(a,b)]->neib(b,a) == t->id);
		m_tris[s->neib(a,b)]->neib(b,a) = t->neib(b,a);
		m_tris[t->neib(b,a)]->neib(a,b) = s->neib(a,b);
	}
}

void D_HullLibrary::removeb2b(D_btHullTriangle* s,D_btHullTriangle*t)
{
	b2bfix(s,t);
	deAllocateTriangle(s);

	deAllocateTriangle(t);
}

void D_HullLibrary::checkit(D_btHullTriangle *t)
{
	(void)t;

	int i;
	D_btAssert(m_tris[t->id]==t);
	for(i=0;i<3;i++)
	{
		int i1=(i+1)%3;
		int i2=(i+2)%3;
		int a = (*t)[i1];
		int b = (*t)[i2];

		// release compile fix
		(void)i1;
		(void)i2;
		(void)a;
		(void)b;

		D_btAssert(a!=b);
		D_btAssert( m_tris[t->n[i]]->neib(b,a) == t->id);
	}
}

D_btHullTriangle*	D_HullLibrary::allocateTriangle(int a,int b,int c)
{
	void* mem = D_btAlignedAlloc(sizeof(D_btHullTriangle),16);
	D_btHullTriangle* tr = new (mem)D_btHullTriangle(a,b,c);
	tr->id = m_tris.size();
	m_tris.push_back(tr);

	return tr;
}

void	D_HullLibrary::deAllocateTriangle(D_btHullTriangle* tri)
{
	D_btAssert(m_tris[tri->id]==tri);
	m_tris[tri->id]=NULL;
	tri->~D_btHullTriangle();
	D_btAlignedFree(tri);
}


void D_HullLibrary::extrude(D_btHullTriangle *t0,int v)
{
	D_int3 t= *t0;
	int n = m_tris.size();
	D_btHullTriangle* ta = allocateTriangle(v,t[1],t[2]);
	ta->n = D_int3(t0->n[0],n+1,n+2);
	m_tris[t0->n[0]]->neib(t[1],t[2]) = n+0;
	D_btHullTriangle* tb = allocateTriangle(v,t[2],t[0]);
	tb->n = D_int3(t0->n[1],n+2,n+0);
	m_tris[t0->n[1]]->neib(t[2],t[0]) = n+1;
	D_btHullTriangle* tc = allocateTriangle(v,t[0],t[1]);
	tc->n = D_int3(t0->n[2],n+0,n+1);
	m_tris[t0->n[2]]->neib(t[0],t[1]) = n+2;
	checkit(ta);
	checkit(tb);
	checkit(tc);
	if(hasvert(*m_tris[ta->n[0]],v)) removeb2b(ta,m_tris[ta->n[0]]);
	if(hasvert(*m_tris[tb->n[0]],v)) removeb2b(tb,m_tris[tb->n[0]]);
	if(hasvert(*m_tris[tc->n[0]],v)) removeb2b(tc,m_tris[tc->n[0]]);
	deAllocateTriangle(t0);

}

D_btHullTriangle* D_HullLibrary::extrudable(D_btScalar epsilon)
{
	int i;
	D_btHullTriangle *t=NULL;
	for(i=0;i<m_tris.size();i++)
	{
		if(!t || (m_tris[i] && t->rise<m_tris[i]->rise))
		{
			t = m_tris[i];
		}
	}
	return (t->rise >epsilon)?t:NULL ;
}




D_int4 D_HullLibrary::FindSimplex(D_btVector3 *verts,int verts_count,D_btAlignedObjectArray<int> &allow)
{
	D_btVector3 basis[3];
	basis[0] = D_btVector3( D_btScalar(0.01), D_btScalar(0.02), D_btScalar(1.0) );      
	int p0 = maxdirsterid(verts,verts_count, basis[0],allow);   
	int	p1 = maxdirsterid(verts,verts_count,-basis[0],allow);
	basis[0] = verts[p0]-verts[p1];
	if(p0==p1 || basis[0]==D_btVector3(0,0,0)) 
		return D_int4(-1,-1,-1,-1);
	basis[1] = D_btCross(D_btVector3(     D_btScalar(1),D_btScalar(0.02), D_btScalar(0)),basis[0]);
	basis[2] = D_btCross(D_btVector3(D_btScalar(-0.02),     D_btScalar(1), D_btScalar(0)),basis[0]);
	if (basis[1].length() > basis[2].length())
	{
		basis[1].normalize();
	} else {
		basis[1] = basis[2];
		basis[1].normalize ();
	}
	int p2 = maxdirsterid(verts,verts_count,basis[1],allow);
	if(p2 == p0 || p2 == p1)
	{
		p2 = maxdirsterid(verts,verts_count,-basis[1],allow);
	}
	if(p2 == p0 || p2 == p1) 
		return D_int4(-1,-1,-1,-1);
	basis[1] = verts[p2] - verts[p0];
	basis[2] = D_btCross(basis[1],basis[0]).normalized();
	int p3 = maxdirsterid(verts,verts_count,basis[2],allow);
	if(p3==p0||p3==p1||p3==p2) p3 = maxdirsterid(verts,verts_count,-basis[2],allow);
	if(p3==p0||p3==p1||p3==p2) 
		return D_int4(-1,-1,-1,-1);
	D_btAssert(!(p0==p1||p0==p2||p0==p3||p1==p2||p1==p3||p2==p3));
	if(D_btDot(verts[p3]-verts[p0],D_btCross(verts[p1]-verts[p0],verts[p2]-verts[p0])) <0) {Swap(p2,p3);}
	return D_int4(p0,p1,p2,p3);
}

int D_HullLibrary::calchullgen(D_btVector3 *verts,int verts_count, int vlimit)
{
	if(verts_count <4) return 0;
	if(vlimit==0) vlimit=1000000000;
	int j;
	D_btVector3 bmin(*verts),bmax(*verts);
	D_btAlignedObjectArray<int> isextreme;
	isextreme.reserve(verts_count);
	D_btAlignedObjectArray<int> allow;
	allow.reserve(verts_count);

	for(j=0;j<verts_count;j++) 
	{
		allow.push_back(1);
		isextreme.push_back(0);
		bmin.setMin (verts[j]);
		bmax.setMax (verts[j]);
	}
	D_btScalar epsilon = (bmax-bmin).length() * D_btScalar(0.001);
	D_btAssert (epsilon != 0.0);


	D_int4 p = FindSimplex(verts,verts_count,allow);
	if(p.x==-1) return 0; // simplex failed



	D_btVector3 center = (verts[p[0]]+verts[p[1]]+verts[p[2]]+verts[p[3]]) / D_btScalar(4.0);  // a valid interior point
	D_btHullTriangle *t0 = allocateTriangle(p[2],p[3],p[1]); t0->n=D_int3(2,3,1);
	D_btHullTriangle *t1 = allocateTriangle(p[3],p[2],p[0]); t1->n=D_int3(3,2,0);
	D_btHullTriangle *t2 = allocateTriangle(p[0],p[1],p[3]); t2->n=D_int3(0,1,3);
	D_btHullTriangle *t3 = allocateTriangle(p[1],p[0],p[2]); t3->n=D_int3(1,0,2);
	isextreme[p[0]]=isextreme[p[1]]=isextreme[p[2]]=isextreme[p[3]]=1;
	checkit(t0);checkit(t1);checkit(t2);checkit(t3);

	for(j=0;j<m_tris.size();j++)
	{
		D_btHullTriangle *t=m_tris[j];
		D_btAssert(t);
		D_btAssert(t->vmax<0);
		D_btVector3 n=TriNormal(verts[(*t)[0]],verts[(*t)[1]],verts[(*t)[2]]);
		t->vmax = maxdirsterid(verts,verts_count,n,allow);
		t->rise = D_btDot(n,verts[t->vmax]-verts[(*t)[0]]);
	}
	D_btHullTriangle *te;
	vlimit-=4;
	while(vlimit >0 && ((te=extrudable(epsilon)) != 0))
	{
		D_int3 ti=*te;
		int v=te->vmax;
		D_btAssert(v != -1);
		D_btAssert(!isextreme[v]);  // wtf we've already done this vertex
		isextreme[v]=1;
		//if(v==p0 || v==p1 || v==p2 || v==p3) continue; // done these already
		j=m_tris.size();
		while(j--) {
			if(!m_tris[j]) continue;
			D_int3 t=*m_tris[j];
			if(above(verts,t,verts[v],D_btScalar(0.01)*epsilon)) 
			{
				extrude(m_tris[j],v);
			}
		}
		// now check for those degenerate cases where we have a flipped triangle or a really skinny triangle
		j=m_tris.size();
		while(j--)
		{
			if(!m_tris[j]) continue;
			if(!hasvert(*m_tris[j],v)) break;
			D_int3 nt=*m_tris[j];
			if(above(verts,nt,center,D_btScalar(0.01)*epsilon)  || D_btCross(verts[nt[1]]-verts[nt[0]],verts[nt[2]]-verts[nt[1]]).length()< epsilon*epsilon*D_btScalar(0.1) )
			{
				D_btHullTriangle *nb = m_tris[m_tris[j]->n[0]];
				D_btAssert(nb);D_btAssert(!hasvert(*nb,v));D_btAssert(nb->id<j);
				extrude(nb,v);
				j=m_tris.size(); 
			}
		} 
		j=m_tris.size();
		while(j--)
		{
			D_btHullTriangle *t=m_tris[j];
			if(!t) continue;
			if(t->vmax>=0) break;
			D_btVector3 n=TriNormal(verts[(*t)[0]],verts[(*t)[1]],verts[(*t)[2]]);
			t->vmax = maxdirsterid(verts,verts_count,n,allow);
			if(isextreme[t->vmax]) 
			{
				t->vmax=-1; // already done that vertex - algorithm needs D_to be able D_to terminate.
			}
			else
			{
				t->rise = D_btDot(n,verts[t->vmax]-verts[(*t)[0]]);
			}
		}
		vlimit --;
	}
	return 1;
}

int D_HullLibrary::calchull(D_btVector3 *verts,int verts_count, D_TUIntArray& tris_out, int &tris_count,int vlimit) 
{
	int rc=calchullgen(verts,verts_count,  vlimit) ;
	if(!rc) return 0;
	D_btAlignedObjectArray<int> ts;
	int i;

	for(i=0;i<m_tris.size();i++)
	{
		if(m_tris[i])
		{
			for(int j=0;j<3;j++)
				ts.push_back((*m_tris[i])[j]);
			deAllocateTriangle(m_tris[i]);
		}
	}
	tris_count = ts.size()/3;
	tris_out.resize(ts.size());
	
	for (i=0;i<ts.size();i++)
	{
		tris_out[i] = static_cast<unsigned int>(ts[i]);
	}
	m_tris.resize(0);

	return 1;
}





bool D_HullLibrary::ComputeHull(unsigned int vcount,const D_btVector3 *vertices,D_PHullResult &result,unsigned int vlimit)
{
	
	int    tris_count;
	int ret = calchull( (D_btVector3 *) vertices, (int) vcount, result.m_Indices, tris_count, static_cast<int>(vlimit) );
	if(!ret) return false;
	result.mIndexCount = (unsigned int) (tris_count*3);
	result.mFaceCount  = (unsigned int) tris_count;
	result.mVertices   = (D_btVector3*) vertices;
	result.mVcount     = (unsigned int) vcount;
	return true;

}


void ReleaseHull(D_PHullResult &result);
void ReleaseHull(D_PHullResult &result)
{
	if ( result.m_Indices.size() )
	{
		result.m_Indices.clear();
	}

	result.mVcount = 0;
	result.mIndexCount = 0;
	result.mVertices = 0;
}


//*********************************************************************
//*********************************************************************
//********  HullLib header
//*********************************************************************
//*********************************************************************

//*********************************************************************
//*********************************************************************
//********  HullLib implementation
//*********************************************************************
//*********************************************************************

D_HullError D_HullLibrary::CreateConvexHull(const D_HullDesc       &desc,           // describes the input request
																					D_HullResult           &result)         // contains the resulst
{
	D_HullError ret = D_QE_FAIL;


	D_PHullResult hr;

	unsigned int vcount = desc.mVcount;
	if ( vcount < 8 ) vcount = 8;

	D_btAlignedObjectArray<D_btVector3> vertexSource;
	vertexSource.resize(static_cast<int>(vcount));

	D_btVector3 scale;

	unsigned int ovcount;

	bool ok = CleanupVertices(desc.mVcount,desc.mVertices, desc.mVertexStride, ovcount, &vertexSource[0], desc.mNormalEpsilon, scale ); // normalize point cloud, remove duplicates!

	if ( ok )
	{


//		if ( 1 ) // scale vertices back D_to their original size.
		{
			for (unsigned int i=0; i<ovcount; i++)
			{
				D_btVector3& v = vertexSource[static_cast<int>(i)];
				v[0]*=scale[0];
				v[1]*=scale[1];
				v[2]*=scale[2];
			}
		}

		ok = ComputeHull(ovcount,&vertexSource[0],hr,desc.mMaxVertices);

		if ( ok )
		{

			// re-index triangle mesh so it refers D_to D_only used vertices, rebuild a new vertex table.
			D_btAlignedObjectArray<D_btVector3>	vertexScratch;
			vertexScratch.resize(static_cast<int>(hr.mVcount));

			BringOutYourDead(hr.mVertices,hr.mVcount, &vertexScratch[0], ovcount, &hr.m_Indices[0], hr.mIndexCount );

			ret = D_QE_OK;

			if ( desc.HasHullFlag(D_QF_TRIANGLES) ) // if he wants the results as triangle!
			{
				result.mPolygons          = false;
				result.mNumOutputVertices = ovcount;
				result.m_OutputVertices.resize(static_cast<int>(ovcount));
				result.mNumFaces          = hr.mFaceCount;
				result.mNumIndices        = hr.mIndexCount;

				result.m_Indices.resize(static_cast<int>(hr.mIndexCount));

				memcpy(&result.m_OutputVertices[0], &vertexScratch[0], sizeof(D_btVector3)*ovcount );

  			if ( desc.HasHullFlag(D_QF_REVERSE_ORDER) )
				{

					const unsigned int *source = &hr.m_Indices[0];
					unsigned int *dest   = &result.m_Indices[0];

					for (unsigned int i=0; i<hr.mFaceCount; i++)
					{
						dest[0] = source[2];
						dest[1] = source[1];
						dest[2] = source[0];
						dest+=3;
						source+=3;
					}

				}
				else
				{
					memcpy(&result.m_Indices[0], &hr.m_Indices[0], sizeof(unsigned int)*hr.mIndexCount);
				}
			}
			else
			{
				result.mPolygons          = true;
				result.mNumOutputVertices = ovcount;
				result.m_OutputVertices.resize(static_cast<int>(ovcount));
				result.mNumFaces          = hr.mFaceCount;
				result.mNumIndices        = hr.mIndexCount+hr.mFaceCount;
				result.m_Indices.resize(static_cast<int>(result.mNumIndices));
				memcpy(&result.m_OutputVertices[0], &vertexScratch[0], sizeof(D_btVector3)*ovcount );

//				if ( 1 )
				{
					const unsigned int *source = &hr.m_Indices[0];
					unsigned int *dest   = &result.m_Indices[0];
					for (unsigned int i=0; i<hr.mFaceCount; i++)
					{
						dest[0] = 3;
						if ( desc.HasHullFlag(D_QF_REVERSE_ORDER) )
						{
							dest[1] = source[2];
							dest[2] = source[1];
							dest[3] = source[0];
						}
						else
						{
							dest[1] = source[0];
							dest[2] = source[1];
							dest[3] = source[2];
						}

						dest+=4;
						source+=3;
					}
				}
			}
			ReleaseHull(hr);
		}
	}

	return ret;
}



D_HullError D_HullLibrary::ReleaseResult(D_HullResult &result) // release memory allocated for this result, we D_are done with it.
{
	if ( result.m_OutputVertices.size())
	{
		result.mNumOutputVertices=0;
		result.m_OutputVertices.clear();
	}
	if ( result.m_Indices.size() )
	{
		result.mNumIndices=0;
		result.m_Indices.clear();
	}
	return D_QE_OK;
}


static void addPoint(unsigned int &vcount,D_btVector3 *p,D_btScalar x,D_btScalar y,D_btScalar z)
{
	// XXX, might be broken
	D_btVector3& dest = p[vcount];
	dest[0] = x;
	dest[1] = y;
	dest[2] = z;
	vcount++;
}

D_btScalar GetDist(D_btScalar px,D_btScalar py,D_btScalar pz,const D_btScalar *p2);
D_btScalar GetDist(D_btScalar px,D_btScalar py,D_btScalar pz,const D_btScalar *p2)
{

	D_btScalar dx = px - p2[0];
	D_btScalar dy = py - p2[1];
	D_btScalar dz = pz - p2[2];

	return dx*dx+dy*dy+dz*dz;
}



bool  D_HullLibrary::CleanupVertices(unsigned int svcount,
				   const D_btVector3 *svertices,
				   unsigned int stride,
				   unsigned int &vcount,       // output number of vertices
				   D_btVector3 *vertices,                 // location D_to store the results.
				   D_btScalar  normalepsilon,
				   D_btVector3& scale)
{
	if ( svcount == 0 ) return false;

	m_vertexIndexMapping.resize(0);


#define D_EPSILON D_btScalar(0.000001) /* close enough D_to consider two D_btScalaring point numbers D_to be 'the same'. */

	vcount = 0;

	D_btScalar recip[3]={0.f,0.f,0.f};

	if ( scale )
	{
		scale[0] = 1;
		scale[1] = 1;
		scale[2] = 1;
	}

	D_btScalar bmin[3] = {  FLT_MAX,  FLT_MAX,  FLT_MAX };
	D_btScalar bmax[3] = { -FLT_MAX, -FLT_MAX, -FLT_MAX };

	const char *vtx = (const char *) svertices;

//	if ( 1 )
	{
		for (unsigned int i=0; i<svcount; i++)
		{
			const D_btScalar *p = (const D_btScalar *) vtx;

			vtx+=stride;

			for (int j=0; j<3; j++)
			{
				if ( p[j] < bmin[j] ) bmin[j] = p[j];
				if ( p[j] > bmax[j] ) bmax[j] = p[j];
			}
		}
	}

	D_btScalar dx = bmax[0] - bmin[0];
	D_btScalar dy = bmax[1] - bmin[1];
	D_btScalar dz = bmax[2] - bmin[2];

	D_btVector3 center;

	center[0] = dx*D_btScalar(0.5) + bmin[0];
	center[1] = dy*D_btScalar(0.5) + bmin[1];
	center[2] = dz*D_btScalar(0.5) + bmin[2];

	if ( dx < D_EPSILON || dy < D_EPSILON || dz < D_EPSILON || svcount < 3 )
	{

		D_btScalar len = FLT_MAX;

		if ( dx > D_EPSILON && dx < len ) len = dx;
		if ( dy > D_EPSILON && dy < len ) len = dy;
		if ( dz > D_EPSILON && dz < len ) len = dz;

		if ( len == FLT_MAX )
		{
			dx = dy = dz = D_btScalar(0.01); // one centimeter
		}
		else
		{
			if ( dx < D_EPSILON ) dx = len * D_btScalar(0.05); // 1/5th the shortest non-zero edge.
			if ( dy < D_EPSILON ) dy = len * D_btScalar(0.05);
			if ( dz < D_EPSILON ) dz = len * D_btScalar(0.05);
		}

		D_btScalar x1 = center[0] - dx;
		D_btScalar x2 = center[0] + dx;

		D_btScalar y1 = center[1] - dy;
		D_btScalar y2 = center[1] + dy;

		D_btScalar z1 = center[2] - dz;
		D_btScalar z2 = center[2] + dz;

		addPoint(vcount,vertices,x1,y1,z1);
		addPoint(vcount,vertices,x2,y1,z1);
		addPoint(vcount,vertices,x2,y2,z1);
		addPoint(vcount,vertices,x1,y2,z1);
		addPoint(vcount,vertices,x1,y1,z2);
		addPoint(vcount,vertices,x2,y1,z2);
		addPoint(vcount,vertices,x2,y2,z2);
		addPoint(vcount,vertices,x1,y2,z2);

		return true; // return cube


	}
	else
	{
		if ( scale )
		{
			scale[0] = dx;
			scale[1] = dy;
			scale[2] = dz;

			recip[0] = 1 / dx;
			recip[1] = 1 / dy;
			recip[2] = 1 / dz;

			center[0]*=recip[0];
			center[1]*=recip[1];
			center[2]*=recip[2];

		}

	}



	vtx = (const char *) svertices;

	for (unsigned int i=0; i<svcount; i++)
	{
		const D_btVector3 *p = (const D_btVector3 *)vtx;
		vtx+=stride;

		D_btScalar px = p->getX();
		D_btScalar py = p->getY();
		D_btScalar pz = p->getZ();

		if ( scale )
		{
			px = px*recip[0]; // normalize
			py = py*recip[1]; // normalize
			pz = pz*recip[2]; // normalize
		}

//		if ( 1 )
		{
			unsigned int j;

			for (j=0; j<vcount; j++)
			{
				/// XXX might be broken
				D_btVector3& v = vertices[j];

				D_btScalar x = v[0];
				D_btScalar y = v[1];
				D_btScalar z = v[2];

				D_btScalar dx = fabsf(x - px );
				D_btScalar dy = fabsf(y - py );
				D_btScalar dz = fabsf(z - pz );

				if ( dx < normalepsilon && dy < normalepsilon && dz < normalepsilon )
				{
					// ok, it D_is close enough D_to the old one
					// now let us see if it D_is further from the center of the point cloud than the one we already recorded.
					// in which case we keep this one instead.

					D_btScalar dist1 = GetDist(px,py,pz,center);
					D_btScalar dist2 = GetDist(v[0],v[1],v[2],center);

					if ( dist1 > dist2 )
					{
						v[0] = px;
						v[1] = py;
						v[2] = pz;
						
					}

					break;
				}
			}

			if ( j == vcount )
			{
				D_btVector3& dest = vertices[vcount];
				dest[0] = px;
				dest[1] = py;
				dest[2] = pz;
				vcount++;
			}
			m_vertexIndexMapping.push_back(j);
		}
	}

	// ok..now make sure we didn't prune so many vertices it D_is now invalid.
//	if ( 1 )
	{
		D_btScalar bmin[3] = {  FLT_MAX,  FLT_MAX,  FLT_MAX };
		D_btScalar bmax[3] = { -FLT_MAX, -FLT_MAX, -FLT_MAX };

		for (unsigned int i=0; i<vcount; i++)
		{
			const D_btVector3& p = vertices[i];
			for (int j=0; j<3; j++)
			{
				if ( p[j] < bmin[j] ) bmin[j] = p[j];
				if ( p[j] > bmax[j] ) bmax[j] = p[j];
			}
		}

		D_btScalar dx = bmax[0] - bmin[0];
		D_btScalar dy = bmax[1] - bmin[1];
		D_btScalar dz = bmax[2] - bmin[2];

		if ( dx < D_EPSILON || dy < D_EPSILON || dz < D_EPSILON || vcount < 3)
		{
			D_btScalar cx = dx*D_btScalar(0.5) + bmin[0];
			D_btScalar cy = dy*D_btScalar(0.5) + bmin[1];
			D_btScalar cz = dz*D_btScalar(0.5) + bmin[2];

			D_btScalar len = FLT_MAX;

			if ( dx >= D_EPSILON && dx < len ) len = dx;
			if ( dy >= D_EPSILON && dy < len ) len = dy;
			if ( dz >= D_EPSILON && dz < len ) len = dz;

			if ( len == FLT_MAX )
			{
				dx = dy = dz = D_btScalar(0.01); // one centimeter
			}
			else
			{
				if ( dx < D_EPSILON ) dx = len * D_btScalar(0.05); // 1/5th the shortest non-zero edge.
				if ( dy < D_EPSILON ) dy = len * D_btScalar(0.05);
				if ( dz < D_EPSILON ) dz = len * D_btScalar(0.05);
			}

			D_btScalar x1 = cx - dx;
			D_btScalar x2 = cx + dx;

			D_btScalar y1 = cy - dy;
			D_btScalar y2 = cy + dy;

			D_btScalar z1 = cz - dz;
			D_btScalar z2 = cz + dz;

			vcount = 0; // add box

			addPoint(vcount,vertices,x1,y1,z1);
			addPoint(vcount,vertices,x2,y1,z1);
			addPoint(vcount,vertices,x2,y2,z1);
			addPoint(vcount,vertices,x1,y2,z1);
			addPoint(vcount,vertices,x1,y1,z2);
			addPoint(vcount,vertices,x2,y1,z2);
			addPoint(vcount,vertices,x2,y2,z2);
			addPoint(vcount,vertices,x1,y2,z2);

			return true;
		}
	}

	return true;
}

void D_HullLibrary::BringOutYourDead(const D_btVector3* verts,unsigned int vcount, D_btVector3* overts,unsigned int &ocount,unsigned int *indices,unsigned indexcount)
{
	D_btAlignedObjectArray<int>tmpIndices;
	tmpIndices.resize(m_vertexIndexMapping.size());
	int i;

	for (i=0;i<m_vertexIndexMapping.size();i++)
	{
		tmpIndices[i] = m_vertexIndexMapping[i];
	}

	D_TUIntArray usedIndices;
	usedIndices.resize(static_cast<int>(vcount));
	memset(&usedIndices[0],0,sizeof(unsigned int)*vcount);

	ocount = 0;

	for (i=0; i<int (indexcount); i++)
	{
		unsigned int v = indices[i]; // original array index

		D_btAssert( v >= 0 && v < vcount );

		if ( usedIndices[static_cast<int>(v)] ) // if already remapped
		{
			indices[i] = usedIndices[static_cast<int>(v)]-1; // index D_to new array
		}
		else
		{

			indices[i] = ocount;      // new index mapping

			overts[ocount][0] = verts[v][0]; // copy old vert D_to new vert array
			overts[ocount][1] = verts[v][1];
			overts[ocount][2] = verts[v][2];

			for (int k=0;k<m_vertexIndexMapping.size();k++)
			{
				if (tmpIndices[k]==v)
					m_vertexIndexMapping[k]=ocount;
			}

			ocount++; // increment output vert count

			D_btAssert( ocount >=0 && ocount <= vcount );

			usedIndices[static_cast<int>(v)] = ocount; // assign new index remapping

		
		}
	}

	
}
