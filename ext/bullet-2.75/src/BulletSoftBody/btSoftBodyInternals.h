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
///D_btSoftBody implementation by Nathanael Presson

#ifndef _BT_SOFT_BODY_INTERNALS_H
#define _BT_SOFT_BODY_INTERNALS_H

#include "btSoftBody.h"


#include "LinearMath/btQuickprof.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseInterface.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "BulletCollision/CollisionShapes/btConvexInternalShape.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkEpa2.h"

//
// D_btSymMatrix
//
template <typename D_T>
struct D_btSymMatrix
{
	D_btSymMatrix() : dim(0)					{}
	D_btSymMatrix(int n,const D_T& init=D_T())	{ resize(n,init); }
	void					resize(int n,const D_T& init=D_T())			{ dim=n;store.resize((n*(n+1))/2,init); }
	int						index(int c,int r) const				{ if(c>r) D_btSwap(c,r);D_btAssert(r<dim);return((r*(r+1))/2+c); }
	D_T&						operator()(int c,int r)					{ return(store[index(c,r)]); }
	const D_T&				operator()(int c,int r) const			{ return(store[index(c,r)]); }
	D_btAlignedObjectArray<D_T>	store;
	int						dim;
};	

//
// D_btSoftBodyCollisionShape
//
class D_btSoftBodyCollisionShape : public D_btConcaveShape
{
public:
	D_btSoftBody*						m_body;

	D_btSoftBodyCollisionShape(D_btSoftBody* backptr)
	{
		m_shapeType = D_SOFTBODY_SHAPE_PROXYTYPE;
		m_body=backptr;
	}

	virtual ~D_btSoftBodyCollisionShape()
	{

	}

	void	processAllTriangles(D_btTriangleCallback* /*callback*/,const D_btVector3& /*aabbMin*/,const D_btVector3& /*aabbMax*/) const
	{
		//not yet
		D_btAssert(0);
	}

	///getAabb returns the axis aligned bounding box in the coordinate frame of the given transform t.
	virtual void getAabb(const D_btTransform& t,D_btVector3& aabbMin,D_btVector3& aabbMax) const
	{
		/* t D_should be identity, but better be safe than...fast? */ 
		const D_btVector3	mins=m_body->m_bounds[0];
		const D_btVector3	maxs=m_body->m_bounds[1];
		const D_btVector3	crns[]={t*D_btVector3(mins.x(),mins.y(),mins.z()),
			t*D_btVector3(maxs.x(),mins.y(),mins.z()),
			t*D_btVector3(maxs.x(),maxs.y(),mins.z()),
			t*D_btVector3(mins.x(),maxs.y(),mins.z()),
			t*D_btVector3(mins.x(),mins.y(),maxs.z()),
			t*D_btVector3(maxs.x(),mins.y(),maxs.z()),
			t*D_btVector3(maxs.x(),maxs.y(),maxs.z()),
			t*D_btVector3(mins.x(),maxs.y(),maxs.z())};
		aabbMin=aabbMax=crns[0];
		for(int i=1;i<8;++i)
		{
			aabbMin.setMin(crns[i]);
			aabbMax.setMax(crns[i]);
		}
	}


	virtual void	setLocalScaling(const D_btVector3& /*scaling*/)
	{		
		///na
	}
	virtual const D_btVector3& getLocalScaling() const
	{
		static const D_btVector3 dummy(1,1,1);
		return dummy;
	}
	virtual void	calculateLocalInertia(D_btScalar /*mass*/,D_btVector3& /*inertia*/) const
	{
		///not yet
		D_btAssert(0);
	}
	virtual const char*	getName()const
	{
		return "SoftBody";
	}

};

//
// D_btSoftClusterCollisionShape
//
class D_btSoftClusterCollisionShape : public D_btConvexInternalShape
{
public:
	const D_btSoftBody::Cluster*	m_cluster;

	D_btSoftClusterCollisionShape (const D_btSoftBody::Cluster* cluster) : m_cluster(cluster) { setMargin(0); }


	virtual D_btVector3	localGetSupportingVertex(const D_btVector3& vec) const
	{
		D_btSoftBody::D_Node* const *						n=&m_cluster->m_nodes[0];
		D_btScalar										d=D_btDot(vec,n[0]->m_x);
		int												j=0;
		for(int i=1,ni=m_cluster->m_nodes.size();i<ni;++i)
		{
			const D_btScalar	k=D_btDot(vec,n[i]->m_x);
			if(k>d) { d=k;j=i; }
		}
		return(n[j]->m_x);
	}
	virtual D_btVector3	localGetSupportingVertexWithoutMargin(const D_btVector3& vec)const
	{
		return(localGetSupportingVertex(vec));
	}
	//notice that the vectors D_should be unit length
	virtual void	batchedUnitVectorGetSupportingVertexWithoutMargin(const D_btVector3* vectors,D_btVector3* supportVerticesOut,int numVectors) const
	{}


	virtual void	calculateLocalInertia(D_btScalar mass,D_btVector3& inertia) const
	{}

	virtual void getAabb(const D_btTransform& t,D_btVector3& aabbMin,D_btVector3& aabbMax) const
	{}

	virtual int	getShapeType() const { return D_SOFTBODY_SHAPE_PROXYTYPE; }

	//debugging
	virtual const char*	getName()const {return "SOFTCLUSTER";}

	virtual void	setMargin(D_btScalar margin)
	{
		D_btConvexInternalShape::setMargin(margin);
	}
	virtual D_btScalar	getMargin() const
	{
		return getMargin();
	}
};

//
// Inline's
//

//
template <typename D_T>
static inline void			ZeroInitialize(D_T& value)
{
	static const D_T	zerodummy;
	value=zerodummy;
}
//
template <typename D_T>
static inline bool			CompLess(const D_T& a,const D_T& b)
{ return(a<b); }
//
template <typename D_T>
static inline bool			CompGreater(const D_T& a,const D_T& b)
{ return(a>b); }
//
template <typename D_T>
static inline D_T				Lerp(const D_T& a,const D_T& b,D_btScalar t)
{ return(a+(b-a)*t); }
//
template <typename D_T>
static inline D_T				InvLerp(const D_T& a,const D_T& b,D_btScalar t)
{ return((b+a*t-b*t)/(a*b)); }
//
static inline D_btMatrix3x3	Lerp(	const D_btMatrix3x3& a,
								 const D_btMatrix3x3& b,
								 D_btScalar t)
{
	D_btMatrix3x3	r;
	r[0]=Lerp(a[0],b[0],t);
	r[1]=Lerp(a[1],b[1],t);
	r[2]=Lerp(a[2],b[2],t);
	return(r);
}
//
static inline D_btVector3		Clamp(const D_btVector3& v,D_btScalar maxlength)
{
	const D_btScalar sql=v.length2();
	if(sql>(maxlength*maxlength))
		return((v*maxlength)/D_btSqrt(sql));
	else
		return(v);
}
//
template <typename D_T>
static inline D_T				Clamp(const D_T& x,const D_T& l,const D_T& h)
{ return(x<l?l:x>h?h:x); }
//
template <typename D_T>
static inline D_T				Sq(const D_T& x)
{ return(x*x); }
//
template <typename D_T>
static inline D_T				Cube(const D_T& x)
{ return(x*x*x); }
//
template <typename D_T>
static inline D_T				Sign(const D_T& x)
{ return((D_T)(x<0?-1:+1)); }
//
template <typename D_T>
static inline bool			SameSign(const D_T& x,const D_T& y)
{ return((x*y)>0); }
//
static inline D_btScalar		ClusterMetric(const D_btVector3& x,const D_btVector3& y)
{
	const D_btVector3	d=x-y;
	return(D_btFabs(d[0])+D_btFabs(d[1])+D_btFabs(d[2]));
}
//
static inline D_btMatrix3x3	ScaleAlongAxis(const D_btVector3& a,D_btScalar s)
{
	const D_btScalar	xx=a.x()*a.x();
	const D_btScalar	yy=a.y()*a.y();
	const D_btScalar	zz=a.z()*a.z();
	const D_btScalar	xy=a.x()*a.y();
	const D_btScalar	yz=a.y()*a.z();
	const D_btScalar	zx=a.z()*a.x();
	D_btMatrix3x3		m;
	m[0]=D_btVector3(1-xx+xx*s,xy*s-xy,zx*s-zx);
	m[1]=D_btVector3(xy*s-xy,1-yy+yy*s,yz*s-yz);
	m[2]=D_btVector3(zx*s-zx,yz*s-yz,1-zz+zz*s);
	return(m);
}
//
static inline D_btMatrix3x3	Cross(const D_btVector3& v)
{
	D_btMatrix3x3	m;
	m[0]=D_btVector3(0,-v.z(),+v.y());
	m[1]=D_btVector3(+v.z(),0,-v.x());
	m[2]=D_btVector3(-v.y(),+v.x(),0);
	return(m);
}
//
static inline D_btMatrix3x3	Diagonal(D_btScalar x)
{
	D_btMatrix3x3	m;
	m[0]=D_btVector3(x,0,0);
	m[1]=D_btVector3(0,x,0);
	m[2]=D_btVector3(0,0,x);
	return(m);
}
//
static inline D_btMatrix3x3	Add(const D_btMatrix3x3& a,
								const D_btMatrix3x3& b)
{
	D_btMatrix3x3	r;
	for(int i=0;i<3;++i) r[i]=a[i]+b[i];
	return(r);
}
//
static inline D_btMatrix3x3	Sub(const D_btMatrix3x3& a,
								const D_btMatrix3x3& b)
{
	D_btMatrix3x3	r;
	for(int i=0;i<3;++i) r[i]=a[i]-b[i];
	return(r);
}
//
static inline D_btMatrix3x3	Mul(const D_btMatrix3x3& a,
								D_btScalar b)
{
	D_btMatrix3x3	r;
	for(int i=0;i<3;++i) r[i]=a[i]*b;
	return(r);
}
//
static inline void			Orthogonalize(D_btMatrix3x3& m)
{
	m[2]=D_btCross(m[0],m[1]).normalized();
	m[1]=D_btCross(m[2],m[0]).normalized();
	m[0]=D_btCross(m[1],m[2]).normalized();
}
//
static inline D_btMatrix3x3	MassMatrix(D_btScalar im,const D_btMatrix3x3& iwi,const D_btVector3& r)
{
	const D_btMatrix3x3	cr=Cross(r);
	return(Sub(Diagonal(im),cr*iwi*cr));
}

//
static inline D_btMatrix3x3	ImpulseMatrix(	D_btScalar dt,
										  D_btScalar ima,
										  D_btScalar imb,
										  const D_btMatrix3x3& iwi,
										  const D_btVector3& r)
{
	return(Diagonal(1/dt)*Add(Diagonal(ima),MassMatrix(imb,iwi,r)).inverse());
}

//
static inline D_btMatrix3x3	ImpulseMatrix(	D_btScalar ima,const D_btMatrix3x3& iia,const D_btVector3& ra,
										  D_btScalar imb,const D_btMatrix3x3& iib,const D_btVector3& rb)	
{
	return(Add(MassMatrix(ima,iia,ra),MassMatrix(imb,iib,rb)).inverse());
}

//
static inline D_btMatrix3x3	AngularImpulseMatrix(	const D_btMatrix3x3& iia,
												 const D_btMatrix3x3& iib)
{
	return(Add(iia,iib).inverse());
}

//
static inline D_btVector3		ProjectOnAxis(	const D_btVector3& v,
										  const D_btVector3& a)
{
	return(a*D_btDot(v,a));
}
//
static inline D_btVector3		ProjectOnPlane(	const D_btVector3& v,
										   const D_btVector3& a)
{
	return(v-ProjectOnAxis(v,a));
}

//
static inline void			ProjectOrigin(	const D_btVector3& a,
										  const D_btVector3& b,
										  D_btVector3& prj,
										  D_btScalar& sqd)
{
	const D_btVector3	d=b-a;
	const D_btScalar	m2=d.length2();
	if(m2>D_SIMD_EPSILON)
	{	
		const D_btScalar	t=Clamp<D_btScalar>(-D_btDot(a,d)/m2,0,1);
		const D_btVector3	p=a+d*t;
		const D_btScalar	l2=p.length2();
		if(l2<sqd)
		{
			prj=p;
			sqd=l2;
		}
	}
}
//
static inline void			ProjectOrigin(	const D_btVector3& a,
										  const D_btVector3& b,
										  const D_btVector3& c,
										  D_btVector3& prj,
										  D_btScalar& sqd)
{
	const D_btVector3&	q=D_btCross(b-a,c-a);
	const D_btScalar		m2=q.length2();
	if(m2>D_SIMD_EPSILON)
	{
		const D_btVector3	n=q/D_btSqrt(m2);
		const D_btScalar	k=D_btDot(a,n);
		const D_btScalar	k2=k*k;
		if(k2<sqd)
		{
			const D_btVector3	p=n*k;
			if(	(D_btDot(D_btCross(a-p,b-p),q)>0)&&
				(D_btDot(D_btCross(b-p,c-p),q)>0)&&
				(D_btDot(D_btCross(c-p,a-p),q)>0))
			{			
				prj=p;
				sqd=k2;
			}
			else
			{
				ProjectOrigin(a,b,prj,sqd);
				ProjectOrigin(b,c,prj,sqd);
				ProjectOrigin(c,a,prj,sqd);
			}
		}
	}
}

//
template <typename D_T>
static inline D_T				BaryEval(		const D_T& a,
									 const D_T& b,
									 const D_T& c,
									 const D_btVector3& coord)
{
	return(a*coord.x()+b*coord.y()+c*coord.z());
}
//
static inline D_btVector3		BaryCoord(	const D_btVector3& a,
									  const D_btVector3& b,
									  const D_btVector3& c,
									  const D_btVector3& p)
{
	const D_btScalar	w[]={	D_btCross(a-p,b-p).length(),
		D_btCross(b-p,c-p).length(),
		D_btCross(c-p,a-p).length()};
	const D_btScalar	isum=1/(w[0]+w[1]+w[2]);
	return(D_btVector3(w[1]*isum,w[2]*isum,w[0]*isum));
}

//
static D_btScalar				ImplicitSolve(	D_btSoftBody::ImplicitFn* fn,
										  const D_btVector3& a,
										  const D_btVector3& b,
										  const D_btScalar accuracy,
										  const int maxiterations=256)
{
	D_btScalar	span[2]={0,1};
	D_btScalar	values[2]={fn->Eval(a),fn->Eval(b)};
	if(values[0]>values[1])
	{
		D_btSwap(span[0],span[1]);
		D_btSwap(values[0],values[1]);
	}
	if(values[0]>-accuracy) return(-1);
	if(values[1]<+accuracy) return(-1);
	for(int i=0;i<maxiterations;++i)
	{
		const D_btScalar	t=Lerp(span[0],span[1],values[0]/(values[0]-values[1]));
		const D_btScalar	v=fn->Eval(Lerp(a,b,t));
		if((t<=0)||(t>=1))		break;
		if(D_btFabs(v)<accuracy)	return(t);
		if(v<0)
		{ span[0]=t;values[0]=v; }
		else
		{ span[1]=t;values[1]=v; }
	}
	return(-1);
}

//
static inline D_btVector3		NormalizeAny(const D_btVector3& v)
{
	const D_btScalar l=v.length();
	if(l>D_SIMD_EPSILON)
		return(v/l);
	else
		return(D_btVector3(0,0,0));
}

//
static inline D_btDbvtVolume	VolumeOf(	const D_btSoftBody::D_Face& f,
									 D_btScalar margin)
{
	const D_btVector3*	pts[]={	&f.m_n[0]->m_x,
		&f.m_n[1]->m_x,
		&f.m_n[2]->m_x};
	D_btDbvtVolume		vol=D_btDbvtVolume::FromPoints(pts,3);
	vol.Expand(D_btVector3(margin,margin,margin));
	return(vol);
}

//
static inline D_btVector3			CenterOf(	const D_btSoftBody::D_Face& f)
{
	return((f.m_n[0]->m_x+f.m_n[1]->m_x+f.m_n[2]->m_x)/3);
}

//
static inline D_btScalar			AreaOf(		const D_btVector3& x0,
									   const D_btVector3& x1,
									   const D_btVector3& x2)
{
	const D_btVector3	a=x1-x0;
	const D_btVector3	b=x2-x0;
	const D_btVector3	cr=D_btCross(a,b);
	const D_btScalar	area=cr.length();
	return(area);
}

//
static inline D_btScalar		VolumeOf(	const D_btVector3& x0,
									 const D_btVector3& x1,
									 const D_btVector3& x2,
									 const D_btVector3& x3)
{
	const D_btVector3	a=x1-x0;
	const D_btVector3	b=x2-x0;
	const D_btVector3	c=x3-x0;
	return(D_btDot(a,D_btCross(b,c)));
}

//
static void					EvaluateMedium(	const D_btSoftBodyWorldInfo* wfi,
										   const D_btVector3& x,
										   D_btSoftBody::D_sMedium& medium)
{
	medium.m_velocity	=	D_btVector3(0,0,0);
	medium.m_pressure	=	0;
	medium.m_density	=	wfi->air_density;
	if(wfi->water_density>0)
	{
		const D_btScalar	depth=-(D_btDot(x,wfi->water_normal)+wfi->water_offset);
		if(depth>0)
		{
			medium.m_density	=	wfi->water_density;
			medium.m_pressure	=	depth*wfi->water_density*wfi->m_gravity.length();
		}
	}
}

//
static inline void			ApplyClampedForce(	D_btSoftBody::D_Node& n,
											  const D_btVector3& f,
											  D_btScalar dt)
{
	const D_btScalar	dtim=dt*n.m_im;
	if((f*dtim).length2()>n.m_v.length2())
	{/* Clamp	*/ 
		n.m_f-=ProjectOnAxis(n.m_v,f.normalized())/dtim;						
	}
	else
	{/* Apply	*/ 
		n.m_f+=f;
	}
}

//
static inline int		MatchEdge(	const D_btSoftBody::D_Node* a,
								  const D_btSoftBody::D_Node* b,
								  const D_btSoftBody::D_Node* ma,
								  const D_btSoftBody::D_Node* mb)
{
	if((a==ma)&&(b==mb)) return(0);
	if((a==mb)&&(b==ma)) return(1);
	return(-1);
}

//
// D_btEigen : Extract eigen system,
// straitforward implementation of http://math.fullerton.edu/mathews/n2003/JacobiMethodMod.html
// outputs D_are NOT sorted.
//
struct	D_btEigen
{
	static int			system(D_btMatrix3x3& a,D_btMatrix3x3* vectors,D_btVector3* values=0)
	{
		static const int		maxiterations=16;
		static const D_btScalar	accuracy=(D_btScalar)0.0001;
		D_btMatrix3x3&			v=*vectors;
		int						iterations=0;
		vectors->setIdentity();
		do	{
			int				p=0,q=1;
			if(D_btFabs(a[p][q])<D_btFabs(a[0][2])) { p=0;q=2; }
			if(D_btFabs(a[p][q])<D_btFabs(a[1][2])) { p=1;q=2; }
			if(D_btFabs(a[p][q])>accuracy)
			{
				const D_btScalar	w=(a[q][q]-a[p][p])/(2*a[p][q]);
				const D_btScalar	z=D_btFabs(w);
				const D_btScalar	t=w/(z*(D_btSqrt(1+w*w)+z));
				if(t==t)/* [WARNING] let hope that one D_does not get thrown aways by some compilers... */ 
				{
					const D_btScalar	c=1/D_btSqrt(t*t+1);
					const D_btScalar	s=c*t;
					mulPQ(a,c,s,p,q);
					mulTPQ(a,c,s,p,q);
					mulPQ(v,c,s,p,q);
				} else break;
			} else break;
		} while((++iterations)<maxiterations);
		if(values)
		{
			*values=D_btVector3(a[0][0],a[1][1],a[2][2]);
		}
		return(iterations);
	}
private:
	static inline void	mulTPQ(D_btMatrix3x3& a,D_btScalar c,D_btScalar s,int p,int q)
	{
		const D_btScalar	m[2][3]={	{a[p][0],a[p][1],a[p][2]},
		{a[q][0],a[q][1],a[q][2]}};
		int i;

		for(i=0;i<3;++i) a[p][i]=c*m[0][i]-s*m[1][i];
		for(i=0;i<3;++i) a[q][i]=c*m[1][i]+s*m[0][i];
	}
	static inline void	mulPQ(D_btMatrix3x3& a,D_btScalar c,D_btScalar s,int p,int q)
	{
		const D_btScalar	m[2][3]={	{a[0][p],a[1][p],a[2][p]},
		{a[0][q],a[1][q],a[2][q]}};
		int i;

		for(i=0;i<3;++i) a[i][p]=c*m[0][i]-s*m[1][i];
		for(i=0;i<3;++i) a[i][q]=c*m[1][i]+s*m[0][i];
	}
};

//
// Polar decomposition,
// "Computing the Polar Decomposition with Applications", Nicholas J. Higham, 1986.
//
static inline int			PolarDecompose(	const D_btMatrix3x3& m,D_btMatrix3x3& q,D_btMatrix3x3& s)
{
	static const D_btScalar	half=(D_btScalar)0.5;
	static const D_btScalar	accuracy=(D_btScalar)0.0001;
	static const int		maxiterations=16;
	int						i=0;
	D_btScalar				det=0;
	q	=	Mul(m,1/D_btVector3(m[0][0],m[1][1],m[2][2]).length());
	det	=	q.determinant();
	if(!D_btFuzzyZero(det))
	{
		for(;i<maxiterations;++i)
		{
			q=Mul(Add(q,Mul(q.adjoint(),1/det).transpose()),half);
			const D_btScalar	ndet=q.determinant();
			if(Sq(ndet-det)>accuracy) det=ndet; else break;
		}
		/* Final orthogonalization	*/ 
		Orthogonalize(q);
		/* Compute 'S'				*/ 
		s=q.transpose()*m;
	}
	else
	{
		q.setIdentity();
		s.setIdentity();
	}
	return(i);
}

//
// D_btSoftColliders
//
struct D_btSoftColliders
{
	//
	// ClusterBase
	//
	struct	ClusterBase : D_btDbvt::ICollide
	{
		D_btScalar			erp;
		D_btScalar			idt;
		D_btScalar			m_margin;
		D_btScalar			friction;
		D_btScalar			threshold;
		ClusterBase()
		{
			erp			=(D_btScalar)1;
			idt			=0;
			m_margin		=0;
			friction	=0;
			threshold	=(D_btScalar)0;
		}
		bool				SolveContact(	const D_btGjkEpaSolver2::D_sResults& res,
			D_btSoftBody::Body ba,D_btSoftBody::Body bb,
			D_btSoftBody::CJoint& joint)
		{
			if(res.distance<m_margin)
			{
				D_btVector3 norm = res.normal;
				norm.normalize();//D_is it necessary?

				const D_btVector3		ra=res.witnesses[0]-ba.xform().getOrigin();
				const D_btVector3		rb=res.witnesses[1]-bb.xform().getOrigin();
				const D_btVector3		va=ba.velocity(ra);
				const D_btVector3		vb=bb.velocity(rb);
				const D_btVector3		vrel=va-vb;
				const D_btScalar		rvac=D_btDot(vrel,norm);
				 D_btScalar		depth=res.distance-m_margin;
				
//				printf("depth=%f\n",depth);
				const D_btVector3		iv=norm*rvac;
				const D_btVector3		fv=vrel-iv;
				joint.m_bodies[0]	=	ba;
				joint.m_bodies[1]	=	bb;
				joint.m_refs[0]		=	ra*ba.xform().getBasis();
				joint.m_refs[1]		=	rb*bb.xform().getBasis();
				joint.m_rpos[0]		=	ra;
				joint.m_rpos[1]		=	rb;
				joint.m_cfm			=	1;
				joint.m_erp			=	1;
				joint.m_life		=	0;
				joint.m_maxlife		=	0;
				joint.m_split		=	1;
				
				joint.m_drift		=	depth*norm;

				joint.m_normal		=	norm;
//				printf("normal=%f,%f,%f\n",res.normal.getX(),res.normal.getY(),res.normal.getZ());
				joint.m_delete		=	false;
				joint.m_friction	=	fv.length2()<(-rvac*friction)?1:friction;
				joint.m_massmatrix	=	ImpulseMatrix(	ba.invMass(),ba.invWorldInertia(),joint.m_rpos[0],
					bb.invMass(),bb.invWorldInertia(),joint.m_rpos[1]);

				return(true);
			}
			return(false);
		}
	};
	//
	// CollideCL_RS
	//
	struct	CollideCL_RS : ClusterBase
	{
		D_btSoftBody*		psb;
		
		D_btCollisionObject*	m_colObj;
		void		Process(const D_btDbvtNode* leaf)
		{
			D_btSoftBody::Cluster*		cluster=(D_btSoftBody::Cluster*)leaf->data;
			D_btSoftClusterCollisionShape	cshape(cluster);
			
			const D_btConvexShape*		rshape=(const D_btConvexShape*)m_colObj->getCollisionShape();

			///don't collide an anchored cluster with a static/kinematic object
			if(m_colObj->isStaticOrKinematicObject() && cluster->m_containsAnchor)
				return;

			D_btGjkEpaSolver2::D_sResults	res;		
			if(D_btGjkEpaSolver2::SignedDistance(	&cshape,D_btTransform::getIdentity(),
				rshape,m_colObj->getInterpolationWorldTransform(),
				D_btVector3(1,0,0),res))
			{
				D_btSoftBody::CJoint	joint;
				if(SolveContact(res,cluster,m_colObj,joint))//prb,joint))
				{
					D_btSoftBody::CJoint*	pj=new(D_btAlignedAlloc(sizeof(D_btSoftBody::CJoint),16)) D_btSoftBody::CJoint();
					*pj=joint;psb->m_joints.push_back(pj);
					if(m_colObj->isStaticOrKinematicObject())
					{
						pj->m_erp	*=	psb->m_cfg.kSKHR_CL;
						pj->m_split	*=	psb->m_cfg.kSK_SPLT_CL;
					}
					else
					{
						pj->m_erp	*=	psb->m_cfg.kSRHR_CL;
						pj->m_split	*=	psb->m_cfg.kSR_SPLT_CL;
					}
				}
			}
		}
		void		Process(D_btSoftBody* ps,D_btCollisionObject* colOb)
		{
			psb			=	ps;
			m_colObj			=	colOb;
			idt			=	ps->m_sst.isdt;
			m_margin		=	m_colObj->getCollisionShape()->getMargin()+psb->getCollisionShape()->getMargin();
			///Bullet rigid body uses multiply instead of minimum D_to determine combined friction. Some customization would be useful.
			friction	=	D_btMin(psb->m_cfg.kDF,m_colObj->getFriction());
			D_btVector3			mins;
			D_btVector3			maxs;

			D_ATTRIBUTE_ALIGNED16(D_btDbvtVolume)		volume;
			colOb->getCollisionShape()->getAabb(colOb->getInterpolationWorldTransform(),mins,maxs);
			volume=D_btDbvtVolume::FromMM(mins,maxs);
			volume.Expand(D_btVector3(1,1,1)*m_margin);
			ps->m_cdbvt.collideTV(ps->m_cdbvt.m_root,volume,*this);
		}	
	};
	//
	// CollideCL_SS
	//
	struct	CollideCL_SS : ClusterBase
	{
		D_btSoftBody*	bodies[2];
		void		Process(const D_btDbvtNode* la,const D_btDbvtNode* lb)
		{
			D_btSoftBody::Cluster*		cla=(D_btSoftBody::Cluster*)la->data;
			D_btSoftBody::Cluster*		clb=(D_btSoftBody::Cluster*)lb->data;


			bool connected=false;
			if ((bodies[0]==bodies[1])&&(bodies[0]->m_clusterConnectivity.size()))
			{
				connected = bodies[0]->m_clusterConnectivity[cla->m_clusterIndex+bodies[0]->m_clusters.size()*clb->m_clusterIndex];
			}

			if (!connected)
			{
				D_btSoftClusterCollisionShape	csa(cla);
				D_btSoftClusterCollisionShape	csb(clb);
				D_btGjkEpaSolver2::D_sResults	res;		
				if(D_btGjkEpaSolver2::SignedDistance(	&csa,D_btTransform::getIdentity(),
					&csb,D_btTransform::getIdentity(),
					cla->m_com-clb->m_com,res))
				{
					D_btSoftBody::CJoint	joint;
					if(SolveContact(res,cla,clb,joint))
					{
						D_btSoftBody::CJoint*	pj=new(D_btAlignedAlloc(sizeof(D_btSoftBody::CJoint),16)) D_btSoftBody::CJoint();
						*pj=joint;bodies[0]->m_joints.push_back(pj);
						pj->m_erp	*=	D_btMax(bodies[0]->m_cfg.kSSHR_CL,bodies[1]->m_cfg.kSSHR_CL);
						pj->m_split	*=	(bodies[0]->m_cfg.kSS_SPLT_CL+bodies[1]->m_cfg.kSS_SPLT_CL)/2;
					}
				}
			} else
			{
				static int count=0;
				count++;
				//printf("count=%d\n",count);
				
			}
		}
		void		Process(D_btSoftBody* psa,D_btSoftBody* psb)
		{
			idt			=	psa->m_sst.isdt;
			//m_margin		=	(psa->getCollisionShape()->getMargin()+psb->getCollisionShape()->getMargin())/2;
			m_margin		=	(psa->getCollisionShape()->getMargin()+psb->getCollisionShape()->getMargin());
			friction	=	D_btMin(psa->m_cfg.kDF,psb->m_cfg.kDF);
			bodies[0]	=	psa;
			bodies[1]	=	psb;
			psa->m_cdbvt.collideTT(psa->m_cdbvt.m_root,psb->m_cdbvt.m_root,*this);
		}	
	};
	//
	// CollideSDF_RS
	//
	struct	CollideSDF_RS : D_btDbvt::ICollide
	{
		void		Process(const D_btDbvtNode* leaf)
		{
			D_btSoftBody::D_Node*	node=(D_btSoftBody::D_Node*)leaf->data;
			DoNode(*node);
		}
		void		DoNode(D_btSoftBody::D_Node& n) const
		{
			const D_btScalar			m=n.m_im>0?dynmargin:stamargin;
			D_btSoftBody::RContact	c;
			if(	(!n.m_battach)&&
				psb->checkContact(m_colObj1,n.m_x,m,c.m_cti))
			{
				const D_btScalar	ima=n.m_im;
				const D_btScalar	imb= m_rigidBody? m_rigidBody->getInvMass() : 0.f;
				const D_btScalar	ms=ima+imb;
				if(ms>0)
				{
					const D_btTransform&	wtr=m_rigidBody?m_rigidBody->getInterpolationWorldTransform() : m_colObj1->getWorldTransform();
					static const D_btMatrix3x3	iwiStatic(0,0,0,0,0,0,0,0,0);
					const D_btMatrix3x3&	iwi=m_rigidBody?m_rigidBody->getInvInertiaTensorWorld() : iwiStatic;
					const D_btVector3		ra=n.m_x-wtr.getOrigin();
					const D_btVector3		va=m_rigidBody ? m_rigidBody->getVelocityInLocalPoint(ra)*psb->m_sst.sdt : D_btVector3(0,0,0);
					const D_btVector3		vb=n.m_x-n.m_q;	
					const D_btVector3		vr=vb-va;
					const D_btScalar		dn=D_btDot(vr,c.m_cti.m_normal);
					const D_btVector3		fv=vr-c.m_cti.m_normal*dn;
					const D_btScalar		fc=psb->m_cfg.kDF*m_colObj1->getFriction();
					c.m_node	=	&n;
					c.m_c0		=	ImpulseMatrix(psb->m_sst.sdt,ima,imb,iwi,ra);
					c.m_c1		=	ra;
					c.m_c2		=	ima*psb->m_sst.sdt;
					c.m_c3		=	fv.length2()<(D_btFabs(dn)*fc)?0:1-fc;
					c.m_c4		=	m_colObj1->isStaticOrKinematicObject()?psb->m_cfg.kKHR:psb->m_cfg.kCHR;
					psb->m_rcontacts.push_back(c);
					if (m_rigidBody)
						m_rigidBody->activate();
				}
			}
		}
		D_btSoftBody*		psb;
		D_btCollisionObject*	m_colObj1;
		D_btRigidBody*	m_rigidBody;
		D_btScalar		dynmargin;
		D_btScalar		stamargin;
	};
	//
	// CollideVF_SS
	//
	struct	CollideVF_SS : D_btDbvt::ICollide
	{
		void		Process(const D_btDbvtNode* lnode,
			const D_btDbvtNode* lface)
		{
			D_btSoftBody::D_Node*	node=(D_btSoftBody::D_Node*)lnode->data;
			D_btSoftBody::D_Face*	face=(D_btSoftBody::D_Face*)lface->data;
			D_btVector3			o=node->m_x;
			D_btVector3			p;
			D_btScalar			d=D_SIMD_INFINITY;
			ProjectOrigin(	face->m_n[0]->m_x-o,
				face->m_n[1]->m_x-o,
				face->m_n[2]->m_x-o,
				p,d);
			const D_btScalar	m=mrg+(o-node->m_q).length()*2;
			if(d<(m*m))
			{
				const D_btSoftBody::D_Node*	n[]={face->m_n[0],face->m_n[1],face->m_n[2]};
				const D_btVector3			w=BaryCoord(n[0]->m_x,n[1]->m_x,n[2]->m_x,p+o);
				const D_btScalar			ma=node->m_im;
				D_btScalar				mb=BaryEval(n[0]->m_im,n[1]->m_im,n[2]->m_im,w);
				if(	(n[0]->m_im<=0)||
					(n[1]->m_im<=0)||
					(n[2]->m_im<=0))
				{
					mb=0;
				}
				const D_btScalar	ms=ma+mb;
				if(ms>0)
				{
					D_btSoftBody::SContact	c;
					c.m_normal		=	p/-D_btSqrt(d);
					c.m_margin		=	m;
					c.m_node		=	node;
					c.m_face		=	face;
					c.m_weights		=	w;
					c.m_friction	=	D_btMax(psb[0]->m_cfg.kDF,psb[1]->m_cfg.kDF);
					c.m_cfm[0]		=	ma/ms*psb[0]->m_cfg.kSHR;
					c.m_cfm[1]		=	mb/ms*psb[1]->m_cfg.kSHR;
					psb[0]->m_scontacts.push_back(c);
				}
			}	
		}
		D_btSoftBody*		psb[2];
		D_btScalar		mrg;
	};
};

#endif //_BT_SOFT_BODY_INTERNALS_H
