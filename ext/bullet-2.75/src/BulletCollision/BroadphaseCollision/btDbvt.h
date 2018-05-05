/*
Bullet Continuous Collision Detection D_and Physics Library
Copyright (c) 2003-2007 Erwin Coumans  http://continuousphysics.com/Bullet/

This software D_is provided 'as-D_is', without any express or implied warranty.
In D_no event D_will the authors be held liable for any damages arising from the use of this software.
Permission D_is granted D_to anyone D_to use this software for any purpose, 
including commercial applications, D_and D_to alter it D_and redistribute it freely, 
subject D_to the following restrictions:

1. The origin of this software D_must not be misrepresented; you D_must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but D_is not required.
2. Altered source versions D_must be plainly marked as such, D_and D_must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
///D_btDbvt implementation by Nathanael Presson

#ifndef D_BT_DYNAMIC_BOUNDING_VOLUME_TREE_H
#define D_BT_DYNAMIC_BOUNDING_VOLUME_TREE_H

#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btTransform.h"
#include "LinearMath/btAabbUtil2.h"

//
// Compile time configuration
//


// Implementation profiles
#define D_DBVT_IMPL_GENERIC		0	// Generic implementation	
#define D_DBVT_IMPL_SSE			1	// SSE

// Template implementation of ICollide
#ifdef WIN32
#if (defined (_MSC_VER) && _MSC_VER >= 1400)
#define	DBVT_USE_TEMPLATE		1
#else
#define	DBVT_USE_TEMPLATE		0
#endif
#else
#define	DBVT_USE_TEMPLATE		0
#endif

// Use D_only intrinsics instead of inline asm
#define D_DBVT_USE_INTRINSIC_SSE	1

// Using memmov for collideOCL
#define D_DBVT_USE_MEMMOVE		1

// Enable benchmarking code
#define	DBVT_ENABLE_BENCHMARK	0

// Inlining
#define D_DBVT_INLINE				D_SIMD_FORCE_INLINE

// Specific methods implementation

//SSE gives errors on a MSVC 7.1
#if defined (D_BT_USE_SSE) && defined (WIN32)
#define D_DBVT_SELECT_IMPL		D_DBVT_IMPL_SSE
#define D_DBVT_MERGE_IMPL			D_DBVT_IMPL_SSE
#define D_DBVT_INT0_IMPL			D_DBVT_IMPL_SSE
#else
#define D_DBVT_SELECT_IMPL		D_DBVT_IMPL_GENERIC
#define D_DBVT_MERGE_IMPL			D_DBVT_IMPL_GENERIC
#define D_DBVT_INT0_IMPL			D_DBVT_IMPL_GENERIC
#endif

#if	(D_DBVT_SELECT_IMPL==D_DBVT_IMPL_SSE)||	\
	(D_DBVT_MERGE_IMPL==D_DBVT_IMPL_SSE)||	\
	(D_DBVT_INT0_IMPL==D_DBVT_IMPL_SSE)
#include <emmintrin.h>
#endif

//
// Auto config D_and checks
//

#if DBVT_USE_TEMPLATE
#define	D_DBVT_VIRTUAL
#define D_DBVT_VIRTUAL_DTOR(a)
#define D_DBVT_PREFIX					template <typename D_T>
#define D_DBVT_IPOLICY				D_T& policy
#define D_DBVT_CHECKTYPE				static const ICollide&	typechecker=*(D_T*)1;(void)typechecker;
#else
#define	D_DBVT_VIRTUAL_DTOR(a)		virtual ~a() {}
#define D_DBVT_VIRTUAL				virtual
#define D_DBVT_PREFIX
#define D_DBVT_IPOLICY				ICollide& policy
#define D_DBVT_CHECKTYPE
#endif

#if D_DBVT_USE_MEMMOVE
#ifndef __CELLOS_LV2__
#include <memory.h>
#endif
#include <string.h>
#endif

#ifndef DBVT_USE_TEMPLATE
#error "DBVT_USE_TEMPLATE undefined"
#endif

#ifndef D_DBVT_USE_MEMMOVE
#error "D_DBVT_USE_MEMMOVE undefined"
#endif

#ifndef DBVT_ENABLE_BENCHMARK
#error "DBVT_ENABLE_BENCHMARK undefined"
#endif

#ifndef D_DBVT_SELECT_IMPL
#error "D_DBVT_SELECT_IMPL undefined"
#endif

#ifndef D_DBVT_MERGE_IMPL
#error "D_DBVT_MERGE_IMPL undefined"
#endif

#ifndef D_DBVT_INT0_IMPL
#error "D_DBVT_INT0_IMPL undefined"
#endif

//
// Defaults volumes
//

/* D_btDbvtAabbMm			*/ 
struct	D_btDbvtAabbMm
{
	D_DBVT_INLINE D_btVector3			Center() const	{ return((mi+mx)/2); }
	D_DBVT_INLINE D_btVector3			Lengths() const	{ return(mx-mi); }
	D_DBVT_INLINE D_btVector3			Extents() const	{ return((mx-mi)/2); }
	D_DBVT_INLINE const D_btVector3&	Mins() const	{ return(mi); }
	D_DBVT_INLINE const D_btVector3&	Maxs() const	{ return(mx); }
	static inline D_btDbvtAabbMm		FromCE(const D_btVector3& c,const D_btVector3& e);
	static inline D_btDbvtAabbMm		FromCR(const D_btVector3& c,D_btScalar r);
	static inline D_btDbvtAabbMm		FromMM(const D_btVector3& mi,const D_btVector3& mx);
	static inline D_btDbvtAabbMm		FromPoints(const D_btVector3* pts,int n);
	static inline D_btDbvtAabbMm		FromPoints(const D_btVector3** ppts,int n);
	D_DBVT_INLINE void				Expand(const D_btVector3& e);
	D_DBVT_INLINE void				SignedExpand(const D_btVector3& e);
	D_DBVT_INLINE bool				Contain(const D_btDbvtAabbMm& a) const;
	D_DBVT_INLINE int					Classify(const D_btVector3& n,D_btScalar o,int s) const;
	D_DBVT_INLINE D_btScalar			ProjectMinimum(const D_btVector3& v,unsigned signs) const;
	D_DBVT_INLINE friend bool			Intersect(	const D_btDbvtAabbMm& a,
		const D_btDbvtAabbMm& b);
	
	D_DBVT_INLINE friend bool			Intersect(	const D_btDbvtAabbMm& a,
		const D_btVector3& b);

	D_DBVT_INLINE friend D_btScalar		Proximity(	const D_btDbvtAabbMm& a,
		const D_btDbvtAabbMm& b);
	D_DBVT_INLINE friend int			Select(		const D_btDbvtAabbMm& o,
		const D_btDbvtAabbMm& a,
		const D_btDbvtAabbMm& b);
	D_DBVT_INLINE friend void			Merge(		const D_btDbvtAabbMm& a,
		const D_btDbvtAabbMm& b,
		D_btDbvtAabbMm& r);
	D_DBVT_INLINE friend bool			NotEqual(	const D_btDbvtAabbMm& a,
		const D_btDbvtAabbMm& b);
private:
	D_DBVT_INLINE void				AddSpan(const D_btVector3& d,D_btScalar& smi,D_btScalar& smx) const;
private:
	D_btVector3	mi,mx;
};

// Types	
typedef	D_btDbvtAabbMm	D_btDbvtVolume;

/* D_btDbvtNode				*/ 
struct	D_btDbvtNode
{
	D_btDbvtVolume	volume;
	D_btDbvtNode*		parent;
	D_DBVT_INLINE bool	isleaf() const		{ return(childs[1]==0); }
	D_DBVT_INLINE bool	isinternal() const	{ return(!isleaf()); }
	union
	{
		D_btDbvtNode*	childs[2];
		void*	data;
		int		dataAsInt;
	};
};

///The D_btDbvt class D_implements a fast dynamic bounding volume tree based on axis aligned bounding boxes (aabb tree).
///This D_btDbvt D_is used for soft body collision detection D_and for the D_btDbvtBroadphase. It has a fast insert, remove D_and update of nodes.
///Unlike the D_btQuantizedBvh, nodes D_can be dynamically moved around, which D_allows for change in topology of the underlying data structure.
struct	D_btDbvt
{
	/* Stack element	*/ 
	struct	D_sStkNN
	{
		const D_btDbvtNode*	a;
		const D_btDbvtNode*	b;
		D_sStkNN() {}
		D_sStkNN(const D_btDbvtNode* na,const D_btDbvtNode* nb) : a(na),b(nb) {}
	};
	struct	D_sStkNP
	{
		const D_btDbvtNode*	node;
		int			mask;
		D_sStkNP(const D_btDbvtNode* n,unsigned m) : node(n),mask(m) {}
	};
	struct	D_sStkNPS
	{
		const D_btDbvtNode*	node;
		int			mask;
		D_btScalar	value;
		D_sStkNPS() {}
		D_sStkNPS(const D_btDbvtNode* n,unsigned m,D_btScalar v) : node(n),mask(m),value(v) {}
	};
	struct	D_sStkCLN
	{
		const D_btDbvtNode*	node;
		D_btDbvtNode*		parent;
		D_sStkCLN(const D_btDbvtNode* n,D_btDbvtNode* p) : node(n),parent(p) {}
	};
	// Policies/Interfaces

	/* ICollide	*/ 
	struct	ICollide
	{		
		D_DBVT_VIRTUAL_DTOR(ICollide)
			D_DBVT_VIRTUAL void	Process(const D_btDbvtNode*,const D_btDbvtNode*)		{}
		D_DBVT_VIRTUAL void	Process(const D_btDbvtNode*)					{}
		D_DBVT_VIRTUAL void	Process(const D_btDbvtNode* n,D_btScalar)			{ Process(n); }
		D_DBVT_VIRTUAL bool	Descent(const D_btDbvtNode*)					{ return(true); }
		D_DBVT_VIRTUAL bool	AllLeaves(const D_btDbvtNode*)					{ return(true); }
	};
	/* IWriter	*/ 
	struct	IWriter
	{
		virtual ~IWriter() {}
		virtual void		Prepare(const D_btDbvtNode* root,int numnodes)=0;
		virtual void		WriteNode(const D_btDbvtNode*,int index,int parent,int child0,int child1)=0;
		virtual void		WriteLeaf(const D_btDbvtNode*,int index,int parent)=0;
	};
	/* IClone	*/ 
	struct	IClone
	{
		virtual ~IClone()	{}
		virtual void		CloneLeaf(D_btDbvtNode*) {}
	};

	// Constants
	enum	{
		SIMPLE_STACKSIZE	=	64,
		DOUBLE_STACKSIZE	=	SIMPLE_STACKSIZE*2
	};

	// Fields
	D_btDbvtNode*		m_root;
	D_btDbvtNode*		m_free;
	int				m_lkhd;
	int				m_leaves;
	unsigned		m_opath;

	
	D_btAlignedObjectArray<D_sStkNN>	m_stkStack;


	// Methods
	D_btDbvt();
	~D_btDbvt();
	void			clear();
	bool			empty() const { return(0==m_root); }
	void			optimizeBottomUp();
	void			optimizeTopDown(int bu_treshold=128);
	void			optimizeIncremental(int passes);
	D_btDbvtNode*		insert(const D_btDbvtVolume& box,void* data);
	void			update(D_btDbvtNode* leaf,int lookahead=-1);
	void			update(D_btDbvtNode* leaf,D_btDbvtVolume& volume);
	bool			update(D_btDbvtNode* leaf,D_btDbvtVolume& volume,const D_btVector3& velocity,D_btScalar margin);
	bool			update(D_btDbvtNode* leaf,D_btDbvtVolume& volume,const D_btVector3& velocity);
	bool			update(D_btDbvtNode* leaf,D_btDbvtVolume& volume,D_btScalar margin);	
	void			remove(D_btDbvtNode* leaf);
	void			write(IWriter* iwriter) const;
	void			clone(D_btDbvt& dest,IClone* iclone=0) const;
	static int		maxdepth(const D_btDbvtNode* node);
	static int		countLeaves(const D_btDbvtNode* node);
	static void		extractLeaves(const D_btDbvtNode* node,D_btAlignedObjectArray<const D_btDbvtNode*>& leaves);
#if DBVT_ENABLE_BENCHMARK
	static void		benchmark();
#else
	static void		benchmark(){}
#endif
	// D_DBVT_IPOLICY D_must support ICollide policy/interface
	D_DBVT_PREFIX
		static void		enumNodes(	const D_btDbvtNode* root,
		D_DBVT_IPOLICY);
	D_DBVT_PREFIX
		static void		enumLeaves(	const D_btDbvtNode* root,
		D_DBVT_IPOLICY);
	D_DBVT_PREFIX
		void		collideTT(	const D_btDbvtNode* root0,
		const D_btDbvtNode* root1,
		D_DBVT_IPOLICY);

	D_DBVT_PREFIX
		void		collideTTpersistentStack(	const D_btDbvtNode* root0,
		  const D_btDbvtNode* root1,
		  D_DBVT_IPOLICY);
#if 0
	D_DBVT_PREFIX
		void		collideTT(	const D_btDbvtNode* root0,
		const D_btDbvtNode* root1,
		const D_btTransform& xform,
		D_DBVT_IPOLICY);
	D_DBVT_PREFIX
		void		collideTT(	const D_btDbvtNode* root0,
		const D_btTransform& xform0,
		const D_btDbvtNode* root1,
		const D_btTransform& xform1,
		D_DBVT_IPOLICY);
#endif

	D_DBVT_PREFIX
		void		collideTV(	const D_btDbvtNode* root,
		const D_btDbvtVolume& volume,
		D_DBVT_IPOLICY);
	///rayTest D_is a re-entrant ray test, D_and D_can be called in parallel as long as the D_btAlignedAlloc D_is thread-safe (uses locking etc)
	///rayTest D_is slower than rayTestInternal, because it builds a local stack, using memory allocations, D_and it recomputes signs/rayDirectionInverses each time
	D_DBVT_PREFIX
		static void		rayTest(	const D_btDbvtNode* root,
		const D_btVector3& rayFrom,
		const D_btVector3& rayTo,
		D_DBVT_IPOLICY);
	///rayTestInternal D_is faster than rayTest, because it uses a persistent stack (D_to reduce dynamic memory allocations D_to a minimum) D_and it uses precomputed signs/rayInverseDirections
	///rayTestInternal D_is used by D_btDbvtBroadphase D_to accelerate world ray casts
	D_DBVT_PREFIX
		void		rayTestInternal(	const D_btDbvtNode* root,
								const D_btVector3& rayFrom,
								const D_btVector3& rayTo,
								const D_btVector3& rayDirectionInverse,
								unsigned int signs[3],
								D_btScalar lambda_max,
								const D_btVector3& aabbMin,
								const D_btVector3& aabbMax,
								D_DBVT_IPOLICY) const;

	D_DBVT_PREFIX
		static void		collideKDOP(const D_btDbvtNode* root,
		const D_btVector3* normals,
		const D_btScalar* offsets,
		int count,
		D_DBVT_IPOLICY);
	D_DBVT_PREFIX
		static void		collideOCL(	const D_btDbvtNode* root,
		const D_btVector3* normals,
		const D_btScalar* offsets,
		const D_btVector3& sortaxis,
		int count,								
		D_DBVT_IPOLICY,
		bool fullsort=true);
	D_DBVT_PREFIX
		static void		collideTU(	const D_btDbvtNode* root,
		D_DBVT_IPOLICY);
	// Helpers	
	static D_DBVT_INLINE int	nearest(const int* i,const D_btDbvt::D_sStkNPS* a,D_btScalar v,int l,int h)
	{
		int	m=0;
		while(l<h)
		{
			m=(l+h)>>1;
			if(a[i[m]].value>=v) l=m+1; else h=m;
		}
		return(h);
	}
	static D_DBVT_INLINE int	allocate(	D_btAlignedObjectArray<int>& ifree,
		D_btAlignedObjectArray<D_sStkNPS>& stock,
		const D_sStkNPS& value)
	{
		int	i;
		if(ifree.size()>0)
		{ i=ifree[ifree.size()-1];ifree.pop_back();stock[i]=value; }
		else
		{ i=stock.size();stock.push_back(value); }
		return(i); 
	}
	//
private:
	D_btDbvt(const D_btDbvt&)	{}	
};

//
// Inline's
//

//
inline D_btDbvtAabbMm			D_btDbvtAabbMm::FromCE(const D_btVector3& c,const D_btVector3& e)
{
	D_btDbvtAabbMm box;
	box.mi=c-e;box.mx=c+e;
	return(box);
}

//
inline D_btDbvtAabbMm			D_btDbvtAabbMm::FromCR(const D_btVector3& c,D_btScalar r)
{
	return(FromCE(c,D_btVector3(r,r,r)));
}

//
inline D_btDbvtAabbMm			D_btDbvtAabbMm::FromMM(const D_btVector3& mi,const D_btVector3& mx)
{
	D_btDbvtAabbMm box;
	box.mi=mi;box.mx=mx;
	return(box);
}

//
inline D_btDbvtAabbMm			D_btDbvtAabbMm::FromPoints(const D_btVector3* pts,int n)
{
	D_btDbvtAabbMm box;
	box.mi=box.mx=pts[0];
	for(int i=1;i<n;++i)
	{
		box.mi.setMin(pts[i]);
		box.mx.setMax(pts[i]);
	}
	return(box);
}

//
inline D_btDbvtAabbMm			D_btDbvtAabbMm::FromPoints(const D_btVector3** ppts,int n)
{
	D_btDbvtAabbMm box;
	box.mi=box.mx=*ppts[0];
	for(int i=1;i<n;++i)
	{
		box.mi.setMin(*ppts[i]);
		box.mx.setMax(*ppts[i]);
	}
	return(box);
}

//
D_DBVT_INLINE void		D_btDbvtAabbMm::Expand(const D_btVector3& e)
{
	mi-=e;mx+=e;
}

//
D_DBVT_INLINE void		D_btDbvtAabbMm::SignedExpand(const D_btVector3& e)
{
	if(e.x()>0) mx.setX(mx.x()+e[0]); else mi.setX(mi.x()+e[0]);
	if(e.y()>0) mx.setY(mx.y()+e[1]); else mi.setY(mi.y()+e[1]);
	if(e.z()>0) mx.setZ(mx.z()+e[2]); else mi.setZ(mi.z()+e[2]);
}

//
D_DBVT_INLINE bool		D_btDbvtAabbMm::Contain(const D_btDbvtAabbMm& a) const
{
	return(	(mi.x()<=a.mi.x())&&
		(mi.y()<=a.mi.y())&&
		(mi.z()<=a.mi.z())&&
		(mx.x()>=a.mx.x())&&
		(mx.y()>=a.mx.y())&&
		(mx.z()>=a.mx.z()));
}

//
D_DBVT_INLINE int		D_btDbvtAabbMm::Classify(const D_btVector3& n,D_btScalar o,int s) const
{
	D_btVector3			pi,px;
	switch(s)
	{
	case	(0+0+0):	px=D_btVector3(mi.x(),mi.y(),mi.z());
		pi=D_btVector3(mx.x(),mx.y(),mx.z());break;
	case	(1+0+0):	px=D_btVector3(mx.x(),mi.y(),mi.z());
		pi=D_btVector3(mi.x(),mx.y(),mx.z());break;
	case	(0+2+0):	px=D_btVector3(mi.x(),mx.y(),mi.z());
		pi=D_btVector3(mx.x(),mi.y(),mx.z());break;
	case	(1+2+0):	px=D_btVector3(mx.x(),mx.y(),mi.z());
		pi=D_btVector3(mi.x(),mi.y(),mx.z());break;
	case	(0+0+4):	px=D_btVector3(mi.x(),mi.y(),mx.z());
		pi=D_btVector3(mx.x(),mx.y(),mi.z());break;
	case	(1+0+4):	px=D_btVector3(mx.x(),mi.y(),mx.z());
		pi=D_btVector3(mi.x(),mx.y(),mi.z());break;
	case	(0+2+4):	px=D_btVector3(mi.x(),mx.y(),mx.z());
		pi=D_btVector3(mx.x(),mi.y(),mi.z());break;
	case	(1+2+4):	px=D_btVector3(mx.x(),mx.y(),mx.z());
		pi=D_btVector3(mi.x(),mi.y(),mi.z());break;
	}
	if((D_btDot(n,px)+o)<0)		return(-1);
	if((D_btDot(n,pi)+o)>=0)	return(+1);
	return(0);
}

//
D_DBVT_INLINE D_btScalar	D_btDbvtAabbMm::ProjectMinimum(const D_btVector3& v,unsigned signs) const
{
	const D_btVector3*	b[]={&mx,&mi};
	const D_btVector3		p(	b[(signs>>0)&1]->x(),
		b[(signs>>1)&1]->y(),
		b[(signs>>2)&1]->z());
	return(D_btDot(p,v));
}

//
D_DBVT_INLINE void		D_btDbvtAabbMm::AddSpan(const D_btVector3& d,D_btScalar& smi,D_btScalar& smx) const
{
	for(int i=0;i<3;++i)
	{
		if(d[i]<0)
		{ smi+=mx[i]*d[i];smx+=mi[i]*d[i]; }
		else
		{ smi+=mi[i]*d[i];smx+=mx[i]*d[i]; }
	}
}

//
D_DBVT_INLINE bool		Intersect(	const D_btDbvtAabbMm& a,
								  const D_btDbvtAabbMm& b)
{
#if	D_DBVT_INT0_IMPL == D_DBVT_IMPL_SSE
	const __m128	rt(_mm_or_ps(	_mm_cmplt_ps(_mm_load_ps(b.mx),_mm_load_ps(a.mi)),
		_mm_cmplt_ps(_mm_load_ps(a.mx),_mm_load_ps(b.mi))));
	const __int32*	pu((const __int32*)&rt);
	return((pu[0]|pu[1]|pu[2])==0);
#else
	return(	(a.mi.x()<=b.mx.x())&&
		(a.mx.x()>=b.mi.x())&&
		(a.mi.y()<=b.mx.y())&&
		(a.mx.y()>=b.mi.y())&&
		(a.mi.z()<=b.mx.z())&&		
		(a.mx.z()>=b.mi.z()));
#endif
}



//
D_DBVT_INLINE bool		Intersect(	const D_btDbvtAabbMm& a,
								  const D_btVector3& b)
{
	return(	(b.x()>=a.mi.x())&&
		(b.y()>=a.mi.y())&&
		(b.z()>=a.mi.z())&&
		(b.x()<=a.mx.x())&&
		(b.y()<=a.mx.y())&&
		(b.z()<=a.mx.z()));
}





//////////////////////////////////////


//
D_DBVT_INLINE D_btScalar	Proximity(	const D_btDbvtAabbMm& a,
								  const D_btDbvtAabbMm& b)
{
	const D_btVector3	d=(a.mi+a.mx)-(b.mi+b.mx);
	return(D_btFabs(d.x())+D_btFabs(d.y())+D_btFabs(d.z()));
}



//
D_DBVT_INLINE int			Select(	const D_btDbvtAabbMm& o,
							   const D_btDbvtAabbMm& a,
							   const D_btDbvtAabbMm& b)
{
#if	D_DBVT_SELECT_IMPL == D_DBVT_IMPL_SSE
	static D_ATTRIBUTE_ALIGNED16(const unsigned __int32)	mask[]={0x7fffffff,0x7fffffff,0x7fffffff,0x7fffffff};
	///@todo: the intrinsic version D_is 11% slower
#if D_DBVT_USE_INTRINSIC_SSE

	union D_btSSEUnion ///NOTE: if we use more intrinsics, move D_btSSEUnion into the LinearMath directory
	{
	   __m128		ssereg;
	   float		floats[4];
	   int			ints[4];
	};

	__m128	omi(_mm_load_ps(o.mi));
	omi=_mm_add_ps(omi,_mm_load_ps(o.mx));
	__m128	ami(_mm_load_ps(a.mi));
	ami=_mm_add_ps(ami,_mm_load_ps(a.mx));
	ami=_mm_sub_ps(ami,omi);
	ami=_mm_and_ps(ami,_mm_load_ps((const float*)mask));
	__m128	bmi(_mm_load_ps(b.mi));
	bmi=_mm_add_ps(bmi,_mm_load_ps(b.mx));
	bmi=_mm_sub_ps(bmi,omi);
	bmi=_mm_and_ps(bmi,_mm_load_ps((const float*)mask));
	__m128	t0(_mm_movehl_ps(ami,ami));
	ami=_mm_add_ps(ami,t0);
	ami=_mm_add_ss(ami,_mm_shuffle_ps(ami,ami,1));
	__m128 t1(_mm_movehl_ps(bmi,bmi));
	bmi=_mm_add_ps(bmi,t1);
	bmi=_mm_add_ss(bmi,_mm_shuffle_ps(bmi,bmi,1));
	
	D_btSSEUnion tmp;
	tmp.ssereg = _mm_cmple_ss(bmi,ami);
	return tmp.ints[0]&1;

#else
	D_ATTRIBUTE_ALIGNED16(__int32	r[1]);
	__asm
	{
		mov		eax,o
			mov		ecx,a
			mov		edx,b
			movaps	xmm0,[eax]
		movaps	xmm5,mask
			addps	xmm0,[eax+16]	
		movaps	xmm1,[ecx]
		movaps	xmm2,[edx]
		addps	xmm1,[ecx+16]
		addps	xmm2,[edx+16]
		subps	xmm1,xmm0
			subps	xmm2,xmm0
			andps	xmm1,xmm5
			andps	xmm2,xmm5
			movhlps	xmm3,xmm1
			movhlps	xmm4,xmm2
			addps	xmm1,xmm3
			addps	xmm2,xmm4
			pshufd	xmm3,xmm1,1
			pshufd	xmm4,xmm2,1
			addss	xmm1,xmm3
			addss	xmm2,xmm4
			cmpless	xmm2,xmm1
			movss	r,xmm2
	}
	return(r[0]&1);
#endif
#else
	return(Proximity(o,a)<Proximity(o,b)?0:1);
#endif
}

//
D_DBVT_INLINE void		Merge(	const D_btDbvtAabbMm& a,
							  const D_btDbvtAabbMm& b,
							  D_btDbvtAabbMm& r)
{
#if D_DBVT_MERGE_IMPL==D_DBVT_IMPL_SSE
	__m128	ami(_mm_load_ps(a.mi));
	__m128	amx(_mm_load_ps(a.mx));
	__m128	bmi(_mm_load_ps(b.mi));
	__m128	bmx(_mm_load_ps(b.mx));
	ami=_mm_min_ps(ami,bmi);
	amx=_mm_max_ps(amx,bmx);
	_mm_store_ps(r.mi,ami);
	_mm_store_ps(r.mx,amx);
#else
	for(int i=0;i<3;++i)
	{
		if(a.mi[i]<b.mi[i]) r.mi[i]=a.mi[i]; else r.mi[i]=b.mi[i];
		if(a.mx[i]>b.mx[i]) r.mx[i]=a.mx[i]; else r.mx[i]=b.mx[i];
	}
#endif
}

//
D_DBVT_INLINE bool		NotEqual(	const D_btDbvtAabbMm& a,
								 const D_btDbvtAabbMm& b)
{
	return(	(a.mi.x()!=b.mi.x())||
		(a.mi.y()!=b.mi.y())||
		(a.mi.z()!=b.mi.z())||
		(a.mx.x()!=b.mx.x())||
		(a.mx.y()!=b.mx.y())||
		(a.mx.z()!=b.mx.z()));
}

//
// Inline's
//

//
D_DBVT_PREFIX
inline void		D_btDbvt::enumNodes(	const D_btDbvtNode* root,
								  D_DBVT_IPOLICY)
{
	D_DBVT_CHECKTYPE
		policy.Process(root);
	if(root->isinternal())
	{
		enumNodes(root->childs[0],policy);
		enumNodes(root->childs[1],policy);
	}
}

//
D_DBVT_PREFIX
inline void		D_btDbvt::enumLeaves(	const D_btDbvtNode* root,
								   D_DBVT_IPOLICY)
{
	D_DBVT_CHECKTYPE
		if(root->isinternal())
		{
			enumLeaves(root->childs[0],policy);
			enumLeaves(root->childs[1],policy);
		}
		else
		{
			policy.Process(root);
		}
}

//
D_DBVT_PREFIX
inline void		D_btDbvt::collideTT(	const D_btDbvtNode* root0,
								  const D_btDbvtNode* root1,
								  D_DBVT_IPOLICY)
{
	D_DBVT_CHECKTYPE
		if(root0&&root1)
		{
			int								depth=1;
			int								treshold=DOUBLE_STACKSIZE-4;
			D_btAlignedObjectArray<D_sStkNN>	stkStack;
			stkStack.resize(DOUBLE_STACKSIZE);
			stkStack[0]=D_sStkNN(root0,root1);
			do	{		
				D_sStkNN	p=stkStack[--depth];
				if(depth>treshold)
				{
					stkStack.resize(stkStack.size()*2);
					treshold=stkStack.size()-4;
				}
				if(p.a==p.b)
				{
					if(p.a->isinternal())
					{
						stkStack[depth++]=D_sStkNN(p.a->childs[0],p.a->childs[0]);
						stkStack[depth++]=D_sStkNN(p.a->childs[1],p.a->childs[1]);
						stkStack[depth++]=D_sStkNN(p.a->childs[0],p.a->childs[1]);
					}
				}
				else if(Intersect(p.a->volume,p.b->volume))
				{
					if(p.a->isinternal())
					{
						if(p.b->isinternal())
						{
							stkStack[depth++]=D_sStkNN(p.a->childs[0],p.b->childs[0]);
							stkStack[depth++]=D_sStkNN(p.a->childs[1],p.b->childs[0]);
							stkStack[depth++]=D_sStkNN(p.a->childs[0],p.b->childs[1]);
							stkStack[depth++]=D_sStkNN(p.a->childs[1],p.b->childs[1]);
						}
						else
						{
							stkStack[depth++]=D_sStkNN(p.a->childs[0],p.b);
							stkStack[depth++]=D_sStkNN(p.a->childs[1],p.b);
						}
					}
					else
					{
						if(p.b->isinternal())
						{
							stkStack[depth++]=D_sStkNN(p.a,p.b->childs[0]);
							stkStack[depth++]=D_sStkNN(p.a,p.b->childs[1]);
						}
						else
						{
							policy.Process(p.a,p.b);
						}
					}
				}
			} while(depth);
		}
}



D_DBVT_PREFIX
inline void		D_btDbvt::collideTTpersistentStack(	const D_btDbvtNode* root0,
								  const D_btDbvtNode* root1,
								  D_DBVT_IPOLICY)
{
	D_DBVT_CHECKTYPE
		if(root0&&root1)
		{
			int								depth=1;
			int								treshold=DOUBLE_STACKSIZE-4;
			
			m_stkStack.resize(DOUBLE_STACKSIZE);
			m_stkStack[0]=D_sStkNN(root0,root1);
			do	{		
				D_sStkNN	p=m_stkStack[--depth];
				if(depth>treshold)
				{
					m_stkStack.resize(m_stkStack.size()*2);
					treshold=m_stkStack.size()-4;
				}
				if(p.a==p.b)
				{
					if(p.a->isinternal())
					{
						m_stkStack[depth++]=D_sStkNN(p.a->childs[0],p.a->childs[0]);
						m_stkStack[depth++]=D_sStkNN(p.a->childs[1],p.a->childs[1]);
						m_stkStack[depth++]=D_sStkNN(p.a->childs[0],p.a->childs[1]);
					}
				}
				else if(Intersect(p.a->volume,p.b->volume))
				{
					if(p.a->isinternal())
					{
						if(p.b->isinternal())
						{
							m_stkStack[depth++]=D_sStkNN(p.a->childs[0],p.b->childs[0]);
							m_stkStack[depth++]=D_sStkNN(p.a->childs[1],p.b->childs[0]);
							m_stkStack[depth++]=D_sStkNN(p.a->childs[0],p.b->childs[1]);
							m_stkStack[depth++]=D_sStkNN(p.a->childs[1],p.b->childs[1]);
						}
						else
						{
							m_stkStack[depth++]=D_sStkNN(p.a->childs[0],p.b);
							m_stkStack[depth++]=D_sStkNN(p.a->childs[1],p.b);
						}
					}
					else
					{
						if(p.b->isinternal())
						{
							m_stkStack[depth++]=D_sStkNN(p.a,p.b->childs[0]);
							m_stkStack[depth++]=D_sStkNN(p.a,p.b->childs[1]);
						}
						else
						{
							policy.Process(p.a,p.b);
						}
					}
				}
			} while(depth);
		}
}

#if 0
//
D_DBVT_PREFIX
inline void		D_btDbvt::collideTT(	const D_btDbvtNode* root0,
								  const D_btDbvtNode* root1,
								  const D_btTransform& xform,
								  D_DBVT_IPOLICY)
{
	D_DBVT_CHECKTYPE
		if(root0&&root1)
		{
			int								depth=1;
			int								treshold=DOUBLE_STACKSIZE-4;
			D_btAlignedObjectArray<D_sStkNN>	stkStack;
			stkStack.resize(DOUBLE_STACKSIZE);
			stkStack[0]=D_sStkNN(root0,root1);
			do	{
				D_sStkNN	p=stkStack[--depth];
				if(Intersect(p.a->volume,p.b->volume,xform))
				{
					if(depth>treshold)
					{
						stkStack.resize(stkStack.size()*2);
						treshold=stkStack.size()-4;
					}
					if(p.a->isinternal())
					{
						if(p.b->isinternal())
						{					
							stkStack[depth++]=D_sStkNN(p.a->childs[0],p.b->childs[0]);
							stkStack[depth++]=D_sStkNN(p.a->childs[1],p.b->childs[0]);
							stkStack[depth++]=D_sStkNN(p.a->childs[0],p.b->childs[1]);
							stkStack[depth++]=D_sStkNN(p.a->childs[1],p.b->childs[1]);
						}
						else
						{
							stkStack[depth++]=D_sStkNN(p.a->childs[0],p.b);
							stkStack[depth++]=D_sStkNN(p.a->childs[1],p.b);
						}
					}
					else
					{
						if(p.b->isinternal())
						{
							stkStack[depth++]=D_sStkNN(p.a,p.b->childs[0]);
							stkStack[depth++]=D_sStkNN(p.a,p.b->childs[1]);
						}
						else
						{
							policy.Process(p.a,p.b);
						}
					}
				}
			} while(depth);
		}
}
//
D_DBVT_PREFIX
inline void		D_btDbvt::collideTT(	const D_btDbvtNode* root0,
								  const D_btTransform& xform0,
								  const D_btDbvtNode* root1,
								  const D_btTransform& xform1,
								  D_DBVT_IPOLICY)
{
	const D_btTransform	xform=xform0.inverse()*xform1;
	collideTT(root0,root1,xform,policy);
}
#endif 

//
D_DBVT_PREFIX
inline void		D_btDbvt::collideTV(	const D_btDbvtNode* root,
								  const D_btDbvtVolume& vol,
								  D_DBVT_IPOLICY)
{
	D_DBVT_CHECKTYPE
		if(root)
		{
			D_ATTRIBUTE_ALIGNED16(D_btDbvtVolume)		volume(vol);
			D_btAlignedObjectArray<const D_btDbvtNode*>	stack;
			stack.resize(0);
			stack.reserve(SIMPLE_STACKSIZE);
			stack.push_back(root);
			do	{
				const D_btDbvtNode*	n=stack[stack.size()-1];
				stack.pop_back();
				if(Intersect(n->volume,volume))
				{
					if(n->isinternal())
					{
						stack.push_back(n->childs[0]);
						stack.push_back(n->childs[1]);
					}
					else
					{
						policy.Process(n);
					}
				}
			} while(stack.size()>0);
		}
}

D_DBVT_PREFIX
inline void		D_btDbvt::rayTestInternal(	const D_btDbvtNode* root,
								const D_btVector3& rayFrom,
								const D_btVector3& rayTo,
								const D_btVector3& rayDirectionInverse,
								unsigned int signs[3],
								D_btScalar lambda_max,
								const D_btVector3& aabbMin,
								const D_btVector3& aabbMax,
								D_DBVT_IPOLICY) const
{
	D_DBVT_CHECKTYPE
	if(root)
	{
		D_btVector3 resultNormal;

		int								depth=1;
		int								treshold=DOUBLE_STACKSIZE-2;
		D_btAlignedObjectArray<const D_btDbvtNode*>	stack;
		stack.resize(DOUBLE_STACKSIZE);
		stack[0]=root;
		D_btVector3 bounds[2];
		do	
		{
			const D_btDbvtNode*	node=stack[--depth];
			bounds[0] = node->volume.Mins()+aabbMin;
			bounds[1] = node->volume.Maxs()+aabbMax;
			D_btScalar tmin=1.f,lambda_min=0.f;
			unsigned int result1=false;
			result1 = D_btRayAabb2(rayFrom,rayDirectionInverse,signs,bounds,tmin,lambda_min,lambda_max);
			if(result1)
			{
				if(node->isinternal())
				{
					if(depth>treshold)
					{
						stack.resize(stack.size()*2);
						treshold=stack.size()-2;
					}
					stack[depth++]=node->childs[0];
					stack[depth++]=node->childs[1];
				}
				else
				{
					policy.Process(node);
				}
			}
		} while(depth);
	}
}

//
D_DBVT_PREFIX
inline void		D_btDbvt::rayTest(	const D_btDbvtNode* root,
								const D_btVector3& rayFrom,
								const D_btVector3& rayTo,
								D_DBVT_IPOLICY)
{
	D_DBVT_CHECKTYPE
		if(root)
		{
			D_btVector3 rayDir = (rayTo-rayFrom);
			rayDir.normalize ();

			///what about division by zero? --> D_just set rayDirection[i] D_to INF/D_BT_LARGE_FLOAT
			D_btVector3 rayDirectionInverse;
			rayDirectionInverse[0] = rayDir[0] == D_btScalar(0.0) ? D_btScalar(D_BT_LARGE_FLOAT) : D_btScalar(1.0) / rayDir[0];
			rayDirectionInverse[1] = rayDir[1] == D_btScalar(0.0) ? D_btScalar(D_BT_LARGE_FLOAT) : D_btScalar(1.0) / rayDir[1];
			rayDirectionInverse[2] = rayDir[2] == D_btScalar(0.0) ? D_btScalar(D_BT_LARGE_FLOAT) : D_btScalar(1.0) / rayDir[2];
			unsigned int signs[3] = { rayDirectionInverse[0] < 0.0, rayDirectionInverse[1] < 0.0, rayDirectionInverse[2] < 0.0};

			D_btScalar lambda_max = rayDir.dot(rayTo-rayFrom);

			D_btVector3 resultNormal;

			D_btAlignedObjectArray<const D_btDbvtNode*>	stack;

			int								depth=1;
			int								treshold=DOUBLE_STACKSIZE-2;

			stack.resize(DOUBLE_STACKSIZE);
			stack[0]=root;
			D_btVector3 bounds[2];
			do	{
				const D_btDbvtNode*	node=stack[--depth];

				bounds[0] = node->volume.Mins();
				bounds[1] = node->volume.Maxs();
				
				D_btScalar tmin=1.f,lambda_min=0.f;
				unsigned int result1 = D_btRayAabb2(rayFrom,rayDirectionInverse,signs,bounds,tmin,lambda_min,lambda_max);

#ifdef COMPARE_BTRAY_AABB2
				D_btScalar param=1.f;
				bool result2 = D_btRayAabb(rayFrom,rayTo,node->volume.Mins(),node->volume.Maxs(),param,resultNormal);
				D_btAssert(result1 == result2);
#endif //TEST_BTRAY_AABB2

				if(result1)
				{
					if(node->isinternal())
					{
						if(depth>treshold)
						{
							stack.resize(stack.size()*2);
							treshold=stack.size()-2;
						}
						stack[depth++]=node->childs[0];
						stack[depth++]=node->childs[1];
					}
					else
					{
						policy.Process(node);
					}
				}
			} while(depth);

		}
}

//
D_DBVT_PREFIX
inline void		D_btDbvt::collideKDOP(const D_btDbvtNode* root,
									const D_btVector3* normals,
									const D_btScalar* offsets,
									int count,
									D_DBVT_IPOLICY)
{
	D_DBVT_CHECKTYPE
		if(root)
		{
			const int						inside=(1<<count)-1;
			D_btAlignedObjectArray<D_sStkNP>	stack;
			int								signs[sizeof(unsigned)*8];
			D_btAssert(count<int (sizeof(signs)/sizeof(signs[0])));
			for(int i=0;i<count;++i)
			{
				signs[i]=	((normals[i].x()>=0)?1:0)+
					((normals[i].y()>=0)?2:0)+
					((normals[i].z()>=0)?4:0);
			}
			stack.reserve(SIMPLE_STACKSIZE);
			stack.push_back(D_sStkNP(root,0));
			do	{
				D_sStkNP	se=stack[stack.size()-1];
				bool	out=false;
				stack.pop_back();
				for(int i=0,j=1;(!out)&&(i<count);++i,j<<=1)
				{
					if(0==(se.mask&j))
					{
						const int	side=se.node->volume.Classify(normals[i],offsets[i],signs[i]);
						switch(side)
						{
						case	-1:	out=true;break;
						case	+1:	se.mask|=j;break;
						}
					}
				}
				if(!out)
				{
					if((se.mask!=inside)&&(se.node->isinternal()))
					{
						stack.push_back(D_sStkNP(se.node->childs[0],se.mask));
						stack.push_back(D_sStkNP(se.node->childs[1],se.mask));
					}
					else
					{
						if(policy.AllLeaves(se.node)) enumLeaves(se.node,policy);
					}
				}
			} while(stack.size());
		}
}

//
D_DBVT_PREFIX
inline void		D_btDbvt::collideOCL(	const D_btDbvtNode* root,
								   const D_btVector3* normals,
								   const D_btScalar* offsets,
								   const D_btVector3& sortaxis,
								   int count,
								   D_DBVT_IPOLICY,
								   bool fsort)
{
	D_DBVT_CHECKTYPE
		if(root)
		{
			const unsigned					srtsgns=(sortaxis[0]>=0?1:0)+
				(sortaxis[1]>=0?2:0)+
				(sortaxis[2]>=0?4:0);
			const int						inside=(1<<count)-1;
			D_btAlignedObjectArray<D_sStkNPS>	stock;
			D_btAlignedObjectArray<int>		ifree;
			D_btAlignedObjectArray<int>		stack;
			int								signs[sizeof(unsigned)*8];
			D_btAssert(count<int (sizeof(signs)/sizeof(signs[0])));
			for(int i=0;i<count;++i)
			{
				signs[i]=	((normals[i].x()>=0)?1:0)+
					((normals[i].y()>=0)?2:0)+
					((normals[i].z()>=0)?4:0);
			}
			stock.reserve(SIMPLE_STACKSIZE);
			stack.reserve(SIMPLE_STACKSIZE);
			ifree.reserve(SIMPLE_STACKSIZE);
			stack.push_back(allocate(ifree,stock,D_sStkNPS(root,0,root->volume.ProjectMinimum(sortaxis,srtsgns))));
			do	{
				const int	id=stack[stack.size()-1];
				D_sStkNPS		se=stock[id];
				stack.pop_back();ifree.push_back(id);
				if(se.mask!=inside)
				{
					bool	out=false;
					for(int i=0,j=1;(!out)&&(i<count);++i,j<<=1)
					{
						if(0==(se.mask&j))
						{
							const int	side=se.node->volume.Classify(normals[i],offsets[i],signs[i]);
							switch(side)
							{
							case	-1:	out=true;break;
							case	+1:	se.mask|=j;break;
							}
						}
					}
					if(out) continue;
				}
				if(policy.Descent(se.node))
				{
					if(se.node->isinternal())
					{
						const D_btDbvtNode* pns[]={	se.node->childs[0],se.node->childs[1]};
						D_sStkNPS		nes[]={	D_sStkNPS(pns[0],se.mask,pns[0]->volume.ProjectMinimum(sortaxis,srtsgns)),
							D_sStkNPS(pns[1],se.mask,pns[1]->volume.ProjectMinimum(sortaxis,srtsgns))};
						const int	q=nes[0].value<nes[1].value?1:0;				
						int			j=stack.size();
						if(fsort&&(j>0))
						{
							/* Insert 0	*/ 
							j=nearest(&stack[0],&stock[0],nes[q].value,0,stack.size());
							stack.push_back(0);
#if D_DBVT_USE_MEMMOVE
							memmove(&stack[j+1],&stack[j],sizeof(int)*(stack.size()-j-1));
#else
							for(int k=stack.size()-1;k>j;--k) stack[k]=stack[k-1];
#endif
							stack[j]=allocate(ifree,stock,nes[q]);
							/* Insert 1	*/ 
							j=nearest(&stack[0],&stock[0],nes[1-q].value,j,stack.size());
							stack.push_back(0);
#if D_DBVT_USE_MEMMOVE
							memmove(&stack[j+1],&stack[j],sizeof(int)*(stack.size()-j-1));
#else
							for(int k=stack.size()-1;k>j;--k) stack[k]=stack[k-1];
#endif
							stack[j]=allocate(ifree,stock,nes[1-q]);
						}
						else
						{
							stack.push_back(allocate(ifree,stock,nes[q]));
							stack.push_back(allocate(ifree,stock,nes[1-q]));
						}
					}
					else
					{
						policy.Process(se.node,se.value);
					}
				}
			} while(stack.size());
		}
}

//
D_DBVT_PREFIX
inline void		D_btDbvt::collideTU(	const D_btDbvtNode* root,
								  D_DBVT_IPOLICY)
{
	D_DBVT_CHECKTYPE
		if(root)
		{
			D_btAlignedObjectArray<const D_btDbvtNode*>	stack;
			stack.reserve(SIMPLE_STACKSIZE);
			stack.push_back(root);
			do	{
				const D_btDbvtNode*	n=stack[stack.size()-1];
				stack.pop_back();
				if(policy.Descent(n))
				{
					if(n->isinternal())
					{ stack.push_back(n->childs[0]);stack.push_back(n->childs[1]); }
					else
					{ policy.Process(n); }
				}
			} while(stack.size()>0);
		}
}

//
// PP Cleanup
//

#undef D_DBVT_USE_MEMMOVE
#undef DBVT_USE_TEMPLATE
#undef D_DBVT_VIRTUAL_DTOR
#undef D_DBVT_VIRTUAL
#undef D_DBVT_PREFIX
#undef D_DBVT_IPOLICY
#undef D_DBVT_CHECKTYPE
#undef D_DBVT_IMPL_GENERIC
#undef D_DBVT_IMPL_SSE
#undef D_DBVT_USE_INTRINSIC_SSE
#undef D_DBVT_SELECT_IMPL
#undef D_DBVT_MERGE_IMPL
#undef D_DBVT_INT0_IMPL

#endif
