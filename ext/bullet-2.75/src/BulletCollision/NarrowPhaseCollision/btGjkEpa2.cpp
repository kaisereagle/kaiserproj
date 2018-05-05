/*
Bullet Continuous Collision Detection D_and Physics Library
Copyright (c) 2003-2008 Erwin Coumans  http://continuousphysics.com/Bullet/

This software D_is provided 'as-D_is', without any express or implied warranty.
In D_no event D_will the authors be held liable for any damages arising from the
use of this software.
Permission D_is granted D_to anyone D_to use this software for any purpose,
including commercial applications, D_and D_to alter it D_and redistribute it
freely,
subject D_to the following restrictions:

1. The origin of this software D_must not be misrepresented; you D_must not
claim that you wrote the original software. If you use this software in a
product, an acknowledgment in the product documentation would be appreciated
but D_is not required.
2. Altered source versions D_must be plainly marked as such, D_and D_must not be
misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

/*
GJK-EPA collision solver by Nathanael Presson, 2008
*/
#include "BulletCollision/CollisionShapes/btConvexInternalShape.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "btGjkEpa2.h"

#if defined(DEBUG) || defined (_DEBUG)
#include <stdio.h> //for D_debug printf
#ifdef __SPU__
#include <D_spu_printf.h>
#define printf D_spu_printf
#endif //__SPU__
#endif

namespace D_gjkepa2_impl
{

	// Config

	/* GJK	*/ 
#define D_GJK_MAX_ITERATIONS	128
#define D_GJK_ACCURARY		((D_btScalar)0.0001)
#define D_GJK_MIN_DISTANCE	((D_btScalar)0.0001)
#define D_GJK_DUPLICATED_EPS	((D_btScalar)0.0001)
#define D_GJK_SIMPLEX2_EPS	((D_btScalar)0.0)
#define D_GJK_SIMPLEX3_EPS	((D_btScalar)0.0)
#define D_GJK_SIMPLEX4_EPS	((D_btScalar)0.0)

	/* EPA	*/ 
#define D_EPA_MAX_VERTICES	64
#define D_EPA_MAX_FACES		(D_EPA_MAX_VERTICES*2)
#define D_EPA_MAX_ITERATIONS	255
#define D_EPA_ACCURACY		((D_btScalar)0.0001)
#define D_EPA_FALLBACK		(10*D_EPA_ACCURACY)
#define D_EPA_PLANE_EPS		((D_btScalar)0.00001)
#define D_EPA_INSIDE_EPS		((D_btScalar)0.01)


	// Shorthands
	typedef unsigned int	D_U;
	typedef unsigned char	D_U1;

	// MinkowskiDiff
	struct	MinkowskiDiff
	{
		const D_btConvexShape*	m_shapes[2];
		D_btMatrix3x3				m_toshape1;
		D_btTransform				m_toshape0;
#ifdef __SPU__
		bool					m_enableMargin;
#else
		D_btVector3				(D_btConvexShape::*Ls)(const D_btVector3&) const;
#endif//__SPU__
		

		MinkowskiDiff()
		{

		}
#ifdef __SPU__
			void					EnableMargin(bool enable)
		{
			m_enableMargin = enable;
		}	
		inline D_btVector3		Support0(const D_btVector3& d) const
		{
			if (m_enableMargin)
			{
				return m_shapes[0]->localGetSupportVertexNonVirtual(d);
			} else
			{
				return m_shapes[0]->localGetSupportVertexWithoutMarginNonVirtual(d);
			}
		}
		inline D_btVector3		Support1(const D_btVector3& d) const
		{
			if (m_enableMargin)
			{
				return m_toshape0*(m_shapes[1]->localGetSupportVertexNonVirtual(m_toshape1*d));
			} else
			{
				return m_toshape0*(m_shapes[1]->localGetSupportVertexWithoutMarginNonVirtual(m_toshape1*d));
			}
		}
#else
		void					EnableMargin(bool enable)
		{
			if(enable)
				Ls=&D_btConvexShape::localGetSupportVertexNonVirtual;
			else
				Ls=&D_btConvexShape::localGetSupportVertexWithoutMarginNonVirtual;
		}	
		inline D_btVector3		Support0(const D_btVector3& d) const
		{
			return(((m_shapes[0])->*(Ls))(d));
		}
		inline D_btVector3		Support1(const D_btVector3& d) const
		{
			return(m_toshape0*((m_shapes[1])->*(Ls))(m_toshape1*d));
		}
#endif //__SPU__

		inline D_btVector3		Support(const D_btVector3& d) const
		{
			return(Support0(d)-Support1(-d));
		}
		D_btVector3				Support(const D_btVector3& d,D_U index) const
		{
			if(index)
				return(Support1(d));
			else
				return(Support0(d));
		}
	};

	typedef	MinkowskiDiff	D_tShape;


	// GJK
	struct	GJK
	{
		/* Types		*/ 
		struct	D_sSV
		{
			D_btVector3	d,w;
		};
		struct	D_sSimplex
		{
			D_sSV*		c[4];
			D_btScalar	p[4];
			D_U			rank;
		};
		struct	D_eStatus	{ enum D__ {
			D_Valid,
			D_Inside,
			D_Failed		};};
			/* Fields		*/ 
			D_tShape			m_shape;
			D_btVector3		m_ray;
			D_btScalar		m_distance;
			D_sSimplex		m_simplices[2];
			D_sSV				m_store[4];
			D_sSV*			m_free[4];
			D_U				m_nfree;
			D_U				m_current;
			D_sSimplex*		m_simplex;
			D_eStatus::D__		m_status;
			/* Methods		*/ 
			GJK()
			{
				Initialize();
			}
			void				Initialize()
			{
				m_ray		=	D_btVector3(0,0,0);
				m_nfree		=	0;
				m_status	=	D_eStatus::D_Failed;
				m_current	=	0;
				m_distance	=	0;
			}
			D_eStatus::D__			Evaluate(const D_tShape& shapearg,const D_btVector3& guess)
			{
				D_U			iterations=0;
				D_btScalar	sqdist=0;
				D_btScalar	alpha=0;
				D_btVector3	lastw[4];
				D_U			clastw=0;
				/* Initialize solver		*/ 
				m_free[0]			=	&m_store[0];
				m_free[1]			=	&m_store[1];
				m_free[2]			=	&m_store[2];
				m_free[3]			=	&m_store[3];
				m_nfree				=	4;
				m_current			=	0;
				m_status			=	D_eStatus::D_Valid;
				m_shape				=	shapearg;
				m_distance			=	0;
				/* Initialize simplex		*/ 
				m_simplices[0].rank	=	0;
				m_ray				=	guess;
				const D_btScalar	sqrl=	m_ray.length2();
				appendvertice(m_simplices[0],sqrl>0?-m_ray:D_btVector3(1,0,0));
				m_simplices[0].p[0]	=	1;
				m_ray				=	m_simplices[0].c[0]->w;	
				sqdist				=	sqrl;
				lastw[0]			=
					lastw[1]			=
					lastw[2]			=
					lastw[3]			=	m_ray;
				/* Loop						*/ 
				do	{
					const D_U		next=1-m_current;
					D_sSimplex&	cs=m_simplices[m_current];
					D_sSimplex&	ns=m_simplices[next];
					/* Check zero							*/ 
					const D_btScalar	rl=m_ray.length();
					if(rl<D_GJK_MIN_DISTANCE)
					{/* D_Touching or inside				*/ 
						m_status=D_eStatus::D_Inside;
						break;
					}
					/* Append new vertice in -'v' direction	*/ 
					appendvertice(cs,-m_ray);
					const D_btVector3&	w=cs.c[cs.rank-1]->w;
					bool				found=false;
					for(D_U i=0;i<4;++i)
					{
						if((w-lastw[i]).length2()<D_GJK_DUPLICATED_EPS)
						{ found=true;break; }
					}
					if(found)
					{/* Return old simplex				*/ 
						removevertice(m_simplices[m_current]);
						break;
					}
					else
					{/* Update lastw					*/ 
						lastw[clastw=(clastw+1)&3]=w;
					}
					/* Check for termination				*/ 
					const D_btScalar	omega=D_btDot(m_ray,w)/rl;
					alpha=D_btMax(omega,alpha);
					if(((rl-alpha)-(D_GJK_ACCURARY*rl))<=0)
					{/* Return old simplex				*/ 
						removevertice(m_simplices[m_current]);
						break;
					}		
					/* Reduce simplex						*/ 
					D_btScalar	weights[4];
					D_U			mask=0;
					switch(cs.rank)
					{
					case	2:	sqdist=projectorigin(	cs.c[0]->w,
									cs.c[1]->w,
									weights,mask);break;
					case	3:	sqdist=projectorigin(	cs.c[0]->w,
									cs.c[1]->w,
									cs.c[2]->w,
									weights,mask);break;
					case	4:	sqdist=projectorigin(	cs.c[0]->w,
									cs.c[1]->w,
									cs.c[2]->w,
									cs.c[3]->w,
									weights,mask);break;
					}
					if(sqdist>=0)
					{/* D_Valid	*/ 
						ns.rank		=	0;
						m_ray		=	D_btVector3(0,0,0);
						m_current	=	next;
						for(D_U i=0,ni=cs.rank;i<ni;++i)
						{
							if(mask&(1<<i))
							{
								ns.c[ns.rank]		=	cs.c[i];
								ns.p[ns.rank++]		=	weights[i];
								m_ray				+=	cs.c[i]->w*weights[i];
							}
							else
							{
								m_free[m_nfree++]	=	cs.c[i];
							}
						}
						if(mask==15) m_status=D_eStatus::D_Inside;
					}
					else
					{/* Return old simplex				*/ 
						removevertice(m_simplices[m_current]);
						break;
					}
					m_status=((++iterations)<D_GJK_MAX_ITERATIONS)?m_status:D_eStatus::D_Failed;
				} while(m_status==D_eStatus::D_Valid);
				m_simplex=&m_simplices[m_current];
				switch(m_status)
				{
				case	D_eStatus::D_Valid:		m_distance=m_ray.length();break;
				case	D_eStatus::D_Inside:	m_distance=0;break;
				default:
					{
					}
				}	
				return(m_status);
			}
			bool					EncloseOrigin()
			{
				switch(m_simplex->rank)
				{
				case	1:
					{
						for(D_U i=0;i<3;++i)
						{
							D_btVector3		axis=D_btVector3(0,0,0);
							axis[i]=1;
							appendvertice(*m_simplex, axis);
							if(EncloseOrigin())	return(true);
							removevertice(*m_simplex);
							appendvertice(*m_simplex,-axis);
							if(EncloseOrigin())	return(true);
							removevertice(*m_simplex);
						}
					}
					break;
				case	2:
					{
						const D_btVector3	d=m_simplex->c[1]->w-m_simplex->c[0]->w;
						for(D_U i=0;i<3;++i)
						{
							D_btVector3		axis=D_btVector3(0,0,0);
							axis[i]=1;
							const D_btVector3	p=D_btCross(d,axis);
							if(p.length2()>0)
							{
								appendvertice(*m_simplex, p);
								if(EncloseOrigin())	return(true);
								removevertice(*m_simplex);
								appendvertice(*m_simplex,-p);
								if(EncloseOrigin())	return(true);
								removevertice(*m_simplex);
							}
						}
					}
					break;
				case	3:
					{
						const D_btVector3	n=D_btCross(m_simplex->c[1]->w-m_simplex->c[0]->w,
							m_simplex->c[2]->w-m_simplex->c[0]->w);
						if(n.length2()>0)
						{
							appendvertice(*m_simplex,n);
							if(EncloseOrigin())	return(true);
							removevertice(*m_simplex);
							appendvertice(*m_simplex,-n);
							if(EncloseOrigin())	return(true);
							removevertice(*m_simplex);
						}
					}
					break;
				case	4:
					{
						if(D_btFabs(det(	m_simplex->c[0]->w-m_simplex->c[3]->w,
							m_simplex->c[1]->w-m_simplex->c[3]->w,
							m_simplex->c[2]->w-m_simplex->c[3]->w))>0)
							return(true);
					}
					break;
				}
				return(false);
			}
			/* Internals	*/ 
			void				getsupport(const D_btVector3& d,D_sSV& sv) const
			{
				sv.d	=	d/d.length();
				sv.w	=	m_shape.Support(sv.d);
			}
			void				removevertice(D_sSimplex& simplex)
			{
				m_free[m_nfree++]=simplex.c[--simplex.rank];
			}
			void				appendvertice(D_sSimplex& simplex,const D_btVector3& v)
			{
				simplex.p[simplex.rank]=0;
				simplex.c[simplex.rank]=m_free[--m_nfree];
				getsupport(v,*simplex.c[simplex.rank++]);
			}
			static D_btScalar		det(const D_btVector3& a,const D_btVector3& b,const D_btVector3& c)
			{
				return(	a.y()*b.z()*c.x()+a.z()*b.x()*c.y()-
					a.x()*b.z()*c.y()-a.y()*b.x()*c.z()+
					a.x()*b.y()*c.z()-a.z()*b.y()*c.x());
			}
			static D_btScalar		projectorigin(	const D_btVector3& a,
				const D_btVector3& b,
				D_btScalar* w,D_U& m)
			{
				const D_btVector3	d=b-a;
				const D_btScalar	l=d.length2();
				if(l>D_GJK_SIMPLEX2_EPS)
				{
					const D_btScalar	t(l>0?-D_btDot(a,d)/l:0);
					if(t>=1)		{ w[0]=0;w[1]=1;m=2;return(b.length2()); }
					else if(t<=0)	{ w[0]=1;w[1]=0;m=1;return(a.length2()); }
					else			{ w[0]=1-(w[1]=t);m=3;return((a+d*t).length2()); }
				}
				return(-1);
			}
			static D_btScalar		projectorigin(	const D_btVector3& a,
				const D_btVector3& b,
				const D_btVector3& c,
				D_btScalar* w,D_U& m)
			{
				static const D_U		imd3[]={1,2,0};
				const D_btVector3*	vt[]={&a,&b,&c};
				const D_btVector3		dl[]={a-b,b-c,c-a};
				const D_btVector3		n=D_btCross(dl[0],dl[1]);
				const D_btScalar		l=n.length2();
				if(l>D_GJK_SIMPLEX3_EPS)
				{
					D_btScalar	mindist=-1;
					D_btScalar	subw[2]={0.f,0.f};
					D_U			subm(0);
					for(D_U i=0;i<3;++i)
					{
						if(D_btDot(*vt[i],D_btCross(dl[i],n))>0)
						{
							const D_U			j=imd3[i];
							const D_btScalar	subd(projectorigin(*vt[i],*vt[j],subw,subm));
							if((mindist<0)||(subd<mindist))
							{
								mindist		=	subd;
								m			=	static_cast<D_U>(((subm&1)?1<<i:0)+((subm&2)?1<<j:0));
								w[i]		=	subw[0];
								w[j]		=	subw[1];
								w[imd3[j]]	=	0;				
							}
						}
					}
					if(mindist<0)
					{
						const D_btScalar	d=D_btDot(a,n);	
						const D_btScalar	s=D_btSqrt(l);
						const D_btVector3	p=n*(d/l);
						mindist	=	p.length2();
						m		=	7;
						w[0]	=	(D_btCross(dl[1],b-p)).length()/s;
						w[1]	=	(D_btCross(dl[2],c-p)).length()/s;
						w[2]	=	1-(w[0]+w[1]);
					}
					return(mindist);
				}
				return(-1);
			}
			static D_btScalar		projectorigin(	const D_btVector3& a,
				const D_btVector3& b,
				const D_btVector3& c,
				const D_btVector3& d,
				D_btScalar* w,D_U& m)
			{
				static const D_U		imd3[]={1,2,0};
				const D_btVector3*	vt[]={&a,&b,&c,&d};
				const D_btVector3		dl[]={a-d,b-d,c-d};
				const D_btScalar		vl=det(dl[0],dl[1],dl[2]);
				const bool			ng=(vl*D_btDot(a,D_btCross(b-c,a-b)))<=0;
				if(ng&&(D_btFabs(vl)>D_GJK_SIMPLEX4_EPS))
				{
					D_btScalar	mindist=-1;
					D_btScalar	subw[3]={0.f,0.f,0.f};
					D_U			subm(0);
					for(D_U i=0;i<3;++i)
					{
						const D_U			j=imd3[i];
						const D_btScalar	s=vl*D_btDot(d,D_btCross(dl[i],dl[j]));
						if(s>0)
						{
							const D_btScalar	subd=projectorigin(*vt[i],*vt[j],d,subw,subm);
							if((mindist<0)||(subd<mindist))
							{
								mindist		=	subd;
								m			=	static_cast<D_U>((subm&1?1<<i:0)+
									(subm&2?1<<j:0)+
									(subm&4?8:0));
								w[i]		=	subw[0];
								w[j]		=	subw[1];
								w[imd3[j]]	=	0;
								w[3]		=	subw[2];
							}
						}
					}
					if(mindist<0)
					{
						mindist	=	0;
						m		=	15;
						w[0]	=	det(c,b,d)/vl;
						w[1]	=	det(a,c,d)/vl;
						w[2]	=	det(b,a,d)/vl;
						w[3]	=	1-(w[0]+w[1]+w[2]);
					}
					return(mindist);
				}
				return(-1);
			}
	};

	// EPA
	struct	EPA
	{
		/* Types		*/ 
		typedef	GJK::D_sSV	D_sSV;
		struct	D_sFace
		{
			D_btVector3	n;
			D_btScalar	d;
			D_btScalar	p;
			D_sSV*		c[3];
			D_sFace*		f[3];
			D_sFace*		l[2];
			D_U1			e[3];
			D_U1			pass;
		};
		struct	D_sList
		{
			D_sFace*		root;
			D_U			count;
			D_sList() : root(0),count(0)	{}
		};
		struct	D_sHorizon
		{
			D_sFace*		cf;
			D_sFace*		ff;
			D_U			nf;
			D_sHorizon() : cf(0),ff(0),nf(0)	{}
		};
		struct	D_eStatus { enum D__ {
			D_Valid,
			D_Touching,
			D_Degenerated,
			D_NonConvex,
			D_InvalidHull,		
			D_OutOfFaces,
			D_OutOfVertices,
			D_AccuraryReached,
			D_FallBack,
			D_Failed		};};
			/* Fields		*/ 
			D_eStatus::D__		m_status;
			GJK::D_sSimplex	m_result;
			D_btVector3		m_normal;
			D_btScalar		m_depth;
			D_sSV				m_sv_store[D_EPA_MAX_VERTICES];
			D_sFace			m_fc_store[D_EPA_MAX_FACES];
			D_U				m_nextsv;
			D_sList			m_hull;
			D_sList			m_stock;
			/* Methods		*/ 
			EPA()
			{
				Initialize();	
			}


			static inline void		bind(D_sFace* fa,D_U ea,D_sFace* fb,D_U eb)
			{
				fa->e[ea]=(D_U1)eb;fa->f[ea]=fb;
				fb->e[eb]=(D_U1)ea;fb->f[eb]=fa;
			}
			static inline void		append(D_sList& list,D_sFace* face)
			{
				face->l[0]	=	0;
				face->l[1]	=	list.root;
				if(list.root) list.root->l[0]=face;
				list.root	=	face;
				++list.count;
			}
			static inline void		remove(D_sList& list,D_sFace* face)
			{
				if(face->l[1]) face->l[1]->l[0]=face->l[0];
				if(face->l[0]) face->l[0]->l[1]=face->l[1];
				if(face==list.root) list.root=face->l[1];
				--list.count;
			}


			void				Initialize()
			{
				m_status	=	D_eStatus::D_Failed;
				m_normal	=	D_btVector3(0,0,0);
				m_depth		=	0;
				m_nextsv	=	0;
				for(D_U i=0;i<D_EPA_MAX_FACES;++i)
				{
					append(m_stock,&m_fc_store[D_EPA_MAX_FACES-i-1]);
				}
			}
			D_eStatus::D__			Evaluate(GJK& gjk,const D_btVector3& guess)
			{
				GJK::D_sSimplex&	simplex=*gjk.m_simplex;
				if((simplex.rank>1)&&gjk.EncloseOrigin())
				{

					/* Clean up				*/ 
					while(m_hull.root)
					{
						D_sFace*	f = m_hull.root;
						remove(m_hull,f);
						append(m_stock,f);
					}
					m_status	=	D_eStatus::D_Valid;
					m_nextsv	=	0;
					/* Orient simplex		*/ 
					if(gjk.det(	simplex.c[0]->w-simplex.c[3]->w,
						simplex.c[1]->w-simplex.c[3]->w,
						simplex.c[2]->w-simplex.c[3]->w)<0)
					{
						D_btSwap(simplex.c[0],simplex.c[1]);
						D_btSwap(simplex.p[0],simplex.p[1]);
					}
					/* Build initial hull	*/ 
					D_sFace*	tetra[]={newface(simplex.c[0],simplex.c[1],simplex.c[2],true),
						newface(simplex.c[1],simplex.c[0],simplex.c[3],true),
						newface(simplex.c[2],simplex.c[1],simplex.c[3],true),
						newface(simplex.c[0],simplex.c[2],simplex.c[3],true)};
					if(m_hull.count==4)
					{
						D_sFace*		best=findbest();
						D_sFace		outer=*best;
						D_U			pass=0;
						D_U			iterations=0;
						bind(tetra[0],0,tetra[1],0);
						bind(tetra[0],1,tetra[2],0);
						bind(tetra[0],2,tetra[3],0);
						bind(tetra[1],1,tetra[3],2);
						bind(tetra[1],2,tetra[2],1);
						bind(tetra[2],2,tetra[3],1);
						m_status=D_eStatus::D_Valid;
						for(;iterations<D_EPA_MAX_ITERATIONS;++iterations)
						{
							if(m_nextsv<D_EPA_MAX_VERTICES)
							{	
								D_sHorizon		horizon;
								D_sSV*			w=&m_sv_store[m_nextsv++];
								bool			valid=true;					
								best->pass	=	(D_U1)(++pass);
								gjk.getsupport(best->n,*w);
								const D_btScalar	wdist=D_btDot(best->n,w->w)-best->d;
								if(wdist>D_EPA_ACCURACY)
								{
									for(D_U j=0;(j<3)&&valid;++j)
									{
										valid&=expand(	pass,w,
											best->f[j],best->e[j],
											horizon);
									}
									if(valid&&(horizon.nf>=3))
									{
										bind(horizon.cf,1,horizon.ff,2);
										remove(m_hull,best);
										append(m_stock,best);
										best=findbest();
										if(best->p>=outer.p) outer=*best;
									} else { m_status=D_eStatus::D_InvalidHull;break; }
								} else { m_status=D_eStatus::D_AccuraryReached;break; }
							} else { m_status=D_eStatus::D_OutOfVertices;break; }
						}
						const D_btVector3	projection=outer.n*outer.d;
						m_normal	=	outer.n;
						m_depth		=	outer.d;
						m_result.rank	=	3;
						m_result.c[0]	=	outer.c[0];
						m_result.c[1]	=	outer.c[1];
						m_result.c[2]	=	outer.c[2];
						m_result.p[0]	=	D_btCross(	outer.c[1]->w-projection,
							outer.c[2]->w-projection).length();
						m_result.p[1]	=	D_btCross(	outer.c[2]->w-projection,
							outer.c[0]->w-projection).length();
						m_result.p[2]	=	D_btCross(	outer.c[0]->w-projection,
							outer.c[1]->w-projection).length();
						const D_btScalar	sum=m_result.p[0]+m_result.p[1]+m_result.p[2];
						m_result.p[0]	/=	sum;
						m_result.p[1]	/=	sum;
						m_result.p[2]	/=	sum;
						return(m_status);
					}
				}
				/* Fallback		*/ 
				m_status	=	D_eStatus::D_FallBack;
				m_normal	=	-guess;
				const D_btScalar	nl=m_normal.length();
				if(nl>0)
					m_normal	=	m_normal/nl;
				else
					m_normal	=	D_btVector3(1,0,0);
				m_depth	=	0;
				m_result.rank=1;
				m_result.c[0]=simplex.c[0];
				m_result.p[0]=1;	
				return(m_status);
			}
			D_sFace*				newface(D_sSV* a,D_sSV* b,D_sSV* c,bool forced)
			{
				if(m_stock.root)
				{
					D_sFace*	face=m_stock.root;
					remove(m_stock,face);
					append(m_hull,face);
					face->pass	=	0;
					face->c[0]	=	a;
					face->c[1]	=	b;
					face->c[2]	=	c;
					face->n		=	D_btCross(b->w-a->w,c->w-a->w);
					const D_btScalar	l=face->n.length();
					const bool		v=l>D_EPA_ACCURACY;
					face->p		=	D_btMin(D_btMin(
						D_btDot(a->w,D_btCross(face->n,a->w-b->w)),
						D_btDot(b->w,D_btCross(face->n,b->w-c->w))),
						D_btDot(c->w,D_btCross(face->n,c->w-a->w)))	/
						(v?l:1);
					face->p		=	face->p>=-D_EPA_INSIDE_EPS?0:face->p;
					if(v)
					{
						face->d		=	D_btDot(a->w,face->n)/l;
						face->n		/=	l;
						if(forced||(face->d>=-D_EPA_PLANE_EPS))
						{
							return(face);
						} else m_status=D_eStatus::D_NonConvex;
					} else m_status=D_eStatus::D_Degenerated;
					remove(m_hull,face);
					append(m_stock,face);
					return(0);
				}
				m_status=m_stock.root?D_eStatus::D_OutOfVertices:D_eStatus::D_OutOfFaces;
				return(0);
			}
			D_sFace*				findbest()
			{
				D_sFace*		minf=m_hull.root;
				D_btScalar	mind=minf->d*minf->d;
				D_btScalar	maxp=minf->p;
				for(D_sFace* f=minf->l[1];f;f=f->l[1])
				{
					const D_btScalar	sqd=f->d*f->d;
					if((f->p>=maxp)&&(sqd<mind))
					{
						minf=f;
						mind=sqd;
						maxp=f->p;
					}
				}
				return(minf);
			}
			bool				expand(D_U pass,D_sSV* w,D_sFace* f,D_U e,D_sHorizon& horizon)
			{
				static const D_U	i1m3[]={1,2,0};
				static const D_U	i2m3[]={2,0,1};
				if(f->pass!=pass)
				{
					const D_U	e1=i1m3[e];
					if((D_btDot(f->n,w->w)-f->d)<-D_EPA_PLANE_EPS)
					{
						D_sFace*	nf=newface(f->c[e1],f->c[e],w,false);
						if(nf)
						{
							bind(nf,0,f,e);
							if(horizon.cf) bind(horizon.cf,1,nf,2); else horizon.ff=nf;
							horizon.cf=nf;
							++horizon.nf;
							return(true);
						}
					}
					else
					{
						const D_U	e2=i2m3[e];
						f->pass		=	(D_U1)pass;
						if(	expand(pass,w,f->f[e1],f->e[e1],horizon)&&
							expand(pass,w,f->f[e2],f->e[e2],horizon))
						{
							remove(m_hull,f);
							append(m_stock,f);
							return(true);
						}
					}
				}
				return(false);
			}

	};

	//
	static void	Initialize(	const D_btConvexShape* shape0,const D_btTransform& wtrs0,
		const D_btConvexShape* shape1,const D_btTransform& wtrs1,
		D_btGjkEpaSolver2::D_sResults& results,
		D_tShape& shape,
		bool withmargins)
	{
		/* Results		*/ 
		results.witnesses[0]	=
			results.witnesses[1]	=	D_btVector3(0,0,0);
		results.status			=	D_btGjkEpaSolver2::D_sResults::D_Separated;
		/* Shape		*/ 
		shape.m_shapes[0]		=	shape0;
		shape.m_shapes[1]		=	shape1;
		shape.m_toshape1		=	wtrs1.getBasis().transposeTimes(wtrs0.getBasis());
		shape.m_toshape0		=	wtrs0.inverseTimes(wtrs1);
		shape.EnableMargin(withmargins);
	}

}

//
// Api
//

using namespace	D_gjkepa2_impl;

//
int			D_btGjkEpaSolver2::StackSizeRequirement()
{
	return(sizeof(GJK)+sizeof(EPA));
}

//
bool		D_btGjkEpaSolver2::Distance(	const D_btConvexShape*	shape0,
									  const D_btTransform&		wtrs0,
									  const D_btConvexShape*	shape1,
									  const D_btTransform&		wtrs1,
									  const D_btVector3&		guess,
									  D_sResults&				results)
{
	D_tShape			shape;
	Initialize(shape0,wtrs0,shape1,wtrs1,results,shape,false);
	GJK				gjk;
	GJK::D_eStatus::D__	gjk_status=gjk.Evaluate(shape,guess);
	if(gjk_status==GJK::D_eStatus::D_Valid)
	{
		D_btVector3	w0=D_btVector3(0,0,0);
		D_btVector3	w1=D_btVector3(0,0,0);
		for(D_U i=0;i<gjk.m_simplex->rank;++i)
		{
			const D_btScalar	p=gjk.m_simplex->p[i];
			w0+=shape.Support( gjk.m_simplex->c[i]->d,0)*p;
			w1+=shape.Support(-gjk.m_simplex->c[i]->d,1)*p;
		}
		results.witnesses[0]	=	wtrs0*w0;
		results.witnesses[1]	=	wtrs0*w1;
		results.normal			=	w0-w1;
		results.distance		=	results.normal.length();
		results.normal			/=	results.distance>D_GJK_MIN_DISTANCE?results.distance:1;
		return(true);
	}
	else
	{
		results.status	=	gjk_status==GJK::D_eStatus::D_Inside?
			D_sResults::Penetrating	:
		D_sResults::GJK_Failed	;
		return(false);
	}
}

//
bool	D_btGjkEpaSolver2::Penetration(	const D_btConvexShape*	shape0,
									 const D_btTransform&		wtrs0,
									 const D_btConvexShape*	shape1,
									 const D_btTransform&		wtrs1,
									 const D_btVector3&		guess,
									 D_sResults&				results,
									 bool					usemargins)
{
	D_tShape			shape;
	Initialize(shape0,wtrs0,shape1,wtrs1,results,shape,usemargins);
	GJK				gjk;	
	GJK::D_eStatus::D__	gjk_status=gjk.Evaluate(shape,-guess);
	switch(gjk_status)
	{
	case	GJK::D_eStatus::D_Inside:
		{
			EPA				epa;
			EPA::D_eStatus::D__	epa_status=epa.Evaluate(gjk,-guess);
			if(epa_status!=EPA::D_eStatus::D_Failed)
			{
				D_btVector3	w0=D_btVector3(0,0,0);
				for(D_U i=0;i<epa.m_result.rank;++i)
				{
					w0+=shape.Support(epa.m_result.c[i]->d,0)*epa.m_result.p[i];
				}
				results.status			=	D_sResults::Penetrating;
				results.witnesses[0]	=	wtrs0*w0;
				results.witnesses[1]	=	wtrs0*(w0-epa.m_normal*epa.m_depth);
				results.normal			=	-epa.m_normal;
				results.distance		=	-epa.m_depth;
				return(true);
			} else results.status=D_sResults::EPA_Failed;
		}
		break;
	case	GJK::D_eStatus::D_Failed:
		results.status=D_sResults::GJK_Failed;
		break;
		default:
					{
					}
	}
	return(false);
}

#ifndef __SPU__
//
D_btScalar	D_btGjkEpaSolver2::SignedDistance(const D_btVector3& position,
											D_btScalar margin,
											const D_btConvexShape* shape0,
											const D_btTransform& wtrs0,
											D_sResults& results)
{
	D_tShape			shape;
	D_btSphereShape	shape1(margin);
	D_btTransform		wtrs1(D_btQuaternion(0,0,0,1),position);
	Initialize(shape0,wtrs0,&shape1,wtrs1,results,shape,false);
	GJK				gjk;	
	GJK::D_eStatus::D__	gjk_status=gjk.Evaluate(shape,D_btVector3(1,1,1));
	if(gjk_status==GJK::D_eStatus::D_Valid)
	{
		D_btVector3	w0=D_btVector3(0,0,0);
		D_btVector3	w1=D_btVector3(0,0,0);
		for(D_U i=0;i<gjk.m_simplex->rank;++i)
		{
			const D_btScalar	p=gjk.m_simplex->p[i];
			w0+=shape.Support( gjk.m_simplex->c[i]->d,0)*p;
			w1+=shape.Support(-gjk.m_simplex->c[i]->d,1)*p;
		}
		results.witnesses[0]	=	wtrs0*w0;
		results.witnesses[1]	=	wtrs0*w1;
		const D_btVector3	delta=	results.witnesses[1]-
			results.witnesses[0];
		const D_btScalar	margin=	shape0->getMarginNonVirtual()+
			shape1.getMarginNonVirtual();
		const D_btScalar	length=	delta.length();	
		results.normal			=	delta/length;
		results.witnesses[0]	+=	results.normal*margin;
		return(length-margin);
	}
	else
	{
		if(gjk_status==GJK::D_eStatus::D_Inside)
		{
			if(Penetration(shape0,wtrs0,&shape1,wtrs1,gjk.m_ray,results))
			{
				const D_btVector3	delta=	results.witnesses[0]-
					results.witnesses[1];
				const D_btScalar	length=	delta.length();
				if (length >= D_SIMD_EPSILON)
					results.normal	=	delta/length;			
				return(-length);
			}
		}	
	}
	return(D_SIMD_INFINITY);
}

//
bool	D_btGjkEpaSolver2::SignedDistance(const D_btConvexShape*	shape0,
										const D_btTransform&		wtrs0,
										const D_btConvexShape*	shape1,
										const D_btTransform&		wtrs1,
										const D_btVector3&		guess,
										D_sResults&				results)
{
	if(!Distance(shape0,wtrs0,shape1,wtrs1,guess,results))
		return(Penetration(shape0,wtrs0,shape1,wtrs1,guess,results,false));
	else
		return(true);
}
#endif //__SPU__

/* Symbols cleanup		*/ 

#undef D_GJK_MAX_ITERATIONS
#undef D_GJK_ACCURARY
#undef D_GJK_MIN_DISTANCE
#undef D_GJK_DUPLICATED_EPS
#undef D_GJK_SIMPLEX2_EPS
#undef D_GJK_SIMPLEX3_EPS
#undef D_GJK_SIMPLEX4_EPS

#undef D_EPA_MAX_VERTICES
#undef D_EPA_MAX_FACES
#undef D_EPA_MAX_ITERATIONS
#undef D_EPA_ACCURACY
#undef D_EPA_FALLBACK
#undef D_EPA_PLANE_EPS
#undef D_EPA_INSIDE_EPS
