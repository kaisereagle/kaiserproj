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
///D_btSoftBodyHelpers.cpp by Nathanael Presson

#include "btSoftBodyInternals.h"
#include <stdio.h>
#include <string.h>
#include "btSoftBodyHelpers.h"
#include "LinearMath/btConvexHull.h"

//
static void				drawVertex(	D_btIDebugDraw* idraw,
								   const D_btVector3& x,D_btScalar s,const D_btVector3& c)
{
	idraw->drawLine(x-D_btVector3(s,0,0),x+D_btVector3(s,0,0),c);
	idraw->drawLine(x-D_btVector3(0,s,0),x+D_btVector3(0,s,0),c);
	idraw->drawLine(x-D_btVector3(0,0,s),x+D_btVector3(0,0,s),c);
}

//
static void				drawBox(	D_btIDebugDraw* idraw,
								const D_btVector3& mins,
								const D_btVector3& maxs,
								const D_btVector3& color)
{
	const D_btVector3	c[]={	D_btVector3(mins.x(),mins.y(),mins.z()),
		D_btVector3(maxs.x(),mins.y(),mins.z()),
		D_btVector3(maxs.x(),maxs.y(),mins.z()),
		D_btVector3(mins.x(),maxs.y(),mins.z()),
		D_btVector3(mins.x(),mins.y(),maxs.z()),
		D_btVector3(maxs.x(),mins.y(),maxs.z()),
		D_btVector3(maxs.x(),maxs.y(),maxs.z()),
		D_btVector3(mins.x(),maxs.y(),maxs.z())};
	idraw->drawLine(c[0],c[1],color);idraw->drawLine(c[1],c[2],color);
	idraw->drawLine(c[2],c[3],color);idraw->drawLine(c[3],c[0],color);
	idraw->drawLine(c[4],c[5],color);idraw->drawLine(c[5],c[6],color);
	idraw->drawLine(c[6],c[7],color);idraw->drawLine(c[7],c[4],color);
	idraw->drawLine(c[0],c[4],color);idraw->drawLine(c[1],c[5],color);
	idraw->drawLine(c[2],c[6],color);idraw->drawLine(c[3],c[7],color);
}

//
static void				drawTree(	D_btIDebugDraw* idraw,
								 const D_btDbvtNode* node,
								 int depth,
								 const D_btVector3& ncolor,
								 const D_btVector3& lcolor,
								 int mindepth,
								 int maxdepth)
{
	if(node)
	{
		if(node->isinternal()&&((depth<maxdepth)||(maxdepth<0)))
		{
			drawTree(idraw,node->childs[0],depth+1,ncolor,lcolor,mindepth,maxdepth);
			drawTree(idraw,node->childs[1],depth+1,ncolor,lcolor,mindepth,maxdepth);
		}
		if(depth>=mindepth)
		{
			const D_btScalar	scl=(D_btScalar)(node->isinternal()?1:1);
			const D_btVector3	mi=node->volume.Center()-node->volume.Extents()*scl;
			const D_btVector3	mx=node->volume.Center()+node->volume.Extents()*scl;
			drawBox(idraw,mi,mx,node->isleaf()?lcolor:ncolor);
		}
	}
}

//
template <typename D_T>
static inline D_T				sum(const D_btAlignedObjectArray<D_T>& items)
{
	D_T	v;
	if(items.size())
	{
		v=items[0];
		for(int i=1,ni=items.size();i<ni;++i)
		{
			v+=items[i];
		}
	}
	return(v);
}

//
template <typename D_T,typename Q>
static inline void			add(D_btAlignedObjectArray<D_T>& items,const Q& value)
{
	for(int i=0,ni=items.size();i<ni;++i)
	{
		items[i]+=value;
	}
}

//
template <typename D_T,typename Q>
static inline void			mul(D_btAlignedObjectArray<D_T>& items,const Q& value)
{
	for(int i=0,ni=items.size();i<ni;++i)
	{
		items[i]*=value;
	}
}

//
template <typename D_T>
static inline D_T				average(const D_btAlignedObjectArray<D_T>& items)
{
	const D_btScalar	n=(D_btScalar)(items.size()>0?items.size():1);
	return(sum(items)/n);
}

//
static inline D_btScalar		tetravolume(const D_btVector3& x0,
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
#if 0
static D_btVector3		stresscolor(D_btScalar stress)
{
	static const D_btVector3	spectrum[]=	{	D_btVector3(1,0,1),
		D_btVector3(0,0,1),
		D_btVector3(0,1,1),
		D_btVector3(0,1,0),
		D_btVector3(1,1,0),
		D_btVector3(1,0,0),
		D_btVector3(1,0,0)};
	static const int		ncolors=sizeof(spectrum)/sizeof(spectrum[0])-1;
	static const D_btScalar	one=1;
	stress=D_btMax<D_btScalar>(0,D_btMin<D_btScalar>(1,stress))*ncolors;
	const int				sel=(int)stress;
	const D_btScalar			frc=stress-sel;
	return(spectrum[sel]+(spectrum[sel+1]-spectrum[sel])*frc);
}
#endif

//
void			D_btSoftBodyHelpers::Draw(	D_btSoftBody* psb,
										D_btIDebugDraw* idraw,
										int drawflags)
{
	const D_btScalar		scl=(D_btScalar)0.1;
	const D_btScalar		nscl=scl*5;
	const D_btVector3		lcolor=D_btVector3(0,0,0);
	const D_btVector3		ncolor=D_btVector3(1,1,1);
	const D_btVector3		ccolor=D_btVector3(1,0,0);
	int i,j,nj;

	/* D_Nodes	*/ 
	if(0!=(drawflags&fDrawFlags::D_Nodes))
	{
		for(i=0;i<psb->m_nodes.size();++i)
		{
			const D_btSoftBody::D_Node&	n=psb->m_nodes[i];
			if(0==(n.m_material->m_flags&D_btSoftBody::D_fMaterial::D_DebugDraw)) continue;
			idraw->drawLine(n.m_x-D_btVector3(scl,0,0),n.m_x+D_btVector3(scl,0,0),D_btVector3(1,0,0));
			idraw->drawLine(n.m_x-D_btVector3(0,scl,0),n.m_x+D_btVector3(0,scl,0),D_btVector3(0,1,0));
			idraw->drawLine(n.m_x-D_btVector3(0,0,scl),n.m_x+D_btVector3(0,0,scl),D_btVector3(0,0,1));
		}
	}
	/* D_Links	*/ 
	if(0!=(drawflags&fDrawFlags::D_Links))
	{
		for(i=0;i<psb->m_links.size();++i)
		{
			const D_btSoftBody::D_Link&	l=psb->m_links[i];
			if(0==(l.m_material->m_flags&D_btSoftBody::D_fMaterial::D_DebugDraw)) continue;
			idraw->drawLine(l.m_n[0]->m_x,l.m_n[1]->m_x,lcolor);
		}
	}
	/* D_Normals	*/ 
	if(0!=(drawflags&fDrawFlags::D_Normals))
	{
		for(i=0;i<psb->m_nodes.size();++i)
		{
			const D_btSoftBody::D_Node&	n=psb->m_nodes[i];
			if(0==(n.m_material->m_flags&D_btSoftBody::D_fMaterial::D_DebugDraw)) continue;
			const D_btVector3			d=n.m_n*nscl;
			idraw->drawLine(n.m_x,n.m_x+d,ncolor);
			idraw->drawLine(n.m_x,n.m_x-d,ncolor*0.5);
		}
	}
	/* D_Contacts	*/ 
	if(0!=(drawflags&fDrawFlags::D_Contacts))
	{
		static const D_btVector3		axis[]={D_btVector3(1,0,0),
			D_btVector3(0,1,0),
			D_btVector3(0,0,1)};
		for(i=0;i<psb->m_rcontacts.size();++i)
		{		
			const D_btSoftBody::RContact&	c=psb->m_rcontacts[i];
			const D_btVector3				o=	c.m_node->m_x-c.m_cti.m_normal*
				(D_btDot(c.m_node->m_x,c.m_cti.m_normal)+c.m_cti.m_offset);
			const D_btVector3				x=D_btCross(c.m_cti.m_normal,axis[c.m_cti.m_normal.minAxis()]).normalized();
			const D_btVector3				y=D_btCross(x,c.m_cti.m_normal).normalized();
			idraw->drawLine(o-x*nscl,o+x*nscl,ccolor);
			idraw->drawLine(o-y*nscl,o+y*nscl,ccolor);
			idraw->drawLine(o,o+c.m_cti.m_normal*nscl*3,D_btVector3(1,1,0));
		}
	}
	/* D_Anchors	*/ 
	if(0!=(drawflags&fDrawFlags::D_Anchors))
	{
		for(i=0;i<psb->m_anchors.size();++i)
		{
			const D_btSoftBody::Anchor&	a=psb->m_anchors[i];
			const D_btVector3				q=a.m_body->getWorldTransform()*a.m_local;
			drawVertex(idraw,a.m_node->m_x,0.25,D_btVector3(1,0,0));
			drawVertex(idraw,q,0.25,D_btVector3(0,1,0));
			idraw->drawLine(a.m_node->m_x,q,D_btVector3(1,1,1));
		}
		for(i=0;i<psb->m_nodes.size();++i)
		{
			const D_btSoftBody::D_Node&	n=psb->m_nodes[i];		
			if(0==(n.m_material->m_flags&D_btSoftBody::D_fMaterial::D_DebugDraw)) continue;
			if(n.m_im<=0)
			{
				drawVertex(idraw,n.m_x,0.25,D_btVector3(1,0,0));
			}
		}
	}
	/* D_Faces	*/ 
	if(0!=(drawflags&fDrawFlags::D_Faces))
	{
		const D_btScalar	scl=(D_btScalar)0.8;
		const D_btScalar	alp=(D_btScalar)1;
		const D_btVector3	col(0,(D_btScalar)0.7,0);
		for(i=0;i<psb->m_faces.size();++i)
		{
			const D_btSoftBody::D_Face&	f=psb->m_faces[i];
			if(0==(f.m_material->m_flags&D_btSoftBody::D_fMaterial::D_DebugDraw)) continue;
			const D_btVector3			x[]={f.m_n[0]->m_x,f.m_n[1]->m_x,f.m_n[2]->m_x};
			const D_btVector3			c=(x[0]+x[1]+x[2])/3;
			idraw->drawTriangle((x[0]-c)*scl+c,
				(x[1]-c)*scl+c,
				(x[2]-c)*scl+c,
				col,alp);
		}	
	}
	/* D_Clusters	*/ 
	if(0!=(drawflags&fDrawFlags::D_Clusters))
	{
		srand(1806);
		for(i=0;i<psb->m_clusters.size();++i)
		{
			if(psb->m_clusters[i]->m_collide)
			{
				D_btVector3						color(	rand()/(D_btScalar)RAND_MAX,
					rand()/(D_btScalar)RAND_MAX,
					rand()/(D_btScalar)RAND_MAX);
				color=color.normalized()*0.75;
				D_btAlignedObjectArray<D_btVector3>	vertices;
				vertices.resize(psb->m_clusters[i]->m_nodes.size());
				for(j=0,nj=vertices.size();j<nj;++j)
				{				
					vertices[j]=psb->m_clusters[i]->m_nodes[j]->m_x;
				}
				D_HullDesc		hdsc(D_QF_TRIANGLES,vertices.size(),&vertices[0]);
				D_HullResult		hres;
				D_HullLibrary		hlib;
				hdsc.mMaxVertices=vertices.size();
				hlib.CreateConvexHull(hdsc,hres);
				const D_btVector3	center=average(hres.m_OutputVertices);
				add(hres.m_OutputVertices,-center);
				mul(hres.m_OutputVertices,(D_btScalar)1);
				add(hres.m_OutputVertices,center);
				for(j=0;j<(int)hres.mNumFaces;++j)
				{
					const int idx[]={hres.m_Indices[j*3+0],hres.m_Indices[j*3+1],hres.m_Indices[j*3+2]};
					idraw->drawTriangle(hres.m_OutputVertices[idx[0]],
						hres.m_OutputVertices[idx[1]],
						hres.m_OutputVertices[idx[2]],
						color,1);
				}
				hlib.ReleaseResult(hres);
			}
			/* D_Velocities	*/ 
#if 0
			for(int j=0;j<psb->m_clusters[i].m_nodes.size();++j)
			{
				const D_btSoftBody::Cluster&	c=psb->m_clusters[i];
				const D_btVector3				r=c.m_nodes[j]->m_x-c.m_com;
				const D_btVector3				v=c.m_lv+D_btCross(c.m_av,r);
				idraw->drawLine(c.m_nodes[j]->m_x,c.m_nodes[j]->m_x+v,D_btVector3(1,0,0));
			}
#endif
			/* Frame		*/ 
			D_btSoftBody::Cluster& c=*psb->m_clusters[i];
			idraw->drawLine(c.m_com,c.m_framexform*D_btVector3(10,0,0),D_btVector3(1,0,0));
			idraw->drawLine(c.m_com,c.m_framexform*D_btVector3(0,10,0),D_btVector3(0,1,0));
			idraw->drawLine(c.m_com,c.m_framexform*D_btVector3(0,0,10),D_btVector3(0,0,1));
		}
	}

	/* D_Tetras	*/ 
	if(0!=(drawflags&fDrawFlags::D_Tetras))
	{
		const D_btScalar	scl=(D_btScalar)0.8;
		const D_btScalar	alp=(D_btScalar)1;
		const D_btVector3	col((D_btScalar)0.7,(D_btScalar)0.7,(D_btScalar)0.7);
		for(int i=0;i<psb->m_tetras.size();++i)
		{
			const D_btSoftBody::Tetra&	t=psb->m_tetras[i];
			if(0==(t.m_material->m_flags&D_btSoftBody::D_fMaterial::D_DebugDraw)) continue;
			const D_btVector3				x[]={t.m_n[0]->m_x,t.m_n[1]->m_x,t.m_n[2]->m_x,t.m_n[3]->m_x};
			const D_btVector3				c=(x[0]+x[1]+x[2]+x[3])/4;
			idraw->drawTriangle((x[0]-c)*scl+c,(x[1]-c)*scl+c,(x[2]-c)*scl+c,col,alp);
			idraw->drawTriangle((x[0]-c)*scl+c,(x[1]-c)*scl+c,(x[3]-c)*scl+c,col,alp);
			idraw->drawTriangle((x[1]-c)*scl+c,(x[2]-c)*scl+c,(x[3]-c)*scl+c,col,alp);
			idraw->drawTriangle((x[2]-c)*scl+c,(x[0]-c)*scl+c,(x[3]-c)*scl+c,col,alp);
		}	
	}

	/* D_Notes	*/ 
	if(0!=(drawflags&fDrawFlags::D_Notes))
	{
		for(i=0;i<psb->m_notes.size();++i)
		{
			const D_btSoftBody::Note&	n=psb->m_notes[i];
			D_btVector3				p=n.m_offset;
			for(int j=0;j<n.m_rank;++j)
			{
				p+=n.m_nodes[j]->m_x*n.m_coords[j];
			}
			idraw->draw3dText(p,n.m_text);
		}
	}
	/* D_Node tree	*/ 
	if(0!=(drawflags&fDrawFlags::D_NodeTree))		DrawNodeTree(psb,idraw);
	/* D_Face tree	*/ 
	if(0!=(drawflags&fDrawFlags::D_FaceTree))		DrawFaceTree(psb,idraw);
	/* Cluster tree	*/ 
	if(0!=(drawflags&fDrawFlags::D_ClusterTree))	DrawClusterTree(psb,idraw);
	/* D_Joints		*/ 
	if(0!=(drawflags&fDrawFlags::D_Joints))
	{
		for(i=0;i<psb->m_joints.size();++i)
		{
			const D_btSoftBody::Joint*	pj=psb->m_joints[i];
			switch(pj->Type())
			{
			case	D_btSoftBody::Joint::D_eType::D_Linear:
				{
					const D_btSoftBody::LJoint*	pjl=(const D_btSoftBody::LJoint*)pj;
					const D_btVector3	a0=pj->m_bodies[0].xform()*pjl->m_refs[0];
					const D_btVector3	a1=pj->m_bodies[1].xform()*pjl->m_refs[1];
					idraw->drawLine(pj->m_bodies[0].xform().getOrigin(),a0,D_btVector3(1,1,0));
					idraw->drawLine(pj->m_bodies[1].xform().getOrigin(),a1,D_btVector3(0,1,1));
					drawVertex(idraw,a0,0.25,D_btVector3(1,1,0));
					drawVertex(idraw,a1,0.25,D_btVector3(0,1,1));
				}
				break;
			case	D_btSoftBody::Joint::D_eType::D_Angular:
				{
					//const D_btSoftBody::AJoint*	pja=(const D_btSoftBody::AJoint*)pj;
					const D_btVector3	o0=pj->m_bodies[0].xform().getOrigin();
					const D_btVector3	o1=pj->m_bodies[1].xform().getOrigin();
					const D_btVector3	a0=pj->m_bodies[0].xform().getBasis()*pj->m_refs[0];
					const D_btVector3	a1=pj->m_bodies[1].xform().getBasis()*pj->m_refs[1];
					idraw->drawLine(o0,o0+a0*10,D_btVector3(1,1,0));
					idraw->drawLine(o0,o0+a1*10,D_btVector3(1,1,0));
					idraw->drawLine(o1,o1+a0*10,D_btVector3(0,1,1));
					idraw->drawLine(o1,o1+a1*10,D_btVector3(0,1,1));
				}
			}		
		}
	}
}

//
void			D_btSoftBodyHelpers::DrawInfos(		D_btSoftBody* psb,
											 D_btIDebugDraw* idraw,
											 bool masses,
											 bool areas,
											 bool /*stress*/)
{
	for(int i=0;i<psb->m_nodes.size();++i)
	{
		const D_btSoftBody::D_Node&	n=psb->m_nodes[i];
		char					text[2048]={0};
		char					buff[1024];
		if(masses)
		{
			sprintf(buff," M(%.2f)",1/n.m_im);
			strcat(text,buff);
		}
		if(areas)
		{
			sprintf(buff," A(%.2f)",n.m_area);
			strcat(text,buff);
		}
		if(text[0]) idraw->draw3dText(n.m_x,text);
	}
}

//
void			D_btSoftBodyHelpers::DrawNodeTree(	D_btSoftBody* psb,
												D_btIDebugDraw* idraw,
												int mindepth,
												int maxdepth)
{
	drawTree(idraw,psb->m_ndbvt.m_root,0,D_btVector3(1,0,1),D_btVector3(1,1,1),mindepth,maxdepth);
}

//
void			D_btSoftBodyHelpers::DrawFaceTree(	D_btSoftBody* psb,
												D_btIDebugDraw* idraw,
												int mindepth,
												int maxdepth)
{
	drawTree(idraw,psb->m_fdbvt.m_root,0,D_btVector3(0,1,0),D_btVector3(1,0,0),mindepth,maxdepth);
}

//
void			D_btSoftBodyHelpers::DrawClusterTree(	D_btSoftBody* psb,
												   D_btIDebugDraw* idraw,
												   int mindepth,
												   int maxdepth)
{
	drawTree(idraw,psb->m_cdbvt.m_root,0,D_btVector3(0,1,1),D_btVector3(1,0,0),mindepth,maxdepth);
}

//
void			D_btSoftBodyHelpers::DrawFrame(		D_btSoftBody* psb,
											 D_btIDebugDraw* idraw)
{
	if(psb->m_pose.m_bframe)
	{
		static const D_btScalar	ascl=10;
		static const D_btScalar	nscl=(D_btScalar)0.1;
		const D_btVector3			com=psb->m_pose.m_com;
		const D_btMatrix3x3		trs=psb->m_pose.m_rot*psb->m_pose.m_scl;
		const D_btVector3			Xaxis=(trs*D_btVector3(1,0,0)).normalized();
		const D_btVector3			Yaxis=(trs*D_btVector3(0,1,0)).normalized();
		const D_btVector3			Zaxis=(trs*D_btVector3(0,0,1)).normalized();
		idraw->drawLine(com,com+Xaxis*ascl,D_btVector3(1,0,0));
		idraw->drawLine(com,com+Yaxis*ascl,D_btVector3(0,1,0));
		idraw->drawLine(com,com+Zaxis*ascl,D_btVector3(0,0,1));
		for(int i=0;i<psb->m_pose.m_pos.size();++i)
		{
			const D_btVector3	x=com+trs*psb->m_pose.m_pos[i];
			drawVertex(idraw,x,nscl,D_btVector3(1,0,1));
		}
	}
}

//
D_btSoftBody*		D_btSoftBodyHelpers::CreateRope(	D_btSoftBodyWorldInfo& worldInfo, const D_btVector3& from,
											  const D_btVector3& D_to,
											  int res,
											  int fixeds)
{
	/* Create nodes	*/ 
	const int		r=res+2;
	D_btVector3*		x=new D_btVector3[r];
	D_btScalar*		m=new D_btScalar[r];
	int i;

	for(i=0;i<r;++i)
	{
		const D_btScalar	t=i/(D_btScalar)(r-1);
		x[i]=lerp(from,D_to,t);
		m[i]=1;
	}
	D_btSoftBody*		psb= new D_btSoftBody(&worldInfo,r,x,m);
	if(fixeds&1) psb->setMass(0,0);
	if(fixeds&2) psb->setMass(r-1,0);
	delete[] x;
	delete[] m;
	/* Create links	*/ 
	for(i=1;i<r;++i)
	{
		psb->appendLink(i-1,i);
	}
	/* Finished		*/ 
	return(psb);
}

//
D_btSoftBody*		D_btSoftBodyHelpers::CreatePatch(D_btSoftBodyWorldInfo& worldInfo,const D_btVector3& corner00,
											   const D_btVector3& corner10,
											   const D_btVector3& corner01,
											   const D_btVector3& corner11,
											   int resx,
											   int resy,
											   int fixeds,
											   bool gendiags)
{
#define D_IDX(_x_,_y_)	((_y_)*rx+(_x_))
	/* Create nodes	*/ 
	if((resx<2)||(resy<2)) return(0);
	const int	rx=resx;
	const int	ry=resy;
	const int	tot=rx*ry;
	D_btVector3*	x=new D_btVector3[tot];
	D_btScalar*	m=new D_btScalar[tot];
	int iy;

	for(iy=0;iy<ry;++iy)
	{
		const D_btScalar	ty=iy/(D_btScalar)(ry-1);
		const D_btVector3	py0=lerp(corner00,corner01,ty);
		const D_btVector3	py1=lerp(corner10,corner11,ty);
		for(int ix=0;ix<rx;++ix)
		{
			const D_btScalar	tx=ix/(D_btScalar)(rx-1);
			x[D_IDX(ix,iy)]=lerp(py0,py1,tx);
			m[D_IDX(ix,iy)]=1;
		}
	}
	D_btSoftBody*		psb=new D_btSoftBody(&worldInfo,tot,x,m);
	if(fixeds&1)	psb->setMass(D_IDX(0,0),0);
	if(fixeds&2)	psb->setMass(D_IDX(rx-1,0),0);
	if(fixeds&4)	psb->setMass(D_IDX(0,ry-1),0);
	if(fixeds&8)	psb->setMass(D_IDX(rx-1,ry-1),0);
	delete[] x;
	delete[] m;
	/* Create links	D_and faces */ 
	for(iy=0;iy<ry;++iy)
	{
		for(int ix=0;ix<rx;++ix)
		{
			const int	idx=D_IDX(ix,iy);
			const bool	mdx=(ix+1)<rx;
			const bool	mdy=(iy+1)<ry;
			if(mdx) psb->appendLink(idx,D_IDX(ix+1,iy));
			if(mdy) psb->appendLink(idx,D_IDX(ix,iy+1));
			if(mdx&&mdy)
			{
				if((ix+iy)&1)
				{
					psb->appendFace(D_IDX(ix,iy),D_IDX(ix+1,iy),D_IDX(ix+1,iy+1));
					psb->appendFace(D_IDX(ix,iy),D_IDX(ix+1,iy+1),D_IDX(ix,iy+1));
					if(gendiags)
					{
						psb->appendLink(D_IDX(ix,iy),D_IDX(ix+1,iy+1));
					}
				}
				else
				{
					psb->appendFace(D_IDX(ix,iy+1),D_IDX(ix,iy),D_IDX(ix+1,iy));
					psb->appendFace(D_IDX(ix,iy+1),D_IDX(ix+1,iy),D_IDX(ix+1,iy+1));
					if(gendiags)
					{
						psb->appendLink(D_IDX(ix+1,iy),D_IDX(ix,iy+1));
					}
				}
			}
		}
	}
	/* Finished		*/ 
#undef D_IDX
	return(psb);
}

//
D_btSoftBody*		D_btSoftBodyHelpers::CreatePatchUV(D_btSoftBodyWorldInfo& worldInfo,
												 const D_btVector3& corner00,
												 const D_btVector3& corner10,
												 const D_btVector3& corner01,
												 const D_btVector3& corner11,
												 int resx,
												 int resy,
												 int fixeds,
												 bool gendiags,
												 float* tex_coords)
{

	/*
	*
	*  corners:
	*
	*  [0][0]     corner00 ------- corner01   [resx][0]
	*                |                |
	*                |                |
	*  [0][resy]  corner10 -------- corner11  [resx][resy]
	*
	*
	*
	*
	*
	*
	*   "fixedgs" map:
	*
	*  corner00     -->   +1
	*  corner01     -->   +2
	*  corner10     -->   +4
	*  corner11     -->   +8
	*  upper middle -->  +16
	*  left middle  -->  +32
	*  right middle -->  +64
	*  lower middle --> +128
	*  center       --> +256
	*
	*
	*   tex_coords size   (resx-1)*(resy-1)*12
	*
	*
	*
	*     SINGLE QUAD INTERNALS
	*
	*  1) D_btSoftBody's nodes D_and links,
	*     diagonal link D_is optional ("gendiags")
	*
	*
	*    node00 ------ node01
	*      | .              
	*      |   .            
	*      |     .          
	*      |       .        
	*      |         .      
	*    node10        node11
	*
	*
	*
	*   2) D_Faces:
	*      two triangles,
	*      UV Coordinates (hier example for single quad)
	*      
	*     (0,1)          (0,1)  (1,1)
	*     1 |\            3 \-----| 2
	*       | \              \    |
	*       |  \              \   |
	*       |   \              \  |
	*       |    \              \ |
	*     2 |-----\ 3            \| 1
	*     (0,0)    (1,0)       (1,0)
	*
	*
	*
	*
	*
	*
	*/

#define D_IDX(_x_,_y_)	((_y_)*rx+(_x_))
	/* Create nodes		*/ 
	if((resx<2)||(resy<2)) return(0);
	const int	rx=resx;
	const int	ry=resy;
	const int	tot=rx*ry;
	D_btVector3*	x=new D_btVector3[tot];
	D_btScalar*	m=new D_btScalar[tot];

	int iy;

	for(iy=0;iy<ry;++iy)
	{
		const D_btScalar	ty=iy/(D_btScalar)(ry-1);
		const D_btVector3	py0=lerp(corner00,corner01,ty);
		const D_btVector3	py1=lerp(corner10,corner11,ty);
		for(int ix=0;ix<rx;++ix)
		{
			const D_btScalar	tx=ix/(D_btScalar)(rx-1);
			x[D_IDX(ix,iy)]=lerp(py0,py1,tx);
			m[D_IDX(ix,iy)]=1;
		}
	}
	D_btSoftBody*	psb=new D_btSoftBody(&worldInfo,tot,x,m);
	if(fixeds&1)		psb->setMass(D_IDX(0,0),0);
	if(fixeds&2)		psb->setMass(D_IDX(rx-1,0),0);
	if(fixeds&4)		psb->setMass(D_IDX(0,ry-1),0);
	if(fixeds&8)		psb->setMass(D_IDX(rx-1,ry-1),0);
	if(fixeds&16)		psb->setMass(D_IDX((rx-1)/2,0),0);
	if(fixeds&32)		psb->setMass(D_IDX(0,(ry-1)/2),0);
	if(fixeds&64)		psb->setMass(D_IDX(rx-1,(ry-1)/2),0);
	if(fixeds&128)		psb->setMass(D_IDX((rx-1)/2,ry-1),0);
	if(fixeds&256)		psb->setMass(D_IDX((rx-1)/2,(ry-1)/2),0);
	delete[] x;
	delete[] m;


	int z = 0;
	/* Create links	D_and faces	*/ 
	for(iy=0;iy<ry;++iy)
	{
		for(int ix=0;ix<rx;++ix)
		{
			const bool	mdx=(ix+1)<rx;
			const bool	mdy=(iy+1)<ry;

			int node00=D_IDX(ix,iy);
			int node01=D_IDX(ix+1,iy);
			int node10=D_IDX(ix,iy+1);
			int node11=D_IDX(ix+1,iy+1);

			if(mdx) psb->appendLink(node00,node01);
			if(mdy) psb->appendLink(node00,node10);
			if(mdx&&mdy)
			{
				psb->appendFace(node00,node10,node11);
				if (tex_coords) {
					tex_coords[z+0]=CalculateUV(resx,resy,ix,iy,0);
					tex_coords[z+1]=CalculateUV(resx,resy,ix,iy,1);
					tex_coords[z+2]=CalculateUV(resx,resy,ix,iy,0);
					tex_coords[z+3]=CalculateUV(resx,resy,ix,iy,2);
					tex_coords[z+4]=CalculateUV(resx,resy,ix,iy,3);
					tex_coords[z+5]=CalculateUV(resx,resy,ix,iy,2);
				}
				psb->appendFace(node11,node01,node00);
				if (tex_coords) {
					tex_coords[z+6 ]=CalculateUV(resx,resy,ix,iy,3);
					tex_coords[z+7 ]=CalculateUV(resx,resy,ix,iy,2);
					tex_coords[z+8 ]=CalculateUV(resx,resy,ix,iy,3);
					tex_coords[z+9 ]=CalculateUV(resx,resy,ix,iy,1);
					tex_coords[z+10]=CalculateUV(resx,resy,ix,iy,0);
					tex_coords[z+11]=CalculateUV(resx,resy,ix,iy,1);
				}
				if (gendiags) psb->appendLink(node00,node11);
				z += 12;
			}
		}
	}
	/* Finished	*/ 
#undef D_IDX
	return(psb);
}

float   D_btSoftBodyHelpers::CalculateUV(int resx,int resy,int ix,int iy,int id)
{

	/*
	*
	*
	*    node00 --- node01
	*      |          |
	*    node10 --- node11
	*
	*
	*   ID map:
	*
	*   node00 s --> 0
	*   node00 t --> 1
	*
	*   node01 s --> 3
	*   node01 t --> 1
	*
	*   node10 s --> 0
	*   node10 t --> 2
	*
	*   node11 s --> 3
	*   node11 t --> 2
	*
	*
	*/

	float tc=0.0f;
	if (id == 0) {
		tc = (1.0f/((resx-1))*ix);
	}
	else if (id==1) {
		tc = (1.0f/((resy-1))*(resy-1-iy));
	}
	else if (id==2) {
		tc = (1.0f/((resy-1))*(resy-1-iy-1));
	}
	else if (id==3) {
		tc = (1.0f/((resx-1))*(ix+1));
	}
	return tc;
}
//
D_btSoftBody*		D_btSoftBodyHelpers::CreateEllipsoid(D_btSoftBodyWorldInfo& worldInfo,const D_btVector3& center,
												   const D_btVector3& radius,
												   int res)
{
	struct	Hammersley
	{
		static void	Generate(D_btVector3* x,int n)
		{
			for(int i=0;i<n;i++)
			{
				D_btScalar	p=0.5,t=0;
				for(int j=i;j;p*=0.5,j>>=1) if(j&1) t+=p;
				D_btScalar	w=2*t-1;
				D_btScalar	a=(D_SIMD_PI+2*i*D_SIMD_PI)/n;
				D_btScalar	s=D_btSqrt(1-w*w);
				*x++=D_btVector3(s*D_btCos(a),s*D_btSin(a),w);
			}
		}
	};
	D_btAlignedObjectArray<D_btVector3>	vtx;
	vtx.resize(3+res);
	Hammersley::Generate(&vtx[0],vtx.size());
	for(int i=0;i<vtx.size();++i)
	{
		vtx[i]=vtx[i]*radius+center;
	}
	return(CreateFromConvexHull(worldInfo,&vtx[0],vtx.size()));
}



//
D_btSoftBody*		D_btSoftBodyHelpers::CreateFromTriMesh(D_btSoftBodyWorldInfo& worldInfo,const D_btScalar*	vertices,
													 const int* triangles,
													 int ntriangles)
{
	int		maxidx=0;
	int i,j,ni;

	for(i=0,ni=ntriangles*3;i<ni;++i)
	{
		maxidx=D_btMax(triangles[i],maxidx);
	}
	++maxidx;
	D_btAlignedObjectArray<bool>		chks;
	D_btAlignedObjectArray<D_btVector3>	vtx;
	chks.resize(maxidx*maxidx,false);
	vtx.resize(maxidx);
	for(i=0,j=0,ni=maxidx*3;i<ni;++j,i+=3)
	{
		vtx[j]=D_btVector3(vertices[i],vertices[i+1],vertices[i+2]);
	}
	D_btSoftBody*		psb=new D_btSoftBody(&worldInfo,vtx.size(),&vtx[0],0);
	for( i=0,ni=ntriangles*3;i<ni;i+=3)
	{
		const int idx[]={triangles[i],triangles[i+1],triangles[i+2]};
#define D_IDX(_x_,_y_) ((_y_)*maxidx+(_x_))
		for(int j=2,k=0;k<3;j=k++)
		{
			if(!chks[D_IDX(idx[j],idx[k])])
			{
				chks[D_IDX(idx[j],idx[k])]=true;
				chks[D_IDX(idx[k],idx[j])]=true;
				psb->appendLink(idx[j],idx[k]);
			}
		}
#undef D_IDX
		psb->appendFace(idx[0],idx[1],idx[2]);
	}
	psb->randomizeConstraints();
	return(psb);
}

//
D_btSoftBody*		D_btSoftBodyHelpers::CreateFromConvexHull(D_btSoftBodyWorldInfo& worldInfo,	const D_btVector3* vertices,
														int nvertices)
{
	D_HullDesc		hdsc(D_QF_TRIANGLES,nvertices,vertices);
	D_HullResult		hres;
	D_HullLibrary		hlib;/*??*/ 
	hdsc.mMaxVertices=nvertices;
	hlib.CreateConvexHull(hdsc,hres);
	D_btSoftBody*		psb=new D_btSoftBody(&worldInfo,(int)hres.mNumOutputVertices,
		&hres.m_OutputVertices[0],0);
	for(int i=0;i<(int)hres.mNumFaces;++i)
	{
		const int idx[]={	hres.m_Indices[i*3+0],
			hres.m_Indices[i*3+1],
			hres.m_Indices[i*3+2]};
		if(idx[0]<idx[1]) psb->appendLink(	idx[0],idx[1]);
		if(idx[1]<idx[2]) psb->appendLink(	idx[1],idx[2]);
		if(idx[2]<idx[0]) psb->appendLink(	idx[2],idx[0]);
		psb->appendFace(idx[0],idx[1],idx[2]);
	}
	hlib.ReleaseResult(hres);
	psb->randomizeConstraints();
	return(psb);
}




static int nextLine(const char* buffer)
{
	int numBytesRead=0;

	while (*buffer != '\n')
	{
		buffer++;
		numBytesRead++;
	}

	
	if (buffer[0]==0x0a)
	{
		buffer++;
		numBytesRead++;
	}
	return numBytesRead;
}

/* Create from TetGen .ele, .face, .node data							*/ 
D_btSoftBody*	D_btSoftBodyHelpers::CreateFromTetGenData(D_btSoftBodyWorldInfo& worldInfo,
													const char* ele,
													const char* face,
													const char* node,
													bool bfacelinks,
													bool btetralinks,
													bool bfacesfromtetras)
{
D_btAlignedObjectArray<D_btVector3>	pos;
int								nnode=0;
int								ndims=0;
int								nattrb=0;
int								hasbounds=0;
int result = sscanf(node,"%d %d %d %d",&nnode,&ndims,&nattrb,&hasbounds);
result = sscanf(node,"%d %d %d %d",&nnode,&ndims,&nattrb,&hasbounds);
node += nextLine(node);

pos.resize(nnode);
for(int i=0;i<pos.size();++i)
	{
	int			index=0;
	//int			bound=0;
	float	x,y,z;
	sscanf(node,"%d %f %f %f",&index,&x,&y,&z);

//	sn>>index;
//	sn>>x;sn>>y;sn>>z;
	node += nextLine(node);

	//for(int j=0;j<nattrb;++j) 
	//	sn>>a;

	//if(hasbounds) 
	//	sn>>bound;

	pos[index].setX(D_btScalar(x));
	pos[index].setY(D_btScalar(y));
	pos[index].setZ(D_btScalar(z));
	}
D_btSoftBody*						psb=new D_btSoftBody(&worldInfo,nnode,&pos[0],0);
#if 0
if(face&&face[0])
	{
	int								nface=0;
	sf>>nface;sf>>hasbounds;
	for(int i=0;i<nface;++i)
		{
		int			index=0;
		int			bound=0;
		int			ni[3];
		sf>>index;
		sf>>ni[0];sf>>ni[1];sf>>ni[2];
		sf>>bound;
		psb->appendFace(ni[0],ni[1],ni[2]);	
		if(btetralinks)
			{
			psb->appendLink(ni[0],ni[1],0,true);
			psb->appendLink(ni[1],ni[2],0,true);
			psb->appendLink(ni[2],ni[0],0,true);
			}
		}
	}
#endif

if(ele&&ele[0])
	{
	int								ntetra=0;
	int								ncorner=0;
	int								neattrb=0;
	sscanf(ele,"%d %d %d",&ntetra,&ncorner,&neattrb);
	ele += nextLine(ele);
	
	//se>>ntetra;se>>ncorner;se>>neattrb;
	for(int i=0;i<ntetra;++i)
		{
		int			index=0;
		int			ni[4];

		//se>>index;
		//se>>ni[0];se>>ni[1];se>>ni[2];se>>ni[3];
		sscanf(ele,"%d %d %d %d %d",&index,&ni[0],&ni[1],&ni[2],&ni[3]);
		ele+=nextLine(ele);
		//for(int j=0;j<neattrb;++j) 
		//	se>>a;
		psb->appendTetra(ni[0],ni[1],ni[2],ni[3]);
		if(btetralinks)
			{
			psb->appendLink(ni[0],ni[1],0,true);
			psb->appendLink(ni[1],ni[2],0,true);
			psb->appendLink(ni[2],ni[0],0,true);
			psb->appendLink(ni[0],ni[3],0,true);
			psb->appendLink(ni[1],ni[3],0,true);
			psb->appendLink(ni[2],ni[3],0,true);
			}
		}
	}
printf("D_Nodes:  %u\r\n",psb->m_nodes.size());
printf("D_Links:  %u\r\n",psb->m_links.size());
printf("D_Faces:  %u\r\n",psb->m_faces.size());
printf("D_Tetras: %u\r\n",psb->m_tetras.size());
return(psb);
}

