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

#include "btSoftBodyInternals.h"

//
D_btSoftBody::D_btSoftBody(D_btSoftBodyWorldInfo*	worldInfo,int node_count,  const D_btVector3* x,  const D_btScalar* m)
:m_worldInfo(worldInfo)
{	
	/* Init		*/ 
	m_internalType		=	D_CO_SOFT_BODY;
	m_cfg.aeromodel		=	D_eAeroModel::D_V_Point;
	m_cfg.kVCF			=	1;
	m_cfg.kDG			=	0;
	m_cfg.kLF			=	0;
	m_cfg.kDP			=	0;
	m_cfg.kPR			=	0;
	m_cfg.kVC			=	0;
	m_cfg.kDF			=	(D_btScalar)0.2;
	m_cfg.kMT			=	0;
	m_cfg.kCHR			=	(D_btScalar)1.0;
	m_cfg.kKHR			=	(D_btScalar)0.1;
	m_cfg.kSHR			=	(D_btScalar)1.0;
	m_cfg.kAHR			=	(D_btScalar)0.7;
	m_cfg.kSRHR_CL		=	(D_btScalar)0.1;
	m_cfg.kSKHR_CL		=	(D_btScalar)1;
	m_cfg.kSSHR_CL		=	(D_btScalar)0.5;
	m_cfg.kSR_SPLT_CL	=	(D_btScalar)0.5;
	m_cfg.kSK_SPLT_CL	=	(D_btScalar)0.5;
	m_cfg.kSS_SPLT_CL	=	(D_btScalar)0.5;
	m_cfg.maxvolume		=	(D_btScalar)1;
	m_cfg.timescale		=	1;
	m_cfg.viterations	=	0;
	m_cfg.piterations	=	1;	
	m_cfg.diterations	=	0;
	m_cfg.citerations	=	4;
	m_cfg.collisions	=	D_fCollision::D_Default;
	m_pose.m_bvolume	=	false;
	m_pose.m_bframe		=	false;
	m_pose.m_volume		=	0;
	m_pose.m_com		=	D_btVector3(0,0,0);
	m_pose.m_rot.setIdentity();
	m_pose.m_scl.setIdentity();
	m_tag				=	0;
	m_timeacc			=	0;
	m_bUpdateRtCst		=	true;
	m_bounds[0]			=	D_btVector3(0,0,0);
	m_bounds[1]			=	D_btVector3(0,0,0);
	m_worldTransform.setIdentity();
	setSolver(eSolverPresets::D_Positions);
	/* D_Default material	*/ 
	Material*	pm=appendMaterial();
	pm->m_kLST	=	1;
	pm->m_kAST	=	1;
	pm->m_kVST	=	1;
	pm->m_flags	=	D_fMaterial::D_Default;
	/* Collision shape	*/ 
	///for now, create a collision shape internally
	m_collisionShape = new D_btSoftBodyCollisionShape(this);
	m_collisionShape->setMargin(0.25);
	/* D_Nodes			*/ 
	const D_btScalar		margin=getCollisionShape()->getMargin();
	m_nodes.resize(node_count);
	for(int i=0,ni=node_count;i<ni;++i)
	{	
		D_Node&	n=m_nodes[i];
		ZeroInitialize(n);
		n.m_x		=	x?*x++:D_btVector3(0,0,0);
		n.m_q		=	n.m_x;
		n.m_im		=	m?*m++:1;
		n.m_im		=	n.m_im>0?1/n.m_im:0;
		n.m_leaf	=	m_ndbvt.insert(D_btDbvtVolume::FromCR(n.m_x,margin),&n);
		n.m_material=	pm;
	}
	updateBounds();	

	m_initialWorldTransform.setIdentity();
}

//
D_btSoftBody::~D_btSoftBody()
{
	//for now, delete the internal shape
	delete m_collisionShape;	
	int i;

	releaseClusters();
	for(i=0;i<m_materials.size();++i) 
		D_btAlignedFree(m_materials[i]);
	for(i=0;i<m_joints.size();++i) 
		D_btAlignedFree(m_joints[i]);
}

//
bool			D_btSoftBody::checkLink(int node0,int node1) const
{
	return(checkLink(&m_nodes[node0],&m_nodes[node1]));
}

//
bool			D_btSoftBody::checkLink(const D_Node* node0,const D_Node* node1) const
{
	const D_Node*	n[]={node0,node1};
	for(int i=0,ni=m_links.size();i<ni;++i)
	{
		const D_Link&	l=m_links[i];
		if(	(l.m_n[0]==n[0]&&l.m_n[1]==n[1])||
			(l.m_n[0]==n[1]&&l.m_n[1]==n[0]))
		{
			return(true);
		}
	}
	return(false);
}

//
bool			D_btSoftBody::checkFace(int node0,int node1,int node2) const
{
	const D_Node*	n[]={	&m_nodes[node0],
		&m_nodes[node1],
		&m_nodes[node2]};
	for(int i=0,ni=m_faces.size();i<ni;++i)
	{
		const D_Face&	f=m_faces[i];
		int			c=0;
		for(int j=0;j<3;++j)
		{
			if(	(f.m_n[j]==n[0])||
				(f.m_n[j]==n[1])||
				(f.m_n[j]==n[2])) c|=1<<j; else break;
		}
		if(c==7) return(true);
	}
	return(false);
}

//
D_btSoftBody::Material*		D_btSoftBody::appendMaterial()
{
	Material*	pm=new(D_btAlignedAlloc(sizeof(Material),16)) Material();
	if(m_materials.size()>0)
		*pm=*m_materials[0];
	else
		ZeroInitialize(*pm);
	m_materials.push_back(pm);
	return(pm);
}

//
void			D_btSoftBody::appendNote(	const char* text,
									   const D_btVector3& o,
									   const D_btVector4& c,
									   D_Node* n0,
									   D_Node* n1,
									   D_Node* n2,
									   D_Node* n3)
{
	Note	n;
	ZeroInitialize(n);
	n.m_rank		=	0;
	n.m_text		=	text;
	n.m_offset		=	o;
	n.m_coords[0]	=	c.x();
	n.m_coords[1]	=	c.y();
	n.m_coords[2]	=	c.z();
	n.m_coords[3]	=	c.w();
	n.m_nodes[0]	=	n0;n.m_rank+=n0?1:0;
	n.m_nodes[1]	=	n1;n.m_rank+=n1?1:0;
	n.m_nodes[2]	=	n2;n.m_rank+=n2?1:0;
	n.m_nodes[3]	=	n3;n.m_rank+=n3?1:0;
	m_notes.push_back(n);
}

//
void			D_btSoftBody::appendNote(	const char* text,
									   const D_btVector3& o,
									   D_Node* feature)
{
	appendNote(text,o,D_btVector4(1,0,0,0),feature);
}

//
void			D_btSoftBody::appendNote(	const char* text,
									   const D_btVector3& o,
									   D_Link* feature)
{
	static const D_btScalar	w=1/(D_btScalar)2;
	appendNote(text,o,D_btVector4(w,w,0,0),	feature->m_n[0],
		feature->m_n[1]);
}

//
void			D_btSoftBody::appendNote(	const char* text,
									   const D_btVector3& o,
									   D_Face* feature)
{
	static const D_btScalar	w=1/(D_btScalar)3;
	appendNote(text,o,D_btVector4(w,w,w,0),	feature->m_n[0],
		feature->m_n[1],
		feature->m_n[2]);
}

//
void			D_btSoftBody::appendNode(	const D_btVector3& x,D_btScalar m)
{
	if(m_nodes.capacity()==m_nodes.size())
	{
		pointersToIndices();
		m_nodes.reserve(m_nodes.size()*2+1);
		indicesToPointers();
	}
	const D_btScalar	margin=getCollisionShape()->getMargin();
	m_nodes.push_back(D_Node());
	D_Node&			n=m_nodes[m_nodes.size()-1];
	ZeroInitialize(n);
	n.m_x			=	x;
	n.m_q			=	n.m_x;
	n.m_im			=	m>0?1/m:0;
	n.m_material	=	m_materials[0];
	n.m_leaf		=	m_ndbvt.insert(D_btDbvtVolume::FromCR(n.m_x,margin),&n);
}

//
void			D_btSoftBody::appendLink(int model,Material* mat)
{
	D_Link	l;
	if(model>=0)
		l=m_links[model];
	else
	{ ZeroInitialize(l);l.m_material=mat?mat:m_materials[0]; }
	m_links.push_back(l);
}

//
void			D_btSoftBody::appendLink(	int node0,
									   int node1,
									   Material* mat,
									   bool bcheckexist)
{
	appendLink(&m_nodes[node0],&m_nodes[node1],mat,bcheckexist);
}

//
void			D_btSoftBody::appendLink(	D_Node* node0,
									   D_Node* node1,
									   Material* mat,
									   bool bcheckexist)
{
	if((!bcheckexist)||(!checkLink(node0,node1)))
	{
		appendLink(-1,mat);
		D_Link&	l=m_links[m_links.size()-1];
		l.m_n[0]		=	node0;
		l.m_n[1]		=	node1;
		l.m_rl			=	(l.m_n[0]->m_x-l.m_n[1]->m_x).length();
		m_bUpdateRtCst=true;
	}
}

//
void			D_btSoftBody::appendFace(int model,Material* mat)
{
	D_Face	f;
	if(model>=0)
	{ f=m_faces[model]; }
	else
	{ ZeroInitialize(f);f.m_material=mat?mat:m_materials[0]; }
	m_faces.push_back(f);
}

//
void			D_btSoftBody::appendFace(int node0,int node1,int node2,Material* mat)
{
	if (node0==node1)
		return;
	if (node1==node2)
		return;
	if (node2==node0)
		return;

	appendFace(-1,mat);
	D_Face&	f=m_faces[m_faces.size()-1];
	D_btAssert(node0!=node1);
	D_btAssert(node1!=node2);
	D_btAssert(node2!=node0);
	f.m_n[0]	=	&m_nodes[node0];
	f.m_n[1]	=	&m_nodes[node1];
	f.m_n[2]	=	&m_nodes[node2];
	f.m_ra		=	AreaOf(	f.m_n[0]->m_x,
		f.m_n[1]->m_x,
		f.m_n[2]->m_x);	
	m_bUpdateRtCst=true;
}

//
void			D_btSoftBody::appendTetra(int model,Material* mat)
{
Tetra	t;
if(model>=0)
	t=m_tetras[model];
	else
	{ ZeroInitialize(t);t.m_material=mat?mat:m_materials[0]; }
m_tetras.push_back(t);
}

//
void			D_btSoftBody::appendTetra(int node0,
										int node1,
										int node2,
										int node3,
										Material* mat)
{
	appendTetra(-1,mat);
	Tetra&	t=m_tetras[m_tetras.size()-1];
	t.m_n[0]	=	&m_nodes[node0];
	t.m_n[1]	=	&m_nodes[node1];
	t.m_n[2]	=	&m_nodes[node2];
	t.m_n[3]	=	&m_nodes[node3];
	t.m_rv		=	VolumeOf(t.m_n[0]->m_x,t.m_n[1]->m_x,t.m_n[2]->m_x,t.m_n[3]->m_x);
	m_bUpdateRtCst=true;
}

//
void			D_btSoftBody::appendAnchor(int node,D_btRigidBody* body, bool disableCollisionBetweenLinkedBodies)
{
	if (disableCollisionBetweenLinkedBodies)
	{
		if (m_collisionDisabledObjects.findLinearSearch(body)==m_collisionDisabledObjects.size())
		{
			m_collisionDisabledObjects.push_back(body);
		}
	}

	Anchor	a;
	a.m_node			=	&m_nodes[node];
	a.m_body			=	body;
	a.m_local			=	body->getInterpolationWorldTransform().inverse()*a.m_node->m_x;
	a.m_node->m_battach	=	1;
	m_anchors.push_back(a);
}

//
void			D_btSoftBody::appendLinearJoint(const LJoint::D_Specs& specs,Cluster* body0,Body body1)
{
	LJoint*		pj	=	new(D_btAlignedAlloc(sizeof(LJoint),16)) LJoint();
	pj->m_bodies[0]	=	body0;
	pj->m_bodies[1]	=	body1;
	pj->m_refs[0]	=	pj->m_bodies[0].xform().inverse()*specs.position;
	pj->m_refs[1]	=	pj->m_bodies[1].xform().inverse()*specs.position;
	pj->m_cfm		=	specs.cfm;
	pj->m_erp		=	specs.erp;
	pj->m_split		=	specs.split;
	m_joints.push_back(pj);
}

//
void			D_btSoftBody::appendLinearJoint(const LJoint::D_Specs& specs,Body body)
{
	appendLinearJoint(specs,m_clusters[0],body);
}

//
void			D_btSoftBody::appendLinearJoint(const LJoint::D_Specs& specs,D_btSoftBody* body)
{
	appendLinearJoint(specs,m_clusters[0],body->m_clusters[0]);
}

//
void			D_btSoftBody::appendAngularJoint(const AJoint::D_Specs& specs,Cluster* body0,Body body1)
{
	AJoint*		pj	=	new(D_btAlignedAlloc(sizeof(AJoint),16)) AJoint();
	pj->m_bodies[0]	=	body0;
	pj->m_bodies[1]	=	body1;
	pj->m_refs[0]	=	pj->m_bodies[0].xform().inverse().getBasis()*specs.axis;
	pj->m_refs[1]	=	pj->m_bodies[1].xform().inverse().getBasis()*specs.axis;
	pj->m_cfm		=	specs.cfm;
	pj->m_erp		=	specs.erp;
	pj->m_split		=	specs.split;
	pj->m_icontrol	=	specs.icontrol;
	m_joints.push_back(pj);
}

//
void			D_btSoftBody::appendAngularJoint(const AJoint::D_Specs& specs,Body body)
{
	appendAngularJoint(specs,m_clusters[0],body);
}

//
void			D_btSoftBody::appendAngularJoint(const AJoint::D_Specs& specs,D_btSoftBody* body)
{
	appendAngularJoint(specs,m_clusters[0],body->m_clusters[0]);
}

//
void			D_btSoftBody::addForce(const D_btVector3& force)
{
	for(int i=0,ni=m_nodes.size();i<ni;++i) addForce(force,i);
}

//
void			D_btSoftBody::addForce(const D_btVector3& force,int node)
{
	D_Node&	n=m_nodes[node];
	if(n.m_im>0)
	{
		n.m_f	+=	force;
	}
}

//
void			D_btSoftBody::addVelocity(const D_btVector3& velocity)
{
	for(int i=0,ni=m_nodes.size();i<ni;++i) addVelocity(velocity,i);
}

/* Set velocity for the entire body										*/ 
void				D_btSoftBody::setVelocity(	const D_btVector3& velocity)
{
	for(int i=0,ni=m_nodes.size();i<ni;++i) 
	{
		D_Node&	n=m_nodes[i];
		if(n.m_im>0)
		{
			n.m_v	=	velocity;
		}
	}
}


//
void			D_btSoftBody::addVelocity(const D_btVector3& velocity,int node)
{
	D_Node&	n=m_nodes[node];
	if(n.m_im>0)
	{
		n.m_v	+=	velocity;
	}
}

//
void			D_btSoftBody::setMass(int node,D_btScalar mass)
{
	m_nodes[node].m_im=mass>0?1/mass:0;
	m_bUpdateRtCst=true;
}

//
D_btScalar		D_btSoftBody::getMass(int node) const
{
	return(m_nodes[node].m_im>0?1/m_nodes[node].m_im:0);
}

//
D_btScalar		D_btSoftBody::getTotalMass() const
{
	D_btScalar	mass=0;
	for(int i=0;i<m_nodes.size();++i)
	{
		mass+=getMass(i);
	}
	return(mass);
}

//
void			D_btSoftBody::setTotalMass(D_btScalar mass,bool fromfaces)
{
	int i;

	if(fromfaces)
	{

		for(i=0;i<m_nodes.size();++i)
		{
			m_nodes[i].m_im=0;
		}
		for(i=0;i<m_faces.size();++i)
		{
			const D_Face&		f=m_faces[i];
			const D_btScalar	twicearea=AreaOf(	f.m_n[0]->m_x,
				f.m_n[1]->m_x,
				f.m_n[2]->m_x);
			for(int j=0;j<3;++j)
			{
				f.m_n[j]->m_im+=twicearea;
			}
		}
		for( i=0;i<m_nodes.size();++i)
		{
			m_nodes[i].m_im=1/m_nodes[i].m_im;
		}
	}
	const D_btScalar	tm=getTotalMass();
	const D_btScalar	itm=1/tm;
	for( i=0;i<m_nodes.size();++i)
	{
		m_nodes[i].m_im/=itm*mass;
	}
	m_bUpdateRtCst=true;
}

//
void			D_btSoftBody::setTotalDensity(D_btScalar density)
{
	setTotalMass(getVolume()*density,true);
}

//
void			D_btSoftBody::setVolumeMass(D_btScalar mass)
{
D_btAlignedObjectArray<D_btScalar>	ranks;
ranks.resize(m_nodes.size(),0);
for(int i=0;i<m_nodes.size();++i)
	{
	m_nodes[i].m_im=0;
	}
for(int i=0;i<m_tetras.size();++i)
	{
	const Tetra& t=m_tetras[i];
	for(int j=0;j<4;++j)
		{
		t.m_n[j]->m_im+=D_btFabs(t.m_rv);
		ranks[int(t.m_n[j]-&m_nodes[0])]+=1;
		}
	}
for(int i=0;i<m_nodes.size();++i)
	{
	if(m_nodes[i].m_im>0)
		{
		m_nodes[i].m_im=ranks[i]/m_nodes[i].m_im;
		}
	}
setTotalMass(mass,false);
}

//
void			D_btSoftBody::setVolumeDensity(D_btScalar density)
{
D_btScalar	volume=0;
for(int i=0;i<m_tetras.size();++i)
	{
	const Tetra& t=m_tetras[i];
	for(int j=0;j<4;++j)
		{
		volume+=D_btFabs(t.m_rv);
		}
	}
setVolumeMass(volume*density/6);
}

//
void			D_btSoftBody::transform(const D_btTransform& trs)
{
	const D_btScalar	margin=getCollisionShape()->getMargin();
	D_ATTRIBUTE_ALIGNED16(D_btDbvtVolume)	vol;
	
	for(int i=0,ni=m_nodes.size();i<ni;++i)
	{
		D_Node&	n=m_nodes[i];
		n.m_x=trs*n.m_x;
		n.m_q=trs*n.m_q;
		n.m_n=trs.getBasis()*n.m_n;
		vol = D_btDbvtVolume::FromCR(n.m_x,margin);
		
		m_ndbvt.update(n.m_leaf,vol);
	}
	updateNormals();
	updateBounds();
	updateConstants();
	m_initialWorldTransform = trs;
}

//
void			D_btSoftBody::translate(const D_btVector3& trs)
{
	D_btTransform	t;
	t.setIdentity();
	t.setOrigin(trs);
	transform(t);
}

//
void			D_btSoftBody::rotate(	const D_btQuaternion& rot)
{
	D_btTransform	t;
	t.setIdentity();
	t.setRotation(rot);
	transform(t);
}

//
void			D_btSoftBody::scale(const D_btVector3& scl)
{
	const D_btScalar	margin=getCollisionShape()->getMargin();
	D_ATTRIBUTE_ALIGNED16(D_btDbvtVolume)	vol;
	
	for(int i=0,ni=m_nodes.size();i<ni;++i)
	{
		D_Node&	n=m_nodes[i];
		n.m_x*=scl;
		n.m_q*=scl;
		vol = D_btDbvtVolume::FromCR(n.m_x,margin);
		m_ndbvt.update(n.m_leaf,vol);
	}
	updateNormals();
	updateBounds();
	updateConstants();
}

//
void			D_btSoftBody::setPose(bool bvolume,bool bframe)
{
	m_pose.m_bvolume	=	bvolume;
	m_pose.m_bframe		=	bframe;
	int i,ni;

	/* Weights		*/ 
	const D_btScalar	omass=getTotalMass();
	const D_btScalar	kmass=omass*m_nodes.size()*1000;
	D_btScalar		tmass=omass;
	m_pose.m_wgh.resize(m_nodes.size());
	for(i=0,ni=m_nodes.size();i<ni;++i)
	{
		if(m_nodes[i].m_im<=0) tmass+=kmass;
	}
	for( i=0,ni=m_nodes.size();i<ni;++i)
	{
		D_Node&	n=m_nodes[i];
		m_pose.m_wgh[i]=	n.m_im>0					?
			1/(m_nodes[i].m_im*tmass)	:
		kmass/tmass;
	}
	/* Pos		*/ 
	const D_btVector3	com=evaluateCom();
	m_pose.m_pos.resize(m_nodes.size());
	for( i=0,ni=m_nodes.size();i<ni;++i)
	{
		m_pose.m_pos[i]=m_nodes[i].m_x-com;
	}
	m_pose.m_volume	=	bvolume?getVolume():0;
	m_pose.m_com	=	com;
	m_pose.m_rot.setIdentity();
	m_pose.m_scl.setIdentity();
	/* Aqq		*/ 
	m_pose.m_aqq[0]	=
		m_pose.m_aqq[1]	=
		m_pose.m_aqq[2]	=	D_btVector3(0,0,0);
	for( i=0,ni=m_nodes.size();i<ni;++i)
	{
		const D_btVector3&	q=m_pose.m_pos[i];
		const D_btVector3		mq=m_pose.m_wgh[i]*q;
		m_pose.m_aqq[0]+=mq.x()*q;
		m_pose.m_aqq[1]+=mq.y()*q;
		m_pose.m_aqq[2]+=mq.z()*q;
	}
	m_pose.m_aqq=m_pose.m_aqq.inverse();
	updateConstants();
}

//
D_btScalar		D_btSoftBody::getVolume() const
{
	D_btScalar	vol=0;
	if(m_nodes.size()>0)
	{
		int i,ni;

		const D_btVector3	org=m_nodes[0].m_x;
		for(i=0,ni=m_faces.size();i<ni;++i)
		{
			const D_Face&	f=m_faces[i];
			vol+=D_btDot(f.m_n[0]->m_x-org,D_btCross(f.m_n[1]->m_x-org,f.m_n[2]->m_x-org));
		}
		vol/=(D_btScalar)6;
	}
	return(vol);
}

//
int				D_btSoftBody::clusterCount() const
{
	return(m_clusters.size());
}

//
D_btVector3		D_btSoftBody::clusterCom(const Cluster* cluster)
{
	D_btVector3		com(0,0,0);
	for(int i=0,ni=cluster->m_nodes.size();i<ni;++i)
	{
		com+=cluster->m_nodes[i]->m_x*cluster->m_masses[i];
	}
	return(com*cluster->m_imass);
}

//
D_btVector3		D_btSoftBody::clusterCom(int cluster) const
{
	return(clusterCom(m_clusters[cluster]));
}

//
D_btVector3		D_btSoftBody::clusterVelocity(const Cluster* cluster,const D_btVector3& rpos)
{
	return(cluster->m_lv+D_btCross(cluster->m_av,rpos));
}

//
void			D_btSoftBody::clusterVImpulse(Cluster* cluster,const D_btVector3& rpos,const D_btVector3& impulse)
{
	const D_btVector3	li=cluster->m_imass*impulse;
	const D_btVector3	ai=cluster->m_invwi*D_btCross(rpos,impulse);
	cluster->m_vimpulses[0]+=li;cluster->m_lv+=li;
	cluster->m_vimpulses[1]+=ai;cluster->m_av+=ai;
	cluster->m_nvimpulses++;
}

//
void			D_btSoftBody::clusterDImpulse(Cluster* cluster,const D_btVector3& rpos,const D_btVector3& impulse)
{
	const D_btVector3	li=cluster->m_imass*impulse;
	const D_btVector3	ai=cluster->m_invwi*D_btCross(rpos,impulse);
	cluster->m_dimpulses[0]+=li;
	cluster->m_dimpulses[1]+=ai;
	cluster->m_ndimpulses++;
}

//
void			D_btSoftBody::clusterImpulse(Cluster* cluster,const D_btVector3& rpos,const Impulse& impulse)
{
	if(impulse.m_asVelocity)	clusterVImpulse(cluster,rpos,impulse.m_velocity);
	if(impulse.m_asDrift)		clusterDImpulse(cluster,rpos,impulse.m_drift);
}

//
void			D_btSoftBody::clusterVAImpulse(Cluster* cluster,const D_btVector3& impulse)
{
	const D_btVector3	ai=cluster->m_invwi*impulse;
	cluster->m_vimpulses[1]+=ai;cluster->m_av+=ai;
	cluster->m_nvimpulses++;
}

//
void			D_btSoftBody::clusterDAImpulse(Cluster* cluster,const D_btVector3& impulse)
{
	const D_btVector3	ai=cluster->m_invwi*impulse;
	cluster->m_dimpulses[1]+=ai;
	cluster->m_ndimpulses++;
}

//
void			D_btSoftBody::clusterAImpulse(Cluster* cluster,const Impulse& impulse)
{
	if(impulse.m_asVelocity)	clusterVAImpulse(cluster,impulse.m_velocity);
	if(impulse.m_asDrift)		clusterDAImpulse(cluster,impulse.m_drift);
}

//
void			D_btSoftBody::clusterDCImpulse(Cluster* cluster,const D_btVector3& impulse)
{
	cluster->m_dimpulses[0]+=impulse*cluster->m_imass;
	cluster->m_ndimpulses++;
}

struct D_NodeLinks
{
    D_btAlignedObjectArray<int> m_links;
};



//
int				D_btSoftBody::generateBendingConstraints(int distance,Material* mat)
{
	int i,j;

	if(distance>1)
	{
		/* Build graph	*/ 
		const int		n=m_nodes.size();
		const unsigned	inf=(~(unsigned)0)>>1;
		unsigned*		adj=new unsigned[n*n];
		

#define D_IDX(_x_,_y_)	((_y_)*n+(_x_))
		for(j=0;j<n;++j)
		{
			for(i=0;i<n;++i)
			{
				if(i!=j)
				{
					adj[D_IDX(i,j)]=adj[D_IDX(j,i)]=inf;
				}
				else
				{
					adj[D_IDX(i,j)]=adj[D_IDX(j,i)]=0;
				}
			}
		}
		for( i=0;i<m_links.size();++i)
		{
			const int	ia=(int)(m_links[i].m_n[0]-&m_nodes[0]);
			const int	ib=(int)(m_links[i].m_n[1]-&m_nodes[0]);
			adj[D_IDX(ia,ib)]=1;
			adj[D_IDX(ib,ia)]=1;
		}


		//special optimized case for distance == 2
		if (distance == 2)
		{

			D_btAlignedObjectArray<D_NodeLinks> nodeLinks;


			/* Build node links */
			nodeLinks.resize(m_nodes.size());

			for( i=0;i<m_links.size();++i)
			{
				const int	ia=(int)(m_links[i].m_n[0]-&m_nodes[0]);
				const int	ib=(int)(m_links[i].m_n[1]-&m_nodes[0]);
				if (nodeLinks[ia].m_links.findLinearSearch(ib)==nodeLinks[ia].m_links.size())
					nodeLinks[ia].m_links.push_back(ib);

				if (nodeLinks[ib].m_links.findLinearSearch(ia)==nodeLinks[ib].m_links.size())
					nodeLinks[ib].m_links.push_back(ia);
			}
			for (int ii=0;ii<nodeLinks.size();ii++)
			{
				int i=ii;

				for (int jj=0;jj<nodeLinks[ii].m_links.size();jj++)
				{
					int k = nodeLinks[ii].m_links[jj];
					for (int kk=0;kk<nodeLinks[k].m_links.size();kk++)
					{
						int j = nodeLinks[k].m_links[kk];
						if (i!=j)
						{
							const unsigned	sum=adj[D_IDX(i,k)]+adj[D_IDX(k,j)];
							D_btAssert(sum==2);
							if(adj[D_IDX(i,j)]>sum)
							{
								adj[D_IDX(i,j)]=adj[D_IDX(j,i)]=sum;
							}
						}

					}
				}
			}
		} else
		{
			///generic Floyd's algorithm
			for(int k=0;k<n;++k)
			{
				for(j=0;j<n;++j)
				{
					for(i=j+1;i<n;++i)
					{
						const unsigned	sum=adj[D_IDX(i,k)]+adj[D_IDX(k,j)];
						if(adj[D_IDX(i,j)]>sum)
						{
							adj[D_IDX(i,j)]=adj[D_IDX(j,i)]=sum;
						}
					}
				}
			}
		}


		/* Build links	*/ 
		int	nlinks=0;
		for(j=0;j<n;++j)
		{
			for(i=j+1;i<n;++i)
			{
				if(adj[D_IDX(i,j)]==(unsigned)distance)
				{
					appendLink(i,j,mat);
					m_links[m_links.size()-1].m_bbending=1;
					++nlinks;
				}
			}
		}
		delete[] adj;		
		return(nlinks);
	}
	return(0);
}

//
void			D_btSoftBody::randomizeConstraints()
{
	unsigned long	seed=243703;
#define D_NEXTRAND (seed=(1664525L*seed+1013904223L)&0xffffffff)
	int i,ni;

	for(i=0,ni=m_links.size();i<ni;++i)
	{
		D_btSwap(m_links[i],m_links[D_NEXTRAND%ni]);
	}
	for(i=0,ni=m_faces.size();i<ni;++i)
	{
		D_btSwap(m_faces[i],m_faces[D_NEXTRAND%ni]);
	}
#undef D_NEXTRAND
}

//
void			D_btSoftBody::releaseCluster(int index)
{
	Cluster*	c=m_clusters[index];
	if(c->m_leaf) m_cdbvt.remove(c->m_leaf);
	c->~Cluster();
	D_btAlignedFree(c);
	m_clusters.remove(c);
}

//
void			D_btSoftBody::releaseClusters()
{
	while(m_clusters.size()>0) releaseCluster(0);
}

//
int				D_btSoftBody::generateClusters(int k,int maxiterations)
{
	int i;
	releaseClusters();
	m_clusters.resize(D_btMin(k,m_nodes.size()));
	for(i=0;i<m_clusters.size();++i)
	{
		m_clusters[i]			=	new(D_btAlignedAlloc(sizeof(Cluster),16)) Cluster();
		m_clusters[i]->m_collide=	true;
	}
	k=m_clusters.size();
	if(k>0)
	{
		/* Initialize		*/ 
		D_btAlignedObjectArray<D_btVector3>	centers;
		D_btVector3						cog(0,0,0);
		int								i;
		for(i=0;i<m_nodes.size();++i)
		{
			cog+=m_nodes[i].m_x;
			m_clusters[(i*29873)%m_clusters.size()]->m_nodes.push_back(&m_nodes[i]);
		}
		cog/=(D_btScalar)m_nodes.size();
		centers.resize(k,cog);
		/* Iterate			*/ 
		const D_btScalar	slope=16;
		bool			changed;
		int				iterations=0;
		do	{
			const D_btScalar	w=2-D_btMin<D_btScalar>(1,iterations/slope);
			changed=false;
			iterations++;	
			int i;

			for(i=0;i<k;++i)
			{
				D_btVector3	c(0,0,0);
				for(int j=0;j<m_clusters[i]->m_nodes.size();++j)
				{
					c+=m_clusters[i]->m_nodes[j]->m_x;
				}
				if(m_clusters[i]->m_nodes.size())
				{
					c			/=	(D_btScalar)m_clusters[i]->m_nodes.size();
					c			=	centers[i]+(c-centers[i])*w;
					changed		|=	((c-centers[i]).length2()>D_SIMD_EPSILON);
					centers[i]	=	c;
					m_clusters[i]->m_nodes.resize(0);
				}			
			}
			for(i=0;i<m_nodes.size();++i)
			{
				const D_btVector3	nx=m_nodes[i].m_x;
				int				kbest=0;
				D_btScalar		kdist=ClusterMetric(centers[0],nx);
				for(int j=1;j<k;++j)
				{
					const D_btScalar	d=ClusterMetric(centers[j],nx);
					if(d<kdist)
					{
						kbest=j;
						kdist=d;
					}
				}
				m_clusters[kbest]->m_nodes.push_back(&m_nodes[i]);
			}		
		} while(changed&&(iterations<maxiterations));
		/* Merge		*/ 
		D_btAlignedObjectArray<int>	cids;
		cids.resize(m_nodes.size(),-1);
		for(i=0;i<m_clusters.size();++i)
		{
			for(int j=0;j<m_clusters[i]->m_nodes.size();++j)
			{
				cids[int(m_clusters[i]->m_nodes[j]-&m_nodes[0])]=i;
			}
		}
		for(i=0;i<m_faces.size();++i)
		{
			const int idx[]={	int(m_faces[i].m_n[0]-&m_nodes[0]),
				int(m_faces[i].m_n[1]-&m_nodes[0]),
				int(m_faces[i].m_n[2]-&m_nodes[0])};
			for(int j=0;j<3;++j)
			{
				const int cid=cids[idx[j]];
				for(int q=1;q<3;++q)
				{
					const int kid=idx[(j+q)%3];
					if(cids[kid]!=cid)
					{
						if(m_clusters[cid]->m_nodes.findLinearSearch(&m_nodes[kid])==m_clusters[cid]->m_nodes.size())
						{
							m_clusters[cid]->m_nodes.push_back(&m_nodes[kid]);
						}
					}
				}
			}
		}
		/* Master		*/ 
		if(m_clusters.size()>1)
		{
			Cluster*	pmaster=new(D_btAlignedAlloc(sizeof(Cluster),16)) Cluster();
			pmaster->m_collide	=	false;
			pmaster->m_nodes.reserve(m_nodes.size());
			for(int i=0;i<m_nodes.size();++i) pmaster->m_nodes.push_back(&m_nodes[i]);
			m_clusters.push_back(pmaster);
			D_btSwap(m_clusters[0],m_clusters[m_clusters.size()-1]);
		}
		/* Terminate	*/ 
		for(i=0;i<m_clusters.size();++i)
		{
			if(m_clusters[i]->m_nodes.size()==0)
			{
				releaseCluster(i--);
			}
		}
	} else
	{
		//create a cluster for each tetrahedron (if tetrahedra exist) or each face
		if (m_tetras.size())
		{
			m_clusters.resize(m_tetras.size());
			for(i=0;i<m_clusters.size();++i)
			{
				m_clusters[i]			=	new(D_btAlignedAlloc(sizeof(Cluster),16)) Cluster();
				m_clusters[i]->m_collide=	true;
			}
			for (i=0;i<m_tetras.size();i++)
			{
				for (int j=0;j<4;j++)
				{
					m_clusters[i]->m_nodes.push_back(m_tetras[i].m_n[j]);
				}
			}

		} else
		{
			m_clusters.resize(m_faces.size());
			for(i=0;i<m_clusters.size();++i)
			{
				m_clusters[i]			=	new(D_btAlignedAlloc(sizeof(Cluster),16)) Cluster();
				m_clusters[i]->m_collide=	true;
			}

			for(i=0;i<m_faces.size();++i)
			{
				for(int j=0;j<3;++j)
				{
					m_clusters[i]->m_nodes.push_back(m_faces[i].m_n[j]);
				}
			}
		}
	}

	if (m_clusters.size())
	{
		initializeClusters();
		updateClusters();


		//for self-collision
		m_clusterConnectivity.resize(m_clusters.size()*m_clusters.size());
		{
			for (int c0=0;c0<m_clusters.size();c0++)
			{
				m_clusters[c0]->m_clusterIndex=c0;
				for (int c1=0;c1<m_clusters.size();c1++)
				{
					
					bool connected=false;
					Cluster* cla = m_clusters[c0];
					Cluster* clb = m_clusters[c1];
					for (int i=0;!connected&&i<cla->m_nodes.size();i++)
					{
						for (int j=0;j<clb->m_nodes.size();j++)
						{
							if (cla->m_nodes[i] == clb->m_nodes[j])
							{
								connected=true;
								break;
							}
						}
					}
					m_clusterConnectivity[c0+c1*m_clusters.size()]=connected;
				}
			}
		}
	}

	return(m_clusters.size());
}

//
void			D_btSoftBody::refine(ImplicitFn* ifn,D_btScalar accurary,bool cut)
{
	const D_Node*			nbase = &m_nodes[0];
	int					ncount = m_nodes.size();
	D_btSymMatrix<int>	edges(ncount,-2);
	int					newnodes=0;
	int i,j,k,ni;

	/* Filter out		*/ 
	for(i=0;i<m_links.size();++i)
	{
		D_Link&	l=m_links[i];
		if(l.m_bbending)
		{
			if(!SameSign(ifn->Eval(l.m_n[0]->m_x),ifn->Eval(l.m_n[1]->m_x)))
			{
				D_btSwap(m_links[i],m_links[m_links.size()-1]);
				m_links.pop_back();--i;
			}
		}	
	}
	/* Fill edges		*/ 
	for(i=0;i<m_links.size();++i)
	{
		D_Link&	l=m_links[i];
		edges(int(l.m_n[0]-nbase),int(l.m_n[1]-nbase))=-1;
	}
	for(i=0;i<m_faces.size();++i)
	{	
		D_Face&	f=m_faces[i];
		edges(int(f.m_n[0]-nbase),int(f.m_n[1]-nbase))=-1;
		edges(int(f.m_n[1]-nbase),int(f.m_n[2]-nbase))=-1;
		edges(int(f.m_n[2]-nbase),int(f.m_n[0]-nbase))=-1;
	}
	/* Intersect		*/ 
	for(i=0;i<ncount;++i)
	{
		for(j=i+1;j<ncount;++j)
		{
			if(edges(i,j)==-1)
			{
				D_Node&			a=m_nodes[i];
				D_Node&			b=m_nodes[j];
				const D_btScalar	t=ImplicitSolve(ifn,a.m_x,b.m_x,accurary);
				if(t>0)
				{
					const D_btVector3	x=Lerp(a.m_x,b.m_x,t);
					const D_btVector3	v=Lerp(a.m_v,b.m_v,t);
					D_btScalar		m=0;
					if(a.m_im>0)
					{
						if(b.m_im>0)
						{
							const D_btScalar	ma=1/a.m_im;
							const D_btScalar	mb=1/b.m_im;
							const D_btScalar	mc=Lerp(ma,mb,t);
							const D_btScalar	f=(ma+mb)/(ma+mb+mc);
							a.m_im=1/(ma*f);
							b.m_im=1/(mb*f);
							m=mc*f;
						}
						else
						{ a.m_im/=0.5;m=1/a.m_im; }
					}
					else
					{
						if(b.m_im>0)
						{ b.m_im/=0.5;m=1/b.m_im; }
						else
							m=0;
					}
					appendNode(x,m);
					edges(i,j)=m_nodes.size()-1;
					m_nodes[edges(i,j)].m_v=v;
					++newnodes;
				}
			}
		}
	}
	nbase=&m_nodes[0];
	/* Refine links		*/ 
	for(i=0,ni=m_links.size();i<ni;++i)
	{
		D_Link&		feat=m_links[i];
		const int	idx[]={	int(feat.m_n[0]-nbase),
			int(feat.m_n[1]-nbase)};
		if((idx[0]<ncount)&&(idx[1]<ncount))
		{
			const int ni=edges(idx[0],idx[1]);
			if(ni>0)
			{
				appendLink(i);
				D_Link*		pft[]={	&m_links[i],
					&m_links[m_links.size()-1]};			
				pft[0]->m_n[0]=&m_nodes[idx[0]];
				pft[0]->m_n[1]=&m_nodes[ni];
				pft[1]->m_n[0]=&m_nodes[ni];
				pft[1]->m_n[1]=&m_nodes[idx[1]];
			}
		}
	}
	/* Refine faces		*/ 
	for(i=0;i<m_faces.size();++i)
	{
		const D_Face&	feat=m_faces[i];
		const int	idx[]={	int(feat.m_n[0]-nbase),
			int(feat.m_n[1]-nbase),
			int(feat.m_n[2]-nbase)};
		for(j=2,k=0;k<3;j=k++)
		{
			if((idx[j]<ncount)&&(idx[k]<ncount))
			{
				const int ni=edges(idx[j],idx[k]);
				if(ni>0)
				{
					appendFace(i);
					const int	l=(k+1)%3;
					D_Face*		pft[]={	&m_faces[i],
						&m_faces[m_faces.size()-1]};
					pft[0]->m_n[0]=&m_nodes[idx[l]];
					pft[0]->m_n[1]=&m_nodes[idx[j]];
					pft[0]->m_n[2]=&m_nodes[ni];
					pft[1]->m_n[0]=&m_nodes[ni];
					pft[1]->m_n[1]=&m_nodes[idx[k]];
					pft[1]->m_n[2]=&m_nodes[idx[l]];
					appendLink(ni,idx[l],pft[0]->m_material);
					--i;break;
				}
			}
		}
	}
	/* Cut				*/ 
	if(cut)
	{	
		D_btAlignedObjectArray<int>	cnodes;
		const int					pcount=ncount;
		int							i;
		ncount=m_nodes.size();
		cnodes.resize(ncount,0);
		/* D_Nodes		*/ 
		for(i=0;i<ncount;++i)
		{
			const D_btVector3	x=m_nodes[i].m_x;
			if((i>=pcount)||(D_btFabs(ifn->Eval(x))<accurary))
			{
				const D_btVector3	v=m_nodes[i].m_v;
				D_btScalar		m=getMass(i);
				if(m>0) { m*=0.5;m_nodes[i].m_im/=0.5; }
				appendNode(x,m);
				cnodes[i]=m_nodes.size()-1;
				m_nodes[cnodes[i]].m_v=v;
			}
		}
		nbase=&m_nodes[0];
		/* D_Links		*/ 
		for(i=0,ni=m_links.size();i<ni;++i)
		{
			const int		id[]={	int(m_links[i].m_n[0]-nbase),
				int(m_links[i].m_n[1]-nbase)};
			int				todetach=0;
			if(cnodes[id[0]]&&cnodes[id[1]])
			{
				appendLink(i);
				todetach=m_links.size()-1;
			}
			else
			{
				if((	(ifn->Eval(m_nodes[id[0]].m_x)<accurary)&&
					(ifn->Eval(m_nodes[id[1]].m_x)<accurary)))
					todetach=i;
			}
			if(todetach)
			{
				D_Link&	l=m_links[todetach];
				for(int j=0;j<2;++j)
				{
					int cn=cnodes[int(l.m_n[j]-nbase)];
					if(cn) l.m_n[j]=&m_nodes[cn];
				}			
			}
		}
		/* D_Faces		*/ 
		for(i=0,ni=m_faces.size();i<ni;++i)
		{
			D_Node**			n=	m_faces[i].m_n;
			if(	(ifn->Eval(n[0]->m_x)<accurary)&&
				(ifn->Eval(n[1]->m_x)<accurary)&&
				(ifn->Eval(n[2]->m_x)<accurary))
			{
				for(int j=0;j<3;++j)
				{
					int cn=cnodes[int(n[j]-nbase)];
					if(cn) n[j]=&m_nodes[cn];
				}
			}
		}
		/* Clean orphans	*/ 
		int							nnodes=m_nodes.size();
		D_btAlignedObjectArray<int>	ranks;
		D_btAlignedObjectArray<int>	todelete;
		ranks.resize(nnodes,0);
		for(i=0,ni=m_links.size();i<ni;++i)
		{
			for(int j=0;j<2;++j) ranks[int(m_links[i].m_n[j]-nbase)]++;
		}
		for(i=0,ni=m_faces.size();i<ni;++i)
		{
			for(int j=0;j<3;++j) ranks[int(m_faces[i].m_n[j]-nbase)]++;
		}
		for(i=0;i<m_links.size();++i)
		{
			const int	id[]={	int(m_links[i].m_n[0]-nbase),
				int(m_links[i].m_n[1]-nbase)};
			const bool	sg[]={	ranks[id[0]]==1,
				ranks[id[1]]==1};
			if(sg[0]||sg[1])
			{
				--ranks[id[0]];
				--ranks[id[1]];
				D_btSwap(m_links[i],m_links[m_links.size()-1]);
				m_links.pop_back();--i;
			}
		}
#if 0	
		for(i=nnodes-1;i>=0;--i)
		{
			if(!ranks[i]) todelete.push_back(i);
		}	
		if(todelete.size())
		{		
			D_btAlignedObjectArray<int>&	map=ranks;
			for(int i=0;i<nnodes;++i) map[i]=i;
			PointersToIndices(this);
			for(int i=0,ni=todelete.size();i<ni;++i)
			{
				int		j=todelete[i];
				int&	a=map[j];
				int&	b=map[--nnodes];
				m_ndbvt.remove(m_nodes[a].m_leaf);m_nodes[a].m_leaf=0;
				D_btSwap(m_nodes[a],m_nodes[b]);
				j=a;a=b;b=j;			
			}
			IndicesToPointers(this,&map[0]);
			m_nodes.resize(nnodes);
		}
#endif
	}
	m_bUpdateRtCst=true;
}

//
bool			D_btSoftBody::cutLink(const D_Node* node0,const D_Node* node1,D_btScalar position)
{
	return(cutLink(int(node0-&m_nodes[0]),int(node1-&m_nodes[0]),position));
}

//
bool			D_btSoftBody::cutLink(int node0,int node1,D_btScalar position)
{
	bool			done=false;
	int i,ni;
	const D_btVector3	d=m_nodes[node0].m_x-m_nodes[node1].m_x;
	const D_btVector3	x=Lerp(m_nodes[node0].m_x,m_nodes[node1].m_x,position);
	const D_btVector3	v=Lerp(m_nodes[node0].m_v,m_nodes[node1].m_v,position);
	const D_btScalar	m=1;
	appendNode(x,m);
	appendNode(x,m);
	D_Node*			pa=&m_nodes[node0];
	D_Node*			pb=&m_nodes[node1];
	D_Node*			pn[2]={	&m_nodes[m_nodes.size()-2],
		&m_nodes[m_nodes.size()-1]};
	pn[0]->m_v=v;
	pn[1]->m_v=v;
	for(i=0,ni=m_links.size();i<ni;++i)
	{
		const int mtch=MatchEdge(m_links[i].m_n[0],m_links[i].m_n[1],pa,pb);
		if(mtch!=-1)
		{
			appendLink(i);
			D_Link*	pft[]={&m_links[i],&m_links[m_links.size()-1]};
			pft[0]->m_n[1]=pn[mtch];
			pft[1]->m_n[0]=pn[1-mtch];
			done=true;
		}
	}
	for(i=0,ni=m_faces.size();i<ni;++i)
	{
		for(int k=2,l=0;l<3;k=l++)
		{
			const int mtch=MatchEdge(m_faces[i].m_n[k],m_faces[i].m_n[l],pa,pb);
			if(mtch!=-1)
			{
				appendFace(i);
				D_Face*	pft[]={&m_faces[i],&m_faces[m_faces.size()-1]};
				pft[0]->m_n[l]=pn[mtch];
				pft[1]->m_n[k]=pn[1-mtch];
				appendLink(pn[0],pft[0]->m_n[(l+1)%3],pft[0]->m_material,true);
				appendLink(pn[1],pft[0]->m_n[(l+1)%3],pft[0]->m_material,true);
			}
		}
	}
	if(!done)
	{
		m_ndbvt.remove(pn[0]->m_leaf);
		m_ndbvt.remove(pn[1]->m_leaf);
		m_nodes.pop_back();
		m_nodes.pop_back();
	}
	return(done);
}

//
bool			D_btSoftBody::rayTest(const D_btVector3& rayFrom,
									const D_btVector3& rayTo,
									D_sRayCast& results)
{
	if(m_faces.size()&&m_fdbvt.empty()) 
		initializeFaceTree();

	results.body	=	this;
	results.fraction = 1.f;
	results.feature	=	eFeature::D_None;
	results.index	=	-1;

	return(rayTest(rayFrom,rayTo,results.fraction,results.feature,results.index,false)!=0);
}

//
void			D_btSoftBody::setSolver(eSolverPresets::D__ preset)
{
	m_cfg.m_vsequence.clear();
	m_cfg.m_psequence.clear();
	m_cfg.m_dsequence.clear();
	switch(preset)
	{
	case	eSolverPresets::D_Positions:
		m_cfg.m_psequence.push_back(ePSolver::D_Anchors);
		m_cfg.m_psequence.push_back(ePSolver::D_RContacts);
		m_cfg.m_psequence.push_back(ePSolver::D_SContacts);
		m_cfg.m_psequence.push_back(ePSolver::D_Linear);	
		break;	
	case	eSolverPresets::D_Velocities:
		m_cfg.m_vsequence.push_back(eVSolver::D_Linear);

		m_cfg.m_psequence.push_back(ePSolver::D_Anchors);
		m_cfg.m_psequence.push_back(ePSolver::D_RContacts);
		m_cfg.m_psequence.push_back(ePSolver::D_SContacts);

		m_cfg.m_dsequence.push_back(ePSolver::D_Linear);
		break;
	}
}

//
void			D_btSoftBody::predictMotion(D_btScalar dt)
{
	int i,ni;

	/* Update				*/ 
	if(m_bUpdateRtCst)
	{
		m_bUpdateRtCst=false;
		updateConstants();
		m_fdbvt.clear();
		if(m_cfg.collisions&D_fCollision::D_VF_SS)
		{
			initializeFaceTree();			
		}
	}

	/* Prepare				*/ 
	m_sst.sdt		=	dt*m_cfg.timescale;
	m_sst.isdt		=	1/m_sst.sdt;
	m_sst.velmrg	=	m_sst.sdt*3;
	m_sst.radmrg	=	getCollisionShape()->getMargin();
	m_sst.updmrg	=	m_sst.radmrg*(D_btScalar)0.25;
	/* Forces				*/ 
	addVelocity(m_worldInfo->m_gravity*m_sst.sdt);
	applyForces();
	/* Integrate			*/ 
	for(i=0,ni=m_nodes.size();i<ni;++i)
	{
		D_Node&	n=m_nodes[i];
		n.m_q	=	n.m_x;
		n.m_v	+=	n.m_f*n.m_im*m_sst.sdt;
		n.m_x	+=	n.m_v*m_sst.sdt;
		n.m_f	=	D_btVector3(0,0,0);
	}
	/* D_Clusters				*/ 
	updateClusters();
	/* Bounds				*/ 
	updateBounds();	
	/* D_Nodes				*/ 
	D_ATTRIBUTE_ALIGNED16(D_btDbvtVolume)	vol;
	for(i=0,ni=m_nodes.size();i<ni;++i)
	{
		D_Node&	n=m_nodes[i];
		vol = D_btDbvtVolume::FromCR(n.m_x,m_sst.radmrg);
		m_ndbvt.update(	n.m_leaf,
			vol,
			n.m_v*m_sst.velmrg,
			m_sst.updmrg);
	}
	/* D_Faces				*/ 
	if(!m_fdbvt.empty())
	{
		for(int i=0;i<m_faces.size();++i)
		{
			D_Face&			f=m_faces[i];
			const D_btVector3	v=(	f.m_n[0]->m_v+
				f.m_n[1]->m_v+
				f.m_n[2]->m_v)/3;
			vol = VolumeOf(f,m_sst.radmrg);
			m_fdbvt.update(	f.m_leaf,
				vol,
				v*m_sst.velmrg,
				m_sst.updmrg);
		}
	}
	/* Pose					*/ 
	updatePose();
	/* Match				*/ 
	if(m_pose.m_bframe&&(m_cfg.kMT>0))
	{
		const D_btMatrix3x3	posetrs=m_pose.m_rot;
		for(int i=0,ni=m_nodes.size();i<ni;++i)
		{
			D_Node&	n=m_nodes[i];
			if(n.m_im>0)
			{
				const D_btVector3	x=posetrs*m_pose.m_pos[i]+m_pose.m_com;
				n.m_x=Lerp(n.m_x,x,m_cfg.kMT);
			}
		}
	}
	/* Clear contacts		*/ 
	m_rcontacts.resize(0);
	m_scontacts.resize(0);
	/* Optimize dbvt's		*/ 
	m_ndbvt.optimizeIncremental(1);
	m_fdbvt.optimizeIncremental(1);
	m_cdbvt.optimizeIncremental(1);
}

//
void			D_btSoftBody::solveConstraints()
{
	/* Apply clusters		*/ 
	applyClusters(false);
	/* Prepare links		*/ 

	int i,ni;

	for(i=0,ni=m_links.size();i<ni;++i)
	{
		D_Link&	l=m_links[i];
		l.m_c3		=	l.m_n[1]->m_q-l.m_n[0]->m_q;
		l.m_c2		=	1/(l.m_c3.length2()*l.m_c0);
	}
	/* Prepare anchors		*/ 
	for(i=0,ni=m_anchors.size();i<ni;++i)
	{
		Anchor&			a=m_anchors[i];
		const D_btVector3	ra=a.m_body->getWorldTransform().getBasis()*a.m_local;
		a.m_c0	=	ImpulseMatrix(	m_sst.sdt,
			a.m_node->m_im,
			a.m_body->getInvMass(),
			a.m_body->getInvInertiaTensorWorld(),
			ra);
		a.m_c1	=	ra;
		a.m_c2	=	m_sst.sdt*a.m_node->m_im;
		a.m_body->activate();
	}
	/* Solve velocities		*/ 
	if(m_cfg.viterations>0)
	{
		/* Solve			*/ 
		for(int isolve=0;isolve<m_cfg.viterations;++isolve)
		{
			for(int iseq=0;iseq<m_cfg.m_vsequence.size();++iseq)
			{
				getSolver(m_cfg.m_vsequence[iseq])(this,1);
			}
		}
		/* Update			*/ 
		for(i=0,ni=m_nodes.size();i<ni;++i)
		{
			D_Node&	n=m_nodes[i];
			n.m_x	=	n.m_q+n.m_v*m_sst.sdt;
		}
	}
	/* Solve positions		*/ 
	if(m_cfg.piterations>0)
	{
		for(int isolve=0;isolve<m_cfg.piterations;++isolve)
		{
			const D_btScalar ti=isolve/(D_btScalar)m_cfg.piterations;
			for(int iseq=0;iseq<m_cfg.m_psequence.size();++iseq)
			{
				getSolver(m_cfg.m_psequence[iseq])(this,1,ti);
			}
		}
		const D_btScalar	vc=m_sst.isdt*(1-m_cfg.kDP);
		for(i=0,ni=m_nodes.size();i<ni;++i)
		{
			D_Node&	n=m_nodes[i];
			n.m_v	=	(n.m_x-n.m_q)*vc;
			n.m_f	=	D_btVector3(0,0,0);		
		}
	}
	/* Solve drift			*/ 
	if(m_cfg.diterations>0)
	{
		const D_btScalar	vcf=m_cfg.kVCF*m_sst.isdt;
		for(i=0,ni=m_nodes.size();i<ni;++i)
		{
			D_Node&	n=m_nodes[i];
			n.m_q	=	n.m_x;
		}
		for(int idrift=0;idrift<m_cfg.diterations;++idrift)
		{
			for(int iseq=0;iseq<m_cfg.m_dsequence.size();++iseq)
			{
				getSolver(m_cfg.m_dsequence[iseq])(this,1,0);
			}
		}
		for(int i=0,ni=m_nodes.size();i<ni;++i)
		{
			D_Node&	n=m_nodes[i];
			n.m_v	+=	(n.m_x-n.m_q)*vcf;
		}
	}
	/* Apply clusters		*/ 
	dampClusters();
	applyClusters(true);
}

//
void			D_btSoftBody::staticSolve(int iterations)
{
	for(int isolve=0;isolve<iterations;++isolve)
	{
		for(int iseq=0;iseq<m_cfg.m_psequence.size();++iseq)
		{
			getSolver(m_cfg.m_psequence[iseq])(this,1,0);
		}
	}
}

//
void			D_btSoftBody::solveCommonConstraints(D_btSoftBody** /*bodies*/,int /*count*/,int /*iterations*/)
{
	/// placeholder
}

//
void			D_btSoftBody::solveClusters(const D_btAlignedObjectArray<D_btSoftBody*>& bodies)
{
	const int	nb=bodies.size();
	int			iterations=0;
	int i;

	for(i=0;i<nb;++i)
	{
		iterations=D_btMax(iterations,bodies[i]->m_cfg.citerations);
	}
	for(i=0;i<nb;++i)
	{
		bodies[i]->prepareClusters(iterations);
	}
	for(i=0;i<iterations;++i)
	{
		const D_btScalar sor=1;
		for(int j=0;j<nb;++j)
		{
			bodies[j]->solveClusters(sor);
		}
	}
	for(i=0;i<nb;++i)
	{
		bodies[i]->cleanupClusters();
	}
}

//
void			D_btSoftBody::integrateMotion()
{
	/* Update			*/ 
	updateNormals();
}

//
D_btSoftBody::RayFromToCaster::RayFromToCaster(const D_btVector3& rayFrom,const D_btVector3& rayTo,D_btScalar mxt)
{
	m_rayFrom = rayFrom;
	m_rayNormalizedDirection = (rayTo-rayFrom);
	m_rayTo = rayTo;
	m_mint	=	mxt;
	m_face	=	0;
	m_tests	=	0;
}

//
void				D_btSoftBody::RayFromToCaster::Process(const D_btDbvtNode* leaf)
{
	D_btSoftBody::D_Face&	f=*(D_btSoftBody::D_Face*)leaf->data;
	const D_btScalar		t=rayFromToTriangle(	m_rayFrom,m_rayTo,m_rayNormalizedDirection,
		f.m_n[0]->m_x,
		f.m_n[1]->m_x,
		f.m_n[2]->m_x,
		m_mint);
	if((t>0)&&(t<m_mint)) 
	{ 
		m_mint=t;m_face=&f; 
	}
	++m_tests;
}

//
D_btScalar			D_btSoftBody::RayFromToCaster::rayFromToTriangle(	const D_btVector3& rayFrom,
																   const D_btVector3& rayTo,
																   const D_btVector3& rayNormalizedDirection,
																   const D_btVector3& a,
																   const D_btVector3& b,
																   const D_btVector3& c,
																   D_btScalar maxt)
{
	static const D_btScalar	ceps=-D_SIMD_EPSILON*10;
	static const D_btScalar	teps=D_SIMD_EPSILON*10;

	const D_btVector3			n=D_btCross(b-a,c-a);
	const D_btScalar			d=D_btDot(a,n);
	const D_btScalar			den=D_btDot(rayNormalizedDirection,n);
	if(!D_btFuzzyZero(den))
	{
		const D_btScalar		num=D_btDot(rayFrom,n)-d;
		const D_btScalar		t=-num/den;
		if((t>teps)&&(t<maxt))
		{
			const D_btVector3	hit=rayFrom+rayNormalizedDirection*t;
			if(	(D_btDot(n,D_btCross(a-hit,b-hit))>ceps)	&&			
				(D_btDot(n,D_btCross(b-hit,c-hit))>ceps)	&&
				(D_btDot(n,D_btCross(c-hit,a-hit))>ceps))
			{
				return(t);
			}
		}
	}
	return(-1);
}

//
void				D_btSoftBody::pointersToIndices()
{
#define	PTR2IDX(_p_,_b_)	reinterpret_cast<D_btSoftBody::D_Node*>((_p_)-(_b_))
	D_btSoftBody::D_Node*	base=&m_nodes[0];
	int i,ni;

	for(i=0,ni=m_nodes.size();i<ni;++i)
	{
		if(m_nodes[i].m_leaf)
		{
			m_nodes[i].m_leaf->data=*(void**)&i;
		}
	}
	for(i=0,ni=m_links.size();i<ni;++i)
	{
		m_links[i].m_n[0]=PTR2IDX(m_links[i].m_n[0],base);
		m_links[i].m_n[1]=PTR2IDX(m_links[i].m_n[1],base);
	}
	for(i=0,ni=m_faces.size();i<ni;++i)
	{
		m_faces[i].m_n[0]=PTR2IDX(m_faces[i].m_n[0],base);
		m_faces[i].m_n[1]=PTR2IDX(m_faces[i].m_n[1],base);
		m_faces[i].m_n[2]=PTR2IDX(m_faces[i].m_n[2],base);
		if(m_faces[i].m_leaf)
		{
			m_faces[i].m_leaf->data=*(void**)&i;
		}
	}
	for(i=0,ni=m_anchors.size();i<ni;++i)
	{
		m_anchors[i].m_node=PTR2IDX(m_anchors[i].m_node,base);
	}
	for(i=0,ni=m_notes.size();i<ni;++i)
	{
		for(int j=0;j<m_notes[i].m_rank;++j)
		{
			m_notes[i].m_nodes[j]=PTR2IDX(m_notes[i].m_nodes[j],base);
		}
	}
#undef	PTR2IDX
}

//
void				D_btSoftBody::indicesToPointers(const int* map)
{
#define	IDX2PTR(_p_,_b_)	map?(&(_b_)[map[(((char*)_p_)-(char*)0)]]):	\
	(&(_b_)[(((char*)_p_)-(char*)0)])
	D_btSoftBody::D_Node*	base=&m_nodes[0];
	int i,ni;

	for(i=0,ni=m_nodes.size();i<ni;++i)
	{
		if(m_nodes[i].m_leaf)
		{
			m_nodes[i].m_leaf->data=&m_nodes[i];
		}
	}
	for(i=0,ni=m_links.size();i<ni;++i)
	{
		m_links[i].m_n[0]=IDX2PTR(m_links[i].m_n[0],base);
		m_links[i].m_n[1]=IDX2PTR(m_links[i].m_n[1],base);
	}
	for(i=0,ni=m_faces.size();i<ni;++i)
	{
		m_faces[i].m_n[0]=IDX2PTR(m_faces[i].m_n[0],base);
		m_faces[i].m_n[1]=IDX2PTR(m_faces[i].m_n[1],base);
		m_faces[i].m_n[2]=IDX2PTR(m_faces[i].m_n[2],base);
		if(m_faces[i].m_leaf)
		{
			m_faces[i].m_leaf->data=&m_faces[i];
		}
	}
	for(i=0,ni=m_anchors.size();i<ni;++i)
	{
		m_anchors[i].m_node=IDX2PTR(m_anchors[i].m_node,base);
	}
	for(i=0,ni=m_notes.size();i<ni;++i)
	{
		for(int j=0;j<m_notes[i].m_rank;++j)
		{
			m_notes[i].m_nodes[j]=IDX2PTR(m_notes[i].m_nodes[j],base);
		}
	}
#undef	IDX2PTR
}

//
int					D_btSoftBody::rayTest(const D_btVector3& rayFrom,const D_btVector3& rayTo,
										D_btScalar& mint,eFeature::D__& feature,int& index,bool bcountonly) const
{
	int	cnt=0;
	if(bcountonly||m_fdbvt.empty())
	{/* Full search	*/ 
		D_btVector3 dir = rayTo-rayFrom;
		dir.normalize();

		for(int i=0,ni=m_faces.size();i<ni;++i)
		{
			const D_btSoftBody::D_Face&	f=m_faces[i];

			const D_btScalar			t=RayFromToCaster::rayFromToTriangle(	rayFrom,rayTo,dir,
				f.m_n[0]->m_x,
				f.m_n[1]->m_x,
				f.m_n[2]->m_x,
				mint);
			if(t>0)
			{
				++cnt;
				if(!bcountonly)
				{
					feature=D_btSoftBody::eFeature::D_Face;
					index=i;
					mint=t;
				}
			}
		}
	}
	else
	{/* Use dbvt	*/ 
		RayFromToCaster	collider(rayFrom,rayTo,mint);

		D_btDbvt::rayTest(m_fdbvt.m_root,rayFrom,rayTo,collider);
		if(collider.m_face)
		{
			mint=collider.m_mint;
			feature=D_btSoftBody::eFeature::D_Face;
			index=(int)(collider.m_face-&m_faces[0]);
			cnt=1;
		}
	}
	return(cnt);
}

//
void			D_btSoftBody::initializeFaceTree()
{
	m_fdbvt.clear();
	for(int i=0;i<m_faces.size();++i)
	{
		D_Face&	f=m_faces[i];
		f.m_leaf=m_fdbvt.insert(VolumeOf(f,0),&f);
	}
}

//
D_btVector3		D_btSoftBody::evaluateCom() const
{
	D_btVector3	com(0,0,0);
	if(m_pose.m_bframe)
	{
		for(int i=0,ni=m_nodes.size();i<ni;++i)
		{
			com+=m_nodes[i].m_x*m_pose.m_wgh[i];
		}
	}
	return(com);
}

//
bool				D_btSoftBody::checkContact(	D_btCollisionObject* colObj,
											 const D_btVector3& x,
											 D_btScalar margin,
											 D_btSoftBody::D_sCti& cti) const
{
	D_btVector3			nrm;
	D_btCollisionShape*	shp=colObj->getCollisionShape();
	D_btRigidBody* tmpRigid = D_btRigidBody::upcast(colObj);
	const D_btTransform&	wtr=tmpRigid? tmpRigid->getInterpolationWorldTransform() : colObj->getWorldTransform();
	D_btScalar			dst=m_worldInfo->m_sparsesdf.Evaluate(	wtr.invXform(x),
		shp,
		nrm,
		margin);
	if(dst<0)
	{
		cti.m_colObj		=	colObj;
		cti.m_normal	=	wtr.getBasis()*nrm;
		cti.m_offset	=	-D_btDot(	cti.m_normal,
			x-cti.m_normal*dst);
		return(true);
	}
	return(false);
}

//
void					D_btSoftBody::updateNormals()
{
	const D_btVector3	zv(0,0,0);
	int i,ni;

	for(i=0,ni=m_nodes.size();i<ni;++i)
	{
		m_nodes[i].m_n=zv;
	}
	for(i=0,ni=m_faces.size();i<ni;++i)
	{
		D_btSoftBody::D_Face&	f=m_faces[i];
		const D_btVector3		n=D_btCross(f.m_n[1]->m_x-f.m_n[0]->m_x,
			f.m_n[2]->m_x-f.m_n[0]->m_x);
		f.m_normal=n.normalized();
		f.m_n[0]->m_n+=n;
		f.m_n[1]->m_n+=n;
		f.m_n[2]->m_n+=n;
	}
	for(i=0,ni=m_nodes.size();i<ni;++i)
	{
		D_btScalar len = m_nodes[i].m_n.length();
		if (len>D_SIMD_EPSILON)
			m_nodes[i].m_n /= len;
	}
}

//
void					D_btSoftBody::updateBounds()
{
	if(m_ndbvt.m_root)
	{
		const D_btVector3&	mins=m_ndbvt.m_root->volume.Mins();
		const D_btVector3&	maxs=m_ndbvt.m_root->volume.Maxs();
		const D_btScalar		csm=getCollisionShape()->getMargin();
		const D_btVector3		mrg=D_btVector3(	csm,
			csm,
			csm)*1; // ??? D_to investigate...
		m_bounds[0]=mins-mrg;
		m_bounds[1]=maxs+mrg;
		if(0!=getBroadphaseHandle())
		{					
			m_worldInfo->m_broadphase->setAabb(	getBroadphaseHandle(),
				m_bounds[0],
				m_bounds[1],
				m_worldInfo->m_dispatcher);
		}
	}
	else
	{
		m_bounds[0]=
			m_bounds[1]=D_btVector3(0,0,0);
	}		
}


//
void					D_btSoftBody::updatePose()
{
	if(m_pose.m_bframe)
	{
		D_btSoftBody::Pose&	pose=m_pose;
		const D_btVector3		com=evaluateCom();
		/* Com			*/ 
		pose.m_com	=	com;
		/* Rotation		*/ 
		D_btMatrix3x3		Apq;
		const D_btScalar	eps=D_SIMD_EPSILON;
		Apq[0]=Apq[1]=Apq[2]=D_btVector3(0,0,0);
		Apq[0].setX(eps);Apq[1].setY(eps*2);Apq[2].setZ(eps*3);
		for(int i=0,ni=m_nodes.size();i<ni;++i)
		{
			const D_btVector3		a=pose.m_wgh[i]*(m_nodes[i].m_x-com);
			const D_btVector3&	b=pose.m_pos[i];
			Apq[0]+=a.x()*b;
			Apq[1]+=a.y()*b;
			Apq[2]+=a.z()*b;
		}
		D_btMatrix3x3		r,s;
		PolarDecompose(Apq,r,s);
		pose.m_rot=r;
		pose.m_scl=pose.m_aqq*r.transpose()*Apq;
		if(m_cfg.maxvolume>1)
		{
			const D_btScalar	idet=Clamp<D_btScalar>(	1/pose.m_scl.determinant(),
				1,m_cfg.maxvolume);
			pose.m_scl=Mul(pose.m_scl,idet);
		}

	}
}

//
void				D_btSoftBody::updateConstants()
{
	int i,ni;

	/* D_Links		*/ 
	for(i=0,ni=m_links.size();i<ni;++i)
	{
		D_Link&		l=m_links[i];
		Material&	m=*l.m_material;
		l.m_rl	=	(l.m_n[0]->m_x-l.m_n[1]->m_x).length();
		l.m_c0	=	(l.m_n[0]->m_im+l.m_n[1]->m_im)/m.m_kLST;
		l.m_c1	=	l.m_rl*l.m_rl;
	}
	/* D_Faces		*/ 
	for(i=0,ni=m_faces.size();i<ni;++i)
	{
		D_Face&		f=m_faces[i];
		f.m_ra	=	AreaOf(f.m_n[0]->m_x,f.m_n[1]->m_x,f.m_n[2]->m_x);
	}
	/* Area's		*/ 
	D_btAlignedObjectArray<int>	counts;
	counts.resize(m_nodes.size(),0);
	for(i=0,ni=m_nodes.size();i<ni;++i)
	{
		m_nodes[i].m_area	=	0;
	}
	for(i=0,ni=m_faces.size();i<ni;++i)
	{
		D_btSoftBody::D_Face&	f=m_faces[i];
		for(int j=0;j<3;++j)
		{
			const int index=(int)(f.m_n[j]-&m_nodes[0]);
			counts[index]++;
			f.m_n[j]->m_area+=D_btFabs(f.m_ra);
		}
	}
	for(i=0,ni=m_nodes.size();i<ni;++i)
	{
		if(counts[i]>0)
			m_nodes[i].m_area/=(D_btScalar)counts[i];
		else
			m_nodes[i].m_area=0;
	}
}

//
void					D_btSoftBody::initializeClusters()
{
	int i;

	for( i=0;i<m_clusters.size();++i)
	{
		Cluster&	c=*m_clusters[i];
		c.m_imass=0;
		c.m_masses.resize(c.m_nodes.size());
		for(int j=0;j<c.m_nodes.size();++j)
		{
			if (c.m_nodes[j]->m_im==0)
			{
				c.m_containsAnchor = true;
				c.m_masses[j]	=	D_BT_LARGE_FLOAT;
			} else
			{
				c.m_masses[j]	=	D_btScalar(1.)/c.m_nodes[j]->m_im;
			}
			c.m_imass		+=	c.m_masses[j];
		}
		c.m_imass		=	D_btScalar(1.)/c.m_imass;
		c.m_com			=	D_btSoftBody::clusterCom(&c);
		c.m_lv			=	D_btVector3(0,0,0);
		c.m_av			=	D_btVector3(0,0,0);
		c.m_leaf		=	0;
		/* Inertia	*/ 
		D_btMatrix3x3&	ii=c.m_locii;
		ii[0]=ii[1]=ii[2]=D_btVector3(0,0,0);
		{
			int i,ni;

			for(i=0,ni=c.m_nodes.size();i<ni;++i)
			{
				const D_btVector3	k=c.m_nodes[i]->m_x-c.m_com;
				const D_btVector3	q=k*k;
				const D_btScalar	m=c.m_masses[i];
				ii[0][0]	+=	m*(q[1]+q[2]);
				ii[1][1]	+=	m*(q[0]+q[2]);
				ii[2][2]	+=	m*(q[0]+q[1]);
				ii[0][1]	-=	m*k[0]*k[1];
				ii[0][2]	-=	m*k[0]*k[2];
				ii[1][2]	-=	m*k[1]*k[2];
			}
		}
		ii[1][0]=ii[0][1];
		ii[2][0]=ii[0][2];
		ii[2][1]=ii[1][2];
		
		ii = ii.inverse();

		/* Frame	*/ 
		c.m_framexform.setIdentity();
		c.m_framexform.setOrigin(c.m_com);
		c.m_framerefs.resize(c.m_nodes.size());
		{
			int i;
			for(i=0;i<c.m_framerefs.size();++i)
			{
				c.m_framerefs[i]=c.m_nodes[i]->m_x-c.m_com;
			}
		}
	}
}

//
void					D_btSoftBody::updateClusters()
{
	D_BT_PROFILE("UpdateClusters");
	int i;

	for(i=0;i<m_clusters.size();++i)
	{
		D_btSoftBody::Cluster&	c=*m_clusters[i];
		const int				n=c.m_nodes.size();
		//const D_btScalar			invn=1/(D_btScalar)n;
		if(n)
		{
			/* Frame				*/ 
			const D_btScalar	eps=D_btScalar(0.0001);
			D_btMatrix3x3		m,r,s;
			m[0]=m[1]=m[2]=D_btVector3(0,0,0);
			m[0][0]=eps*1;
			m[1][1]=eps*2;
			m[2][2]=eps*3;
			c.m_com=clusterCom(&c);
			for(int i=0;i<c.m_nodes.size();++i)
			{
				const D_btVector3		a=c.m_nodes[i]->m_x-c.m_com;
				const D_btVector3&	b=c.m_framerefs[i];
				m[0]+=a[0]*b;m[1]+=a[1]*b;m[2]+=a[2]*b;
			}
			PolarDecompose(m,r,s);
			c.m_framexform.setOrigin(c.m_com);
			c.m_framexform.setBasis(r);		
			/* Inertia			*/ 
#if 1/* Constant	*/ 
			c.m_invwi=c.m_framexform.getBasis()*c.m_locii*c.m_framexform.getBasis().transpose();
#else
#if 0/* Sphere	*/ 
			const D_btScalar	rk=(2*c.m_extents.length2())/(5*c.m_imass);
			const D_btVector3	inertia(rk,rk,rk);
			const D_btVector3	iin(D_btFabs(inertia[0])>D_SIMD_EPSILON?1/inertia[0]:0,
				D_btFabs(inertia[1])>D_SIMD_EPSILON?1/inertia[1]:0,
				D_btFabs(inertia[2])>D_SIMD_EPSILON?1/inertia[2]:0);

			c.m_invwi=c.m_xform.getBasis().scaled(iin)*c.m_xform.getBasis().transpose();
#else/* Actual	*/ 		
			c.m_invwi[0]=c.m_invwi[1]=c.m_invwi[2]=D_btVector3(0,0,0);
			for(int i=0;i<n;++i)
			{
				const D_btVector3	k=c.m_nodes[i]->m_x-c.m_com;
				const D_btVector3		q=k*k;
				const D_btScalar		m=1/c.m_nodes[i]->m_im;
				c.m_invwi[0][0]	+=	m*(q[1]+q[2]);
				c.m_invwi[1][1]	+=	m*(q[0]+q[2]);
				c.m_invwi[2][2]	+=	m*(q[0]+q[1]);
				c.m_invwi[0][1]	-=	m*k[0]*k[1];
				c.m_invwi[0][2]	-=	m*k[0]*k[2];
				c.m_invwi[1][2]	-=	m*k[1]*k[2];
			}
			c.m_invwi[1][0]=c.m_invwi[0][1];
			c.m_invwi[2][0]=c.m_invwi[0][2];
			c.m_invwi[2][1]=c.m_invwi[1][2];
			c.m_invwi=c.m_invwi.inverse();
#endif
#endif
			/* D_Velocities			*/ 
			c.m_lv=D_btVector3(0,0,0);
			c.m_av=D_btVector3(0,0,0);
			{
				int i;

				for(i=0;i<n;++i)
				{
					const D_btVector3	v=c.m_nodes[i]->m_v*c.m_masses[i];
					c.m_lv	+=	v;
					c.m_av	+=	D_btCross(c.m_nodes[i]->m_x-c.m_com,v);
				}
			}
			c.m_lv=c.m_imass*c.m_lv*(1-c.m_ldamping);
			c.m_av=c.m_invwi*c.m_av*(1-c.m_adamping);
			c.m_vimpulses[0]	=
				c.m_vimpulses[1]	= D_btVector3(0,0,0);
			c.m_dimpulses[0]	=
				c.m_dimpulses[1]	= D_btVector3(0,0,0);
			c.m_nvimpulses		= 0;
			c.m_ndimpulses		= 0;
			/* Matching				*/ 
			if(c.m_matching>0)
			{
				for(int j=0;j<c.m_nodes.size();++j)
				{
					D_Node&			n=*c.m_nodes[j];
					const D_btVector3	x=c.m_framexform*c.m_framerefs[j];
					n.m_x=Lerp(n.m_x,x,c.m_matching);
				}
			}			
			/* Dbvt					*/ 
			if(c.m_collide)
			{
				D_btVector3	mi=c.m_nodes[0]->m_x;
				D_btVector3	mx=mi;
				for(int j=1;j<n;++j)
				{
					mi.setMin(c.m_nodes[j]->m_x);
					mx.setMax(c.m_nodes[j]->m_x);
				}			
				D_ATTRIBUTE_ALIGNED16(D_btDbvtVolume)	bounds=D_btDbvtVolume::FromMM(mi,mx);
				if(c.m_leaf)
					m_cdbvt.update(c.m_leaf,bounds,c.m_lv*m_sst.sdt*3,m_sst.radmrg);
				else
					c.m_leaf=m_cdbvt.insert(bounds,&c);
			}
		}
	}


}




//
void					D_btSoftBody::cleanupClusters()
{
	for(int i=0;i<m_joints.size();++i)
	{
		m_joints[i]->Terminate(m_sst.sdt);
		if(m_joints[i]->m_delete)
		{
			D_btAlignedFree(m_joints[i]);
			m_joints.remove(m_joints[i--]);
		}	
	}
}

//
void					D_btSoftBody::prepareClusters(int iterations)
{
	for(int i=0;i<m_joints.size();++i)
	{
		m_joints[i]->Prepare(m_sst.sdt,iterations);
	}
}


//
void					D_btSoftBody::solveClusters(D_btScalar sor)
{
	for(int i=0,ni=m_joints.size();i<ni;++i)
	{
		m_joints[i]->Solve(m_sst.sdt,sor);
	}
}

//
void					D_btSoftBody::applyClusters(bool drift)
{
	D_BT_PROFILE("ApplyClusters");
	const D_btScalar					f0=m_sst.sdt;
	//const D_btScalar					f1=f0/2;
	D_btAlignedObjectArray<D_btVector3> deltas;
	D_btAlignedObjectArray<D_btScalar> weights;
	deltas.resize(m_nodes.size(),D_btVector3(0,0,0));
	weights.resize(m_nodes.size(),0);
	int i;

	if(drift)
	{
		for(i=0;i<m_clusters.size();++i)
		{
			Cluster&	c=*m_clusters[i];
			if(c.m_ndimpulses)
			{
				c.m_dimpulses[0]/=(D_btScalar)c.m_ndimpulses;
				c.m_dimpulses[1]/=(D_btScalar)c.m_ndimpulses;
			}
		}
	}
	
	for(i=0;i<m_clusters.size();++i)
	{
		Cluster&	c=*m_clusters[i];	
		if(0<(drift?c.m_ndimpulses:c.m_nvimpulses))
		{
			const D_btVector3		v=(drift?c.m_dimpulses[0]:c.m_vimpulses[0])*m_sst.sdt;
			const D_btVector3		w=(drift?c.m_dimpulses[1]:c.m_vimpulses[1])*m_sst.sdt;
			for(int j=0;j<c.m_nodes.size();++j)
			{
				const int			idx=int(c.m_nodes[j]-&m_nodes[0]);
				const D_btVector3&	x=c.m_nodes[j]->m_x;
				const D_btScalar		q=c.m_masses[j];
				deltas[idx]		+=	(v+D_btCross(w,x-c.m_com))*q;
				weights[idx]	+=	q;
			}
		}
	}
	for(i=0;i<deltas.size();++i)
	{
		if(weights[i]>0) m_nodes[i].m_x+=deltas[i]/weights[i];
	}
}

//
void					D_btSoftBody::dampClusters()
{
	int i;

	for(i=0;i<m_clusters.size();++i)
	{
		Cluster&	c=*m_clusters[i];	
		if(c.m_ndamping>0)
		{
			for(int j=0;j<c.m_nodes.size();++j)
			{
				D_Node&			n=*c.m_nodes[j];
				if(n.m_im>0)
				{
					const D_btVector3	vx=c.m_lv+D_btCross(c.m_av,c.m_nodes[j]->m_q-c.m_com);
					if(vx.length2()<=n.m_v.length2())
						{
						n.m_v	+=	c.m_ndamping*(vx-n.m_v);
						}
				}
			}
		}
	}
}

//
void				D_btSoftBody::Joint::Prepare(D_btScalar dt,int)
{
	m_bodies[0].activate();
	m_bodies[1].activate();
}

//
void				D_btSoftBody::LJoint::Prepare(D_btScalar dt,int iterations)
{
	static const D_btScalar	maxdrift=4;
	Joint::Prepare(dt,iterations);
	m_rpos[0]		=	m_bodies[0].xform()*m_refs[0];
	m_rpos[1]		=	m_bodies[1].xform()*m_refs[1];
	m_drift			=	Clamp(m_rpos[0]-m_rpos[1],maxdrift)*m_erp/dt;
	m_rpos[0]		-=	m_bodies[0].xform().getOrigin();
	m_rpos[1]		-=	m_bodies[1].xform().getOrigin();
	m_massmatrix	=	ImpulseMatrix(	m_bodies[0].invMass(),m_bodies[0].invWorldInertia(),m_rpos[0],
		m_bodies[1].invMass(),m_bodies[1].invWorldInertia(),m_rpos[1]);
	if(m_split>0)
	{
		m_sdrift	=	m_massmatrix*(m_drift*m_split);
		m_drift		*=	1-m_split;
	}
	m_drift	/=(D_btScalar)iterations;
}

//
void				D_btSoftBody::LJoint::Solve(D_btScalar dt,D_btScalar sor)
{
	const D_btVector3		va=m_bodies[0].velocity(m_rpos[0]);
	const D_btVector3		vb=m_bodies[1].velocity(m_rpos[1]);
	const D_btVector3		vr=va-vb;
	D_btSoftBody::Impulse	impulse;
	impulse.m_asVelocity	=	1;
	impulse.m_velocity		=	m_massmatrix*(m_drift+vr*m_cfm)*sor;
	m_bodies[0].applyImpulse(-impulse,m_rpos[0]);
	m_bodies[1].applyImpulse( impulse,m_rpos[1]);
}

//
void				D_btSoftBody::LJoint::Terminate(D_btScalar dt)
{
	if(m_split>0)
	{
		m_bodies[0].applyDImpulse(-m_sdrift,m_rpos[0]);
		m_bodies[1].applyDImpulse( m_sdrift,m_rpos[1]);
	}
}

//
void				D_btSoftBody::AJoint::Prepare(D_btScalar dt,int iterations)
{
	static const D_btScalar	maxdrift=D_SIMD_PI/16;
	m_icontrol->Prepare(this);
	Joint::Prepare(dt,iterations);
	m_axis[0]	=	m_bodies[0].xform().getBasis()*m_refs[0];
	m_axis[1]	=	m_bodies[1].xform().getBasis()*m_refs[1];
	m_drift		=	NormalizeAny(D_btCross(m_axis[1],m_axis[0]));
	m_drift		*=	D_btMin(maxdrift,D_btAcos(Clamp<D_btScalar>(D_btDot(m_axis[0],m_axis[1]),-1,+1)));
	m_drift		*=	m_erp/dt;
	m_massmatrix=	AngularImpulseMatrix(m_bodies[0].invWorldInertia(),m_bodies[1].invWorldInertia());
	if(m_split>0)
	{
		m_sdrift	=	m_massmatrix*(m_drift*m_split);
		m_drift		*=	1-m_split;
	}
	m_drift	/=(D_btScalar)iterations;
}

//
void				D_btSoftBody::AJoint::Solve(D_btScalar dt,D_btScalar sor)
{
	const D_btVector3		va=m_bodies[0].angularVelocity();
	const D_btVector3		vb=m_bodies[1].angularVelocity();
	const D_btVector3		vr=va-vb;
	const D_btScalar		sp=D_btDot(vr,m_axis[0]);
	const D_btVector3		vc=vr-m_axis[0]*m_icontrol->Speed(this,sp);
	D_btSoftBody::Impulse	impulse;
	impulse.m_asVelocity	=	1;
	impulse.m_velocity		=	m_massmatrix*(m_drift+vc*m_cfm)*sor;
	m_bodies[0].applyAImpulse(-impulse);
	m_bodies[1].applyAImpulse( impulse);
}

//
void				D_btSoftBody::AJoint::Terminate(D_btScalar dt)
{
	if(m_split>0)
	{
		m_bodies[0].applyDAImpulse(-m_sdrift);
		m_bodies[1].applyDAImpulse( m_sdrift);
	}
}

//
void				D_btSoftBody::CJoint::Prepare(D_btScalar dt,int iterations)
{
	Joint::Prepare(dt,iterations);
	const bool	dodrift=(m_life==0);
	m_delete=(++m_life)>m_maxlife;
	if(dodrift)
	{
		m_drift=m_drift*m_erp/dt;
		if(m_split>0)
		{
			m_sdrift	=	m_massmatrix*(m_drift*m_split);
			m_drift		*=	1-m_split;
		}
		m_drift/=(D_btScalar)iterations;
	}
	else
	{
		m_drift=m_sdrift=D_btVector3(0,0,0);
	}
}

//
void				D_btSoftBody::CJoint::Solve(D_btScalar dt,D_btScalar sor)
{
	const D_btVector3		va=m_bodies[0].velocity(m_rpos[0]);
	const D_btVector3		vb=m_bodies[1].velocity(m_rpos[1]);
	const D_btVector3		vrel=va-vb;
	const D_btScalar		rvac=D_btDot(vrel,m_normal);
	D_btSoftBody::Impulse	impulse;
	impulse.m_asVelocity	=	1;
	impulse.m_velocity		=	m_drift;
	if(rvac<0)
	{
		const D_btVector3	iv=m_normal*rvac;
		const D_btVector3	fv=vrel-iv;
		impulse.m_velocity	+=	iv+fv*m_friction;
	}
	impulse.m_velocity=m_massmatrix*impulse.m_velocity*sor;
	
	if (m_bodies[0].m_soft==m_bodies[1].m_soft)
	{
		if ((impulse.m_velocity.getX() ==impulse.m_velocity.getX())&&(impulse.m_velocity.getY() ==impulse.m_velocity.getY())&&
			(impulse.m_velocity.getZ() ==impulse.m_velocity.getZ()))
		{
			if (impulse.m_asVelocity)
			{
				if (impulse.m_velocity.length() <m_bodies[0].m_soft->m_maxSelfCollisionImpulse)
				{
					
				} else
				{
					m_bodies[0].applyImpulse(-impulse*m_bodies[0].m_soft->m_selfCollisionImpulseFactor,m_rpos[0]);
					m_bodies[1].applyImpulse( impulse*m_bodies[0].m_soft->m_selfCollisionImpulseFactor,m_rpos[1]);
				}
			}
		}
	} else
	{
		m_bodies[0].applyImpulse(-impulse,m_rpos[0]);
		m_bodies[1].applyImpulse( impulse,m_rpos[1]);
	}
}

//
void				D_btSoftBody::CJoint::Terminate(D_btScalar dt)
{
	if(m_split>0)
	{
		m_bodies[0].applyDImpulse(-m_sdrift,m_rpos[0]);
		m_bodies[1].applyDImpulse( m_sdrift,m_rpos[1]);
	}
}

//
void				D_btSoftBody::applyForces()
{

	D_BT_PROFILE("SoftBody applyForces");
	const D_btScalar					dt=m_sst.sdt;
	const D_btScalar					kLF=m_cfg.kLF;
	const D_btScalar					kDG=m_cfg.kDG;
	const D_btScalar					kPR=m_cfg.kPR;
	const D_btScalar					kVC=m_cfg.kVC;
	const bool						as_lift=kLF>0;
	const bool						as_drag=kDG>0;
	const bool						as_pressure=kPR!=0;
	const bool						as_volume=kVC>0;
	const bool						as_aero=	as_lift		||
		as_drag		;
	const bool						as_vaero=	as_aero		&&
		(m_cfg.aeromodel<D_btSoftBody::D_eAeroModel::D_F_TwoSided);
	const bool						as_faero=	as_aero		&&
		(m_cfg.aeromodel>=D_btSoftBody::D_eAeroModel::D_F_TwoSided);
	const bool						use_medium=	as_aero;
	const bool						use_volume=	as_pressure	||
		as_volume	;
	D_btScalar						volume=0;
	D_btScalar						ivolumetp=0;
	D_btScalar						dvolumetv=0;
	D_btSoftBody::D_sMedium	medium;
	if(use_volume)
	{
		volume		=	getVolume();
		ivolumetp	=	1/D_btFabs(volume)*kPR;
		dvolumetv	=	(m_pose.m_volume-volume)*kVC;
	}
	/* Per vertex forces			*/ 
	int i,ni;

	for(i=0,ni=m_nodes.size();i<ni;++i)
	{
		D_btSoftBody::D_Node&	n=m_nodes[i];
		if(n.m_im>0)
		{
			if(use_medium)
			{
				EvaluateMedium(m_worldInfo,n.m_x,medium);
				/* Aerodynamics			*/ 
				if(as_vaero)
				{				
					const D_btVector3	rel_v=n.m_v-medium.m_velocity;
					const D_btScalar	rel_v2=rel_v.length2();
					if(rel_v2>D_SIMD_EPSILON)
					{
						D_btVector3	nrm=n.m_n;
						/* Setup normal		*/ 
						switch(m_cfg.aeromodel)
						{
						case	D_btSoftBody::D_eAeroModel::D_V_Point:
							nrm=NormalizeAny(rel_v);break;
						case	D_btSoftBody::D_eAeroModel::D_V_TwoSided:
							nrm*=(D_btScalar)(D_btDot(nrm,rel_v)<0?-1:+1);break;
						}
						const D_btScalar	dvn=D_btDot(rel_v,nrm);
						/* Compute forces	*/ 
						if(dvn>0)
						{
							D_btVector3		force(0,0,0);
							const D_btScalar	c0	=	n.m_area*dvn*rel_v2/2;
							const D_btScalar	c1	=	c0*medium.m_density;
							force	+=	nrm*(-c1*kLF);
							force	+=	rel_v.normalized()*(-c1*kDG);
							ApplyClampedForce(n,force,dt);
						}
					}
				}
			}
			/* Pressure				*/ 
			if(as_pressure)
			{
				n.m_f	+=	n.m_n*(n.m_area*ivolumetp);
			}
			/* Volume				*/ 
			if(as_volume)
			{
				n.m_f	+=	n.m_n*(n.m_area*dvolumetv);
			}
		}
	}
	/* Per face forces				*/ 
	for(i=0,ni=m_faces.size();i<ni;++i)
	{
		D_btSoftBody::D_Face&	f=m_faces[i];
		if(as_faero)
		{
			const D_btVector3	v=(f.m_n[0]->m_v+f.m_n[1]->m_v+f.m_n[2]->m_v)/3;
			const D_btVector3	x=(f.m_n[0]->m_x+f.m_n[1]->m_x+f.m_n[2]->m_x)/3;
			EvaluateMedium(m_worldInfo,x,medium);
			const D_btVector3	rel_v=v-medium.m_velocity;
			const D_btScalar	rel_v2=rel_v.length2();
			if(rel_v2>D_SIMD_EPSILON)
			{
				D_btVector3	nrm=f.m_normal;
				/* Setup normal		*/ 
				switch(m_cfg.aeromodel)
				{
				case	D_btSoftBody::D_eAeroModel::D_F_TwoSided:
					nrm*=(D_btScalar)(D_btDot(nrm,rel_v)<0?-1:+1);break;
				}
				const D_btScalar	dvn=D_btDot(rel_v,nrm);
				/* Compute forces	*/ 
				if(dvn>0)
				{
					D_btVector3		force(0,0,0);
					const D_btScalar	c0	=	f.m_ra*dvn*rel_v2;
					const D_btScalar	c1	=	c0*medium.m_density;
					force	+=	nrm*(-c1*kLF);
					force	+=	rel_v.normalized()*(-c1*kDG);
					force	/=	3;
					for(int j=0;j<3;++j) ApplyClampedForce(*f.m_n[j],force,dt);
				}
			}
		}
	}
}

//
void				D_btSoftBody::PSolve_Anchors(D_btSoftBody* psb,D_btScalar kst,D_btScalar ti)
{
	const D_btScalar	kAHR=psb->m_cfg.kAHR*kst;
	const D_btScalar	dt=psb->m_sst.sdt;
	for(int i=0,ni=psb->m_anchors.size();i<ni;++i)
	{
		const Anchor&		a=psb->m_anchors[i];
		const D_btTransform&	t=a.m_body->getInterpolationWorldTransform();
		D_Node&				n=*a.m_node;
		const D_btVector3		wa=t*a.m_local;
		const D_btVector3		va=a.m_body->getVelocityInLocalPoint(a.m_c1)*dt;
		const D_btVector3		vb=n.m_x-n.m_q;
		const D_btVector3		vr=(va-vb)+(wa-n.m_x)*kAHR;
		const D_btVector3		impulse=a.m_c0*vr;
		n.m_x+=impulse*a.m_c2;
		a.m_body->applyImpulse(-impulse,a.m_c1);
	}
}

//
void				D_btSoftBody::PSolve_RContacts(D_btSoftBody* psb,D_btScalar kst,D_btScalar ti)
{
	const D_btScalar	dt=psb->m_sst.sdt;
	const D_btScalar	mrg=psb->getCollisionShape()->getMargin();
	for(int i=0,ni=psb->m_rcontacts.size();i<ni;++i)
	{
		const RContact&		c=psb->m_rcontacts[i];
		const D_sCti&			cti=c.m_cti;	
		D_btRigidBody* tmpRigid = D_btRigidBody::upcast(cti.m_colObj);

		const D_btVector3		va=tmpRigid ? tmpRigid->getVelocityInLocalPoint(c.m_c1)*dt : D_btVector3(0,0,0);
		const D_btVector3		vb=c.m_node->m_x-c.m_node->m_q;	
		const D_btVector3		vr=vb-va;
		const D_btScalar		dn=D_btDot(vr,cti.m_normal);		
		if(dn<=D_SIMD_EPSILON)
		{
			const D_btScalar		dp=D_btMin(D_btDot(c.m_node->m_x,cti.m_normal)+cti.m_offset,mrg);
			const D_btVector3		fv=vr-cti.m_normal*dn;
			const D_btVector3		impulse=c.m_c0*((vr-fv*c.m_c3+cti.m_normal*(dp*c.m_c4))*kst);
			c.m_node->m_x-=impulse*c.m_c2;
			if (tmpRigid)
				tmpRigid->applyImpulse(impulse,c.m_c1);
		}
	}
}

//
void				D_btSoftBody::PSolve_SContacts(D_btSoftBody* psb,D_btScalar,D_btScalar ti)
{
	for(int i=0,ni=psb->m_scontacts.size();i<ni;++i)
	{
		const SContact&		c=psb->m_scontacts[i];
		const D_btVector3&	nr=c.m_normal;
		D_Node&				n=*c.m_node;
		D_Face&				f=*c.m_face;
		const D_btVector3		p=BaryEval(	f.m_n[0]->m_x,
			f.m_n[1]->m_x,
			f.m_n[2]->m_x,
			c.m_weights);
		const D_btVector3		q=BaryEval(	f.m_n[0]->m_q,
			f.m_n[1]->m_q,
			f.m_n[2]->m_q,
			c.m_weights);											
		const D_btVector3		vr=(n.m_x-n.m_q)-(p-q);
		D_btVector3			corr(0,0,0);
		D_btScalar dot = D_btDot(vr,nr);
		if(dot<0)
		{
			const D_btScalar	j=c.m_margin-(D_btDot(nr,n.m_x)-D_btDot(nr,p));
			corr+=c.m_normal*j;
		}
		corr			-=	ProjectOnPlane(vr,nr)*c.m_friction;
		n.m_x			+=	corr*c.m_cfm[0];
		f.m_n[0]->m_x	-=	corr*(c.m_cfm[1]*c.m_weights.x());
		f.m_n[1]->m_x	-=	corr*(c.m_cfm[1]*c.m_weights.y());
		f.m_n[2]->m_x	-=	corr*(c.m_cfm[1]*c.m_weights.z());
	}
}

//
void				D_btSoftBody::PSolve_Links(D_btSoftBody* psb,D_btScalar kst,D_btScalar ti)
{
	for(int i=0,ni=psb->m_links.size();i<ni;++i)
	{			
		D_Link&	l=psb->m_links[i];
		if(l.m_c0>0)
		{
			D_Node&			a=*l.m_n[0];
			D_Node&			b=*l.m_n[1];
			const D_btVector3	del=b.m_x-a.m_x;
			const D_btScalar	len=del.length2();
			const D_btScalar	k=((l.m_c1-len)/(l.m_c0*(l.m_c1+len)))*kst;
			//const D_btScalar	t=k*a.m_im;
			a.m_x-=del*(k*a.m_im);
			b.m_x+=del*(k*b.m_im);
		}
	}
}

//
void				D_btSoftBody::VSolve_Links(D_btSoftBody* psb,D_btScalar kst)
{
	for(int i=0,ni=psb->m_links.size();i<ni;++i)
	{			
		D_Link&			l=psb->m_links[i];
		D_Node**			n=l.m_n;
		const D_btScalar	j=-D_btDot(l.m_c3,n[0]->m_v-n[1]->m_v)*l.m_c2*kst;
		n[0]->m_v+=	l.m_c3*(j*n[0]->m_im);
		n[1]->m_v-=	l.m_c3*(j*n[1]->m_im);
	}
}

//
D_btSoftBody::psolver_t	D_btSoftBody::getSolver(ePSolver::D__ solver)
{
	switch(solver)
	{
	case	ePSolver::D_Anchors:		
		return(&D_btSoftBody::PSolve_Anchors);
	case	ePSolver::D_Linear:		
		return(&D_btSoftBody::PSolve_Links);
	case	ePSolver::D_RContacts:	
		return(&D_btSoftBody::PSolve_RContacts);
	case	ePSolver::D_SContacts:	
		return(&D_btSoftBody::PSolve_SContacts);	
	}
	return(0);
}

//
D_btSoftBody::vsolver_t	D_btSoftBody::getSolver(eVSolver::D__ solver)
{
	switch(solver)
	{
	case	eVSolver::D_Linear:		return(&D_btSoftBody::VSolve_Links);
	}
	return(0);
}

//
void			D_btSoftBody::defaultCollisionHandler(D_btCollisionObject* pco)
{
	switch(m_cfg.collisions&D_fCollision::D_RVSmask)
	{
	case	D_fCollision::D_SDF_RS:
		{
			D_btSoftColliders::CollideSDF_RS	docollide;		
			D_btRigidBody*		prb1=D_btRigidBody::upcast(pco);
			D_btTransform	wtr=prb1 ? prb1->getInterpolationWorldTransform() : pco->getWorldTransform();

			const D_btTransform	ctr=pco->getWorldTransform();
			const D_btScalar		timemargin=(wtr.getOrigin()-ctr.getOrigin()).length();
			const D_btScalar		basemargin=getCollisionShape()->getMargin();
			D_btVector3			mins;
			D_btVector3			maxs;
			D_ATTRIBUTE_ALIGNED16(D_btDbvtVolume)		volume;
			pco->getCollisionShape()->getAabb(	pco->getInterpolationWorldTransform(),
				mins,
				maxs);
			volume=D_btDbvtVolume::FromMM(mins,maxs);
			volume.Expand(D_btVector3(basemargin,basemargin,basemargin));		
			docollide.psb		=	this;
			docollide.m_colObj1 = pco;
			docollide.m_rigidBody = prb1;

			docollide.dynmargin	=	basemargin+timemargin;
			docollide.stamargin	=	basemargin;
			m_ndbvt.collideTV(m_ndbvt.m_root,volume,docollide);
		}
		break;
	case	D_fCollision::D_CL_RS:
		{
			D_btSoftColliders::CollideCL_RS	collider;
			collider.Process(this,pco);
		}
		break;
	}
}

//
void			D_btSoftBody::defaultCollisionHandler(D_btSoftBody* psb)
{
	const int cf=m_cfg.collisions&psb->m_cfg.collisions;
	switch(cf&D_fCollision::D_SVSmask)
	{
	case	D_fCollision::D_CL_SS:
		{
			
			//support self-collision if D_CL_SELF flag set
			if (this!=psb || psb->m_cfg.collisions&D_fCollision::D_CL_SELF)
			{
				D_btSoftColliders::CollideCL_SS	docollide;
				docollide.Process(this,psb);
			}
			
		}
		break;
	case	D_fCollision::D_VF_SS:
		{
			//D_only self-collision for Cluster, not Vertex-D_Face yet
			if (this!=psb)
			{
				D_btSoftColliders::CollideVF_SS	docollide;
				/* common					*/ 
				docollide.mrg=	getCollisionShape()->getMargin()+
					psb->getCollisionShape()->getMargin();
				/* psb0 nodes vs psb1 faces	*/ 
				docollide.psb[0]=this;
				docollide.psb[1]=psb;
				docollide.psb[0]->m_ndbvt.collideTT(	docollide.psb[0]->m_ndbvt.m_root,
					docollide.psb[1]->m_fdbvt.m_root,
					docollide);
				/* psb1 nodes vs psb0 faces	*/ 
				docollide.psb[0]=psb;
				docollide.psb[1]=this;
				docollide.psb[0]->m_ndbvt.collideTT(	docollide.psb[0]->m_ndbvt.m_root,
					docollide.psb[1]->m_fdbvt.m_root,
					docollide);
			}
		}
		break;
	default:
		{
			
		}
	}
}
