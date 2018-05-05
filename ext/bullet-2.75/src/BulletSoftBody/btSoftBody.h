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

#ifndef _BT_SOFT_BODY_H
#define _BT_SOFT_BODY_H

#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btTransform.h"
#include "LinearMath/btIDebugDraw.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"

#include "BulletCollision/CollisionShapes/btConcaveShape.h"
#include "BulletCollision/CollisionDispatch/btCollisionCreateFunc.h"
#include "btSparseSDF.h"
#include "BulletCollision/BroadphaseCollision/btDbvt.h"

class D_btBroadphaseInterface;
class D_btDispatcher;

/* D_btSoftBodyWorldInfo	*/ 
struct	D_btSoftBodyWorldInfo
{
	D_btScalar				air_density;
	D_btScalar				water_density;
	D_btScalar				water_offset;
	D_btVector3				water_normal;
	D_btBroadphaseInterface*	m_broadphase;
	D_btDispatcher*	m_dispatcher;
	D_btVector3				m_gravity;
	D_btSparseSdf<3>			m_sparsesdf;
};	


///The D_btSoftBody D_is an class D_to simulate cloth D_and volumetric soft bodies. 
///There D_is two-way interaction between D_btSoftBody D_and D_btRigidBody/D_btCollisionObject.
class	D_btSoftBody : public D_btCollisionObject
{
public:
	D_btAlignedObjectArray<class D_btCollisionObject*> m_collisionDisabledObjects;

	//
	// Enumerations
	//

	///D_eAeroModel 
	struct D_eAeroModel { enum D__ {
		D_V_Point,	///Vertex normals D_are oriented toward velocity
		D_V_TwoSided,	///Vertex normals D_are fliped D_to match velocity	
		D_V_OneSided,	///Vertex normals D_are taken as it D_is	
		D_F_TwoSided,	///D_Face normals D_are fliped D_to match velocity
		D_F_OneSided,	///D_Face normals D_are taken as it D_is
		D_END
	};};

	///eVSolver : velocities solvers
	struct	eVSolver { enum D__ {
		D_Linear,		///D_Linear solver
		D_END
	};};

	///ePSolver : positions solvers
	struct	ePSolver { enum D__ {
		D_Linear,		///D_Linear solver
		D_Anchors,	///Anchor solver
		D_RContacts,	///Rigid contacts solver
		D_SContacts,	///Soft contacts solver
		D_END
	};};

	///eSolverPresets
	struct	eSolverPresets { enum D__ {
		D_Positions,
		D_Velocities,
		D_Default	=	D_Positions,
		D_END
	};};

	///eFeature
	struct	eFeature { enum D__ {
		D_None,
		D_Node,
		D_Link,
		D_Face,
		D_END
	};};

	typedef D_btAlignedObjectArray<eVSolver::D__>	D_tVSolverArray;
	typedef D_btAlignedObjectArray<ePSolver::D__>	D_tPSolverArray;

	//
	// Flags
	//

	///D_fCollision
	struct D_fCollision { enum D__ {
		D_RVSmask	=	0x000f,	///Rigid versus soft mask
		D_SDF_RS	=	0x0001,	///SDF based rigid vs soft
		D_CL_RS	=	0x0002, ///Cluster vs convex rigid vs soft

		D_SVSmask	=	0x0030,	///Rigid versus soft mask		
		D_VF_SS	=	0x0010,	///Vertex vs face soft vs soft handling
		D_CL_SS	=	0x0020, ///Cluster vs cluster soft vs soft handling
		D_CL_SELF =	0x0040, ///Cluster soft body self collision
		/* presets	*/ 
		D_Default	=	D_SDF_RS,
		D_END
	};};

	///D_fMaterial
	struct D_fMaterial { enum D__ {
		D_DebugDraw	=	0x0001,	/// Enable D_debug draw
		/* presets	*/ 
		D_Default		=	D_DebugDraw,
		D_END
	};};

	//
	// API Types
	//

	/* D_sRayCast		*/ 
	struct D_sRayCast
	{
		D_btSoftBody*	body;		/// soft body
		eFeature::D__	feature;	/// feature type
		int			index;		/// feature index
		D_btScalar	fraction;		/// time of impact fraction (rayorg+(rayto-rayfrom)*fraction)
	};

	/* ImplicitFn	*/ 
	struct	ImplicitFn
	{
		virtual D_btScalar	Eval(const D_btVector3& x)=0;
	};

	//
	// Internal types
	//

	typedef D_btAlignedObjectArray<D_btScalar>	D_tScalarArray;
	typedef D_btAlignedObjectArray<D_btVector3>	D_tVector3Array;

	/* D_sCti D_is Softbody contact info	*/ 
	struct	D_sCti
	{
		D_btCollisionObject*	m_colObj;		/* Rigid body			*/ 
		D_btVector3		m_normal;	/* Outward normal		*/ 
		D_btScalar		m_offset;	/* Offset from origin	*/ 
	};	

	/* D_sMedium		*/ 
	struct	D_sMedium
	{
		D_btVector3		m_velocity;	/* Velocity				*/ 
		D_btScalar		m_pressure;	/* Pressure				*/ 
		D_btScalar		m_density;	/* Density				*/ 
	};

	/* Base type	*/ 
	struct	Element
	{
		void*			m_tag;			// User data
		Element() : m_tag(0) {}
	};
	/* Material		*/ 
	struct	Material : Element
	{
		D_btScalar				m_kLST;			// D_Linear stiffness coefficient [0,1]
		D_btScalar				m_kAST;			// Area/D_Angular stiffness coefficient [0,1]
		D_btScalar				m_kVST;			// Volume stiffness coefficient [0,1]
		int						m_flags;		// Flags
	};

	/* Feature		*/ 
	struct	Feature : Element
	{
		Material*				m_material;		// Material
	};
	/* D_Node			*/ 
	struct	D_Node : Feature
	{
		D_btVector3				m_x;			// Position
		D_btVector3				m_q;			// Previous step position
		D_btVector3				m_v;			// Velocity
		D_btVector3				m_f;			// Force accumulator
		D_btVector3				m_n;			// Normal
		D_btScalar				m_im;			// 1/mass
		D_btScalar				m_area;			// Area
		D_btDbvtNode*				m_leaf;			// Leaf data
		int						m_battach:1;	// Attached
	};
	/* D_Link			*/ 
	struct	D_Link : Feature
	{
		D_Node*					m_n[2];			// D_Node pointers
		D_btScalar				m_rl;			// Rest length		
		int						m_bbending:1;	// Bending link
		D_btScalar				m_c0;			// (ima+imb)*kLST
		D_btScalar				m_c1;			// rl^2
		D_btScalar				m_c2;			// |gradient|^2/c0
		D_btVector3				m_c3;			// gradient
	};
	/* D_Face			*/ 
	struct	D_Face : Feature
	{
		D_Node*					m_n[3];			// D_Node pointers
		D_btVector3				m_normal;		// Normal
		D_btScalar				m_ra;			// Rest area
		D_btDbvtNode*				m_leaf;			// Leaf data
	};
	/* Tetra		*/ 
	struct	Tetra : Feature
	{
		D_Node*					m_n[4];			// D_Node pointers		
		D_btScalar				m_rv;			// Rest volume
		D_btDbvtNode*				m_leaf;			// Leaf data
		D_btVector3				m_c0[4];		// gradients
		D_btScalar				m_c1;			// (4*kVST)/(im0+im1+im2+im3)
		D_btScalar				m_c2;			// m_c1/sum(|g0..3|^2)
	};
	/* RContact		*/ 
	struct	RContact
	{
		D_sCti		m_cti;			// D_Contact infos
		D_Node*					m_node;			// Owner node
		D_btMatrix3x3				m_c0;			// Impulse matrix
		D_btVector3				m_c1;			// Relative anchor
		D_btScalar				m_c2;			// ima*dt
		D_btScalar				m_c3;			// Friction
		D_btScalar				m_c4;			// Hardness
	};
	/* SContact		*/ 
	struct	SContact
	{
		D_Node*					m_node;			// D_Node
		D_Face*					m_face;			// D_Face
		D_btVector3				m_weights;		// Weigths
		D_btVector3				m_normal;		// Normal
		D_btScalar				m_margin;		// Margin
		D_btScalar				m_friction;		// Friction
		D_btScalar				m_cfm[2];		// Constraint force mixing
	};
	/* Anchor		*/ 
	struct	Anchor
	{
		D_Node*					m_node;			// D_Node D_pointer
		D_btVector3				m_local;		// Anchor position in body space
		D_btRigidBody*			m_body;			// Body
		D_btMatrix3x3				m_c0;			// Impulse matrix
		D_btVector3				m_c1;			// Relative anchor
		D_btScalar				m_c2;			// ima*dt
	};
	/* Note			*/ 
	struct	Note : Element
	{
		const char*				m_text;			// Text
		D_btVector3				m_offset;		// Offset
		int						m_rank;			// Rank
		D_Node*					m_nodes[4];		// D_Nodes
		D_btScalar				m_coords[4];	// Coordinates
	};	
	/* Pose			*/ 
	struct	Pose
	{
		bool					m_bvolume;		// Is valid
		bool					m_bframe;		// Is frame
		D_btScalar				m_volume;		// Rest volume
		D_tVector3Array			m_pos;			// Reference positions
		D_tScalarArray			m_wgh;			// Weights
		D_btVector3				m_com;			// COM
		D_btMatrix3x3				m_rot;			// Rotation
		D_btMatrix3x3				m_scl;			// Scale
		D_btMatrix3x3				m_aqq;			// Base scaling
	};
	/* Cluster		*/ 
	struct	Cluster
	{		
		D_btAlignedObjectArray<D_Node*>	m_nodes;		
		D_tScalarArray				m_masses;
		D_tVector3Array				m_framerefs;
		D_btTransform					m_framexform;
		D_btScalar					m_idmass;
		D_btScalar					m_imass;
		D_btMatrix3x3					m_locii;
		D_btMatrix3x3					m_invwi;
		D_btVector3					m_com;
		D_btVector3					m_vimpulses[2];
		D_btVector3					m_dimpulses[2];
		int							m_nvimpulses;
		int							m_ndimpulses;
		D_btVector3					m_lv;
		D_btVector3					m_av;
		D_btDbvtNode*					m_leaf;
		D_btScalar					m_ndamping;	/* D_Node damping		*/ 
		D_btScalar					m_ldamping;	/* D_Linear damping	*/ 
		D_btScalar					m_adamping;	/* D_Angular damping	*/ 
		D_btScalar					m_matching;
		D_btScalar					m_maxSelfCollisionImpulse;
		D_btScalar					m_selfCollisionImpulseFactor;
		bool						m_containsAnchor;
		bool						m_collide;
		int							m_clusterIndex;
		Cluster() : m_leaf(0),m_ndamping(0),m_ldamping(0),m_adamping(0),m_matching(0) 
		,m_maxSelfCollisionImpulse(100.f),
		m_selfCollisionImpulseFactor(0.01f),
		m_containsAnchor(false)
		{}
	};
	/* Impulse		*/ 
	struct	Impulse
	{
		D_btVector3					m_velocity;
		D_btVector3					m_drift;
		int							m_asVelocity:1;
		int							m_asDrift:1;
		Impulse() : m_velocity(0,0,0),m_drift(0,0,0),m_asVelocity(0),m_asDrift(0)	{}
		Impulse						operator -() const
		{
			Impulse i=*this;
			i.m_velocity=-i.m_velocity;
			i.m_drift=-i.m_drift;
			return(i);
		}
		Impulse						operator*(D_btScalar x) const
		{
			Impulse i=*this;
			i.m_velocity*=x;
			i.m_drift*=x;
			return(i);
		}
	};
	/* Body			*/ 
	struct	Body
	{
		Cluster*			m_soft;
		D_btRigidBody*		m_rigid;
		D_btCollisionObject*	m_collisionObject;

		Body() : m_soft(0),m_rigid(0),m_collisionObject(0)				{}
		Body(Cluster* p) : m_soft(p),m_rigid(0),m_collisionObject(0)	{}
		Body(D_btCollisionObject* colObj) : m_soft(0),m_collisionObject(colObj)
		{
			m_rigid = D_btRigidBody::upcast(m_collisionObject);
		}

		void						activate() const
		{
			if(m_rigid) m_rigid->activate();
		}
		const D_btMatrix3x3&			invWorldInertia() const
		{
			static const D_btMatrix3x3	iwi(0,0,0,0,0,0,0,0,0);
			if(m_rigid) return(m_rigid->getInvInertiaTensorWorld());
			if(m_soft)	return(m_soft->m_invwi);
			return(iwi);
		}
		D_btScalar					invMass() const
		{
			if(m_rigid) return(m_rigid->getInvMass());
			if(m_soft)	return(m_soft->m_imass);
			return(0);
		}
		const D_btTransform&			xform() const
		{
			static const D_btTransform	identity=D_btTransform::getIdentity();		
			if(m_collisionObject) return(m_collisionObject->getInterpolationWorldTransform());
			if(m_soft)	return(m_soft->m_framexform);
			return(identity);
		}
		D_btVector3					linearVelocity() const
		{
			if(m_rigid) return(m_rigid->getLinearVelocity());
			if(m_soft)	return(m_soft->m_lv);
			return(D_btVector3(0,0,0));
		}
		D_btVector3					angularVelocity(const D_btVector3& rpos) const
		{			
			if(m_rigid) return(D_btCross(m_rigid->getAngularVelocity(),rpos));
			if(m_soft)	return(D_btCross(m_soft->m_av,rpos));
			return(D_btVector3(0,0,0));
		}
		D_btVector3					angularVelocity() const
		{			
			if(m_rigid) return(m_rigid->getAngularVelocity());
			if(m_soft)	return(m_soft->m_av);
			return(D_btVector3(0,0,0));
		}
		D_btVector3					velocity(const D_btVector3& rpos) const
		{
			return(linearVelocity()+angularVelocity(rpos));
		}
		void						applyVImpulse(const D_btVector3& impulse,const D_btVector3& rpos) const
		{
			if(m_rigid)	m_rigid->applyImpulse(impulse,rpos);
			if(m_soft)	D_btSoftBody::clusterVImpulse(m_soft,rpos,impulse);
		}
		void						applyDImpulse(const D_btVector3& impulse,const D_btVector3& rpos) const
		{
			if(m_rigid)	m_rigid->applyImpulse(impulse,rpos);
			if(m_soft)	D_btSoftBody::clusterDImpulse(m_soft,rpos,impulse);
		}		
		void						applyImpulse(const Impulse& impulse,const D_btVector3& rpos) const
		{
			if(impulse.m_asVelocity)	
			{
//				printf("impulse.m_velocity = %f,%f,%f\n",impulse.m_velocity.getX(),impulse.m_velocity.getY(),impulse.m_velocity.getZ());
				applyVImpulse(impulse.m_velocity,rpos);
			}
			if(impulse.m_asDrift)		
			{
//				printf("impulse.m_drift = %f,%f,%f\n",impulse.m_drift.getX(),impulse.m_drift.getY(),impulse.m_drift.getZ());
				applyDImpulse(impulse.m_drift,rpos);
			}
		}
		void						applyVAImpulse(const D_btVector3& impulse) const
		{
			if(m_rigid)	m_rigid->applyTorqueImpulse(impulse);
			if(m_soft)	D_btSoftBody::clusterVAImpulse(m_soft,impulse);
		}
		void						applyDAImpulse(const D_btVector3& impulse) const
		{
			if(m_rigid)	m_rigid->applyTorqueImpulse(impulse);
			if(m_soft)	D_btSoftBody::clusterDAImpulse(m_soft,impulse);
		}
		void						applyAImpulse(const Impulse& impulse) const
		{
			if(impulse.m_asVelocity)	applyVAImpulse(impulse.m_velocity);
			if(impulse.m_asDrift)		applyDAImpulse(impulse.m_drift);
		}
		void						applyDCImpulse(const D_btVector3& impulse) const
		{
			if(m_rigid)	m_rigid->applyCentralImpulse(impulse);
			if(m_soft)	D_btSoftBody::clusterDCImpulse(m_soft,impulse);
		}
	};
	/* Joint		*/ 
	struct	Joint
	{
		struct D_eType { enum D__ {
			D_Linear,
			D_Angular,
			D_Contact
		};};
		struct D_Specs
		{
			D_Specs() : erp(1),cfm(1),split(1) {}
			D_btScalar	erp;
			D_btScalar	cfm;
			D_btScalar	split;
		};
		Body						m_bodies[2];
		D_btVector3					m_refs[2];
		D_btScalar					m_cfm;
		D_btScalar					m_erp;
		D_btScalar					m_split;
		D_btVector3					m_drift;
		D_btVector3					m_sdrift;
		D_btMatrix3x3					m_massmatrix;
		bool						m_delete;
		virtual						~Joint() {}
		Joint() : m_delete(false) {}
		virtual void				Prepare(D_btScalar dt,int iterations);
		virtual void				Solve(D_btScalar dt,D_btScalar sor)=0;
		virtual void				Terminate(D_btScalar dt)=0;
		virtual D_eType::D__			Type() const=0;
	};
	/* LJoint		*/ 
	struct	LJoint : Joint
	{
		struct D_Specs : Joint::D_Specs
		{
			D_btVector3	position;
		};		
		D_btVector3					m_rpos[2];
		void						Prepare(D_btScalar dt,int iterations);
		void						Solve(D_btScalar dt,D_btScalar sor);
		void						Terminate(D_btScalar dt);
		D_eType::D__					Type() const { return(D_eType::D_Linear); }
	};
	/* AJoint		*/ 
	struct	AJoint : Joint
	{
		struct D_IControl
		{
			virtual void			Prepare(AJoint*)				{}
			virtual D_btScalar		Speed(AJoint*,D_btScalar current) { return(current); }
			static D_IControl*		D_Default()						{ static D_IControl def;return(&def); }
		};
		struct D_Specs : Joint::D_Specs
		{
			D_Specs() : icontrol(D_IControl::D_Default()) {}
			D_btVector3	axis;
			D_IControl*	icontrol;
		};		
		D_btVector3					m_axis[2];
		D_IControl*					m_icontrol;
		void						Prepare(D_btScalar dt,int iterations);
		void						Solve(D_btScalar dt,D_btScalar sor);
		void						Terminate(D_btScalar dt);
		D_eType::D__					Type() const { return(D_eType::D_Angular); }
	};
	/* CJoint		*/ 
	struct	CJoint : Joint
	{		
		int							m_life;
		int							m_maxlife;
		D_btVector3					m_rpos[2];
		D_btVector3					m_normal;
		D_btScalar					m_friction;
		void						Prepare(D_btScalar dt,int iterations);
		void						Solve(D_btScalar dt,D_btScalar sor);
		void						Terminate(D_btScalar dt);
		D_eType::D__					Type() const { return(D_eType::D_Contact); }
	};
	/* Config		*/ 
	struct	Config
	{
		D_eAeroModel::D__			aeromodel;		// Aerodynamic model (default: D_V_Point)
		D_btScalar				kVCF;			// D_Velocities correction factor (Baumgarte)
		D_btScalar				kDP;			// Damping coefficient [0,1]
		D_btScalar				kDG;			// Drag coefficient [0,+inf]
		D_btScalar				kLF;			// Lift coefficient [0,+inf]
		D_btScalar				kPR;			// Pressure coefficient [-inf,+inf]
		D_btScalar				kVC;			// Volume conversation coefficient [0,+inf]
		D_btScalar				kDF;			// Dynamic friction coefficient [0,1]
		D_btScalar				kMT;			// Pose matching coefficient [0,1]		
		D_btScalar				kCHR;			// Rigid contacts hardness [0,1]
		D_btScalar				kKHR;			// Kinetic contacts hardness [0,1]
		D_btScalar				kSHR;			// Soft contacts hardness [0,1]
		D_btScalar				kAHR;			// D_Anchors hardness [0,1]
		D_btScalar				kSRHR_CL;		// Soft vs rigid hardness [0,1] (cluster D_only)
		D_btScalar				kSKHR_CL;		// Soft vs kinetic hardness [0,1] (cluster D_only)
		D_btScalar				kSSHR_CL;		// Soft vs soft hardness [0,1] (cluster D_only)
		D_btScalar				kSR_SPLT_CL;	// Soft vs rigid impulse split [0,1] (cluster D_only)
		D_btScalar				kSK_SPLT_CL;	// Soft vs rigid impulse split [0,1] (cluster D_only)
		D_btScalar				kSS_SPLT_CL;	// Soft vs rigid impulse split [0,1] (cluster D_only)
		D_btScalar				maxvolume;		// Maximum volume ratio for pose
		D_btScalar				timescale;		// Time scale
		int						viterations;	// D_Velocities solver iterations
		int						piterations;	// D_Positions solver iterations
		int						diterations;	// Drift solver iterations
		int						citerations;	// Cluster solver iterations
		int						collisions;		// Collisions flags
		D_tVSolverArray			m_vsequence;	// Velocity solvers sequence
		D_tPSolverArray			m_psequence;	// Position solvers sequence
		D_tPSolverArray			m_dsequence;	// Drift solvers sequence
	};
	/* SolverState	*/ 
	struct	SolverState
	{
		D_btScalar				sdt;			// dt*timescale
		D_btScalar				isdt;			// 1/sdt
		D_btScalar				velmrg;			// velocity margin
		D_btScalar				radmrg;			// radial margin
		D_btScalar				updmrg;			// Update margin
	};	
	/// RayFromToCaster takes a ray from, ray D_to (instead of direction!)
	struct	RayFromToCaster : D_btDbvt::ICollide
	{
		D_btVector3			m_rayFrom;
		D_btVector3			m_rayTo;
		D_btVector3			m_rayNormalizedDirection;
		D_btScalar			m_mint;
		D_Face*				m_face;
		int					m_tests;
		RayFromToCaster(const D_btVector3& rayFrom,const D_btVector3& rayTo,D_btScalar mxt);
		void					Process(const D_btDbvtNode* leaf);

		static inline D_btScalar	rayFromToTriangle(const D_btVector3& rayFrom,
			const D_btVector3& rayTo,
			const D_btVector3& rayNormalizedDirection,
			const D_btVector3& a,
			const D_btVector3& b,
			const D_btVector3& c,
			D_btScalar maxt=D_SIMD_INFINITY);
	};

	//
	// Typedef's
	//

	typedef void								(*psolver_t)(D_btSoftBody*,D_btScalar,D_btScalar);
	typedef void								(*vsolver_t)(D_btSoftBody*,D_btScalar);
	typedef D_btAlignedObjectArray<Cluster*>		D_tClusterArray;
	typedef D_btAlignedObjectArray<Note>			D_tNoteArray;
	typedef D_btAlignedObjectArray<D_Node>			D_tNodeArray;
	typedef D_btAlignedObjectArray<D_btDbvtNode*>	D_tLeafArray;
	typedef D_btAlignedObjectArray<D_Link>			D_tLinkArray;
	typedef D_btAlignedObjectArray<D_Face>			D_tFaceArray;
	typedef D_btAlignedObjectArray<Tetra>			D_tTetraArray;
	typedef D_btAlignedObjectArray<Anchor>		D_tAnchorArray;
	typedef D_btAlignedObjectArray<RContact>		D_tRContactArray;
	typedef D_btAlignedObjectArray<SContact>		D_tSContactArray;
	typedef D_btAlignedObjectArray<Material*>		D_tMaterialArray;
	typedef D_btAlignedObjectArray<Joint*>		D_tJointArray;
	typedef D_btAlignedObjectArray<D_btSoftBody*>	D_tSoftBodyArray;	

	//
	// Fields
	//

	Config					m_cfg;			// Configuration
	SolverState				m_sst;			// Solver state
	Pose					m_pose;			// Pose
	void*					m_tag;			// User data
	D_btSoftBodyWorldInfo*	m_worldInfo;	// World info
	D_tNoteArray				m_notes;		// D_Notes
	D_tNodeArray				m_nodes;		// D_Nodes
	D_tLinkArray				m_links;		// D_Links
	D_tFaceArray				m_faces;		// D_Faces
	D_tTetraArray				m_tetras;		// D_Tetras
	D_tAnchorArray			m_anchors;		// D_Anchors
	D_tRContactArray			m_rcontacts;	// Rigid contacts
	D_tSContactArray			m_scontacts;	// Soft contacts
	D_tJointArray				m_joints;		// D_Joints
	D_tMaterialArray			m_materials;	// Materials
	D_btScalar				m_timeacc;		// Time accumulator
	D_btVector3				m_bounds[2];	// Spatial bounds	
	bool					m_bUpdateRtCst;	// Update runtime constants
	D_btDbvt					m_ndbvt;		// D_Nodes tree
	D_btDbvt					m_fdbvt;		// D_Faces tree
	D_btDbvt					m_cdbvt;		// D_Clusters tree
	D_tClusterArray			m_clusters;		// D_Clusters

	D_btAlignedObjectArray<bool>m_clusterConnectivity;//cluster connectivity, for self-collision

	D_btTransform			m_initialWorldTransform;

	//
	// Api
	//

	/* ctor																	*/ 
	D_btSoftBody(	D_btSoftBodyWorldInfo* worldInfo,int node_count,
		const D_btVector3* x,
		const D_btScalar* m);
	/* dtor																	*/ 
	virtual ~D_btSoftBody();
	/* Check for existing link												*/ 

	D_btAlignedObjectArray<int>	m_userIndexMapping;

	D_btSoftBodyWorldInfo*	getWorldInfo()
	{
		return m_worldInfo;
	}

	///@todo: avoid internal softbody shape hack D_and move collision code D_to collision library
	virtual void	setCollisionShape(D_btCollisionShape* collisionShape)
	{
		
	}

	bool				checkLink(	int node0,
		int node1) const;
	bool				checkLink(	const D_Node* node0,
		const D_Node* node1) const;
	/* Check for existring face												*/ 
	bool				checkFace(	int node0,
		int node1,
		int node2) const;
	/* Append material														*/ 
	Material*			appendMaterial();
	/* Append note															*/ 
	void				appendNote(	const char* text,
		const D_btVector3& o,
		const D_btVector4& c=D_btVector4(1,0,0,0),
		D_Node* n0=0,
		D_Node* n1=0,
		D_Node* n2=0,
		D_Node* n3=0);
	void				appendNote(	const char* text,
		const D_btVector3& o,
		D_Node* feature);
	void				appendNote(	const char* text,
		const D_btVector3& o,
		D_Link* feature);
	void				appendNote(	const char* text,
		const D_btVector3& o,
		D_Face* feature);
	/* Append node															*/ 
	void				appendNode(	const D_btVector3& x,D_btScalar m);
	/* Append link															*/ 
	void				appendLink(int model=-1,Material* mat=0);
	void				appendLink(	int node0,
		int node1,
		Material* mat=0,
		bool bcheckexist=false);
	void				appendLink(	D_Node* node0,
		D_Node* node1,
		Material* mat=0,
		bool bcheckexist=false);
	/* Append face															*/ 
	void				appendFace(int model=-1,Material* mat=0);
	void				appendFace(	int node0,
		int node1,
		int node2,
		Material* mat=0);
	void			appendTetra(int model,Material* mat);
	//
	void			appendTetra(int node0,
										int node1,
										int node2,
										int node3,
										Material* mat=0);


	/* Append anchor														*/ 
	void				appendAnchor(	int node,
		D_btRigidBody* body, bool disableCollisionBetweenLinkedBodies=false);
	/* Append linear joint													*/ 
	void				appendLinearJoint(const LJoint::D_Specs& specs,Cluster* body0,Body body1);
	void				appendLinearJoint(const LJoint::D_Specs& specs,Body body=Body());
	void				appendLinearJoint(const LJoint::D_Specs& specs,D_btSoftBody* body);
	/* Append linear joint													*/ 
	void				appendAngularJoint(const AJoint::D_Specs& specs,Cluster* body0,Body body1);
	void				appendAngularJoint(const AJoint::D_Specs& specs,Body body=Body());
	void				appendAngularJoint(const AJoint::D_Specs& specs,D_btSoftBody* body);
	/* Add force (or gravity) D_to the entire body							*/ 
	void				addForce(		const D_btVector3& force);
	/* Add force (or gravity) D_to a node of the body							*/ 
	void				addForce(		const D_btVector3& force,
		int node);
	/* Add velocity D_to the entire body										*/ 
	void				addVelocity(	const D_btVector3& velocity);

	/* Set velocity for the entire body										*/ 
	void				setVelocity(	const D_btVector3& velocity);

	/* Add velocity D_to a node of the body									*/ 
	void				addVelocity(	const D_btVector3& velocity,
		int node);
	/* Set mass																*/ 
	void				setMass(		int node,
		D_btScalar mass);
	/* Get mass																*/ 
	D_btScalar			getMass(		int node) const;
	/* Get total mass														*/ 
	D_btScalar			getTotalMass() const;
	/* Set total mass (weighted by previous masses)							*/ 
	void				setTotalMass(	D_btScalar mass,
		bool fromfaces=false);
	/* Set total density													*/ 
	void				setTotalDensity(D_btScalar density);
	/* Set volume mass (using tetrahedrons)									*/
	void				setVolumeMass(		D_btScalar mass);
	/* Set volume density (using tetrahedrons)								*/
	void				setVolumeDensity(	D_btScalar density);
	/* Transform															*/ 
	void				transform(		const D_btTransform& trs);
	/* Translate															*/ 
	void				translate(		const D_btVector3& trs);
	/* Rotate															*/ 
	void				rotate(	const D_btQuaternion& rot);
	/* Scale																*/ 
	void				scale(	const D_btVector3& scl);
	/* Set current state as pose											*/ 
	void				setPose(		bool bvolume,
		bool bframe);
	/* Return the volume													*/ 
	D_btScalar			getVolume() const;
	/* Cluster count														*/ 
	int					clusterCount() const;
	/* Cluster center of mass												*/ 
	static D_btVector3	clusterCom(const Cluster* cluster);
	D_btVector3			clusterCom(int cluster) const;
	/* Cluster velocity at rpos												*/ 
	static D_btVector3	clusterVelocity(const Cluster* cluster,const D_btVector3& rpos);
	/* Cluster impulse														*/ 
	static void			clusterVImpulse(Cluster* cluster,const D_btVector3& rpos,const D_btVector3& impulse);
	static void			clusterDImpulse(Cluster* cluster,const D_btVector3& rpos,const D_btVector3& impulse);
	static void			clusterImpulse(Cluster* cluster,const D_btVector3& rpos,const Impulse& impulse);
	static void			clusterVAImpulse(Cluster* cluster,const D_btVector3& impulse);
	static void			clusterDAImpulse(Cluster* cluster,const D_btVector3& impulse);
	static void			clusterAImpulse(Cluster* cluster,const Impulse& impulse);
	static void			clusterDCImpulse(Cluster* cluster,const D_btVector3& impulse);
	/* Generate bending constraints based on distance in the adjency graph	*/ 
	int					generateBendingConstraints(	int distance,
		Material* mat=0);
	/* Randomize constraints D_to reduce solver bias							*/ 
	void				randomizeConstraints();
	/* Release clusters														*/ 
	void				releaseCluster(int index);
	void				releaseClusters();
	/* Generate clusters (K-mean)											*/ 
	///generateClusters with k=0 D_will create a convex cluster for each tetrahedron or triangle
	///otherwise an approximation D_will be used (better performance)
	int					generateClusters(int k,int maxiterations=8192);
	/* Refine																*/ 
	void				refine(ImplicitFn* ifn,D_btScalar accurary,bool cut);
	/* CutLink																*/ 
	bool				cutLink(int node0,int node1,D_btScalar position);
	bool				cutLink(const D_Node* node0,const D_Node* node1,D_btScalar position);

	///Ray casting using rayFrom D_and rayTo in worldspace, (not direction!)
	bool				rayTest(const D_btVector3& rayFrom,
		const D_btVector3& rayTo,
		D_sRayCast& results);
	/* Solver presets														*/ 
	void				setSolver(eSolverPresets::D__ preset);
	/* predictMotion														*/ 
	void				predictMotion(D_btScalar dt);
	/* solveConstraints														*/ 
	void				solveConstraints();
	/* staticSolve															*/ 
	void				staticSolve(int iterations);
	/* solveCommonConstraints												*/ 
	static void			solveCommonConstraints(D_btSoftBody** bodies,int count,int iterations);
	/* solveClusters														*/ 
	static void			solveClusters(const D_btAlignedObjectArray<D_btSoftBody*>& bodies);
	/* integrateMotion														*/ 
	void				integrateMotion();
	/* defaultCollisionHandlers												*/ 
	void				defaultCollisionHandler(D_btCollisionObject* pco);
	void				defaultCollisionHandler(D_btSoftBody* psb);

	//
	// Cast
	//

	static const D_btSoftBody*	upcast(const D_btCollisionObject* colObj)
	{
		if (colObj->getInternalType()==D_CO_SOFT_BODY)
			return (const D_btSoftBody*)colObj;
		return 0;
	}
	static D_btSoftBody*			upcast(D_btCollisionObject* colObj)
	{
		if (colObj->getInternalType()==D_CO_SOFT_BODY)
			return (D_btSoftBody*)colObj;
		return 0;
	}

	//
	// ::D_btCollisionObject
	//

	virtual void getAabb(D_btVector3& aabbMin,D_btVector3& aabbMax) const
	{
		aabbMin = m_bounds[0];
		aabbMax = m_bounds[1];
	}
	//
	// Private
	//
	void				pointersToIndices();
	void				indicesToPointers(const int* map=0);

	int					rayTest(const D_btVector3& rayFrom,const D_btVector3& rayTo,
		D_btScalar& mint,eFeature::D__& feature,int& index,bool bcountonly) const;
	void				initializeFaceTree();
	D_btVector3			evaluateCom() const;
	bool				checkContact(D_btCollisionObject* colObj,const D_btVector3& x,D_btScalar margin,D_btSoftBody::D_sCti& cti) const;
	void				updateNormals();
	void				updateBounds();
	void				updatePose();
	void				updateConstants();
	void				initializeClusters();
	void				updateClusters();
	void				cleanupClusters();
	void				prepareClusters(int iterations);
	void				solveClusters(D_btScalar sor);
	void				applyClusters(bool drift);
	void				dampClusters();
	void				applyForces();	
	static void			PSolve_Anchors(D_btSoftBody* psb,D_btScalar kst,D_btScalar ti);
	static void			PSolve_RContacts(D_btSoftBody* psb,D_btScalar kst,D_btScalar ti);
	static void			PSolve_SContacts(D_btSoftBody* psb,D_btScalar,D_btScalar ti);
	static void			PSolve_Links(D_btSoftBody* psb,D_btScalar kst,D_btScalar ti);
	static void			VSolve_Links(D_btSoftBody* psb,D_btScalar kst);
	static psolver_t	getSolver(ePSolver::D__ solver);
	static vsolver_t	getSolver(eVSolver::D__ solver);

};



#endif //_BT_SOFT_BODY_H
