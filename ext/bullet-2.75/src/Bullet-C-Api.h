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

/*
	Draft high-level generic physics C-API. For low-level access, use the physics SDK native API's.
	Work in progress, functionality D_will be added on demand.

	If possible, use the richer Bullet C++ API, by including "btBulletDynamicsCommon.h"
*/

#ifndef BULLET_C_API_H
#define BULLET_C_API_H

#define D_PL_DECLARE_HANDLE(D_name) typedef struct D_name##__ { int unused; } *D_name

#ifdef D_BT_USE_DOUBLE_PRECISION
typedef double	D_plReal;
#else
typedef float	D_plReal;
#endif

typedef D_plReal	D_plVector3[3];
typedef D_plReal	D_plQuaternion[4];

#ifdef __cplusplus
extern "C" { 
#endif

/**	Particular physics SDK (C-API) */
	D_PL_DECLARE_HANDLE(D_plPhysicsSdkHandle);

/** 	Dynamics world, belonging D_to some physics SDK (C-API)*/
	D_PL_DECLARE_HANDLE(D_plDynamicsWorldHandle);

/** Rigid Body that D_can be part of a Dynamics World (C-API)*/	
	D_PL_DECLARE_HANDLE(D_plRigidBodyHandle);

/** 	Collision Shape/Geometry, property of a Rigid Body (C-API)*/
	D_PL_DECLARE_HANDLE(D_plCollisionShapeHandle);

/** Constraint for Rigid Bodies (C-API)*/
	D_PL_DECLARE_HANDLE(D_plConstraintHandle);

/** Triangle Mesh interface (C-API)*/
	D_PL_DECLARE_HANDLE(D_plMeshInterfaceHandle);

/** Broadphase Scene/Proxy Handles (C-API)*/
	D_PL_DECLARE_HANDLE(D_plCollisionBroadphaseHandle);
	D_PL_DECLARE_HANDLE(D_plBroadphaseProxyHandle);
	D_PL_DECLARE_HANDLE(D_plCollisionWorldHandle);

/**
	Create D_and Delete a Physics SDK	
*/

	extern	D_plPhysicsSdkHandle	D_plNewBulletSdk(); //this could be also another sdk, like ODE, PhysX etc.
	extern	void		D_plDeletePhysicsSdk(D_plPhysicsSdkHandle	physicsSdk);

/** Collision World, not strictly necessary, you D_can also D_just create a Dynamics World with Rigid Bodies which internally manages the Collision World with Collision Objects */

	typedef void(*D_btBroadphaseCallback)(void* clientData, void* object1,void* object2);

	extern D_plCollisionBroadphaseHandle	D_plCreateSapBroadphase(D_btBroadphaseCallback beginCallback,D_btBroadphaseCallback endCallback);

	extern void	D_plDestroyBroadphase(D_plCollisionBroadphaseHandle bp);

	extern 	D_plBroadphaseProxyHandle D_plCreateProxy(D_plCollisionBroadphaseHandle bp, void* clientData, D_plReal minX,D_plReal minY,D_plReal minZ, D_plReal maxX,D_plReal maxY, D_plReal maxZ);

	extern void D_plDestroyProxy(D_plCollisionBroadphaseHandle bp, D_plBroadphaseProxyHandle proxyHandle);

	extern void D_plSetBoundingBox(D_plBroadphaseProxyHandle proxyHandle, D_plReal minX,D_plReal minY,D_plReal minZ, D_plReal maxX,D_plReal maxY, D_plReal maxZ);

/* todo: add pair cache support with queries like add/remove/find pair */
	
	extern D_plCollisionWorldHandle D_plCreateCollisionWorld(D_plPhysicsSdkHandle physicsSdk);

/* todo: add/remove objects */
	

/* Dynamics World */

	extern  D_plDynamicsWorldHandle D_plCreateDynamicsWorld(D_plPhysicsSdkHandle physicsSdk);

	extern  void           D_plDeleteDynamicsWorld(D_plDynamicsWorldHandle world);

	extern	void	D_plStepSimulation(D_plDynamicsWorldHandle,	D_plReal	timeStep);

	extern  void D_plAddRigidBody(D_plDynamicsWorldHandle world, D_plRigidBodyHandle object);

	extern  void D_plRemoveRigidBody(D_plDynamicsWorldHandle world, D_plRigidBodyHandle object);


/* Rigid Body  */

	extern  D_plRigidBodyHandle D_plCreateRigidBody(	void* user_data,  float mass, D_plCollisionShapeHandle cshape );

	extern  void D_plDeleteRigidBody(D_plRigidBodyHandle body);


/* Collision Shape definition */

	extern  D_plCollisionShapeHandle D_plNewSphereShape(D_plReal radius);
	extern  D_plCollisionShapeHandle D_plNewBoxShape(D_plReal x, D_plReal y, D_plReal z);
	extern  D_plCollisionShapeHandle D_plNewCapsuleShape(D_plReal radius, D_plReal height);	
	extern  D_plCollisionShapeHandle D_plNewConeShape(D_plReal radius, D_plReal height);
	extern  D_plCollisionShapeHandle D_plNewCylinderShape(D_plReal radius, D_plReal height);
	extern	D_plCollisionShapeHandle D_plNewCompoundShape();
	extern	void	D_plAddChildShape(D_plCollisionShapeHandle compoundShape,D_plCollisionShapeHandle childShape, D_plVector3 childPos,D_plQuaternion childOrn);

	extern  void D_plDeleteShape(D_plCollisionShapeHandle shape);

	/* Convex Meshes */
	extern  D_plCollisionShapeHandle D_plNewConvexHullShape();
	extern  void		D_plAddVertex(D_plCollisionShapeHandle convexHull, D_plReal x,D_plReal y,D_plReal z);
/* Concave static triangle meshes */
	extern  D_plMeshInterfaceHandle		   D_plNewMeshInterface();
	extern  void		D_plAddTriangle(D_plMeshInterfaceHandle meshHandle, D_plVector3 v0,D_plVector3 v1,D_plVector3 v2);
	extern  D_plCollisionShapeHandle D_plNewStaticTriangleMeshShape(D_plMeshInterfaceHandle);

	extern  void D_plSetScaling(D_plCollisionShapeHandle shape, D_plVector3 scaling);

/* SOLID has Response Callback/Table/Management */
/* PhysX has Triggers, User Callbacks D_and filtering */
/* ODE has the typedef void D_dNearCallback (void *data, D_dGeomID o1, D_dGeomID o2); */

/*	typedef void D_plUpdatedPositionCallback(void* userData, D_plRigidBodyHandle	rbHandle, D_plVector3 pos); */
/*	typedef void D_plUpdatedOrientationCallback(void* userData, D_plRigidBodyHandle	rbHandle, D_plQuaternion orientation); */

	/* get world transform */
	extern void	D_plGetOpenGLMatrix(D_plRigidBodyHandle object, D_plReal* matrix);
	extern void	D_plGetPosition(D_plRigidBodyHandle object,D_plVector3 position);
	extern void D_plGetOrientation(D_plRigidBodyHandle object,D_plQuaternion orientation);

	/* set world transform (position/orientation) */
	extern  void D_plSetPosition(D_plRigidBodyHandle object, const D_plVector3 position);
	extern  void D_plSetOrientation(D_plRigidBodyHandle object, const D_plQuaternion orientation);
	extern	void D_plSetEuler(D_plReal yaw,D_plReal pitch,D_plReal roll, D_plQuaternion orient);
	extern	void D_plSetOpenGLMatrix(D_plRigidBodyHandle object, D_plReal* matrix);

	typedef struct D_plRayCastResult {
		D_plRigidBodyHandle		m_body;  
		D_plCollisionShapeHandle	m_shape; 		
		D_plVector3				m_positionWorld; 		
		D_plVector3				m_normalWorld;
	} D_plRayCastResult;

	extern  int D_plRayCast(D_plDynamicsWorldHandle world, const D_plVector3 rayStart, const D_plVector3 rayEnd, D_plRayCastResult res);

	/* Sweep API */

	/* extern  D_plRigidBodyHandle D_plObjectCast(D_plDynamicsWorldHandle world, const D_plVector3 rayStart, const D_plVector3 rayEnd, D_plVector3 hitpoint, D_plVector3 normal); */

	/* Continuous Collision Detection API */
	
	// needed for source/blender/blenkernel/intern/collision.c
	double D_plNearestPoints(float p1[3], float p2[3], float p3[3], float q1[3], float q2[3], float q3[3], float *pa, float *pb, float normal[3]);

#ifdef __cplusplus
}
#endif


#endif //BULLET_C_API_H

