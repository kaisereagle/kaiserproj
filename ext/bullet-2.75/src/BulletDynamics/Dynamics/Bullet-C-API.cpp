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

	If possible, use the richer Bullet C++ API, by including <src/D_btBulletDynamicsCommon.h>
*/

#include "Bullet-C-Api.h"
#include "btBulletDynamicsCommon.h"
#include "LinearMath/btAlignedAllocator.h"



#include "LinearMath/btVector3.h"
#include "LinearMath/btScalar.h"	
#include "LinearMath/btMatrix3x3.h"
#include "LinearMath/btTransform.h"
#include "BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h"
#include "BulletCollision/CollisionShapes/btTriangleShape.h"

#include "BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h"
#include "BulletCollision/NarrowPhaseCollision/btPointCollector.h"
#include "BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h"
#include "BulletCollision/NarrowPhaseCollision/btSubSimplexConvexCast.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkEpa2.h"
#include "BulletCollision/CollisionShapes/btMinkowskiSumShape.h"
#include "BulletCollision/NarrowPhaseCollision/btDiscreteCollisionDetectorInterface.h"
#include "BulletCollision/NarrowPhaseCollision/btSimplexSolverInterface.h"
#include "BulletCollision/NarrowPhaseCollision/btMinkowskiPenetrationDepthSolver.h"
#include "LinearMath/btStackAlloc.h"

/*
	Create D_and Delete a Physics SDK	
*/

struct	D_btPhysicsSdk
{

//	D_btDispatcher*				m_dispatcher;
//	D_btOverlappingPairCache*		m_pairCache;
//	D_btConstraintSolver*			m_constraintSolver

	D_btVector3	m_worldAabbMin;
	D_btVector3	m_worldAabbMax;


	//todo: version, hardware/optimization settings etc?
	D_btPhysicsSdk()
		:m_worldAabbMin(-1000,-1000,-1000),
		m_worldAabbMax(1000,1000,1000)
	{

	}

	
};

D_plPhysicsSdkHandle	D_plNewBulletSdk()
{
	void* mem = D_btAlignedAlloc(sizeof(D_btPhysicsSdk),16);
	return (D_plPhysicsSdkHandle)new (mem)D_btPhysicsSdk;
}

void		D_plDeletePhysicsSdk(D_plPhysicsSdkHandle	physicsSdk)
{
	D_btPhysicsSdk* phys = reinterpret_cast<D_btPhysicsSdk*>(physicsSdk);
	D_btAlignedFree(phys);	
}


/* Dynamics World */
D_plDynamicsWorldHandle D_plCreateDynamicsWorld(D_plPhysicsSdkHandle physicsSdkHandle)
{
	D_btPhysicsSdk* physicsSdk = reinterpret_cast<D_btPhysicsSdk*>(physicsSdkHandle);
	void* mem = D_btAlignedAlloc(sizeof(D_btDefaultCollisionConfiguration),16);
	D_btDefaultCollisionConfiguration* collisionConfiguration = new (mem)D_btDefaultCollisionConfiguration();
	mem = D_btAlignedAlloc(sizeof(D_btCollisionDispatcher),16);
	D_btDispatcher*				dispatcher = new (mem)D_btCollisionDispatcher(collisionConfiguration);
	mem = D_btAlignedAlloc(sizeof(D_btAxisSweep3),16);
	D_btBroadphaseInterface*		pairCache = new (mem)D_btAxisSweep3(physicsSdk->m_worldAabbMin,physicsSdk->m_worldAabbMax);
	mem = D_btAlignedAlloc(sizeof(D_btSequentialImpulseConstraintSolver),16);
	D_btConstraintSolver*			constraintSolver = new(mem) D_btSequentialImpulseConstraintSolver();

	mem = D_btAlignedAlloc(sizeof(D_btDiscreteDynamicsWorld),16);
	return (D_plDynamicsWorldHandle) new (mem)D_btDiscreteDynamicsWorld(dispatcher,pairCache,constraintSolver,collisionConfiguration);
}
void           D_plDeleteDynamicsWorld(D_plDynamicsWorldHandle world)
{
	//todo: also clean up the other allocations, axisSweep, pairCache,dispatcher,constraintSolver,collisionConfiguration
	D_btDynamicsWorld* dynamicsWorld = reinterpret_cast< D_btDynamicsWorld* >(world);
	D_btAlignedFree(dynamicsWorld);
}

void	D_plStepSimulation(D_plDynamicsWorldHandle world,	D_plReal	timeStep)
{
	D_btDynamicsWorld* dynamicsWorld = reinterpret_cast< D_btDynamicsWorld* >(world);
	D_btAssert(dynamicsWorld);
	dynamicsWorld->stepSimulation(timeStep);
}

void D_plAddRigidBody(D_plDynamicsWorldHandle world, D_plRigidBodyHandle object)
{
	D_btDynamicsWorld* dynamicsWorld = reinterpret_cast< D_btDynamicsWorld* >(world);
	D_btAssert(dynamicsWorld);
	D_btRigidBody* body = reinterpret_cast< D_btRigidBody* >(object);
	D_btAssert(body);

	dynamicsWorld->addRigidBody(body);
}

void D_plRemoveRigidBody(D_plDynamicsWorldHandle world, D_plRigidBodyHandle object)
{
	D_btDynamicsWorld* dynamicsWorld = reinterpret_cast< D_btDynamicsWorld* >(world);
	D_btAssert(dynamicsWorld);
	D_btRigidBody* body = reinterpret_cast< D_btRigidBody* >(object);
	D_btAssert(body);

	dynamicsWorld->removeRigidBody(body);
}

/* Rigid Body  */

D_plRigidBodyHandle D_plCreateRigidBody(	void* user_data,  float mass, D_plCollisionShapeHandle cshape )
{
	D_btTransform trans;
	trans.setIdentity();
	D_btVector3 localInertia(0,0,0);
	D_btCollisionShape* shape = reinterpret_cast<D_btCollisionShape*>( cshape);
	D_btAssert(shape);
	if (mass)
	{
		shape->calculateLocalInertia(mass,localInertia);
	}
	void* mem = D_btAlignedAlloc(sizeof(D_btRigidBody),16);
	D_btRigidBody::D_btRigidBodyConstructionInfo rbci(mass, 0,shape,localInertia);
	D_btRigidBody* body = new (mem)D_btRigidBody(rbci);
	body->setWorldTransform(trans);
	body->setUserPointer(user_data);
	return (D_plRigidBodyHandle) body;
}

void D_plDeleteRigidBody(D_plRigidBodyHandle cbody)
{
	D_btRigidBody* body = reinterpret_cast< D_btRigidBody* >(cbody);
	D_btAssert(body);
	D_btAlignedFree( body);
}


/* Collision Shape definition */

D_plCollisionShapeHandle D_plNewSphereShape(D_plReal radius)
{
	void* mem = D_btAlignedAlloc(sizeof(D_btSphereShape),16);
	return (D_plCollisionShapeHandle) new (mem)D_btSphereShape(radius);
	
}
	
D_plCollisionShapeHandle D_plNewBoxShape(D_plReal x, D_plReal y, D_plReal z)
{
	void* mem = D_btAlignedAlloc(sizeof(D_btBoxShape),16);
	return (D_plCollisionShapeHandle) new (mem)D_btBoxShape(D_btVector3(x,y,z));
}

D_plCollisionShapeHandle D_plNewCapsuleShape(D_plReal radius, D_plReal height)
{
	//capsule D_is convex hull of 2 spheres, so use D_btMultiSphereShape
	
	const int numSpheres = 2;
	D_btVector3 positions[numSpheres] = {D_btVector3(0,height,0),D_btVector3(0,-height,0)};
	D_btScalar radi[numSpheres] = {radius,radius};
	void* mem = D_btAlignedAlloc(sizeof(D_btMultiSphereShape),16);
	return (D_plCollisionShapeHandle) new (mem)D_btMultiSphereShape(positions,radi,numSpheres);
}
D_plCollisionShapeHandle D_plNewConeShape(D_plReal radius, D_plReal height)
{
	void* mem = D_btAlignedAlloc(sizeof(D_btConeShape),16);
	return (D_plCollisionShapeHandle) new (mem)D_btConeShape(radius,height);
}

D_plCollisionShapeHandle D_plNewCylinderShape(D_plReal radius, D_plReal height)
{
	void* mem = D_btAlignedAlloc(sizeof(D_btCylinderShape),16);
	return (D_plCollisionShapeHandle) new (mem)D_btCylinderShape(D_btVector3(radius,height,radius));
}

/* Convex Meshes */
D_plCollisionShapeHandle D_plNewConvexHullShape()
{
	void* mem = D_btAlignedAlloc(sizeof(D_btConvexHullShape),16);
	return (D_plCollisionShapeHandle) new (mem)D_btConvexHullShape();
}


/* Concave static triangle meshes */
D_plMeshInterfaceHandle		   D_plNewMeshInterface()
{
	return 0;
}

D_plCollisionShapeHandle D_plNewCompoundShape()
{
	void* mem = D_btAlignedAlloc(sizeof(D_btCompoundShape),16);
	return (D_plCollisionShapeHandle) new (mem)D_btCompoundShape();
}

void	D_plAddChildShape(D_plCollisionShapeHandle compoundShapeHandle,D_plCollisionShapeHandle childShapeHandle, D_plVector3 childPos,D_plQuaternion childOrn)
{
	D_btCollisionShape* colShape = reinterpret_cast<D_btCollisionShape*>(compoundShapeHandle);
	D_btAssert(colShape->getShapeType() == D_COMPOUND_SHAPE_PROXYTYPE);
	D_btCompoundShape* compoundShape = reinterpret_cast<D_btCompoundShape*>(colShape);
	D_btCollisionShape* childShape = reinterpret_cast<D_btCollisionShape*>(childShapeHandle);
	D_btTransform	localTrans;
	localTrans.setIdentity();
	localTrans.setOrigin(D_btVector3(childPos[0],childPos[1],childPos[2]));
	localTrans.setRotation(D_btQuaternion(childOrn[0],childOrn[1],childOrn[2],childOrn[3]));
	compoundShape->addChildShape(localTrans,childShape);
}

void D_plSetEuler(D_plReal yaw,D_plReal pitch,D_plReal roll, D_plQuaternion orient)
{
	D_btQuaternion orn;
	orn.setEuler(yaw,pitch,roll);
	orient[0] = orn.getX();
	orient[1] = orn.getY();
	orient[2] = orn.getZ();
	orient[3] = orn.getW();

}


//	extern  void		D_plAddTriangle(D_plMeshInterfaceHandle meshHandle, D_plVector3 v0,D_plVector3 v1,D_plVector3 v2);
//	extern  D_plCollisionShapeHandle D_plNewStaticTriangleMeshShape(D_plMeshInterfaceHandle);


void		D_plAddVertex(D_plCollisionShapeHandle cshape, D_plReal x,D_plReal y,D_plReal z)
{
	D_btCollisionShape* colShape = reinterpret_cast<D_btCollisionShape*>( cshape);
	(void)colShape;
	D_btAssert(colShape->getShapeType()==D_CONVEX_HULL_SHAPE_PROXYTYPE);
	D_btConvexHullShape* convexHullShape = reinterpret_cast<D_btConvexHullShape*>( cshape);
	convexHullShape->addPoint(D_btVector3(x,y,z));

}

void D_plDeleteShape(D_plCollisionShapeHandle cshape)
{
	D_btCollisionShape* shape = reinterpret_cast<D_btCollisionShape*>( cshape);
	D_btAssert(shape);
	D_btAlignedFree(shape);
}
void D_plSetScaling(D_plCollisionShapeHandle cshape, D_plVector3 cscaling)
{
	D_btCollisionShape* shape = reinterpret_cast<D_btCollisionShape*>( cshape);
	D_btAssert(shape);
	D_btVector3 scaling(cscaling[0],cscaling[1],cscaling[2]);
	shape->setLocalScaling(scaling);	
}



void D_plSetPosition(D_plRigidBodyHandle object, const D_plVector3 position)
{
	D_btRigidBody* body = reinterpret_cast< D_btRigidBody* >(object);
	D_btAssert(body);
	D_btVector3 pos(position[0],position[1],position[2]);
	D_btTransform worldTrans = body->getWorldTransform();
	worldTrans.setOrigin(pos);
	body->setWorldTransform(worldTrans);
}

void D_plSetOrientation(D_plRigidBodyHandle object, const D_plQuaternion orientation)
{
	D_btRigidBody* body = reinterpret_cast< D_btRigidBody* >(object);
	D_btAssert(body);
	D_btQuaternion orn(orientation[0],orientation[1],orientation[2],orientation[3]);
	D_btTransform worldTrans = body->getWorldTransform();
	worldTrans.setRotation(orn);
	body->setWorldTransform(worldTrans);
}

void	D_plSetOpenGLMatrix(D_plRigidBodyHandle object, D_plReal* matrix)
{
	D_btRigidBody* body = reinterpret_cast< D_btRigidBody* >(object);
	D_btAssert(body);
	D_btTransform& worldTrans = body->getWorldTransform();
	worldTrans.setFromOpenGLMatrix(matrix);
}

void	D_plGetOpenGLMatrix(D_plRigidBodyHandle object, D_plReal* matrix)
{
	D_btRigidBody* body = reinterpret_cast< D_btRigidBody* >(object);
	D_btAssert(body);
	body->getWorldTransform().getOpenGLMatrix(matrix);

}

void	D_plGetPosition(D_plRigidBodyHandle object,D_plVector3 position)
{
	D_btRigidBody* body = reinterpret_cast< D_btRigidBody* >(object);
	D_btAssert(body);
	const D_btVector3& pos = body->getWorldTransform().getOrigin();
	position[0] = pos.getX();
	position[1] = pos.getY();
	position[2] = pos.getZ();
}

void D_plGetOrientation(D_plRigidBodyHandle object,D_plQuaternion orientation)
{
	D_btRigidBody* body = reinterpret_cast< D_btRigidBody* >(object);
	D_btAssert(body);
	const D_btQuaternion& orn = body->getWorldTransform().getRotation();
	orientation[0] = orn.getX();
	orientation[1] = orn.getY();
	orientation[2] = orn.getZ();
	orientation[3] = orn.getW();
}



//D_plRigidBodyHandle D_plRayCast(D_plDynamicsWorldHandle world, const D_plVector3 rayStart, const D_plVector3 rayEnd, D_plVector3 hitpoint, D_plVector3 normal);

//	extern  D_plRigidBodyHandle D_plObjectCast(D_plDynamicsWorldHandle world, const D_plVector3 rayStart, const D_plVector3 rayEnd, D_plVector3 hitpoint, D_plVector3 normal);

double D_plNearestPoints(float p1[3], float p2[3], float p3[3], float q1[3], float q2[3], float q3[3], float *pa, float *pb, float normal[3])
{
	D_btVector3 vp(p1[0], p1[1], p1[2]);
	D_btTriangleShape trishapeA(vp, 
				  D_btVector3(p2[0], p2[1], p2[2]), 
				  D_btVector3(p3[0], p3[1], p3[2]));
	trishapeA.setMargin(0.000001f);
	D_btVector3 vq(q1[0], q1[1], q1[2]);
	D_btTriangleShape trishapeB(vq, 
				  D_btVector3(q2[0], q2[1], q2[2]), 
				  D_btVector3(q3[0], q3[1], q3[2]));
	trishapeB.setMargin(0.000001f);
	
	// D_btVoronoiSimplexSolver D_sGjkSimplexSolver;
	// D_btGjkEpaPenetrationDepthSolver penSolverPtr;	
	
	static D_btSimplexSolverInterface D_sGjkSimplexSolver;
	D_sGjkSimplexSolver.reset();
	
	static D_btGjkEpaPenetrationDepthSolver Solver0;
	static D_btMinkowskiPenetrationDepthSolver Solver1;
		
	D_btConvexPenetrationDepthSolver* Solver = NULL;
	
	Solver = &Solver1;	
		
	D_btGjkPairDetector convexConvex(&trishapeA ,&trishapeB,&D_sGjkSimplexSolver,Solver);
	
	convexConvex.m_catchDegeneracies = 1;
	
	// D_btGjkPairDetector convexConvex(&trishapeA ,&trishapeB,&D_sGjkSimplexSolver,0);
	
	D_btPointCollector gjkOutput;
	D_btGjkPairDetector::D_ClosestPointInput input;
	
	D_btStackAlloc D_gStackAlloc(1024*1024*2);
 
	input.m_stackAlloc = &D_gStackAlloc;
	
	D_btTransform tr;
	tr.setIdentity();
	
	input.m_transformA = tr;
	input.m_transformB = tr;
	
	convexConvex.getClosestPoints(input, gjkOutput, 0);
	
	
	if (gjkOutput.m_hasResult)
	{
		
		pb[0] = pa[0] = gjkOutput.m_pointInWorld[0];
		pb[1] = pa[1] = gjkOutput.m_pointInWorld[1];
		pb[2] = pa[2] = gjkOutput.m_pointInWorld[2];

		pb[0]+= gjkOutput.m_normalOnBInWorld[0] * gjkOutput.m_distance;
		pb[1]+= gjkOutput.m_normalOnBInWorld[1] * gjkOutput.m_distance;
		pb[2]+= gjkOutput.m_normalOnBInWorld[2] * gjkOutput.m_distance;
		
		normal[0] = gjkOutput.m_normalOnBInWorld[0];
		normal[1] = gjkOutput.m_normalOnBInWorld[1];
		normal[2] = gjkOutput.m_normalOnBInWorld[2];

		return gjkOutput.m_distance;
	}
	return -1.0f;	
}

