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

#ifndef D_BT_SOFTBODY_RIGIDBODY_COLLISION_CONFIGURATION
#define D_BT_SOFTBODY_RIGIDBODY_COLLISION_CONFIGURATION

#include "BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h"

class D_btVoronoiSimplexSolver;
class D_btGjkEpaPenetrationDepthSolver;


///D_btSoftBodyRigidBodyCollisionConfiguration add softbody interaction on top of D_btDefaultCollisionConfiguration
class	D_btSoftBodyRigidBodyCollisionConfiguration : public D_btDefaultCollisionConfiguration
{

	//default CreationFunctions, filling the m_doubleDispatch table
	D_btCollisionAlgorithmCreateFunc*	m_softSoftCreateFunc;
	D_btCollisionAlgorithmCreateFunc*	m_softRigidConvexCreateFunc;
	D_btCollisionAlgorithmCreateFunc*	m_swappedSoftRigidConvexCreateFunc;
	D_btCollisionAlgorithmCreateFunc*	m_softRigidConcaveCreateFunc;
	D_btCollisionAlgorithmCreateFunc*	m_swappedSoftRigidConcaveCreateFunc;

public:

	D_btSoftBodyRigidBodyCollisionConfiguration(const D_btDefaultCollisionConstructionInfo& constructionInfo = D_btDefaultCollisionConstructionInfo());

	virtual ~D_btSoftBodyRigidBodyCollisionConfiguration();

	///creation of soft-soft D_and soft-rigid, D_and otherwise fallback D_to base class implementation
	virtual D_btCollisionAlgorithmCreateFunc* getCollisionAlgorithmCreateFunc(int proxyType0,int proxyType1);

};

#endif //D_BT_SOFTBODY_RIGIDBODY_COLLISION_CONFIGURATION

