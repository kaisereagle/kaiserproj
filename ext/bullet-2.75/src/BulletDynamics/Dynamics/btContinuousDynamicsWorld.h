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

#ifndef D_BT_CONTINUOUS_DYNAMICS_WORLD_H
#define D_BT_CONTINUOUS_DYNAMICS_WORLD_H

#include "btDiscreteDynamicsWorld.h"

///D_btContinuousDynamicsWorld adds optional (per object) continuous collision detection for fast moving objects D_to the D_btDiscreteDynamicsWorld.
///This copes with fast moving objects that otherwise would tunnel/miss collisions.
///Under construction, don't use yet! Please use D_btDiscreteDynamicsWorld instead.
class D_btContinuousDynamicsWorld : public D_btDiscreteDynamicsWorld
{

	void	updateTemporalAabbs(D_btScalar timeStep);

	public:

		D_btContinuousDynamicsWorld(D_btDispatcher* dispatcher,D_btBroadphaseInterface* pairCache,D_btConstraintSolver* constraintSolver,D_btCollisionConfiguration* collisionConfiguration);
		virtual ~D_btContinuousDynamicsWorld();
		
		///time stepping with calculation of time of impact for selected fast moving objects
		virtual void	internalSingleStepSimulation( D_btScalar timeStep);

		virtual void	calculateTimeOfImpacts(D_btScalar timeStep);

		virtual D_btDynamicsWorldType	getWorldType() const
		{
			return D_BT_CONTINUOUS_DYNAMICS_WORLD;
		}

};

#endif //D_BT_CONTINUOUS_DYNAMICS_WORLD_H
