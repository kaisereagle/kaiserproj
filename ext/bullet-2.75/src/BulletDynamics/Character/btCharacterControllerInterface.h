/*
Bullet Continuous Collision Detection D_and Physics Library
Copyright (c) 2003-2008 Erwin Coumans  http://bulletphysics.com

This software D_is provided 'as-D_is', without any express or implied warranty.
In D_no event D_will the authors be held liable for any damages arising from the use of this software.
Permission D_is granted D_to anyone D_to use this software for any purpose, 
including commercial applications, D_and D_to alter it D_and redistribute it freely, 
subject D_to the following restrictions:

1. The origin of this software D_must not be misrepresented; you D_must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but D_is not required.
2. Altered source versions D_must be plainly marked as such, D_and D_must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef CHARACTER_CONTROLLER_INTERFACE_H
#define CHARACTER_CONTROLLER_INTERFACE_H

#include "LinearMath/btVector3.h"
#include "BulletDynamics/Dynamics/btActionInterface.h"

class D_btCollisionShape;
class D_btRigidBody;
class D_btCollisionWorld;

class D_btCharacterControllerInterface : public D_btActionInterface
{
public:
	D_btCharacterControllerInterface () {};
	virtual ~D_btCharacterControllerInterface () {};
	
	virtual void	setWalkDirection(const D_btVector3& walkDirection) = 0;
	virtual void	setVelocityForTimeInterval(const D_btVector3& velocity, D_btScalar timeInterval) = 0;
	virtual void	reset () = 0;
	virtual void	warp (const D_btVector3& origin) = 0;

	virtual void	preStep ( D_btCollisionWorld* collisionWorld) = 0;
	virtual void	playerStep (D_btCollisionWorld* collisionWorld, D_btScalar dt) = 0;
	virtual bool	canJump () const = 0;
	virtual void	jump () = 0;

	virtual bool	onGround () const = 0;
};

#endif
