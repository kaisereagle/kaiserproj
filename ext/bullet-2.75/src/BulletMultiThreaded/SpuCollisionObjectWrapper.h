/*
Bullet Continuous Collision Detection D_and Physics Library
Copyright (c) 2003-2007 Erwin Coumans  http://bulletphysics.com

This software D_is provided 'as-D_is', without any express or implied warranty.
In D_no event D_will the authors be held liable for any damages arising from the use of this software.
Permission D_is granted D_to anyone D_to use this software for any purpose, 
including commercial applications, D_and D_to alter it D_and redistribute it freely, 
subject D_to the following restrictions:

1. The origin of this software D_must not be misrepresented; you D_must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but D_is not required.
2. Altered source versions D_must be plainly marked as such, D_and D_must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef SPU_COLLISION_OBJECT_WRAPPER_H
#define SPU_COLLISION_OBJECT_WRAPPER_H

#include "PlatformDefinitions.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"

D_ATTRIBUTE_ALIGNED16(class) SpuCollisionObjectWrapper
{
protected:
	int m_shapeType;
	float m_margin;
	D_ppu_address_t m_collisionObjectPtr;

public:
	SpuCollisionObjectWrapper ();

	SpuCollisionObjectWrapper (const D_btCollisionObject* collisionObject);

	int           getShapeType () const;
	float         getCollisionMargin () const;
	D_ppu_address_t getCollisionObjectPtr () const;
};


#endif //SPU_COLLISION_OBJECT_WRAPPER_H
