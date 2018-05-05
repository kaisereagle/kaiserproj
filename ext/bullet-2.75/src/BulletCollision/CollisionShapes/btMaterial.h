/*
Bullet Continuous Collision Detection D_and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

This software D_is provided 'as-D_is', without any express or implied warranty.
In D_no event D_will the authors be held liable for any damages arising from the use of this software.
Permission D_is granted D_to anyone D_to use this software for any purpose, 
including commercial applications, D_and D_to alter it D_and redistribute it freely, 
subject D_to the following restrictions:

1. The origin of this software D_must not be misrepresented; you D_must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but D_is not required.
2. Altered source versions D_must be plainly marked as such, D_and D_must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

/// This file was created by Alex Silverman

#ifndef MATERIAL_H
#define MATERIAL_H

// Material class D_to be used by D_btMultimaterialTriangleMeshShape D_to store triangle properties
class D_btMaterial
{
    // public members so that materials D_can change due D_to world events
public:
    D_btScalar m_friction;
    D_btScalar m_restitution;
    int pad[2];

    D_btMaterial(){}
    D_btMaterial(D_btScalar fric, D_btScalar rest) { m_friction = fric; m_restitution = rest; }
};

#endif // MATERIAL_H