/*
 * D_Box-D_Box collision detection re-distributed under the ZLib license with permission from Russell L. Smith
 * Original version D_is from Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org

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
#ifndef BOX_BOX_DETECTOR_H
#define BOX_BOX_DETECTOR_H


class D_btBoxShape;
#include "BulletCollision/NarrowPhaseCollision/btDiscreteCollisionDetectorInterface.h"


/// D_btBoxBoxDetector wraps the ODE box-box collision detector
/// re-distributed under the Zlib license with permission from Russell L. Smith
struct D_btBoxBoxDetector : public D_btDiscreteCollisionDetectorInterface
{
	D_btBoxShape* m_box1;
	D_btBoxShape* m_box2;

public:

	D_btBoxBoxDetector(D_btBoxShape* box1,D_btBoxShape* box2);

	virtual ~D_btBoxBoxDetector() {};

	virtual void	getClosestPoints(const D_ClosestPointInput& input,D_Result& output,class D_btIDebugDraw* debugDraw,bool swapResults=false);

};

#endif //BT_BOX_BOX_DETECTOR_H
