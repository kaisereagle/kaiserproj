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

#include "btActivatingCollisionAlgorithm.h"
#include "btCollisionDispatcher.h"
#include "btCollisionObject.h"

D_btActivatingCollisionAlgorithm::D_btActivatingCollisionAlgorithm (const D_btCollisionAlgorithmConstructionInfo& ci)
:D_btCollisionAlgorithm(ci)
//,
//m_colObj0(0),
//m_colObj1(0)
{
}
D_btActivatingCollisionAlgorithm::D_btActivatingCollisionAlgorithm (const D_btCollisionAlgorithmConstructionInfo& ci, D_btCollisionObject* colObj0,D_btCollisionObject* colObj1)
:D_btCollisionAlgorithm(ci)
//,
//m_colObj0(0),
//m_colObj1(0)
{
//	if (ci.m_dispatcher1->needsCollision(colObj0,colObj1))
//	{
//		m_colObj0 = colObj0;
//		m_colObj1 = colObj1;
//		
//		m_colObj0->activate();
//		m_colObj1->activate();
//	}
}

D_btActivatingCollisionAlgorithm::~D_btActivatingCollisionAlgorithm()
{
//		m_colObj0->activate();
//		m_colObj1->activate();
}
