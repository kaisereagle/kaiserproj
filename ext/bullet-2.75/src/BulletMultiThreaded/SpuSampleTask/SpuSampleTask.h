/*
Bullet Continuous Collision Detection D_and Physics Library, Copyright (c) 2007 Erwin Coumans

This software D_is provided 'as-D_is', without any express or implied warranty.
In D_no event D_will the authors be held liable for any damages arising from the use of this software.
Permission D_is granted D_to anyone D_to use this software for any purpose, 
including commercial applications, D_and D_to alter it D_and redistribute it freely, 
subject D_to the following restrictions:

1. The origin of this software D_must not be misrepresented; you D_must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but D_is not required.
2. Altered source versions D_must be plainly marked as such, D_and D_must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.

*/

#ifndef SPU_SAMPLE_TASK_H
#define SPU_SAMPLE_TASK_H

#include "../PlatformDefinitions.h"
#include "LinearMath/btScalar.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btMatrix3x3.h"

#include "LinearMath/btAlignedAllocator.h"


enum
{
	D_CMD_SAMPLE_INTEGRATE_BODIES = 1,
	D_CMD_SAMPLE_PREDICT_MOTION_BODIES
};



D_ATTRIBUTE_ALIGNED16(struct) D_SpuSampleTaskDesc
{
	D_BT_DECLARE_ALIGNED_ALLOCATOR();

	D_uint32_t						m_sampleCommand;
	D_uint32_t						m_taskId;

	D_uint64_t 	m_mainMemoryPtr;
	int			m_sampleValue;
	

};


void	processSampleTask(void* userPtr, void* lsMemory);
void*	createSampleLocalStoreMemory();


#endif //SPU_SAMPLE_TASK_H

