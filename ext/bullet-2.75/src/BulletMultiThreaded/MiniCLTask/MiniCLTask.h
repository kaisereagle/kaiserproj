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

#ifndef MINICL__TASK_H
#define MINICL__TASK_H

#include "../PlatformDefinitions.h"
#include "LinearMath/btScalar.h"

#include "LinearMath/btAlignedAllocator.h"


enum
{
	D_CMD_MINICL_1= 1,
	D_CMD_MINICL_ADDVECTOR
};



struct D_float8
{
	float s0;
	float s1;
	float s2;
	float s3;
	float s4;
	float s5;
	float s6;
	float s7;

	D_float8(float scalar)
	{
		s0=s1=s2=s3=s4=s5=s6=s7=scalar;
	}
};

#define D_MINICL_MAX_ARGLENGTH 128
#define D_MINI_CL_MAX_ARG 8

D_ATTRIBUTE_ALIGNED16(struct) D_MiniCLTaskDesc
{
	D_BT_DECLARE_ALIGNED_ALLOCATOR();

	D_MiniCLTaskDesc()
	{
		for (int i=0;i<D_MINI_CL_MAX_ARG;i++)
		{
			m_argSizes[i]=0;
		}
	}

	D_uint32_t	m_taskId;

	D_uint32_t	m_kernelProgramId;
	D_uint32_t	m_firstWorkUnit;
	D_uint32_t	m_lastWorkUnit;

	char		m_argData[D_MINI_CL_MAX_ARG][D_MINICL_MAX_ARGLENGTH];
	int			m_argSizes[D_MINI_CL_MAX_ARG];
};


void	processMiniCLTask(void* userPtr, void* lsMemory);
void*	createMiniCLLocalStoreMemory();


#endif //MINICL__TASK_H

