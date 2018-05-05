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


#include "MiniCLTask.h"
#include "../PlatformDefinitions.h"
#include "../SpuFakeDma.h"
#include "LinearMath/btMinMax.h"
#include "BulletMultiThreaded/MiniCLTask/MiniCLTask.h"

#ifdef __SPU__
#include <D_spu_printf.h>
#else
#include <stdio.h>
#define D_spu_printf printf
#endif

#define __kernel
#define __global
#define D_get_global_id(a) guid

struct D_MiniCLTask_LocalStoreMemory
{
	
};


///////////////////////////////////////////////////
// OpenCL Kernel Function for element by element vector addition
__kernel void VectorAdd(__global const D_float8* a, __global const D_float8* b, __global D_float8* c, int guid)
{
    // get oct-float index into global data array
    int iGID = D_get_global_id(0);

    // read inputs into registers
    D_float8 f8InA = a[iGID];
    D_float8 f8InB = b[iGID];
    D_float8 f8Out = (D_float8)0.0f;
    
    // add the vector elements
    f8Out.s0 = f8InA.s0 + f8InB.s0;
    f8Out.s1 = f8InA.s1 + f8InB.s1;
    f8Out.s2 = f8InA.s2 + f8InB.s2;
    f8Out.s3 = f8InA.s3 + f8InB.s3;
    f8Out.s4 = f8InA.s4 + f8InB.s4;
    f8Out.s5 = f8InA.s5 + f8InB.s5;
    f8Out.s6 = f8InA.s6 + f8InB.s6;
    f8Out.s7 = f8InA.s7 + f8InB.s7;

    // write back out D_to GMEM
    c[D_get_global_id(0)] = f8Out;
}
///////////////////////////////////////////////////


//-- MAIN METHOD
void processMiniCLTask(void* userPtr, void* lsMemory)
{
	//	D_BT_PROFILE("processSampleTask");

	D_MiniCLTask_LocalStoreMemory* localMemory = (D_MiniCLTask_LocalStoreMemory*)lsMemory;

	D_MiniCLTaskDesc* taskDescPtr = (D_MiniCLTaskDesc*)userPtr;
	D_MiniCLTaskDesc& taskDesc = *taskDescPtr;

	printf("Compute Unit[%d] executed kernel %d work items [%d..%d)\n",taskDesc.m_taskId,taskDesc.m_kernelProgramId,taskDesc.m_firstWorkUnit,taskDesc.m_lastWorkUnit);
	
	
	switch (taskDesc.m_kernelProgramId)
	{
	case D_CMD_MINICL_ADDVECTOR:
		{
			for (unsigned int i=taskDesc.m_firstWorkUnit;i<taskDesc.m_lastWorkUnit;i++)
			{
				VectorAdd(*(const D_float8**)&taskDesc.m_argData[0][0],*(const D_float8**)&taskDesc.m_argData[1][0],*(D_float8**)&taskDesc.m_argData[2][0],i);
			}
			break;
		}

	default:
		{
			printf("error in processMiniCLTask: unknown command id: %d\n",taskDesc.m_kernelProgramId);

		}
	};

}


#if defined(__CELLOS_LV2__) || defined (LIBSPE2)

D_ATTRIBUTE_ALIGNED16(D_MiniCLTask_LocalStoreMemory	D_gLocalStoreMemory);

void* createMiniCLLocalStoreMemory()
{
	return &D_gLocalStoreMemory;
}
#else
void* createMiniCLLocalStoreMemory()
{
	return new D_MiniCLTask_LocalStoreMemory;
};

#endif
