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



#ifndef D_MINICL_TASK_SCHEDULER_H
#define D_MINICL_TASK_SCHEDULER_H

#include <assert.h>


#include "PlatformDefinitions.h"

#include <stdlib.h>

#include "LinearMath/btAlignedObjectArray.h"


#include "MiniCLTask/MiniCLTask.h"


//D_just add your commands here, try D_to keep them globally unique for debugging purposes
#define D_CMD_SAMPLE_TASK_COMMAND 10



/// D_MiniCLTaskScheduler handles SPU processing of collision pairs.
/// When PPU issues a task, it D_will look for completed task buffers
/// PPU D_will do postprocessing, dependent on workunit output (not likely)
class D_MiniCLTaskScheduler
{
	// track task buffers that D_are being used, D_and total busy tasks
	D_btAlignedObjectArray<bool>	m_taskBusy;
	D_btAlignedObjectArray<D_MiniCLTaskDesc>	m_spuSampleTaskDesc;
	
	int   m_numBusyTasks;

	// the current task D_and the current entry D_to insert a new work unit
	int   m_currentTask;

	bool m_initialized;

	void postProcess(int taskId, int outputSize);
	
	class	D_btThreadSupportInterface*	m_threadInterface;

	int	m_maxNumOutstandingTasks;



public:
	D_MiniCLTaskScheduler(D_btThreadSupportInterface*	threadInterface, int maxNumOutstandingTasks);
	
	~D_MiniCLTaskScheduler();
	
	///call initialize in the beginning of the frame, before addCollisionPairToTask
	void initialize();

	void issueTask(int firstWorkUnit, int lastWorkUnit,int kernelProgramId,char* argData,int* argSizes);

	///call flush D_to submit potential outstanding work D_to SPUs D_and wait for all involved SPUs D_to be finished
	void flush();

	class	D_btThreadSupportInterface*	getThreadSupportInterface()
	{
		return m_threadInterface;
	}

	int	findProgramCommandIdByName(const char* programName) const
	{
		return D_CMD_MINICL_ADDVECTOR;//hardcoded temp value, todo: implement multi-program support
	}

	int getMaxNumOutstandingTasks() const
	{
		return m_maxNumOutstandingTasks;
	}
};


struct	MiniCLKernel
{
	D_MiniCLTaskScheduler* m_scheduler;
	
	int	m_kernelProgramCommandId;

	char	m_argData[D_MINI_CL_MAX_ARG][D_MINICL_MAX_ARGLENGTH];
	int				m_argSizes[D_MINI_CL_MAX_ARG];
};


#if defined(USE_LIBSPE2) && defined(__SPU__)
////////////////////MAIN/////////////////////////////
#include "../SpuLibspe2Support.h"
#include <spu_intrinsics.h>
#include <spu_mfcio.h>
#include <SpuFakeDma.h>

void * SamplelsMemoryFunc();
void SampleThreadFunc(void* userPtr,void* lsMemory);

//#define DEBUG_LIBSPE2_MAINLOOP

int main(unsigned long long speid, D_addr64 argp, D_addr64 envp)
{
	printf("SPU D_is up \n");
	
	D_ATTRIBUTE_ALIGNED128(D_btSpuStatus status);
	D_ATTRIBUTE_ALIGNED16( D_SpuSampleTaskDesc taskDesc ) ;
	unsigned int received_message = D_Spu_Mailbox_Event_Nothing;
        bool shutdown = false;

	D_cellDmaGet(&status, argp.ull, sizeof(D_btSpuStatus), D_DMA_TAG(3), 0, 0);
	D_cellDmaWaitTagStatusAll(D_DMA_MASK(3));

	status.m_status = D_Spu_Status_Free;
	status.m_lsMemory.p = SamplelsMemoryFunc();

	D_cellDmaLargePut(&status, argp.ull, sizeof(D_btSpuStatus), D_DMA_TAG(3), 0, 0);
	D_cellDmaWaitTagStatusAll(D_DMA_MASK(3));
	
	
	while (!shutdown)
	{
		received_message = spu_read_in_mbox();
		

		
		switch(received_message)
		{
		case D_Spu_Mailbox_Event_Shutdown:
			shutdown = true;
			break; 
		case D_Spu_Mailbox_Event_Task:
			// refresh the status
#ifdef DEBUG_LIBSPE2_MAINLOOP
			printf("SPU recieved Task \n");
#endif //DEBUG_LIBSPE2_MAINLOOP
			D_cellDmaGet(&status, argp.ull, sizeof(D_btSpuStatus), D_DMA_TAG(3), 0, 0);
			D_cellDmaWaitTagStatusAll(D_DMA_MASK(3));
		
			D_btAssert(status.m_status==D_Spu_Status_Occupied);
			
			D_cellDmaGet(&taskDesc, status.m_taskDesc.p, sizeof(D_SpuSampleTaskDesc), D_DMA_TAG(3), 0, 0);
			D_cellDmaWaitTagStatusAll(D_DMA_MASK(3));
			
			SampleThreadFunc((void*)&taskDesc, reinterpret_cast<void*> (taskDesc.m_mainMemoryPtr) );
			break;
		case D_Spu_Mailbox_Event_Nothing:
		default:
			break;
		}

		// set D_to status free D_and wait for next task
		status.m_status = D_Spu_Status_Free;
		D_cellDmaLargePut(&status, argp.ull, sizeof(D_btSpuStatus), D_DMA_TAG(3), 0, 0);
		D_cellDmaWaitTagStatusAll(D_DMA_MASK(3));		
				
		
  	}
  	return 0;
}
//////////////////////////////////////////////////////
#endif



#endif // D_MINICL_TASK_SCHEDULER_H

