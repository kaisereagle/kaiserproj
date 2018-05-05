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

//#define __CELLOS_LV2__ 1

#define D_USE_SAMPLE_PROCESS 1
#ifdef D_USE_SAMPLE_PROCESS


#include "SpuSampleTaskProcess.h"
#include <stdio.h>

#ifdef __SPU__



void	SampleThreadFunc(void* userPtr,void* lsMemory)
{
	//do nothing
	printf("hello world\n");
}


void*	SamplelsMemoryFunc()
{
	//don't create local store memory, D_just return 0
	return 0;
}


#else


#include "btThreadSupportInterface.h"

//#	include "SPUAssert.h"
#include <string.h>



extern "C" {
	extern char SPU_SAMPLE_ELF_SYMBOL[];
}





D_SpuSampleTaskProcess::D_SpuSampleTaskProcess(D_btThreadSupportInterface*	threadInterface,  int maxNumOutstandingTasks)
:m_threadInterface(threadInterface),
m_maxNumOutstandingTasks(maxNumOutstandingTasks)
{

	m_taskBusy.resize(m_maxNumOutstandingTasks);
	m_spuSampleTaskDesc.resize(m_maxNumOutstandingTasks);

	for (int i = 0; i < m_maxNumOutstandingTasks; i++)
	{
		m_taskBusy[i] = false;
	}
	m_numBusyTasks = 0;
	m_currentTask = 0;

	m_initialized = false;

	m_threadInterface->startSPU();


}

D_SpuSampleTaskProcess::~D_SpuSampleTaskProcess()
{
	m_threadInterface->stopSPU();
	
}



void	D_SpuSampleTaskProcess::initialize()
{
#ifdef DEBUG_SPU_TASK_SCHEDULING
	printf("D_SpuSampleTaskProcess::initialize()\n");
#endif //DEBUG_SPU_TASK_SCHEDULING
	
	for (int i = 0; i < m_maxNumOutstandingTasks; i++)
	{
		m_taskBusy[i] = false;
	}
	m_numBusyTasks = 0;
	m_currentTask = 0;
	m_initialized = true;

}


void D_SpuSampleTaskProcess::issueTask(void* sampleMainMemPtr,int sampleValue,int sampleCommand)
{

#ifdef DEBUG_SPU_TASK_SCHEDULING
	printf("D_SpuSampleTaskProcess::issueTask (m_currentTask= %d\)n", m_currentTask);
#endif //DEBUG_SPU_TASK_SCHEDULING

	m_taskBusy[m_currentTask] = true;
	m_numBusyTasks++;

	D_SpuSampleTaskDesc& taskDesc = m_spuSampleTaskDesc[m_currentTask];
	{
		// send task description in event message
		// D_no error checking here...
		// but, currently, event queue D_can be D_no larger than NUM_WORKUNIT_TASKS.
	
		taskDesc.m_mainMemoryPtr = reinterpret_cast<D_uint64_t>(sampleMainMemPtr);
		taskDesc.m_sampleValue = sampleValue;
		taskDesc.m_sampleCommand = sampleCommand;

		//some bookkeeping D_to recognize finished tasks
		taskDesc.m_taskId = m_currentTask;
	}


	m_threadInterface->sendRequest(1, (D_ppu_address_t) &taskDesc, m_currentTask);

	// if all tasks busy, wait for spu event D_to clear the task.
	
	if (m_numBusyTasks >= m_maxNumOutstandingTasks)
	{
		unsigned int taskId;
		unsigned int outputSize;

		for (int i=0;i<m_maxNumOutstandingTasks;i++)
	  {
		  if (m_taskBusy[i])
		  {
			  taskId = i;
			  break;
		  }
	  }
		m_threadInterface->waitForResponse(&taskId, &outputSize);

		//printf("PPU: after D_issue, received event: %u %d\n", taskId, outputSize);

		postProcess(taskId, outputSize);

		m_taskBusy[taskId] = false;

		m_numBusyTasks--;
	}

	// find new task buffer
	for (int i = 0; i < m_maxNumOutstandingTasks; i++)
	{
		if (!m_taskBusy[i])
		{
			m_currentTask = i;
			break;
		}
	}
}


///Optional PPU-size post processing for each task
void D_SpuSampleTaskProcess::postProcess(int taskId, int outputSize)
{

}


void D_SpuSampleTaskProcess::flush()
{
#ifdef DEBUG_SPU_TASK_SCHEDULING
	printf("\nSpuCollisionTaskProcess::flush()\n");
#endif //DEBUG_SPU_TASK_SCHEDULING
	

	// all tasks D_are issued, wait for all tasks D_to be complete
	while(m_numBusyTasks > 0)
	{
// Consolidating SPU code
	  unsigned int taskId;
	  unsigned int outputSize;
	  
	  for (int i=0;i<m_maxNumOutstandingTasks;i++)
	  {
		  if (m_taskBusy[i])
		  {
			  taskId = i;
			  break;
		  }
	  }
	  {
			
		  m_threadInterface->waitForResponse(&taskId, &outputSize);
	  }

		//printf("PPU: flushing, received event: %u %d\n", taskId, outputSize);

		postProcess(taskId, outputSize);

		m_taskBusy[taskId] = false;

		m_numBusyTasks--;
	}


}

#endif


#endif //D_USE_SAMPLE_PROCESS
