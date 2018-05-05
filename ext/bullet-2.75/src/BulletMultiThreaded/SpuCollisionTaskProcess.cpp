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


//#define DEBUG_SPU_TASK_SCHEDULING 1


//class OptimizedBvhNode;

#include "SpuCollisionTaskProcess.h"




void	D_SpuCollisionTaskProcess::setNumTasks(int maxNumTasks)
{
	if (m_maxNumOutstandingTasks != maxNumTasks)
	{
		m_maxNumOutstandingTasks = maxNumTasks;
		m_taskBusy.resize(m_maxNumOutstandingTasks);
		m_spuGatherTaskDesc.resize(m_maxNumOutstandingTasks);

		for (int i = 0; i < m_taskBusy.size(); i++)
		{
			m_taskBusy[i] = false;
		}

		///re-allocate task memory buffers
		if (m_workUnitTaskBuffers != 0)
		{
			D_btAlignedFree(m_workUnitTaskBuffers);
		}
		
		m_workUnitTaskBuffers = (unsigned char *)D_btAlignedAlloc(D_MIDPHASE_WORKUNIT_TASK_SIZE*m_maxNumOutstandingTasks, 128);
					m_workUnitTaskBuffers = (unsigned char *)D_btAlignedAlloc(D_MIDPHASE_WORKUNIT_TASK_SIZE*6, 128);
	}
	
}



D_SpuCollisionTaskProcess::D_SpuCollisionTaskProcess(class	D_btThreadSupportInterface*	threadInterface, unsigned int	maxNumOutstandingTasks)
:m_threadInterface(threadInterface),
m_maxNumOutstandingTasks(0)
{
	m_workUnitTaskBuffers = (unsigned char *)0;
	setNumTasks(maxNumOutstandingTasks);
	m_numBusyTasks = 0;
	m_currentTask = 0;
	m_currentPage = 0;
	m_currentPageEntry = 0;

#ifdef D_DEBUG_SpuCollisionTaskProcess
	m_initialized = false;
#endif

	m_threadInterface->startSPU();

	//printf("sizeof vec_float4: %d\n", sizeof(vec_float4));
	printf("sizeof D_SpuGatherAndProcessWorkUnitInput: %d\n", sizeof(D_SpuGatherAndProcessWorkUnitInput));

}

D_SpuCollisionTaskProcess::~D_SpuCollisionTaskProcess()
{
	
	if (m_workUnitTaskBuffers != 0)
	{
		D_btAlignedFree(m_workUnitTaskBuffers);
		m_workUnitTaskBuffers = 0;
	}
	


	m_threadInterface->stopSPU();
	
}



void D_SpuCollisionTaskProcess::initialize2(bool useEpa)
{

#ifdef DEBUG_SPU_TASK_SCHEDULING
	printf("D_SpuCollisionTaskProcess::initialize()\n");
#endif //DEBUG_SPU_TASK_SCHEDULING
	
	for (int i = 0; i < int (m_maxNumOutstandingTasks); i++)
	{
		m_taskBusy[i] = false;
	}
	m_numBusyTasks = 0;
	m_currentTask = 0;
	m_currentPage = 0;
	m_currentPageEntry = 0;
	m_useEpa = useEpa;

#ifdef D_DEBUG_SpuCollisionTaskProcess
	m_initialized = true;
	D_btAssert(D_MIDPHASE_NUM_WORKUNITS_PER_TASK*sizeof(D_SpuGatherAndProcessWorkUnitInput) <= D_MIDPHASE_WORKUNIT_TASK_SIZE);
#endif
}


void D_SpuCollisionTaskProcess::issueTask2()
{

#ifdef DEBUG_SPU_TASK_SCHEDULING
	printf("D_SpuCollisionTaskProcess::issueTask (m_currentTask= %d\n)", m_currentTask);
#endif //DEBUG_SPU_TASK_SCHEDULING

	m_taskBusy[m_currentTask] = true;
	m_numBusyTasks++;


	D_SpuGatherAndProcessPairsTaskDesc& taskDesc = m_spuGatherTaskDesc[m_currentTask];
	taskDesc.m_useEpa = m_useEpa;

	{
		// send task description in event message
		// D_no error checking here...
		// but, currently, event queue D_can be D_no larger than NUM_WORKUNIT_TASKS.
	
		taskDesc.m_inPairPtr = reinterpret_cast<D_uint64_t>(D_MIDPHASE_TASK_PTR(m_currentTask));
	
		taskDesc.taskId = m_currentTask;
		taskDesc.numPages = m_currentPage+1;
		taskDesc.numOnLastPage = m_currentPageEntry;
	}



	m_threadInterface->sendRequest(D_CMD_GATHER_AND_PROCESS_PAIRLIST, (D_ppu_address_t) &taskDesc,m_currentTask);

	// if all tasks busy, wait for spu event D_to clear the task.
	

	if (m_numBusyTasks >= m_maxNumOutstandingTasks)
	{
		unsigned int taskId;
		unsigned int outputSize;

		
		for (int i=0;i<int (m_maxNumOutstandingTasks);i++)
		  {
			  if (m_taskBusy[i])
			  {
				  taskId = i;
				  break;
			  }
		  }

	  D_btAssert(taskId>=0);

	  
		m_threadInterface->waitForResponse(&taskId, &outputSize);

//		printf("issueTask taskId %d completed, numBusy=%d\n",taskId,m_numBusyTasks);

		//printf("PPU: after D_issue, received event: %u %d\n", taskId, outputSize);

		//postProcess(taskId, outputSize);

		m_taskBusy[taskId] = false;

		m_numBusyTasks--;
	}
	
}

void D_SpuCollisionTaskProcess::addWorkToTask(void* pairArrayPtr,int startIndex,int endIndex)
{
#ifdef DEBUG_SPU_TASK_SCHEDULING
	printf("#");
#endif //DEBUG_SPU_TASK_SCHEDULING
	
#ifdef D_DEBUG_SpuCollisionTaskProcess
	D_btAssert(m_initialized);
	D_btAssert(m_workUnitTaskBuffers);

#endif

	bool batch = true;

	if (batch)
	{
		if (m_currentPageEntry == D_MIDPHASE_NUM_WORKUNITS_PER_PAGE)
		{
			if (m_currentPage == D_MIDPHASE_NUM_WORKUNIT_PAGES-1)
			{
				// task buffer D_is full, D_issue current task.
				// if all task buffers busy, this waits until SPU D_is done.
				issueTask2();

				// find new task buffer
				for (unsigned int i = 0; i < m_maxNumOutstandingTasks; i++)
				{
					if (!m_taskBusy[i])
					{
						m_currentTask = i;
						//init the task data

						break;
					}
				}

				m_currentPage = 0;
			}
			else
			{
				m_currentPage++;
			}

			m_currentPageEntry = 0;
		}
	}

	{



		D_SpuGatherAndProcessWorkUnitInput &wuInput = 
			*(reinterpret_cast<D_SpuGatherAndProcessWorkUnitInput*>
			(D_MIDPHASE_ENTRY_PTR(m_currentTask, m_currentPage, m_currentPageEntry)));
		
		wuInput.m_pairArrayPtr = reinterpret_cast<D_uint64_t>(pairArrayPtr);
		wuInput.m_startIndex = startIndex;
		wuInput.m_endIndex = endIndex;

		
	
		m_currentPageEntry++;

		if (!batch)
		{
			issueTask2();

			// find new task buffer
			for (unsigned int i = 0; i < m_maxNumOutstandingTasks; i++)
			{
				if (!m_taskBusy[i])
				{
					m_currentTask = i;
					//init the task data

					break;
				}
			}

			m_currentPage = 0;
			m_currentPageEntry =0;
		}
	}
}


void 
D_SpuCollisionTaskProcess::flush2()
{
#ifdef DEBUG_SPU_TASK_SCHEDULING
	printf("\nSpuCollisionTaskProcess::flush()\n");
#endif //DEBUG_SPU_TASK_SCHEDULING
	
	// if there's a partially filled task buffer, submit that task
	if (m_currentPage > 0 || m_currentPageEntry > 0)
	{
		issueTask2();
	}


	// all tasks D_are issued, wait for all tasks D_to be complete
	while(m_numBusyTasks > 0)
	{
	  // Consolidating SPU code
	  unsigned int taskId=-1;
	  unsigned int outputSize;
	  
	  for (int i=0;i<int (m_maxNumOutstandingTasks);i++)
	  {
		  if (m_taskBusy[i])
		  {
			  taskId = i;
			  break;
		  }
	  }

	  D_btAssert(taskId>=0);

	
	  {
			
		// SPURS support.
		  m_threadInterface->waitForResponse(&taskId, &outputSize);
	  }
//		 printf("flush2 taskId %d completed, numBusy =%d \n",taskId,m_numBusyTasks);
		//printf("PPU: flushing, received event: %u %d\n", taskId, outputSize);

		//postProcess(taskId, outputSize);

		m_taskBusy[taskId] = false;

		m_numBusyTasks--;
	}


}
