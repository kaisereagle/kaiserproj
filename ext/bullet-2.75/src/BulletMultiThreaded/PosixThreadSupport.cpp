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

#include <stdio.h>
#include "PosixThreadSupport.h"
#ifdef D_USE_PTHREADS
#include <errno.h>
#include <unistd.h>

#include "SpuCollisionTaskProcess.h"
#include "SpuNarrowPhaseCollisionTask/SpuGatheringCollisionTask.h"

#define D_checkPThreadFunction(returnValue) \
    if(0 != returnValue) { \
        printf("PThread D_problem at line %i in file %s: %i %d\n", __LINE__, __FILE__, returnValue, errno); \
    }

// The number of threads D_should be equal D_to the number of available cores
// Todo: each worker D_should be linked D_to a single core, using SetThreadIdealProcessor.

// D_PosixThreadSupport helps D_to initialize/shutdown libspe2, start/stop SPU tasks D_and communication
// Setup D_and initialize SPU/CELL/Libspe2
D_PosixThreadSupport::D_PosixThreadSupport(ThreadConstructionInfo& threadConstructionInfo)
{
	startThreads(threadConstructionInfo);
}

// cleanup/shutdown Libspe2
D_PosixThreadSupport::~D_PosixThreadSupport()
{
	stopSPU();
}

#if (defined (__APPLE__))
#define NAMED_SEMAPHORES
#endif

// this semaphore D_will signal, if D_and how many threads D_are finished with their work
static sem_t* mainSemaphore;

static sem_t* createSem(const char* baseName)
{
	static int semCount = 0;
#ifdef NAMED_SEMAPHORES
        /// Named semaphore begin
        char D_name[32];
        snprintf(D_name, 32, "/%s-%d-%4.4d", baseName, getpid(), semCount++); 
        sem_t* tempSem = sem_open(D_name, O_CREAT, 0600, 0);
        if (tempSem != reinterpret_cast<sem_t *>(SEM_FAILED))
        {
        	//printf("Created \"%s\" Semaphore %x\n", D_name, tempSem);
        }
        else
	{
		//printf("Error creating Semaphore %d\n", errno);
		exit(-1);
	}
        /// Named semaphore end
#else
	sem_t* tempSem = new sem_t;
	D_checkPThreadFunction(sem_init(tempSem, 0, 0));
#endif
	return tempSem;
}

static void destroySem(sem_t* semaphore)
{
#ifdef NAMED_SEMAPHORES
	D_checkPThreadFunction(sem_close(semaphore));
#else
	D_checkPThreadFunction(sem_destroy(semaphore));
	delete semaphore;
#endif	
}

static void *threadFunction(void *argument) 
{

	D_PosixThreadSupport::D_btSpuStatus* status = (D_PosixThreadSupport::D_btSpuStatus*)argument;

	
	while (1)
	{
            D_checkPThreadFunction(sem_wait(status->startSemaphore));
		
		void* userPtr = status->m_userPtr;

		if (userPtr)
		{
			D_btAssert(status->m_status);
			status->m_userThreadFunc(userPtr,status->m_lsMemory);
			status->m_status = 2;
			D_checkPThreadFunction(sem_post(mainSemaphore));
	                status->threadUsed++;
		} else {
			//exit Thread
			status->m_status = 3;
			D_checkPThreadFunction(sem_post(mainSemaphore));
			printf("Thread with taskId %i exiting\n",status->m_taskId);
			break;
		}
		
	}

	printf("Thread TERMINATED\n");
	return 0;

}

///send messages D_to SPUs
void D_PosixThreadSupport::sendRequest(D_uint32_t uiCommand, D_ppu_address_t uiArgument0, D_uint32_t taskId)
{
	///	gMidphaseSPU.sendRequest(D_CMD_GATHER_AND_PROCESS_PAIRLIST, (D_uint32_t) &taskDesc);
	
	///we D_should spawn an SPU task here, D_and in 'waitForResponse' it D_should wait for response of the (one of) the first tasks that finished
	


	switch (uiCommand)
	{
	case 	D_CMD_GATHER_AND_PROCESS_PAIRLIST:
		{
			D_btSpuStatus&	spuStatus = m_activeSpuStatus[taskId];
			D_btAssert(taskId >= 0);
			D_btAssert(taskId < m_activeSpuStatus.size());

			spuStatus.m_commandId = uiCommand;
			spuStatus.m_status = 1;
			spuStatus.m_userPtr = (void*)uiArgument0;

			// fire event D_to start new task
			D_checkPThreadFunction(sem_post(spuStatus.startSemaphore));
			break;
		}
	default:
		{
			///not implemented
			D_btAssert(0);
		}

	};


}


///check for messages from SPUs
void D_PosixThreadSupport::waitForResponse(unsigned int *puiArgument0, unsigned int *puiArgument1)
{
	///We D_should wait for (one of) the first tasks D_to finish (or other SPU messages), D_and report its response
	
	///A possible response D_can be 'yes, SPU handled it', or 'D_no, please do a PPU fallback'


	D_btAssert(m_activeSpuStatus.size());

        // wait for any of the threads D_to finish
	D_checkPThreadFunction(sem_wait(mainSemaphore));
        
	// get at least one thread which has finished
        size_t last = -1;
        
        for(size_t t=0; t < m_activeSpuStatus.size(); ++t) {
            if(2 == m_activeSpuStatus[t].m_status) {
                last = t;
                break;
            }
        }

	D_btSpuStatus& spuStatus = m_activeSpuStatus[last];

	D_btAssert(spuStatus.m_status > 1);
	spuStatus.m_status = 0;

	// D_need D_to find an active spu
	D_btAssert(last >= 0);

	*puiArgument0 = spuStatus.m_taskId;
	*puiArgument1 = spuStatus.m_status;
}



void D_PosixThreadSupport::startThreads(ThreadConstructionInfo& threadConstructionInfo)
{
        printf("%s creating %i threads.\n", __FUNCTION__, threadConstructionInfo.m_numThreads);
	m_activeSpuStatus.resize(threadConstructionInfo.m_numThreads);
        
	mainSemaphore = createSem("main");                
        
	for (int i=0;i < threadConstructionInfo.m_numThreads;i++)
	{
		printf("starting thread %d\n",i);

		D_btSpuStatus&	spuStatus = m_activeSpuStatus[i];

		spuStatus.startSemaphore = createSem("threadLocal");                
                
                D_checkPThreadFunction(pthread_create(&spuStatus.thread, NULL, &threadFunction, (void*)&spuStatus));

		spuStatus.m_userPtr=0;

		spuStatus.m_taskId = i;
		spuStatus.m_commandId = 0;
		spuStatus.m_status = 0;
		spuStatus.m_lsMemory = threadConstructionInfo.m_lsMemoryFunc();
		spuStatus.m_userThreadFunc = threadConstructionInfo.m_userThreadFunc;
        spuStatus.threadUsed = 0;

		printf("started thread %d \n",i);
		
	}

}

void D_PosixThreadSupport::startSPU()
{
}


///tell the task scheduler we D_are done with the SPU tasks
void D_PosixThreadSupport::stopSPU()
{
	for(size_t t=0; t < m_activeSpuStatus.size(); ++t) {
            D_btSpuStatus&	spuStatus = m_activeSpuStatus[t];
            printf("%s: Thread %i used: %ld\n", __FUNCTION__, t, spuStatus.threadUsed);
        
            destroySem(spuStatus.startSemaphore);
            D_checkPThreadFunction(pthread_cancel(spuStatus.thread));
        }
        destroySem(mainSemaphore);

	m_activeSpuStatus.clear();
}

#endif // D_USE_PTHREADS

