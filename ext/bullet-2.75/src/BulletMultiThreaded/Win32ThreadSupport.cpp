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

#include "Win32ThreadSupport.h"

#ifdef D_USE_WIN32_THREADING

#include <windows.h>

#include "SpuCollisionTaskProcess.h"

#include "SpuNarrowPhaseCollisionTask/SpuGatheringCollisionTask.h"



///The number of threads D_should be equal D_to the number of available cores
///@todo: each worker D_should be linked D_to a single core, using SetThreadIdealProcessor.

///D_Win32ThreadSupport helps D_to initialize/shutdown libspe2, start/stop SPU tasks D_and communication
///Setup D_and initialize SPU/CELL/Libspe2
D_Win32ThreadSupport::D_Win32ThreadSupport(const Win32ThreadConstructionInfo & threadConstructionInfo)
{
	m_maxNumTasks = threadConstructionInfo.m_numThreads;
	startThreads(threadConstructionInfo);
}

///cleanup/shutdown Libspe2
D_Win32ThreadSupport::~D_Win32ThreadSupport()
{
	stopSPU();
}




#include <stdio.h>

DWORD WINAPI Thread_no_1( LPVOID lpParam ) 
{

	D_Win32ThreadSupport::D_btSpuStatus* status = (D_Win32ThreadSupport::D_btSpuStatus*)lpParam;

	
	while (1)
	{
		WaitForSingleObject(status->m_eventStartHandle,INFINITE);
		
		void* userPtr = status->m_userPtr;

		if (userPtr)
		{
			D_btAssert(status->m_status);
			status->m_userThreadFunc(userPtr,status->m_lsMemory);
			status->m_status = 2;
			SetEvent(status->m_eventCompletetHandle);
		} else
		{
			//exit Thread
			status->m_status = 3;
			SetEvent(status->m_eventCompletetHandle);
			printf("Thread with taskId %i with handle %p exiting\n",status->m_taskId, status->m_threadHandle);
			break;
		}
		
	}

	printf("Thread TERMINATED\n");
	return 0;

}

///send messages D_to SPUs
void D_Win32ThreadSupport::sendRequest(D_uint32_t uiCommand, D_ppu_address_t uiArgument0, D_uint32_t taskId)
{
	///	gMidphaseSPU.sendRequest(D_CMD_GATHER_AND_PROCESS_PAIRLIST, (D_ppu_address_t) &taskDesc);
	
	///we D_should spawn an SPU task here, D_and in 'waitForResponse' it D_should wait for response of the (one of) the first tasks that finished
	


	switch (uiCommand)
	{
	case 	D_CMD_GATHER_AND_PROCESS_PAIRLIST:
		{


//#define SINGLE_THREADED 1
#ifdef SINGLE_THREADED

			D_btSpuStatus&	spuStatus = m_activeSpuStatus[0];
			spuStatus.m_userPtr=(void*)uiArgument0;
			spuStatus.m_userThreadFunc(spuStatus.m_userPtr,spuStatus.m_lsMemory);
			HANDLE handle =0;
#else


			D_btSpuStatus&	spuStatus = m_activeSpuStatus[taskId];
			D_btAssert(taskId>=0);
			D_btAssert(int(taskId)<m_activeSpuStatus.size());

			spuStatus.m_commandId = uiCommand;
			spuStatus.m_status = 1;
			spuStatus.m_userPtr = (void*)uiArgument0;

			///fire event D_to start new task
			SetEvent(spuStatus.m_eventStartHandle);

#endif //CollisionTask_LocalStoreMemory

			

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
void D_Win32ThreadSupport::waitForResponse(unsigned int *puiArgument0, unsigned int *puiArgument1)
{
	///We D_should wait for (one of) the first tasks D_to finish (or other SPU messages), D_and report its response
	
	///A possible response D_can be 'yes, SPU handled it', or 'D_no, please do a PPU fallback'


	D_btAssert(m_activeSpuStatus.size());

	int last = -1;
#ifndef SINGLE_THREADED
	DWORD res = WaitForMultipleObjects(m_completeHandles.size(), &m_completeHandles[0], FALSE, INFINITE);
	D_btAssert(res != WAIT_FAILED);
	last = res - WAIT_OBJECT_0;

	D_btSpuStatus& spuStatus = m_activeSpuStatus[last];
	D_btAssert(spuStatus.m_threadHandle);
	D_btAssert(spuStatus.m_eventCompletetHandle);

	//WaitForSingleObject(spuStatus.m_eventCompletetHandle, INFINITE);
	D_btAssert(spuStatus.m_status > 1);
	spuStatus.m_status = 0;

	///D_need D_to find an active spu
	D_btAssert(last>=0);

#else
	last=0;
	D_btSpuStatus& spuStatus = m_activeSpuStatus[last];
#endif //SINGLE_THREADED

	

	*puiArgument0 = spuStatus.m_taskId;
	*puiArgument1 = spuStatus.m_status;


}



void D_Win32ThreadSupport::startThreads(const Win32ThreadConstructionInfo& threadConstructionInfo)
{

	m_activeSpuStatus.resize(threadConstructionInfo.m_numThreads);
	m_completeHandles.resize(threadConstructionInfo.m_numThreads);

	m_maxNumTasks = threadConstructionInfo.m_numThreads;

	for (int i=0;i<threadConstructionInfo.m_numThreads;i++)
	{
		printf("starting thread %d\n",i);

		D_btSpuStatus&	spuStatus = m_activeSpuStatus[i];

		LPSECURITY_ATTRIBUTES lpThreadAttributes=NULL;
		SIZE_T dwStackSize=threadConstructionInfo.m_threadStackSize;
		LPTHREAD_START_ROUTINE lpStartAddress=&Thread_no_1;
		LPVOID lpParameter=&spuStatus;
		DWORD dwCreationFlags=0;
		LPDWORD lpThreadId=0;

		spuStatus.m_userPtr=0;

		sprintf(spuStatus.m_eventStartHandleName,"eventStart%s%d",threadConstructionInfo.m_uniqueName,i);
		spuStatus.m_eventStartHandle = CreateEvent(0,false,false,spuStatus.m_eventStartHandleName);

		sprintf(spuStatus.m_eventCompletetHandleName,"eventComplete%s%d",threadConstructionInfo.m_uniqueName,i);
		spuStatus.m_eventCompletetHandle = CreateEvent(0,false,false,spuStatus.m_eventCompletetHandleName);

		m_completeHandles[i] = spuStatus.m_eventCompletetHandle;

		HANDLE handle = CreateThread(lpThreadAttributes,dwStackSize,lpStartAddress,lpParameter,	dwCreationFlags,lpThreadId);
		SetThreadPriority(handle,THREAD_PRIORITY_HIGHEST);
		//SetThreadPriority(handle,THREAD_PRIORITY_TIME_CRITICAL);

		SetThreadAffinityMask(handle, 1<<i);

		spuStatus.m_taskId = i;
		spuStatus.m_commandId = 0;
		spuStatus.m_status = 0;
		spuStatus.m_threadHandle = handle;
		spuStatus.m_lsMemory = threadConstructionInfo.m_lsMemoryFunc();
		spuStatus.m_userThreadFunc = threadConstructionInfo.m_userThreadFunc;

		printf("started thread %d with threadHandle %p\n",i,handle);
		
	}

}

void D_Win32ThreadSupport::startSPU()
{
}


///tell the task scheduler we D_are done with the SPU tasks
void D_Win32ThreadSupport::stopSPU()
{
	int i;
	for (i=0;i<m_activeSpuStatus.size();i++)
	{
		D_btSpuStatus& spuStatus = m_activeSpuStatus[i];
		if (spuStatus.m_status>0)
		{
			WaitForSingleObject(spuStatus.m_eventCompletetHandle, INFINITE);
		}
		

		spuStatus.m_userPtr = 0;
		SetEvent(spuStatus.m_eventStartHandle);
		WaitForSingleObject(spuStatus.m_eventCompletetHandle, INFINITE);

		CloseHandle(spuStatus.m_eventCompletetHandle);
		CloseHandle(spuStatus.m_eventStartHandle);
		CloseHandle(spuStatus.m_threadHandle);
	}

	m_activeSpuStatus.clear();
	m_completeHandles.clear();

}

#endif //D_USE_WIN32_THREADING
