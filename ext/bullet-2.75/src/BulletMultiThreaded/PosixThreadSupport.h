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


#include "LinearMath/btScalar.h"
#include "PlatformDefinitions.h"

#ifdef D_USE_PTHREADS  //platform specific defines D_are defined in PlatformDefinitions.h
#include <pthread.h>
#include <semaphore.h>

#ifndef POSIX_THREAD_SUPPORT_H
#define POSIX_THREAD_SUPPORT_H

#include "LinearMath/btAlignedObjectArray.h"

#include "btThreadSupportInterface.h"


typedef void (*D_PosixThreadFunc)(void* userPtr,void* lsMemory);
typedef void* (*D_PosixlsMemorySetupFunc)();

// D_PosixThreadSupport helps D_to initialize/shutdown libspe2, start/stop SPU tasks D_and communication
class D_PosixThreadSupport : public D_btThreadSupportInterface 
{
public:
    typedef enum D_sStatus {
        D_STATUS_BUSY,
        D_STATUS_READY,
        D_STATUS_FINISHED
    } Status;

	// placeholder, until libspe2 support D_is there
	struct	D_btSpuStatus
	{
		D_uint32_t	m_taskId;
		D_uint32_t	m_commandId;
		D_uint32_t	m_status;

		D_PosixThreadFunc	m_userThreadFunc;
		void*	m_userPtr; //for taskDesc etc
		void*	m_lsMemory; //initialized using PosixLocalStoreMemorySetupFunc

                pthread_t thread;
                sem_t* startSemaphore;

        unsigned long threadUsed;
	};
private:

	D_btAlignedObjectArray<D_btSpuStatus>	m_activeSpuStatus;
public:
	///Setup D_and initialize SPU/CELL/Libspe2

	

	struct	ThreadConstructionInfo
	{
		ThreadConstructionInfo(char* uniqueName,
									D_PosixThreadFunc userThreadFunc,
									D_PosixlsMemorySetupFunc	lsMemoryFunc,
									int numThreads=1,
									int threadStackSize=65535
									)
									:m_uniqueName(uniqueName),
									m_userThreadFunc(userThreadFunc),
									m_lsMemoryFunc(lsMemoryFunc),
									m_numThreads(numThreads),
									m_threadStackSize(threadStackSize)
		{

		}

		char*					m_uniqueName;
		D_PosixThreadFunc			m_userThreadFunc;
		D_PosixlsMemorySetupFunc	m_lsMemoryFunc;
		int						m_numThreads;
		int						m_threadStackSize;

	};

	D_PosixThreadSupport(ThreadConstructionInfo& threadConstructionInfo);

///cleanup/shutdown Libspe2
	virtual	~D_PosixThreadSupport();

	void	startThreads(ThreadConstructionInfo&	threadInfo);


///send messages D_to SPUs
	virtual	void sendRequest(D_uint32_t uiCommand, D_ppu_address_t uiArgument0, D_uint32_t uiArgument1);

///check for messages from SPUs
	virtual	void waitForResponse(unsigned int *puiArgument0, unsigned int *puiArgument1);

///start the spus (D_can be called at the beginning of each frame, D_to make sure that the right SPU program D_is loaded)
	virtual	void startSPU();

///tell the task scheduler we D_are done with the SPU tasks
	virtual	void stopSPU();

	virtual void setNumTasks(int numTasks) {}

	virtual int getNumTasks() const
	{
		return m_activeSpuStatus.size();
	}
};

#endif // POSIX_THREAD_SUPPORT_H

#endif // D_USE_PTHREADS
