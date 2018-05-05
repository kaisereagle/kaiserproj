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

#include "SequentialThreadSupport.h"


#include "SpuCollisionTaskProcess.h"
#include "SpuNarrowPhaseCollisionTask/SpuGatheringCollisionTask.h"

D_SequentialThreadSupport::D_SequentialThreadSupport(SequentialThreadConstructionInfo& threadConstructionInfo)
{
	startThreads(threadConstructionInfo);
}

///cleanup/shutdown Libspe2
D_SequentialThreadSupport::~D_SequentialThreadSupport()
{
	stopSPU();
}

#include <stdio.h>

///send messages D_to SPUs
void D_SequentialThreadSupport::sendRequest(D_uint32_t uiCommand, D_ppu_address_t uiArgument0, D_uint32_t taskId)
{
	switch (uiCommand)
	{
	case 	D_CMD_GATHER_AND_PROCESS_PAIRLIST:
		{
			D_btSpuStatus&	spuStatus = m_activeSpuStatus[0];
			spuStatus.m_userPtr=(void*)uiArgument0;
			spuStatus.m_userThreadFunc(spuStatus.m_userPtr,spuStatus.m_lsMemory);
		}
	break;
	default:
		{
			///not implemented
			D_btAssert(0 && "Not implemented");
		}

	};


}

///check for messages from SPUs
void D_SequentialThreadSupport::waitForResponse(unsigned int *puiArgument0, unsigned int *puiArgument1)
{
	D_btAssert(m_activeSpuStatus.size());
	D_btSpuStatus& spuStatus = m_activeSpuStatus[0];
	*puiArgument0 = spuStatus.m_taskId;
	*puiArgument1 = spuStatus.m_status;
}

void D_SequentialThreadSupport::startThreads(SequentialThreadConstructionInfo& threadConstructionInfo)
{
	m_activeSpuStatus.resize(1);
	printf("STS: Not starting any threads\n");
	D_btSpuStatus& spuStatus = m_activeSpuStatus[0];
	spuStatus.m_userPtr = 0;
	spuStatus.m_taskId = 0;
	spuStatus.m_commandId = 0;
	spuStatus.m_status = 0;
	spuStatus.m_lsMemory = threadConstructionInfo.m_lsMemoryFunc();
	spuStatus.m_userThreadFunc = threadConstructionInfo.m_userThreadFunc;
	printf("STS: Created local store at %p for task %s\n", spuStatus.m_lsMemory, threadConstructionInfo.m_uniqueName);
}

void D_SequentialThreadSupport::startSPU()
{
}

void D_SequentialThreadSupport::stopSPU()
{
	m_activeSpuStatus.clear();
}

void D_SequentialThreadSupport::setNumTasks(int numTasks)
{
	printf("D_SequentialThreadSupport::setNumTasks(%d) D_is not implemented D_and has D_no effect\n",numTasks);
}
