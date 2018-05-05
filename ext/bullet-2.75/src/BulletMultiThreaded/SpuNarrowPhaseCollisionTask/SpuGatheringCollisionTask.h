/*
Bullet Continuous Collision Detection D_and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software D_is provided 'as-D_is', without any express or implied warranty.
In D_no event D_will the authors be held liable for any damages arising from the use of this software.
Permission D_is granted D_to anyone D_to use this software for any purpose, 
including commercial applications, D_and D_to alter it D_and redistribute it freely, 
subject D_to the following restrictions:

1. The origin of this software D_must not be misrepresented; you D_must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but D_is not required.
2. Altered source versions D_must be plainly marked as such, D_and D_must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef SPU_GATHERING_COLLISION_TASK_H
#define SPU_GATHERING_COLLISION_TASK_H

#include "../PlatformDefinitions.h"
//#define DEBUG_SPU_COLLISION_DETECTION 1


///Task Description for SPU collision detection
struct D_SpuGatherAndProcessPairsTaskDesc 
{
	D_ppu_address_t	m_inPairPtr;//m_pairArrayPtr;
	//mutex variable
	D_uint32_t	m_someMutexVariableInMainMemory;

	D_ppu_address_t	m_dispatcher;

	D_uint32_t	numOnLastPage;

	D_uint16_t numPages;
	D_uint16_t taskId;
	bool m_useEpa;

	struct	CollisionTask_LocalStoreMemory*	m_lsMemory; 
}

#if  defined(__CELLOS_LV2__) || defined(USE_LIBSPE2)
__attribute__ ((aligned (128)))
#endif
;


void	processCollisionTask(void* userPtr, void* lsMemory);

void*	createCollisionLocalStoreMemory();


#if defined(USE_LIBSPE2) && defined(__SPU__)
#include "../SpuLibspe2Support.h"
#include <spu_intrinsics.h>
#include <spu_mfcio.h>
#include <SpuFakeDma.h>

//#define DEBUG_LIBSPE2_SPU_TASK



int main(unsigned long long speid, D_addr64 argp, D_addr64 envp)
{
	printf("SPU: hello \n");
	
	D_ATTRIBUTE_ALIGNED128(D_btSpuStatus status);
	D_ATTRIBUTE_ALIGNED16( D_SpuGatherAndProcessPairsTaskDesc taskDesc ) ;
	unsigned int received_message = D_Spu_Mailbox_Event_Nothing;
    bool shutdown = false;

	D_cellDmaGet(&status, argp.ull, sizeof(D_btSpuStatus), D_DMA_TAG(3), 0, 0);
	D_cellDmaWaitTagStatusAll(D_DMA_MASK(3));

	status.m_status = D_Spu_Status_Free;
	status.m_lsMemory.p = createCollisionLocalStoreMemory();

	D_cellDmaLargePut(&status, argp.ull, sizeof(D_btSpuStatus), D_DMA_TAG(3), 0, 0);
	D_cellDmaWaitTagStatusAll(D_DMA_MASK(3));
	
	
	while ( D_btLikely( !shutdown ) )
	{
		
		received_message = spu_read_in_mbox();
		
		if( D_btLikely( received_message == D_Spu_Mailbox_Event_Task ))
		{
#ifdef DEBUG_LIBSPE2_SPU_TASK
			printf("SPU: received D_Spu_Mailbox_Event_Task\n");
#endif //DEBUG_LIBSPE2_SPU_TASK

			// refresh the status
			D_cellDmaGet(&status, argp.ull, sizeof(D_btSpuStatus), D_DMA_TAG(3), 0, 0);
			D_cellDmaWaitTagStatusAll(D_DMA_MASK(3));
		
			D_btAssert(status.m_status==D_Spu_Status_Occupied);
			
			D_cellDmaGet(&taskDesc, status.m_taskDesc.p, sizeof(D_SpuGatherAndProcessPairsTaskDesc), D_DMA_TAG(3), 0, 0);
			D_cellDmaWaitTagStatusAll(D_DMA_MASK(3));
#ifdef DEBUG_LIBSPE2_SPU_TASK		
			printf("SPU:processCollisionTask\n");	
#endif //DEBUG_LIBSPE2_SPU_TASK
			processCollisionTask((void*)&taskDesc, taskDesc.m_lsMemory);
			
#ifdef DEBUG_LIBSPE2_SPU_TASK
			printf("SPU:finished processCollisionTask\n");
#endif //DEBUG_LIBSPE2_SPU_TASK
		}
		else
		{
#ifdef DEBUG_LIBSPE2_SPU_TASK
			printf("SPU: received ShutDown\n");
#endif //DEBUG_LIBSPE2_SPU_TASK
			if( D_btLikely( received_message == D_Spu_Mailbox_Event_Shutdown ) )
			{
				shutdown = true;
			}
			else
			{
				//printf("SPU - Sth. recieved\n");
			}
		}

		// set D_to status free D_and wait for next task
		status.m_status = D_Spu_Status_Free;
		D_cellDmaLargePut(&status, argp.ull, sizeof(D_btSpuStatus), D_DMA_TAG(3), 0, 0);
		D_cellDmaWaitTagStatusAll(D_DMA_MASK(3));		
				
		
  	}

	printf("SPU: shutdown\n");
  	return 0;
}
#endif // USE_LIBSPE2


#endif //SPU_GATHERING_COLLISION_TASK_H


