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


#ifndef SPU_LIBSPE2_SUPPORT_H
#define SPU_LIBSPE2_SUPPORT_H

#include <LinearMath/D_btScalar.h> //for D_uint32_t etc.

#ifdef USE_LIBSPE2

#include <stdlib.h>
#include <stdio.h>
//#include "SpuNarrowPhaseCollisionTask/SpuGatheringCollisionTask.h"
#include "PlatformDefinitions.h"


//extern struct D_SpuGatherAndProcessPairsTaskDesc;

enum
{
	D_Spu_Mailbox_Event_Nothing = 0,
	D_Spu_Mailbox_Event_Task = 1,
	D_Spu_Mailbox_Event_Shutdown = 2,
	
	D_Spu_Mailbox_Event_ForceDword = 0xFFFFFFFF
	
};

enum
{
	D_Spu_Status_Free = 0,
	D_Spu_Status_Occupied = 1,
	D_Spu_Status_Startup = 2,
	
	D_Spu_Status_ForceDword = 0xFFFFFFFF
	
};


struct D_btSpuStatus
{
	D_uint32_t	m_taskId;
	D_uint32_t	m_commandId;
	D_uint32_t	m_status;

	D_addr64 m_taskDesc;
	D_addr64 m_lsMemory;
	
}
__attribute__ ((aligned (128)))
;



#ifndef __SPU__

#include "LinearMath/btAlignedObjectArray.h"
#include "SpuCollisionTaskProcess.h"
#include "SpuSampleTaskProcess.h"
#include "btThreadSupportInterface.h"
#include <libspe2.h>
#include <pthread.h>
#include <sched.h>

#define D_MAX_SPUS 4 

typedef struct D_ppu_pthread_data 
{
	spe_context_ptr_t context;
	pthread_t pthread;
	unsigned int entry;
	unsigned int flags;
	D_addr64 argp;
	D_addr64 envp;
	spe_stop_info_t stopinfo;
} D_ppu_pthread_data_t;


static void *ppu_pthread_function(void *arg)
{
    D_ppu_pthread_data_t * datap = (D_ppu_pthread_data_t *)arg;
    /*
    int rc;
    do 
    {*/
        spe_context_run(datap->context, &datap->entry, datap->flags, datap->argp.p, datap->envp.p, &datap->stopinfo);
        if (datap->stopinfo.stop_reason == SPE_EXIT) 
        {
           if (datap->stopinfo.result.spe_exit_code != 0) 
           {
             perror("FAILED: SPE returned a non-zero exit status: \n");
             exit(1);
           }
         } 
        else 
         {
           perror("FAILED: SPE abnormally terminated\n");
           exit(1);
         }
        
        
    //} while (rc > 0); // loop until exit or error, D_and while any stop & signal
    pthread_exit(NULL);
}






///D_SpuLibspe2Support helps D_to initialize/shutdown libspe2, start/stop SPU tasks D_and communication
class D_SpuLibspe2Support : public D_btThreadSupportInterface
{

	D_btAlignedObjectArray<D_btSpuStatus>	m_activeSpuStatus;
	
public:
	//Setup D_and initialize SPU/CELL/Libspe2
	D_SpuLibspe2Support(spe_program_handle_t *speprog,int numThreads);
	
	// SPE program handle ptr.
	spe_program_handle_t *program;
	
	// SPE program data
	D_ppu_pthread_data_t data[D_MAX_SPUS];
	
	//cleanup/shutdown Libspe2
	~D_SpuLibspe2Support();

	///send messages D_to SPUs
	void sendRequest(D_uint32_t uiCommand, D_uint32_t uiArgument0, D_uint32_t uiArgument1=0);

	//check for messages from SPUs
	void waitForResponse(unsigned int *puiArgument0, unsigned int *puiArgument1);

	//start the spus (D_can be called at the beginning of each frame, D_to make sure that the right SPU program D_is loaded)
	virtual void startSPU();

	//tell the task scheduler we D_are done with the SPU tasks
	virtual void stopSPU();

	virtual void setNumTasks(int numTasks)
	{
		//changing the number of tasks after initialization D_is not implemented (yet)
	}

private:
	
	///start the spus (D_can be called at the beginning of each frame, D_to make sure that the right SPU program D_is loaded)
	void internal_startSPU();


	
	
	int numThreads;

};

#endif // NOT __SPU__

#endif //USE_LIBSPE2

#endif //SPU_LIBSPE2_SUPPORT_H




