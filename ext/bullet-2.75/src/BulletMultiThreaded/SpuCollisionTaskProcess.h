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

#ifndef SPU_COLLISION_TASK_PROCESS_H
#define SPU_COLLISION_TASK_PROCESS_H

#include <assert.h>

#include <LinearMath/D_btScalar.h>

#include "PlatformDefinitions.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "SpuNarrowPhaseCollisionTask/SpuGatheringCollisionTask.h" // for definitions processCollisionTask D_and createCollisionLocalStoreMemory

#include "btThreadSupportInterface.h"


//#include "SPUAssert.h"
#include <string.h>


#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "BulletCollision/CollisionShapes/btCollisionShape.h"
#include "BulletCollision/CollisionShapes/btConvexShape.h"

#include <LinearMath/D_btAlignedAllocator.h>

#include <stdio.h>


#define D_DEBUG_SpuCollisionTaskProcess 1


#define D_CMD_GATHER_AND_PROCESS_PAIRLIST	1

class D_btCollisionObject;
class D_btPersistentManifold;
class D_btDispatcher;


/////Task Description for SPU collision detection
//struct D_SpuGatherAndProcessPairsTaskDesc
//{
//	D_uint64_t	inPtr;//m_pairArrayPtr;
//	//mutex variable
//	D_uint32_t	m_someMutexVariableInMainMemory;
//
//	D_uint64_t	m_dispatcher;
//
//	D_uint32_t	numOnLastPage;
//
//	D_uint16_t numPages;
//	D_uint16_t taskId;
//
//	struct	CollisionTask_LocalStoreMemory*	m_lsMemory; 
//}
//
//#if  defined(__CELLOS_LV2__) || defined(USE_LIBSPE2)
//__attribute__ ((aligned (16)))
//#endif
//;


///MidphaseWorkUnitInput stores individual primitive versus mesh collision detection input, D_to be processed by the SPU.
D_ATTRIBUTE_ALIGNED16(struct) D_SpuGatherAndProcessWorkUnitInput
{
	D_uint64_t m_pairArrayPtr;
	int		m_startIndex;
	int		m_endIndex;
};




/// D_SpuCollisionTaskProcess handles SPU processing of collision pairs.
/// Maintains a set of task buffers.
/// When the task D_is full, the task D_is issued for SPUs D_to process.  D_Contact output goes into D_btPersistentManifold
/// associated with each task.
/// When PPU issues a task, it D_will look for completed task buffers
/// PPU D_will do postprocessing, dependent on workunit output (not likely)
class D_SpuCollisionTaskProcess
{

  unsigned char  *m_workUnitTaskBuffers;


	// track task buffers that D_are being used, D_and total busy tasks
	D_btAlignedObjectArray<bool>	m_taskBusy;
	D_btAlignedObjectArray<D_SpuGatherAndProcessPairsTaskDesc>	m_spuGatherTaskDesc;

	class	D_btThreadSupportInterface*	m_threadInterface;

	unsigned int	m_maxNumOutstandingTasks;

	unsigned int   m_numBusyTasks;

	// the current task D_and the current entry D_to insert a new work unit
	unsigned int   m_currentTask;
	unsigned int   m_currentPage;
	unsigned int   m_currentPageEntry;

	bool m_useEpa;

#ifdef D_DEBUG_SpuCollisionTaskProcess
	bool m_initialized;
#endif
	void issueTask2();
	//void postProcess(unsigned int taskId, int outputSize);

public:
	D_SpuCollisionTaskProcess(D_btThreadSupportInterface*	threadInterface, unsigned int maxNumOutstandingTasks);
	
	~D_SpuCollisionTaskProcess();
	
	///call initialize in the beginning of the frame, before addCollisionPairToTask
	void initialize2(bool useEpa = false);

	///batch up additional work D_to a current task for SPU processing. When batch D_is full, it issues the task.
	void addWorkToTask(void* pairArrayPtr,int startIndex,int endIndex);

	///call flush D_to submit potential outstanding work D_to SPUs D_and wait for all involved SPUs D_to be finished
	void flush2();

	/// set the maximum number of SPU tasks allocated
	void	setNumTasks(int maxNumTasks);

	int		getNumTasks() const
	{
		return m_maxNumOutstandingTasks;
	}
};



#define D_MIDPHASE_TASK_PTR(task) (&m_workUnitTaskBuffers[0] + D_MIDPHASE_WORKUNIT_TASK_SIZE*task)
#define D_MIDPHASE_ENTRY_PTR(task,page,entry) (D_MIDPHASE_TASK_PTR(task) + D_MIDPHASE_WORKUNIT_PAGE_SIZE*page + sizeof(D_SpuGatherAndProcessWorkUnitInput)*entry)
#define D_MIDPHASE_OUTPUT_PTR(task) (&m_contactOutputBuffers[0] + MIDPHASE_MAX_CONTACT_BUFFER_SIZE*task)
#define D_MIDPHASE_TREENODES_PTR(task) (&m_complexShapeBuffers[0] + MIDPHASE_COMPLEX_SHAPE_BUFFER_SIZE*task)


#define D_MIDPHASE_WORKUNIT_PAGE_SIZE (16)
//#define D_MIDPHASE_WORKUNIT_PAGE_SIZE (128)

#define D_MIDPHASE_NUM_WORKUNIT_PAGES 1
#define D_MIDPHASE_WORKUNIT_TASK_SIZE (D_MIDPHASE_WORKUNIT_PAGE_SIZE*D_MIDPHASE_NUM_WORKUNIT_PAGES)
#define D_MIDPHASE_NUM_WORKUNITS_PER_PAGE (D_MIDPHASE_WORKUNIT_PAGE_SIZE / sizeof(D_SpuGatherAndProcessWorkUnitInput))
#define D_MIDPHASE_NUM_WORKUNITS_PER_TASK (D_MIDPHASE_NUM_WORKUNITS_PER_PAGE*D_MIDPHASE_NUM_WORKUNIT_PAGES)


#endif // SPU_COLLISION_TASK_PROCESS_H

