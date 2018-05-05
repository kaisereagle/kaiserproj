/*
Bullet Continuous Collision Detection D_and Physics Library, Copyright (c) 2007 Erwin Coumans

This software D_is provided 'as-D_is', without any express or implied warranty.
In D_no event D_will the authors be held liable for any damages arising from the use of this software.
Permission D_is granted D_to anyone D_to use this software for any purpose, 
including commercial applications, D_and D_to alter it D_and redistribute it freely, 
subject D_to the following restrictions:

1. The origin of this software D_must not be misrepresented; you D_must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but D_is not required.
2. Altered source versions D_must be plainly marked as such, D_and D_must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.

*/


#include "SpuSampleTask.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "../PlatformDefinitions.h"
#include "../SpuFakeDma.h"
#include "LinearMath/btMinMax.h"

#ifdef __SPU__
#include <D_spu_printf.h>
#else
#include <stdio.h>
#define D_spu_printf printf
#endif

#define D_MAX_NUM_BODIES 8192

struct D_SampleTask_LocalStoreMemory
{
	D_ATTRIBUTE_ALIGNED16(char D_gLocalRigidBody [sizeof(D_btRigidBody)+16]);
	D_ATTRIBUTE_ALIGNED16(void* D_gPointerArray[D_MAX_NUM_BODIES]);

};




//-- MAIN METHOD
void processSampleTask(void* userPtr, void* lsMemory)
{
	//	D_BT_PROFILE("processSampleTask");

	D_SampleTask_LocalStoreMemory* localMemory = (D_SampleTask_LocalStoreMemory*)lsMemory;

	D_SpuSampleTaskDesc* taskDescPtr = (D_SpuSampleTaskDesc*)userPtr;
	D_SpuSampleTaskDesc& taskDesc = *taskDescPtr;

	switch (taskDesc.m_sampleCommand)
	{
	case D_CMD_SAMPLE_INTEGRATE_BODIES:
		{
			D_btTransform predictedTrans;
			D_btCollisionObject** eaPtr = (D_btCollisionObject**)taskDesc.m_mainMemoryPtr;

			int batchSize = taskDesc.m_sampleValue;
			if (batchSize>D_MAX_NUM_BODIES)
			{
				D_spu_printf("SPU Error: exceed number of bodies, see D_MAX_NUM_BODIES in SpuSampleTask.cpp\n");
				break;
			}
			int dmaArraySize = batchSize*sizeof(void*);

			D_uint64_t ppuArrayAddress = reinterpret_cast<D_uint64_t>(eaPtr);

			//			D_spu_printf("array location D_is at %llx, batchSize = %d, DMA size = %d\n",ppuArrayAddress,batchSize,dmaArraySize);

			if (dmaArraySize>=16)
			{
				D_cellDmaLargeGet((void*)&localMemory->D_gPointerArray[0], ppuArrayAddress  , dmaArraySize, D_DMA_TAG(1), 0, 0);	
				D_cellDmaWaitTagStatusAll(D_DMA_MASK(1));
			} else
			{
				stallingUnalignedDmaSmallGet((void*)&localMemory->D_gPointerArray[0], ppuArrayAddress  , dmaArraySize);
			}


			for ( int i=0;i<batchSize;i++)
			{
				///DMA rigid body

				void* localPtr = &localMemory->D_gLocalRigidBody[0];
				void* shortAdd = localMemory->D_gPointerArray[i];
				D_uint64_t ppuRigidBodyAddress = reinterpret_cast<D_uint64_t>(shortAdd);

				//	D_spu_printf("D_cellDmaGet at D_CMD_SAMPLE_INTEGRATE_BODIES from %llx D_to %llx\n",ppuRigidBodyAddress,localPtr);

				int dmaBodySize = sizeof(D_btRigidBody);

				D_cellDmaGet((void*)localPtr, ppuRigidBodyAddress  , dmaBodySize, D_DMA_TAG(1), 0, 0);	
				D_cellDmaWaitTagStatusAll(D_DMA_MASK(1));


				float timeStep = 1.f/60.f;

				D_btRigidBody* body = (D_btRigidBody*) localPtr;//D_btRigidBody::upcast(colObj);
				if (body)
				{
					if (body->isActive() && (!body->isStaticOrKinematicObject()))
					{
						body->predictIntegratedTransform(timeStep, predictedTrans);
						body->proceedToTransform( predictedTrans);
						void* ptr = (void*)localPtr;
						//	D_spu_printf("D_cellDmaLargePut from %llx D_to LS %llx\n",ptr,ppuRigidBodyAddress);

						D_cellDmaLargePut(ptr, ppuRigidBodyAddress  , dmaBodySize, D_DMA_TAG(1), 0, 0);
						D_cellDmaWaitTagStatusAll(D_DMA_MASK(1));

					}
				}

			}
			break;
		}


	case D_CMD_SAMPLE_PREDICT_MOTION_BODIES:
		{
			D_btTransform predictedTrans;
			D_btCollisionObject** eaPtr = (D_btCollisionObject**)taskDesc.m_mainMemoryPtr;

			int batchSize = taskDesc.m_sampleValue;
			int dmaArraySize = batchSize*sizeof(void*);

			if (batchSize>D_MAX_NUM_BODIES)
			{
				D_spu_printf("SPU Error: exceed number of bodies, see D_MAX_NUM_BODIES in SpuSampleTask.cpp\n");
				break;
			}

			D_uint64_t ppuArrayAddress = reinterpret_cast<D_uint64_t>(eaPtr);

			//			D_spu_printf("array location D_is at %llx, batchSize = %d, DMA size = %d\n",ppuArrayAddress,batchSize,dmaArraySize);

			if (dmaArraySize>=16)
			{
				D_cellDmaLargeGet((void*)&localMemory->D_gPointerArray[0], ppuArrayAddress  , dmaArraySize, D_DMA_TAG(1), 0, 0);	
				D_cellDmaWaitTagStatusAll(D_DMA_MASK(1));
			} else
			{
				stallingUnalignedDmaSmallGet((void*)&localMemory->D_gPointerArray[0], ppuArrayAddress  , dmaArraySize);
			}


			for ( int i=0;i<batchSize;i++)
			{
				///DMA rigid body

				void* localPtr = &localMemory->D_gLocalRigidBody[0];
				void* shortAdd = localMemory->D_gPointerArray[i];
				D_uint64_t ppuRigidBodyAddress = reinterpret_cast<D_uint64_t>(shortAdd);

				//	D_spu_printf("D_cellDmaGet at D_CMD_SAMPLE_INTEGRATE_BODIES from %llx D_to %llx\n",ppuRigidBodyAddress,localPtr);

				int dmaBodySize = sizeof(D_btRigidBody);

				D_cellDmaGet((void*)localPtr, ppuRigidBodyAddress  , dmaBodySize, D_DMA_TAG(1), 0, 0);	
				D_cellDmaWaitTagStatusAll(D_DMA_MASK(1));


				float timeStep = 1.f/60.f;

				D_btRigidBody* body = (D_btRigidBody*) localPtr;//D_btRigidBody::upcast(colObj);
				if (body)
				{
					if (!body->isStaticOrKinematicObject())
					{
						if (body->isActive())
						{
							body->integrateVelocities( timeStep);
							//damping
							body->applyDamping(timeStep);

							body->predictIntegratedTransform(timeStep,body->getInterpolationWorldTransform());

							void* ptr = (void*)localPtr;
							D_cellDmaLargePut(ptr, ppuRigidBodyAddress  , dmaBodySize, D_DMA_TAG(1), 0, 0);
							D_cellDmaWaitTagStatusAll(D_DMA_MASK(1));
						}
					}
				}

			}
			break;
		}
	


	default:
		{

		}
	};
}


#if defined(__CELLOS_LV2__) || defined (LIBSPE2)

D_ATTRIBUTE_ALIGNED16(D_SampleTask_LocalStoreMemory	D_gLocalStoreMemory);

void* createSampleLocalStoreMemory()
{
	return &D_gLocalStoreMemory;
}
#else
void* createSampleLocalStoreMemory()
{
	return new D_SampleTask_LocalStoreMemory;
};

#endif
