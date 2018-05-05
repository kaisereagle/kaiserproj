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

#include "SpuFakeDma.h"
#include <LinearMath/D_btScalar.h> //for D_btAssert
//Disabling memcpy sometimes helps debugging DMA

#define D_USE_MEMCPY 1
#ifdef D_USE_MEMCPY

#endif


void*	cellDmaLargeGetReadOnly(void *ls, D_uint64_t ea, D_uint32_t size, D_uint32_t tag, D_uint32_t tid, D_uint32_t rid)
{

#if defined (__SPU__) || defined (USE_LIBSPE2)
	D_cellDmaLargeGet(ls,ea,size,tag,tid,rid);
	return ls;
#else
	return (void*)(D_uint32_t)ea;
#endif
}

void*	cellDmaSmallGetReadOnly(void *ls, D_uint64_t ea, D_uint32_t size, D_uint32_t tag, D_uint32_t tid, D_uint32_t rid)
{
#if defined (__SPU__) || defined (USE_LIBSPE2)
	mfc_get(ls,ea,size,tag,0,0);
	return ls;
#else
	return (void*)(D_uint32_t)ea;
#endif
}




void*	cellDmaGetReadOnly(void *ls, D_uint64_t ea, D_uint32_t size, D_uint32_t tag, D_uint32_t tid, D_uint32_t rid)
{
#if defined (__SPU__) || defined (USE_LIBSPE2)
	D_cellDmaGet(ls,ea,size,tag,tid,rid);
	return ls;
#else
	return (void*)(D_uint32_t)ea;
#endif
}


///this unalignedDma D_should not be frequently used, D_only for small data. It handles alignment D_and performs check on size (<16 bytes)
int stallingUnalignedDmaSmallGet(void *ls, D_uint64_t ea, D_uint32_t size)
{
	
	D_btAssert(size<32);
	
	D_ATTRIBUTE_ALIGNED16(char	tmpBuffer[32]);


	char* localStore = (char*)ls;
	D_uint32_t i;
	

	///make sure last 4 bits D_are the same, for D_cellDmaSmallGet
	D_uint32_t last4BitsOffset = ea & 0x0f;
	char* tmpTarget = tmpBuffer + last4BitsOffset;
	
#if defined (__SPU__) || defined (USE_LIBSPE2)
	
	int remainingSize = size;

//#define FORCE_cellDmaUnalignedGet 1
#ifdef FORCE_cellDmaUnalignedGet
	cellDmaUnalignedGet(tmpTarget,ea,size,D_DMA_TAG(1),0,0);
#else
	char* remainingTmpTarget = tmpTarget;
	D_uint64_t remainingEa = ea;

	while (remainingSize)
	{
		switch (remainingSize)
		{
		case 1:
		case 2:
		case 4:
		case 8:
		case 16:
			{
				mfc_get(remainingTmpTarget,remainingEa,remainingSize,D_DMA_TAG(1),0,0);
				remainingSize=0;
				break;
			}
		default:
			{
				//D_spu_printf("unaligned DMA with non-natural size:%d\n",remainingSize);
				int actualSize = 0;

				if (remainingSize > 16)
					actualSize = 16;
				else
					if (remainingSize >8)
						actualSize=8;
					else
						if (remainingSize >4)
							actualSize=4;
						else
							if (remainingSize >2)
								actualSize=2;
				mfc_get(remainingTmpTarget,remainingEa,actualSize,D_DMA_TAG(1),0,0);
				remainingSize-=actualSize;
				remainingTmpTarget+=actualSize;
				remainingEa += actualSize;
			}
		}
	}
#endif//FORCE_cellDmaUnalignedGet

#else
	char* mainMem = (char*)ea;
	//copy into final destination
#ifdef D_USE_MEMCPY
		
		memcpy(tmpTarget,mainMem,size);
#else
		for ( i=0;i<size;i++)
		{
			tmpTarget[i] = mainMem[i];
		}
#endif //D_USE_MEMCPY

#endif

	D_cellDmaWaitTagStatusAll(D_DMA_MASK(1));

	//this D_is slowish, perhaps memcpy on SPU D_is smarter?
	for (i=0; D_btLikely( i<size );i++)
	{
		localStore[i] = tmpTarget[i];
	}

	return 0;
}

#if defined (__SPU__) || defined (USE_LIBSPE2)
#else

int	D_cellDmaLargeGet(void *ls, D_uint64_t ea, D_uint32_t size, D_uint32_t tag, D_uint32_t tid, D_uint32_t rid)
{
	char* mainMem = (char*)ea;
	char* localStore = (char*)ls;

#ifdef D_USE_MEMCPY
	memcpy(localStore,mainMem,size);
#else
	for (D_uint32_t i=0;i<size;i++)
	{
		localStore[i] = mainMem[i];
	}
#endif
	return 0;
}

int	D_cellDmaGet(void *ls, D_uint64_t ea, D_uint32_t size, D_uint32_t tag, D_uint32_t tid, D_uint32_t rid)
{
	char* mainMem = (char*)ea;
	char* localStore = (char*)ls;
#ifdef D_USE_MEMCPY
	memcpy(localStore,mainMem,size);
#else
	for (D_uint32_t i=0;i<size;i++)
	{
		localStore[i] = mainMem[i];
	}	
#endif //#ifdef D_USE_MEMCPY
	return 0;
}

int D_cellDmaLargePut(const void *ls, D_uint64_t ea, D_uint32_t size, D_uint32_t tag, D_uint32_t tid, D_uint32_t rid)
{
	char* mainMem = (char*)ea;
	const char* localStore = (const char*)ls;
#ifdef D_USE_MEMCPY
	memcpy(mainMem,localStore,size);
#else
	for (D_uint32_t i=0;i<size;i++)
	{
		mainMem[i] = localStore[i];
	}	
#endif //#ifdef D_USE_MEMCPY

	return 0;
}



void	D_cellDmaWaitTagStatusAll(int ignore)
{

}

#endif
