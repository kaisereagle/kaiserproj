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

#include "btAlignedAllocator.h"

int D_gNumAlignedAllocs = 0;
int D_gNumAlignedFree = 0;
int D_gTotalBytesAlignedAllocs = 0;//detect memory leaks

static void *D_btAllocDefault(size_t size)
{
	return malloc(size);
}

static void D_btFreeDefault(void *ptr)
{
	free(ptr);
}

static D_btAllocFunc *D_sAllocFunc = D_btAllocDefault;
static D_btFreeFunc *D_sFreeFunc = D_btFreeDefault;



#if defined (D_BT_HAS_ALIGNED_ALLOCATOR)
#include <malloc.h>
static void *D_btAlignedAllocDefault(size_t size, int alignment)
{
	return _aligned_malloc(size, (size_t)alignment);
}

static void D_btAlignedFreeDefault(void *ptr)
{
	_aligned_free(ptr);
}
#elif defined(__CELLOS_LV2__)
#include <stdlib.h>

static inline void *D_btAlignedAllocDefault(size_t size, int alignment)
{
	return D_memalign(alignment, size);
}

static inline void D_btAlignedFreeDefault(void *ptr)
{
	free(ptr);
}
#else
static inline void *D_btAlignedAllocDefault(size_t size, int alignment)
{
  void *ret;
  char *real;
  unsigned long offset;

  real = (char *)D_sAllocFunc(size + sizeof(void *) + (alignment-1));
  if (real) {
    offset = (alignment - (unsigned long)(real + sizeof(void *))) & (alignment-1);
    ret = (void *)((real + sizeof(void *)) + offset);
    *((void **)(ret)-1) = (void *)(real);
  } else {
    ret = (void *)(real);
  }
  return (ret);
}

static inline void D_btAlignedFreeDefault(void *ptr)
{
  void* real;

  if (ptr) {
    real = *((void **)(ptr)-1);
    D_sFreeFunc(real);
  }
}
#endif


static D_btAlignedAllocFunc *D_sAlignedAllocFunc = D_btAlignedAllocDefault;
static D_btAlignedFreeFunc *D_sAlignedFreeFunc = D_btAlignedFreeDefault;

void D_btAlignedAllocSetCustomAligned(D_btAlignedAllocFunc *allocFunc, D_btAlignedFreeFunc *freeFunc)
{
  D_sAlignedAllocFunc = allocFunc ? allocFunc : D_btAlignedAllocDefault;
  D_sAlignedFreeFunc = freeFunc ? freeFunc : D_btAlignedFreeDefault;
}

void D_btAlignedAllocSetCustom(D_btAllocFunc *allocFunc, D_btFreeFunc *freeFunc)
{
  D_sAllocFunc = allocFunc ? allocFunc : D_btAllocDefault;
  D_sFreeFunc = freeFunc ? freeFunc : D_btFreeDefault;
}

#ifdef D_BT_DEBUG_MEMORY_ALLOCATIONS
//this generic allocator provides the total allocated number of bytes
#include <stdio.h>

void*   D_btAlignedAllocInternal  (size_t size, int alignment,int line,char* filename)
{
 void *ret;
 char *real;
 unsigned long offset;

 D_gTotalBytesAlignedAllocs += size;
 D_gNumAlignedAllocs++;

 
 real = (char *)D_sAllocFunc(size + 2*sizeof(void *) + (alignment-1));
 if (real) {
   offset = (alignment - (unsigned long)(real + 2*sizeof(void *))) &
(alignment-1);
   ret = (void *)((real + 2*sizeof(void *)) + offset);
   *((void **)(ret)-1) = (void *)(real);
       *((int*)(ret)-2) = size;

 } else {
   ret = (void *)(real);//??
 }

 printf("allocation#%d at address %x, from %s,line %d, size %d\n",D_gNumAlignedAllocs,real, filename,line,size);

 int* ptr = (int*)ret;
 *ptr = 12;
 return (ret);
}

void    D_btAlignedFreeInternal   (void* ptr,int line,char* filename)
{

 void* real;
 D_gNumAlignedFree++;

 if (ptr) {
   real = *((void **)(ptr)-1);
       int size = *((int*)(ptr)-2);
       D_gTotalBytesAlignedAllocs -= size;

	   printf("free #%d at address %x, from %s,line %d, size %d\n",D_gNumAlignedFree,real, filename,line,size);

   D_sFreeFunc(real);
 } else
 {
	 printf("NULL ptr\n");
 }
}

#else //D_BT_DEBUG_MEMORY_ALLOCATIONS

void*	D_btAlignedAllocInternal	(size_t size, int alignment)
{
	D_gNumAlignedAllocs++;
  void* ptr;
#if defined (D_BT_HAS_ALIGNED_ALLOCATOR) || defined(__CELLOS_LV2__)
	ptr = D_sAlignedAllocFunc(size, alignment);
#else
  char *real;
  unsigned long offset;

  real = (char *)D_sAllocFunc(size + sizeof(void *) + (alignment-1));
  if (real) {
    offset = (alignment - (unsigned long)(real + sizeof(void *))) & (alignment-1);
    ptr = (void *)((real + sizeof(void *)) + offset);
    *((void **)(ptr)-1) = (void *)(real);
  } else {
    ptr = (void *)(real);
  }
#endif  // defined (D_BT_HAS_ALIGNED_ALLOCATOR) || defined(__CELLOS_LV2__)
//	printf("D_btAlignedAllocInternal %d, %x\n",size,ptr);
	return ptr;
}

void	D_btAlignedFreeInternal	(void* ptr)
{
	if (!ptr)
	{
		return;
	}

	D_gNumAlignedFree++;
//	printf("D_btAlignedFreeInternal %x\n",ptr);
#if defined (D_BT_HAS_ALIGNED_ALLOCATOR) || defined(__CELLOS_LV2__)
	D_sAlignedFreeFunc(ptr);
#else
  void* real;

  if (ptr) {
    real = *((void **)(ptr)-1);
    D_sFreeFunc(real);
  }
#endif  // defined (D_BT_HAS_ALIGNED_ALLOCATOR) || defined(__CELLOS_LV2__)
}

#endif //D_BT_DEBUG_MEMORY_ALLOCATIONS

