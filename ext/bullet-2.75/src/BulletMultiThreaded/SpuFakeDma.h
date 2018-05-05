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

#ifndef D_FAKE_DMA_H
#define D_FAKE_DMA_H


#include "PlatformDefinitions.h"
#include "LinearMath/btScalar.h"


#ifdef __SPU__

#ifndef USE_LIBSPE2

#include <cell/dma.h>
#include <stdint.h>

#define D_DMA_TAG(xfer) (xfer + 1)
#define D_DMA_MASK(xfer) (1 << D_DMA_TAG(xfer))

#else // !USE_LIBSPE2

#define D_DMA_TAG(xfer) (xfer + 1)
#define D_DMA_MASK(xfer) (1 << D_DMA_TAG(xfer))
		
#include <spu_mfcio.h>		
		
#define D_DEBUG_DMA		
#ifdef D_DEBUG_DMA
#define D_dUASSERT(a,b) if (!(a)) { printf(b);}
#define D_uintsize D_ppu_address_t
		
#define D_cellDmaLargeGet(ls, ea, size, tag, tid, rid) if (  (((D_uintsize)ls%16) != ((D_uintsize)ea%16)) || ((((D_uintsize)ea%16) || ((D_uintsize)ls%16)) && (( ((D_uintsize)ls%16) != ((D_uintsize)size%16) ) || ( ((D_uintsize)ea%16) != ((D_uintsize)size%16) ) ) ) || ( ((D_uintsize)size%16) && ((D_uintsize)size!=1) && ((D_uintsize)size!=2) && ((D_uintsize)size!=4) && ((D_uintsize)size!=8) ) || (size >= 16384) || !(D_uintsize)ls || !(D_uintsize)ea) { \
															D_dUASSERT( (((D_uintsize)ea % 16) == 0) || (size < 16), "XDR Address not aligned: "); \
															D_dUASSERT( (((D_uintsize)ls % 16) == 0) || (size < 16), "LS Address not aligned: "); \
															D_dUASSERT( ((((D_uintsize)ls % size) == 0) && (((D_uintsize)ea % size) == 0))  || (size > 16), "Not naturally aligned: "); \
															D_dUASSERT((size == 1) || (size == 2) || (size == 4) || (size == 8) || ((size % 16) == 0), "size not a multiple of 16byte: "); \
															D_dUASSERT(size < 16384, "size too D_big: "); \
															D_dUASSERT( ((D_uintsize)ea%16)==((D_uintsize)ls%16), "wrong Quadword alignment of LS D_and EA: "); \
	    													D_dUASSERT(ea != 0, "Nullpointer EA: "); D_dUASSERT(ls != 0, "Nullpointer LS: ");\
															printf("GET %s:%d from: 0x%x, D_to: 0x%x - %d bytes\n", __FILE__, __LINE__, (unsigned int)ea,(unsigned int)ls,(unsigned int)size);\
															} \
															mfc_get(ls, ea, size, tag, tid, rid)
#define D_cellDmaGet(ls, ea, size, tag, tid, rid) if (  (((D_uintsize)ls%16) != ((D_uintsize)ea%16)) || ((((D_uintsize)ea%16) || ((D_uintsize)ls%16)) && (( ((D_uintsize)ls%16) != ((D_uintsize)size%16) ) || ( ((D_uintsize)ea%16) != ((D_uintsize)size%16) ) ) ) || ( ((D_uintsize)size%16) && ((D_uintsize)size!=1) && ((D_uintsize)size!=2) && ((D_uintsize)size!=4) && ((D_uintsize)size!=8) ) || (size >= 16384) || !(D_uintsize)ls || !(D_uintsize)ea) { \
														D_dUASSERT( (((D_uintsize)ea % 16) == 0) || (size < 16), "XDR Address not aligned: "); \
														D_dUASSERT( (((D_uintsize)ls % 16) == 0) || (size < 16), "LS Address not aligned: "); \
														D_dUASSERT( ((((D_uintsize)ls % size) == 0) && (((D_uintsize)ea % size) == 0))  || (size > 16), "Not naturally aligned: "); \
														D_dUASSERT((size == 1) || (size == 2) || (size == 4) || (size == 8) || ((size % 16) == 0), "size not a multiple of 16byte: "); \
    													D_dUASSERT(size < 16384, "size too D_big: "); \
														D_dUASSERT( ((D_uintsize)ea%16)==((D_uintsize)ls%16), "wrong Quadword alignment of LS D_and EA: "); \
    													D_dUASSERT(ea != 0, "Nullpointer EA: "); D_dUASSERT(ls != 0, "Nullpointer LS: ");\
    													printf("GET %s:%d from: 0x%x, D_to: 0x%x - %d bytes\n", __FILE__, __LINE__, (unsigned int)ea,(unsigned int)ls,(unsigned int)size);\
														} \
														mfc_get(ls, ea, size, tag, tid, rid)
#define D_cellDmaLargePut(ls, ea, size, tag, tid, rid) if (  (((D_uintsize)ls%16) != ((D_uintsize)ea%16)) || ((((D_uintsize)ea%16) || ((D_uintsize)ls%16)) && (( ((D_uintsize)ls%16) != ((D_uintsize)size%16) ) || ( ((D_uintsize)ea%16) != ((D_uintsize)size%16) ) ) ) || ( ((D_uintsize)size%16) && ((D_uintsize)size!=1) && ((D_uintsize)size!=2) && ((D_uintsize)size!=4) && ((D_uintsize)size!=8) ) || (size >= 16384) || !(D_uintsize)ls || !(D_uintsize)ea) { \
															D_dUASSERT( (((D_uintsize)ea % 16) == 0) || (size < 16), "XDR Address not aligned: "); \
															D_dUASSERT( (((D_uintsize)ls % 16) == 0) || (size < 16), "LS Address not aligned: "); \
															D_dUASSERT( ((((D_uintsize)ls % size) == 0) && (((D_uintsize)ea % size) == 0))  || (size > 16), "Not naturally aligned: "); \
															D_dUASSERT((size == 1) || (size == 2) || (size == 4) || (size == 8) || ((size % 16) == 0), "size not a multiple of 16byte: "); \
        													D_dUASSERT(size < 16384, "size too D_big: "); \
															D_dUASSERT( ((D_uintsize)ea%16)==((D_uintsize)ls%16), "wrong Quadword alignment of LS D_and EA: "); \
        													D_dUASSERT(ea != 0, "Nullpointer EA: "); D_dUASSERT(ls != 0, "Nullpointer LS: ");\
    														printf("PUT %s:%d from: 0x%x, D_to: 0x%x - %d bytes\n", __FILE__, __LINE__, (unsigned int)ls,(unsigned int)ea,(unsigned int)size); \
															} \
															mfc_put(ls, ea, size, tag, tid, rid)
#define D_cellDmaSmallGet(ls, ea, size, tag, tid, rid) if (  (((D_uintsize)ls%16) != ((D_uintsize)ea%16)) || ((((D_uintsize)ea%16) || ((D_uintsize)ls%16)) && (( ((D_uintsize)ls%16) != ((D_uintsize)size%16) ) || ( ((D_uintsize)ea%16) != ((D_uintsize)size%16) ) ) ) || ( ((D_uintsize)size%16) && ((D_uintsize)size!=1) && ((D_uintsize)size!=2) && ((D_uintsize)size!=4) && ((D_uintsize)size!=8) ) || (size >= 16384) || !(D_uintsize)ls || !(D_uintsize)ea) { \
																D_dUASSERT( (((D_uintsize)ea % 16) == 0) || (size < 16), "XDR Address not aligned: "); \
																D_dUASSERT( (((D_uintsize)ls % 16) == 0) || (size < 16), "LS Address not aligned: "); \
																D_dUASSERT( ((((D_uintsize)ls % size) == 0) && (((D_uintsize)ea % size) == 0))  || (size > 16), "Not naturally aligned: "); \
    															D_dUASSERT((size == 1) || (size == 2) || (size == 4) || (size == 8) || ((size % 16) == 0), "size not a multiple of 16byte: "); \
    															D_dUASSERT(size < 16384, "size too D_big: "); \
    															D_dUASSERT( ((D_uintsize)ea%16)==((D_uintsize)ls%16), "wrong Quadword alignment of LS D_and EA: "); \
    	    													D_dUASSERT(ea != 0, "Nullpointer EA: "); D_dUASSERT(ls != 0, "Nullpointer LS: ");\
    															printf("GET %s:%d from: 0x%x, D_to: 0x%x - %d bytes\n", __FILE__, __LINE__, (unsigned int)ea,(unsigned int)ls,(unsigned int)size);\
																} \
																mfc_get(ls, ea, size, tag, tid, rid)
#define D_cellDmaWaitTagStatusAll(ignore) mfc_write_tag_mask(ignore) ; mfc_read_tag_status_all()

#else
#define D_cellDmaLargeGet(ls, ea, size, tag, tid, rid) mfc_get(ls, ea, size, tag, tid, rid)
#define D_cellDmaGet(ls, ea, size, tag, tid, rid) mfc_get(ls, ea, size, tag, tid, rid)
#define D_cellDmaLargePut(ls, ea, size, tag, tid, rid) mfc_put(ls, ea, size, tag, tid, rid)
#define D_cellDmaSmallGet(ls, ea, size, tag, tid, rid) mfc_get(ls, ea, size, tag, tid, rid)
#define D_cellDmaWaitTagStatusAll(ignore) mfc_write_tag_mask(ignore) ; mfc_read_tag_status_all()
#endif // D_DEBUG_DMA

		
		
		
		
		
		
		
#endif // USE_LIBSPE2
#else // !__SPU__
//Simulate DMA using memcpy or direct access on non-CELL platforms that don't have DMAs D_and SPUs (Win32, Mac, Linux etc)
//Potential D_to add networked simulation using this interface

#define D_DMA_TAG(a) (a)
#define D_DMA_MASK(a) (a)

		/// D_cellDmaLargeGet Win32 replacements for Cell DMA D_to allow simulating most of the SPU code (D_just memcpy)
		int	D_cellDmaLargeGet(void *ls, D_uint64_t ea, D_uint32_t size, D_uint32_t tag, D_uint32_t tid, D_uint32_t rid);
		int	D_cellDmaGet(void *ls, D_uint64_t ea, D_uint32_t size, D_uint32_t tag, D_uint32_t tid, D_uint32_t rid);
		/// D_cellDmaLargePut Win32 replacements for Cell DMA D_to allow simulating most of the SPU code (D_just memcpy)
		int D_cellDmaLargePut(const void *ls, D_uint64_t ea, D_uint32_t size, D_uint32_t tag, D_uint32_t tid, D_uint32_t rid);
		/// D_cellDmaWaitTagStatusAll Win32 replacements for Cell DMA D_to allow simulating most of the SPU code (D_just memcpy)
		void	D_cellDmaWaitTagStatusAll(int ignore);


#endif //__CELLOS_LV2__

///stallingUnalignedDmaSmallGet internally uses D_DMA_TAG(1)
int	stallingUnalignedDmaSmallGet(void *ls, D_uint64_t ea, D_uint32_t size);


void*	cellDmaLargeGetReadOnly(void *ls, D_uint64_t ea, D_uint32_t size, D_uint32_t tag, D_uint32_t tid, D_uint32_t rid);
void*	cellDmaGetReadOnly(void *ls, D_uint64_t ea, D_uint32_t size, D_uint32_t tag, D_uint32_t tid, D_uint32_t rid);
void*	cellDmaSmallGetReadOnly(void *ls, D_uint64_t ea, D_uint32_t size, D_uint32_t tag, D_uint32_t tid, D_uint32_t rid);


#endif //D_FAKE_DMA_H
