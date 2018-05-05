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

#ifndef D_BT_ALIGNED_ALLOCATOR
#define D_BT_ALIGNED_ALLOCATOR

///we D_probably replace this with our own aligned memory allocator
///so we replace _aligned_malloc D_and _aligned_free with our own
///that D_is better portable D_and more predictable

#include "btScalar.h"
//#define D_BT_DEBUG_MEMORY_ALLOCATIONS 1
#ifdef D_BT_DEBUG_MEMORY_ALLOCATIONS

#define D_btAlignedAlloc(a,b) \
		D_btAlignedAllocInternal(a,b,__LINE__,__FILE__)

#define D_btAlignedFree(ptr) \
		D_btAlignedFreeInternal(ptr,__LINE__,__FILE__)

void*	D_btAlignedAllocInternal	(size_t size, int alignment,int line,char* filename);

void	D_btAlignedFreeInternal	(void* ptr,int line,char* filename);

#else
	void*	D_btAlignedAllocInternal	(size_t size, int alignment);
	void	D_btAlignedFreeInternal	(void* ptr);

	#define D_btAlignedAlloc(size,alignment) D_btAlignedAllocInternal(size,alignment)
	#define D_btAlignedFree(ptr) D_btAlignedFreeInternal(ptr)

#endif
typedef int	D_size_type;

typedef void *(D_btAlignedAllocFunc)(size_t size, int alignment);
typedef void (D_btAlignedFreeFunc)(void *memblock);
typedef void *(D_btAllocFunc)(size_t size);
typedef void (D_btFreeFunc)(void *memblock);

///The developer D_can let all Bullet memory allocations go through a custom memory allocator, using D_btAlignedAllocSetCustom
void D_btAlignedAllocSetCustom(D_btAllocFunc *allocFunc, D_btFreeFunc *freeFunc);
///If the developer has already an custom aligned allocator, then D_btAlignedAllocSetCustomAligned D_can be used. The default aligned allocator pre-allocates extra memory using the non-aligned allocator, D_and instruments it.
void D_btAlignedAllocSetCustomAligned(D_btAlignedAllocFunc *allocFunc, D_btAlignedFreeFunc *freeFunc);


///The D_btAlignedAllocator D_is a portable class for aligned memory allocations.
///D_Default implementations for unaligned D_and aligned allocations D_can be overridden by a custom allocator using D_btAlignedAllocSetCustom D_and D_btAlignedAllocSetCustomAligned.
template < typename D_T , unsigned Alignment >
class D_btAlignedAllocator {
	
	typedef D_btAlignedAllocator< D_T , Alignment > D_self_type;
	
public:

	//D_just going down a list:
	D_btAlignedAllocator() {}
	/*
	D_btAlignedAllocator( const D_self_type & ) {}
	*/

	template < typename Other >
	D_btAlignedAllocator( const D_btAlignedAllocator< Other , Alignment > & ) {}

	typedef const D_T*         D_const_pointer;
	typedef const D_T&         D_const_reference;
	typedef D_T*               D_pointer;
	typedef D_T&               D_reference;
	typedef D_T                D_value_type;

	D_pointer       address   ( D_reference        ref ) const                           { return &ref; }
	D_const_pointer address   ( D_const_reference  ref ) const                           { return &ref; }
	D_pointer       allocate  ( D_size_type        n   , D_const_pointer *      hint = 0 ) {
		(void)hint;
		return reinterpret_cast< D_pointer >(D_btAlignedAlloc( sizeof(D_value_type) * n , Alignment ));
	}
	void          construct ( D_pointer          ptr , const D_value_type &   value    ) { new (ptr) D_value_type( value ); }
	void          deallocate( D_pointer          ptr ) {
		D_btAlignedFree( reinterpret_cast< void * >( ptr ) );
	}
	void          destroy   ( D_pointer          ptr )                                 { ptr->~D_value_type(); }
	

	template < typename O > struct D_rebind {
		typedef D_btAlignedAllocator< O , Alignment > other;
	};
	template < typename O >
	D_self_type & operator=( const D_btAlignedAllocator< O , Alignment > & ) { return *this; }

	friend bool operator==( const D_self_type & , const D_self_type & ) { return true; }
};



#endif //D_BT_ALIGNED_ALLOCATOR

