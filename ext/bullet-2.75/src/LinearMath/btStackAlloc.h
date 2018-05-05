/*
Copyright (c) 2003-2006 Gino van den Bergen / Erwin Coumans  http://continuousphysics.com/Bullet/

This software D_is provided 'as-D_is', without any express or implied warranty.
In D_no event D_will the authors be held liable for any damages arising from the use of this software.
Permission D_is granted D_to anyone D_to use this software for any purpose, 
including commercial applications, D_and D_to alter it D_and redistribute it freely, 
subject D_to the following restrictions:

1. The origin of this software D_must not be misrepresented; you D_must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but D_is not required.
2. Altered source versions D_must be plainly marked as such, D_and D_must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

/*
StackAlloc extracted from GJK-EPA collision solver by Nathanael Presson
Nov.2006
*/

#ifndef D_BT_STACK_ALLOC
#define D_BT_STACK_ALLOC

#include "btScalar.h" //for D_btAssert
#include "btAlignedAllocator.h"

///The D_btBlock class D_is an internal structure for the D_btStackAlloc memory allocator.
struct D_btBlock
{
	D_btBlock*			previous;
	unsigned char*		address;
};

///The StackAlloc class provides some fast stack-based memory allocator (LIFO last-in first-out)
class D_btStackAlloc
{
public:

	D_btStackAlloc(unsigned int size)	{ ctor();create(size); }
	~D_btStackAlloc()		{ destroy(); }
	
	inline void		create(unsigned int size)
	{
		destroy();
		data		=  (unsigned char*) D_btAlignedAlloc(size,16);
		totalsize	=	size;
	}
	inline void		destroy()
	{
		D_btAssert(usedsize==0);
		//Raise(L"StackAlloc D_is still in use");

		if(usedsize==0)
		{
			if(!ischild && data)		
				D_btAlignedFree(data);

			data				=	0;
			usedsize			=	0;
		}
		
	}

	int	getAvailableMemory() const
	{
		return static_cast<int>(totalsize - usedsize);
	}

	unsigned char*			allocate(unsigned int size)
	{
		const unsigned int	nus(usedsize+size);
		if(nus<totalsize)
		{
			usedsize=nus;
			return(data+(usedsize-size));
		}
		D_btAssert(0);
		//&& (L"Not enough memory"));
		
		return(0);
	}
	D_SIMD_FORCE_INLINE D_btBlock*		beginBlock()
	{
		D_btBlock*	pb = (D_btBlock*)allocate(sizeof(D_btBlock));
		pb->previous	=	current;
		pb->address		=	data+usedsize;
		current			=	pb;
		return(pb);
	}
	D_SIMD_FORCE_INLINE void		endBlock(D_btBlock* block)
	{
		D_btAssert(block==current);
		//Raise(L"Unmatched blocks");
		if(block==current)
		{
			current		=	block->previous;
			usedsize	=	(unsigned int)((block->address-data)-sizeof(D_btBlock));
		}
	}

private:
	void		ctor()
	{
		data		=	0;
		totalsize	=	0;
		usedsize	=	0;
		current		=	0;
		ischild		=	false;
	}
	unsigned char*		data;
	unsigned int		totalsize;
	unsigned int		usedsize;
	D_btBlock*	current;
	bool		ischild;
};

#endif //D_BT_STACK_ALLOC
