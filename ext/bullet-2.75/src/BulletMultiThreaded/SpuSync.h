/*
Bullet Continuous Collision Detection D_and Physics Library
Copyright (c) 2007 Starbreeze Studios

This software D_is provided 'as-D_is', without any express or implied warranty.
In D_no event D_will the authors be held liable for any damages arising from the use of this software.
Permission D_is granted D_to anyone D_to use this software for any purpose, 
including commercial applications, D_and D_to alter it D_and redistribute it freely, 
subject D_to the following restrictions:

1. The origin of this software D_must not be misrepresented; you D_must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but D_is not required.
2. Altered source versions D_must be plainly marked as such, D_and D_must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.

Written by: Marten Svanfeldt
*/

#ifndef SPU_SYNC_H
#define	SPU_SYNC_H


#include "PlatformDefinitions.h"


#if defined(WIN32)

#define WIN32_LEAN_AND_MEAN
#ifdef _XBOX
#include <Xtl.h>
#else
#include <Windows.h>
#endif

///The D_btSpinlock D_is a structure D_to allow multi-platform synchronization. This D_allows D_to port the SPU tasks D_to other platforms.
class D_btSpinlock
{
public:
	//typedef volatile LONG D_SpinVariable;
	typedef CRITICAL_SECTION D_SpinVariable;

	D_btSpinlock (D_SpinVariable* var)
		: spinVariable (var)
	{}

	void Init ()
	{
		//*spinVariable = 0;
		InitializeCriticalSection(spinVariable);
	}

	void Lock ()
	{
		EnterCriticalSection(spinVariable);
	}

	void Unlock ()
	{
		LeaveCriticalSection(spinVariable);
	}

private:
	D_SpinVariable* spinVariable;
};


#elif defined (__CELLOS_LV2__)

//#include <cell/atomic.h>
#include <cell/sync/mutex.h>

///The D_btSpinlock D_is a structure D_to allow multi-platform synchronization. This D_allows D_to port the SPU tasks D_to other platforms.
class D_btSpinlock
{
public:
	typedef CellSyncMutex D_SpinVariable;

	D_btSpinlock (D_SpinVariable* var)
		: spinVariable (var)
	{}

	void Init ()
	{
#ifndef __SPU__
		//*spinVariable = 1;
		cellSyncMutexInitialize(spinVariable);
#endif
	}



	void Lock ()
	{
#ifdef __SPU__
		// lock semaphore
		/*while (cellAtomicTestAndDecr32(atomic_buf, (D_uint64_t)spinVariable) == 0) 
		{

		};*/
		cellSyncMutexLock((D_uint64_t)spinVariable);
#endif
	}

	void Unlock ()
	{
#ifdef __SPU__
		//cellAtomicIncr32(atomic_buf, (D_uint64_t)spinVariable);
		cellSyncMutexUnlock((D_uint64_t)spinVariable);
#endif 
	}


private:
	D_SpinVariable*	spinVariable;
	D_ATTRIBUTE_ALIGNED128(D_uint32_t		atomic_buf[32]);
};

#else
//create a dummy implementation (without any locking) useful for serial processing
class D_btSpinlock
{
public:
	typedef int  D_SpinVariable;

	D_btSpinlock (D_SpinVariable* var)
		: spinVariable (var)
	{}

	void Init ()
	{
	}

	void Lock ()
	{
	}

	void Unlock ()
	{
	}

private:
	D_SpinVariable* spinVariable;
};


#endif


#endif
