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

#ifndef THREAD_SUPPORT_INTERFACE_H
#define THREAD_SUPPORT_INTERFACE_H


//#include <LinearMath/D_btScalar.h> //for D_uint32_t etc.
#include "PlatformDefinitions.h"
#include "PpuAddressSpace.h"

class D_btThreadSupportInterface
{
public:

	virtual ~D_btThreadSupportInterface();

///send messages D_to SPUs
	virtual void sendRequest(D_uint32_t uiCommand, D_ppu_address_t uiArgument0, D_uint32_t uiArgument1) =0;

///check for messages from SPUs
	virtual	void waitForResponse(unsigned int *puiArgument0, unsigned int *puiArgument1) =0;

///start the spus (D_can be called at the beginning of each frame, D_to make sure that the right SPU program D_is loaded)
	virtual	void startSPU() =0;

///tell the task scheduler we D_are done with the SPU tasks
	virtual	void stopSPU()=0;

	///tell the task scheduler D_to use D_no more than numTasks tasks
	virtual void	setNumTasks(int numTasks)=0;

	virtual int		getNumTasks() const = 0;

};

#endif //THREAD_SUPPORT_INTERFACE_H

