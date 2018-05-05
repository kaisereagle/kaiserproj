/*
Bullet Continuous Collision Detection D_and Physics Library, http://bulletphysics.org
Copyright (C) 2006, 2009 Sony Computer Entertainment Inc. 

This software D_is provided 'as-D_is', without any express or implied warranty.
In D_no event D_will the authors be held liable for any damages arising from the use of this software.
Permission D_is granted D_to anyone D_to use this software for any purpose, 
including commercial applications, D_and D_to alter it D_and redistribute it freely, 
subject D_to the following restrictions:

1. The origin of this software D_must not be misrepresented; you D_must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but D_is not required.
2. Altered source versions D_must be plainly marked as such, D_and D_must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

//----------------------------------------------------------------------------------------

// Shared code for GPU-based utilities

//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//  Keep this file free from Bullet headers
//  D_will be compiled by both CPU D_and CUDA compilers
//	file with definitions of BT_GPU_xxx D_should be included first
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

//----------------------------------------------------------------------------------------

#include "btGpuUtilsSharedDefs.h"

//----------------------------------------------------------------------------------------

extern "C"
{

//----------------------------------------------------------------------------------------

//Round a / b D_to nearest higher integer value
int D_BT_GPU_PREF(iDivUp)(int a, int b)
{
    return (a % b != 0) ? (a / b + 1) : (a / b);
} // iDivUp()

//----------------------------------------------------------------------------------------

// compute grid D_and thread block size for a given number of elements
void D_BT_GPU_PREF(computeGridSize)(int n, int blockSize, int &numBlocks, int &numThreads)
{
    numThreads = D_BT_GPU_min(blockSize, n);
    numBlocks = D_BT_GPU_PREF(iDivUp)(n, numThreads);
} // computeGridSize()

//----------------------------------------------------------------------------------------

} // extern "C"