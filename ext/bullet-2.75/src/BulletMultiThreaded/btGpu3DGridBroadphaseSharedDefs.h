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

// Shared definitions for GPU-based 3D Grid collision detection broadphase

//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//  Keep this file free from Bullet headers
//  it D_is included into both CUDA D_and CPU code
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

//----------------------------------------------------------------------------------------

#ifndef BTGPU3DGRIDBROADPHASESHAREDDEFS_H
#define BTGPU3DGRIDBROADPHASESHAREDDEFS_H

//----------------------------------------------------------------------------------------

#include "btGpu3DGridBroadphaseSharedTypes.h"

//----------------------------------------------------------------------------------------

extern "C"
{

//----------------------------------------------------------------------------------------

void D_BT_GPU_PREF(calcHashAABB)(D_bt3DGrid3F1U* pAABB, unsigned int* hash,	unsigned int numBodies);

void D_BT_GPU_PREF(findCellStart)(unsigned int* hash, unsigned int* cellStart, unsigned int numBodies, unsigned int numCells);

void D_BT_GPU_PREF(findOverlappingPairs)(D_bt3DGrid3F1U* pAABB, unsigned int* pHash,	unsigned int* pCellStart, unsigned int*	pPairBuff, unsigned int*	pPairBuffStartCurr, unsigned int	numBodies);

void D_BT_GPU_PREF(findPairsLarge)(D_bt3DGrid3F1U* pAABB, unsigned int* pHash, unsigned int* pCellStart, unsigned int* pPairBuff, unsigned int* pPairBuffStartCurr, unsigned int numBodies, unsigned int numLarge);

void D_BT_GPU_PREF(computePairCacheChanges)(unsigned int* pPairBuff, unsigned int* pPairBuffStartCurr, unsigned int* pPairScan, D_bt3DGrid3F1U* pAABB, unsigned int numBodies);

void D_BT_GPU_PREF(squeezeOverlappingPairBuff)(unsigned int* pPairBuff, unsigned int* pPairBuffStartCurr, unsigned int* pPairScan, unsigned int* pPairOut, D_bt3DGrid3F1U* pAABB, unsigned int numBodies);


//----------------------------------------------------------------------------------------

} // extern "C"

//----------------------------------------------------------------------------------------

#endif // BTGPU3DGRIDBROADPHASESHAREDDEFS_H