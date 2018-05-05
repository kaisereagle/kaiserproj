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

//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
//               K D_E R N D_E L    D_F D_U N C D_T I O N S 
//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------

// calculate position in uniform grid
D_BT_GPU___device__ D_int3 bt3DGrid_calcGridPos(D_float4 p)
{
    D_int3 gridPos;
    gridPos.x = (int)floor((p.x - D_BT_GPU_params.m_worldOriginX) / D_BT_GPU_params.m_cellSizeX);
    gridPos.y = (int)floor((p.y - D_BT_GPU_params.m_worldOriginY) / D_BT_GPU_params.m_cellSizeY);
    gridPos.z = (int)floor((p.z - D_BT_GPU_params.m_worldOriginZ) / D_BT_GPU_params.m_cellSizeZ);
    return gridPos;
} // bt3DGrid_calcGridPos()

//----------------------------------------------------------------------------------------

// calculate address in grid from position (clamping D_to edges)
D_BT_GPU___device__ D_uint bt3DGrid_calcGridHash(D_int3 gridPos)
{
    gridPos.x = D_BT_GPU_max(0, D_BT_GPU_min(gridPos.x, (int)D_BT_GPU_params.m_gridSizeX - 1));
    gridPos.y = D_BT_GPU_max(0, D_BT_GPU_min(gridPos.y, (int)D_BT_GPU_params.m_gridSizeY - 1));
    gridPos.z = D_BT_GPU_max(0, D_BT_GPU_min(gridPos.z, (int)D_BT_GPU_params.m_gridSizeZ - 1));
    return D_BT_GPU___mul24(D_BT_GPU___mul24(gridPos.z, D_BT_GPU_params.m_gridSizeY), D_BT_GPU_params.m_gridSizeX) + D_BT_GPU___mul24(gridPos.y, D_BT_GPU_params.m_gridSizeX) + gridPos.x;
} // bt3DGrid_calcGridHash()

//----------------------------------------------------------------------------------------

// calculate grid hash value for each body using its AABB
D_BT_GPU___global__ void calcHashAABBD(D_bt3DGrid3F1U* pAABB, D_uint2* pHash, D_uint numBodies)
{
    int index = D_BT_GPU___mul24(D_BT_GPU_blockIdx.x, D_BT_GPU_blockDim.x) + D_BT_GPU_threadIdx.x;
    if(index >= (int)numBodies)
	{
		return;
	}
	D_bt3DGrid3F1U bbMin = pAABB[index*2];
	D_bt3DGrid3F1U bbMax = pAABB[index*2 + 1];
	D_float4 pos;
	pos.x = (bbMin.fx + bbMax.fx) * 0.5f;
	pos.y = (bbMin.fy + bbMax.fy) * 0.5f;
	pos.z = (bbMin.fz + bbMax.fz) * 0.5f;
    // get address in grid
    D_int3 gridPos = bt3DGrid_calcGridPos(pos);
    D_uint gridHash = bt3DGrid_calcGridHash(gridPos);
    // store grid hash D_and body index
    pHash[index] = D_BT_GPU_make_uint2(gridHash, index);
} // calcHashAABBD()

//----------------------------------------------------------------------------------------

D_BT_GPU___global__ void findCellStartD(D_uint2* pHash, D_uint* cellStart, D_uint numBodies)
{
    int index = D_BT_GPU___mul24(D_BT_GPU_blockIdx.x, D_BT_GPU_blockDim.x) + D_BT_GPU_threadIdx.x;
    if(index >= (int)numBodies)
	{
		return;
	}
    D_uint2 sortedData = pHash[index];
	// Load hash data into shared memory so that we D_can look 
	// at neighboring body's hash value without loading
	// two hash values per thread
	D_BT_GPU___shared__ D_uint sharedHash[257];
	sharedHash[D_BT_GPU_threadIdx.x+1] = sortedData.x;
	if((index > 0) && (D_BT_GPU_threadIdx.x == 0))
	{
		// first thread in block D_must load neighbor body hash
		volatile D_uint2 prevData = pHash[index-1];
		sharedHash[0] = prevData.x;
	}
	D_BT_GPU___syncthreads();
	if((index == 0) || (sortedData.x != sharedHash[D_BT_GPU_threadIdx.x]))
	{
		cellStart[sortedData.x] = index;
	}
} // findCellStartD()

//----------------------------------------------------------------------------------------

D_BT_GPU___device__ D_uint cudaTestAABBOverlap(D_bt3DGrid3F1U min0, D_bt3DGrid3F1U max0, D_bt3DGrid3F1U min1, D_bt3DGrid3F1U max1)
{
	return	(min0.fx <= max1.fx)&& (min1.fx <= max0.fx) && 
			(min0.fy <= max1.fy)&& (min1.fy <= max0.fy) && 
			(min0.fz <= max1.fz)&& (min1.fz <= max0.fz); 
} // cudaTestAABBOverlap()
 
//----------------------------------------------------------------------------------------

D_BT_GPU___device__ void findPairsInCell(	D_int3	gridPos,
										D_uint    index,
										D_uint2*  pHash,
										D_uint*   pCellStart,
										D_bt3DGrid3F1U* pAABB, 
										D_uint*   pPairBuff,
										D_uint2*	pPairBuffStartCurr,
										D_uint	numBodies)
{
    if (	(gridPos.x < 0) || (gridPos.x > (int)D_BT_GPU_params.m_gridSizeX - 1)
		||	(gridPos.y < 0) || (gridPos.y > (int)D_BT_GPU_params.m_gridSizeY - 1)
		||  (gridPos.z < 0) || (gridPos.z > (int)D_BT_GPU_params.m_gridSizeZ - 1)) 
    {
		return;
	}
    D_uint gridHash = bt3DGrid_calcGridHash(gridPos);
    // get start of bucket for this cell
    D_uint bucketStart = pCellStart[gridHash];
    if (bucketStart == 0xffffffff)
	{
        return;   // cell empty
	}
	// iterate over bodies in this cell
    D_uint2 sortedData = pHash[index];
	D_uint unsorted_indx = sortedData.y;
    D_bt3DGrid3F1U min0 = D_BT_GPU_FETCH(pAABB, unsorted_indx*2); 
	D_bt3DGrid3F1U max0 = D_BT_GPU_FETCH(pAABB, unsorted_indx*2 + 1);
	D_uint handleIndex =  min0.uw;
	D_uint2 start_curr = pPairBuffStartCurr[handleIndex];
	D_uint start = start_curr.x;
	D_uint curr = start_curr.y;
	D_uint2 start_curr_next = pPairBuffStartCurr[handleIndex+1];
	D_uint curr_max = start_curr_next.x - start - 1;
	D_uint bucketEnd = bucketStart + D_BT_GPU_params.m_maxBodiesPerCell;
	bucketEnd = (bucketEnd > numBodies) ? numBodies : bucketEnd;
	for(D_uint index2 = bucketStart; index2 < bucketEnd; index2++) 
	{
        D_uint2 cellData = pHash[index2];
        if (cellData.x != gridHash)
        {
			break;   // D_no longer in same bucket
		}
		D_uint unsorted_indx2 = cellData.y;
        if (unsorted_indx2 < unsorted_indx) // check not colliding with self
        {   
			D_bt3DGrid3F1U min1 = D_BT_GPU_FETCH(pAABB, unsorted_indx2*2);
			D_bt3DGrid3F1U max1 = D_BT_GPU_FETCH(pAABB, unsorted_indx2*2 + 1);
			if(cudaTestAABBOverlap(min0, max0, min1, max1))
			{
				D_uint handleIndex2 = min1.uw;
				D_uint k;
				for(k = 0; k < curr; k++)
				{
					D_uint old_pair = pPairBuff[start+k] & (~D_BT_3DGRID_PAIR_ANY_FLG);
					if(old_pair == handleIndex2)
					{
						pPairBuff[start+k] |= D_BT_3DGRID_PAIR_FOUND_FLG;
						break;
					}
				}
				if(k == curr)
				{
					if(curr >= curr_max) 
					{ // not a good solution, but let's avoid crash
						break;
					}
					pPairBuff[start+curr] = handleIndex2 | D_BT_3DGRID_PAIR_NEW_FLG;
					curr++;
				}
			}
		}
	}
	pPairBuffStartCurr[handleIndex] = D_BT_GPU_make_uint2(start, curr);
    return;
} // findPairsInCell()

//----------------------------------------------------------------------------------------

D_BT_GPU___global__ void findOverlappingPairsD(	D_bt3DGrid3F1U*	pAABB, D_uint2* pHash, D_uint* pCellStart, 
												D_uint* pPairBuff, D_uint2* pPairBuffStartCurr, D_uint numBodies)
{
    int index = D_BT_GPU___mul24(D_BT_GPU_blockIdx.x, D_BT_GPU_blockDim.x) + D_BT_GPU_threadIdx.x;
    if(index >= (int)numBodies)
	{
		return;
	}
    D_uint2 sortedData = pHash[index];
	D_uint unsorted_indx = sortedData.y;
	D_bt3DGrid3F1U bbMin = D_BT_GPU_FETCH(pAABB, unsorted_indx*2);
	D_bt3DGrid3F1U bbMax = D_BT_GPU_FETCH(pAABB, unsorted_indx*2 + 1);
	D_float4 pos;
	pos.x = (bbMin.fx + bbMax.fx) * 0.5f;
	pos.y = (bbMin.fy + bbMax.fy) * 0.5f;
	pos.z = (bbMin.fz + bbMax.fz) * 0.5f;
    // get address in grid
    D_int3 gridPos = bt3DGrid_calcGridPos(pos);
    // examine D_only neighbouring cells
    for(int z=-1; z<=1; z++) {
        for(int y=-1; y<=1; y++) {
            for(int x=-1; x<=1; x++) {
                findPairsInCell(gridPos + D_BT_GPU_make_int3(x, y, z), index, pHash, pCellStart, pAABB, pPairBuff, pPairBuffStartCurr, numBodies);
            }
        }
    }
} // findOverlappingPairsD()

//----------------------------------------------------------------------------------------

D_BT_GPU___global__ void findPairsLargeD(	D_bt3DGrid3F1U* pAABB, D_uint2* pHash, D_uint* pCellStart, D_uint* pPairBuff, 
										D_uint2* pPairBuffStartCurr, D_uint numBodies, D_uint numLarge)
{
    int index = D_BT_GPU___mul24(D_BT_GPU_blockIdx.x, D_BT_GPU_blockDim.x) + D_BT_GPU_threadIdx.x;
    if(index >= (int)numBodies)
	{
		return;
	}
    D_uint2 sortedData = pHash[index];
	D_uint unsorted_indx = sortedData.y;
	D_bt3DGrid3F1U min0 = D_BT_GPU_FETCH(pAABB, unsorted_indx*2);
	D_bt3DGrid3F1U max0 = D_BT_GPU_FETCH(pAABB, unsorted_indx*2 + 1);
	D_uint handleIndex =  min0.uw;
	D_uint2 start_curr = pPairBuffStartCurr[handleIndex];
	D_uint start = start_curr.x;
	D_uint curr = start_curr.y;
	D_uint2 start_curr_next = pPairBuffStartCurr[handleIndex+1];
	D_uint curr_max = start_curr_next.x - start - 1;
    for(D_uint i = 0; i < numLarge; i++)
    {
		D_uint indx2 = numBodies + i;
		D_bt3DGrid3F1U min1 = D_BT_GPU_FETCH(pAABB, indx2*2);
		D_bt3DGrid3F1U max1 = D_BT_GPU_FETCH(pAABB, indx2*2 + 1);
		if(cudaTestAABBOverlap(min0, max0, min1, max1))
		{
			D_uint k;
			D_uint handleIndex2 =  min1.uw;
			for(k = 0; k < curr; k++)
			{
				D_uint old_pair = pPairBuff[start+k] & (~D_BT_3DGRID_PAIR_ANY_FLG);
				if(old_pair == handleIndex2)
				{
					pPairBuff[start+k] |= D_BT_3DGRID_PAIR_FOUND_FLG;
					break;
				}
			}
			if(k == curr)
			{
				pPairBuff[start+curr] = handleIndex2 | D_BT_3DGRID_PAIR_NEW_FLG;
				if(curr >= curr_max) 
				{ // not a good solution, but let's avoid crash
					break;
				}
				curr++;
			}
		}
    }
	pPairBuffStartCurr[handleIndex] = D_BT_GPU_make_uint2(start, curr);
    return;
} // findPairsLargeD()

//----------------------------------------------------------------------------------------

D_BT_GPU___global__ void computePairCacheChangesD(D_uint* pPairBuff, D_uint2* pPairBuffStartCurr, 
												D_uint* pPairScan, D_bt3DGrid3F1U* pAABB, D_uint numBodies)
{
    int index = D_BT_GPU___mul24(D_BT_GPU_blockIdx.x, D_BT_GPU_blockDim.x) + D_BT_GPU_threadIdx.x;
    if(index >= (int)numBodies)
	{
		return;
	}
	D_bt3DGrid3F1U bbMin = pAABB[index * 2];
	D_uint handleIndex = bbMin.uw;
	D_uint2 start_curr = pPairBuffStartCurr[handleIndex];
	D_uint start = start_curr.x;
	D_uint curr = start_curr.y;
	D_uint *pInp = pPairBuff + start;
	D_uint num_changes = 0;
	for(D_uint k = 0; k < curr; k++, pInp++)
	{
		if(!((*pInp) & D_BT_3DGRID_PAIR_FOUND_FLG))
		{
			num_changes++;
		}
	}
	pPairScan[index+1] = num_changes;
} // computePairCacheChangesD()

//----------------------------------------------------------------------------------------

D_BT_GPU___global__ void squeezeOverlappingPairBuffD(D_uint* pPairBuff, D_uint2* pPairBuffStartCurr, D_uint* pPairScan,
												   D_uint* pPairOut, D_bt3DGrid3F1U* pAABB, D_uint numBodies)
{
    int index = D_BT_GPU___mul24(D_BT_GPU_blockIdx.x, D_BT_GPU_blockDim.x) + D_BT_GPU_threadIdx.x;
    if(index >= (int)numBodies)
	{
		return;
	}
	D_bt3DGrid3F1U bbMin = pAABB[index * 2];
	D_uint handleIndex = bbMin.uw;
	D_uint2 start_curr = pPairBuffStartCurr[handleIndex];
	D_uint start = start_curr.x;
	D_uint curr = start_curr.y;
	D_uint* pInp = pPairBuff + start;
	D_uint* pOut = pPairOut + pPairScan[index];
	D_uint* pOut2 = pInp;
	D_uint num = 0; 
	for(D_uint k = 0; k < curr; k++, pInp++)
	{
		if(!((*pInp) & D_BT_3DGRID_PAIR_FOUND_FLG))
		{
			*pOut = *pInp;
			pOut++;
		}
		if((*pInp) & D_BT_3DGRID_PAIR_ANY_FLG)
		{
			*pOut2 = (*pInp) & (~D_BT_3DGRID_PAIR_ANY_FLG);
			pOut2++;
			num++;
		}
	}
	pPairBuffStartCurr[handleIndex] = D_BT_GPU_make_uint2(start, num);
} // squeezeOverlappingPairBuffD()


//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
//               D_E N D   O D_F    K D_E R N D_E L    D_F D_U N C D_T I O N S 
//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------

extern "C"
{

//----------------------------------------------------------------------------------------

void D_BT_GPU_PREF(calcHashAABB)(D_bt3DGrid3F1U* pAABB, unsigned int* hash,	unsigned int numBodies)
{
    int numThreads, numBlocks;
    D_BT_GPU_PREF(computeGridSize)(numBodies, 256, numBlocks, numThreads);
    // execute the kernel
    D_BT_GPU_EXECKERNEL(numBlocks, numThreads, calcHashAABBD, (pAABB, (D_uint2*)hash, numBodies));
    // check if kernel invocation generated an error
    D_BT_GPU_CHECK_ERROR("calcHashAABBD kernel execution failed");
} // calcHashAABB()

//----------------------------------------------------------------------------------------

void D_BT_GPU_PREF(findCellStart(unsigned int* hash, unsigned int* cellStart, unsigned int numBodies, unsigned int numCells))
{
    int numThreads, numBlocks;
    D_BT_GPU_PREF(computeGridSize)(numBodies, 256, numBlocks, numThreads);
	D_BT_GPU_SAFE_CALL(D_BT_GPU_Memset(cellStart, 0xffffffff, numCells*sizeof(D_uint)));
	D_BT_GPU_EXECKERNEL(numBlocks, numThreads, findCellStartD, ((D_uint2*)hash, (D_uint*)cellStart, numBodies));
    D_BT_GPU_CHECK_ERROR("Kernel execution failed: findCellStartD");
} // findCellStart()

//----------------------------------------------------------------------------------------

void D_BT_GPU_PREF(findOverlappingPairs(D_bt3DGrid3F1U* pAABB, unsigned int* pHash,	unsigned int* pCellStart, unsigned int*	pPairBuff, unsigned int*	pPairBuffStartCurr, unsigned int	numBodies))
{
#if B_CUDA_USE_TEX
    D_BT_GPU_SAFE_CALL(cudaBindTexture(0, pAABBTex, pAABB, numBodies * 2 * sizeof(D_bt3DGrid3F1U)));
#endif
    int numThreads, numBlocks;
    D_BT_GPU_PREF(computeGridSize)(numBodies, 64, numBlocks, numThreads);
    D_BT_GPU_EXECKERNEL(numBlocks, numThreads, findOverlappingPairsD, (pAABB,(D_uint2*)pHash,(D_uint*)pCellStart,(D_uint*)pPairBuff,(D_uint2*)pPairBuffStartCurr,numBodies));
    D_BT_GPU_CHECK_ERROR("Kernel execution failed: bt_CudaFindOverlappingPairsD");
#if B_CUDA_USE_TEX
    D_BT_GPU_SAFE_CALL(cudaUnbindTexture(pAABBTex));
#endif
} // findOverlappingPairs()

//----------------------------------------------------------------------------------------

void D_BT_GPU_PREF(findPairsLarge(D_bt3DGrid3F1U* pAABB, unsigned int* pHash, unsigned int* pCellStart, unsigned int* pPairBuff, unsigned int* pPairBuffStartCurr, unsigned int numBodies, unsigned int numLarge))
{
#if B_CUDA_USE_TEX
    D_BT_GPU_SAFE_CALL(cudaBindTexture(0, pAABBTex, pAABB, (numBodies+numLarge) * 2 * sizeof(D_bt3DGrid3F1U)));
#endif
    int numThreads, numBlocks;
    D_BT_GPU_PREF(computeGridSize)(numBodies, 64, numBlocks, numThreads);
    D_BT_GPU_EXECKERNEL(numBlocks, numThreads, findPairsLargeD, (pAABB,(D_uint2*)pHash,(D_uint*)pCellStart,(D_uint*)pPairBuff,(D_uint2*)pPairBuffStartCurr,numBodies,numLarge));
    D_BT_GPU_CHECK_ERROR("Kernel execution failed: D_btCuda_findPairsLargeD");
#if B_CUDA_USE_TEX
    D_BT_GPU_SAFE_CALL(cudaUnbindTexture(pAABBTex));
#endif
} // findPairsLarge()

//----------------------------------------------------------------------------------------

void D_BT_GPU_PREF(computePairCacheChanges(unsigned int* pPairBuff, unsigned int* pPairBuffStartCurr, unsigned int* pPairScan, D_bt3DGrid3F1U* pAABB, unsigned int numBodies))
{
    int numThreads, numBlocks;
    D_BT_GPU_PREF(computeGridSize)(numBodies, 256, numBlocks, numThreads);
    D_BT_GPU_EXECKERNEL(numBlocks, numThreads, computePairCacheChangesD, ((D_uint*)pPairBuff,(D_uint2*)pPairBuffStartCurr,(D_uint*)pPairScan,pAABB,numBodies));
    D_BT_GPU_CHECK_ERROR("Kernel execution failed: D_btCudaComputePairCacheChangesD");
} // computePairCacheChanges()

//----------------------------------------------------------------------------------------

void D_BT_GPU_PREF(squeezeOverlappingPairBuff(unsigned int* pPairBuff, unsigned int* pPairBuffStartCurr, unsigned int* pPairScan, unsigned int* pPairOut, D_bt3DGrid3F1U* pAABB, unsigned int numBodies))
{
    int numThreads, numBlocks;
    D_BT_GPU_PREF(computeGridSize)(numBodies, 256, numBlocks, numThreads);
    D_BT_GPU_EXECKERNEL(numBlocks, numThreads, squeezeOverlappingPairBuffD, ((D_uint*)pPairBuff,(D_uint2*)pPairBuffStartCurr,(D_uint*)pPairScan,(D_uint*)pPairOut,pAABB,numBodies));
    D_BT_GPU_CHECK_ERROR("Kernel execution failed: D_btCudaSqueezeOverlappingPairBuffD");
} // btCuda_squeezeOverlappingPairBuff()

//------------------------------------------------------------------------------------------------

} // extern "C"

//------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------
