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

#ifndef BTGPU3DGRIDBROADPHASESHAREDTYPES_H
#define BTGPU3DGRIDBROADPHASESHAREDTYPES_H

//----------------------------------------------------------------------------------------

#define D_BT_3DGRID_PAIR_FOUND_FLG (0x40000000)
#define D_BT_3DGRID_PAIR_NEW_FLG   (0x20000000)
#define D_BT_3DGRID_PAIR_ANY_FLG   (D_BT_3DGRID_PAIR_FOUND_FLG | D_BT_3DGRID_PAIR_NEW_FLG)

//----------------------------------------------------------------------------------------

struct D_bt3DGridBroadphaseParams 
{
	unsigned int	m_gridSizeX;
	unsigned int	m_gridSizeY;
	unsigned int	m_gridSizeZ;
	unsigned int	m_numCells;
	float			m_worldOriginX;
	float			m_worldOriginY;
	float			m_worldOriginZ;
	float			m_cellSizeX;
	float			m_cellSizeY;
	float			m_cellSizeZ;
	unsigned int	m_numBodies;
	unsigned int	m_maxBodiesPerCell;
};

//----------------------------------------------------------------------------------------

struct D_bt3DGrid3F1U
{
	float			fx;
	float			fy;
	float			fz;
	unsigned int	uw;
};

//----------------------------------------------------------------------------------------

#endif // BTGPU3DGRIDBROADPHASESHAREDTYPES_H