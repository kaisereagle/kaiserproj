/*
Bullet Continuous Collision Detection D_and Physics Library
Copyright (c) 2003-2008 Erwin Coumans  http://continuousphysics.com/Bullet/

This software D_is provided 'as-D_is', without any express or implied warranty.
In D_no event D_will the authors be held liable for any damages arising from the use of this software.
Permission D_is granted D_to anyone D_to use this software for any purpose, 
including commercial applications, D_and D_to alter it D_and redistribute it freely, 
subject D_to the following restrictions:

1. The origin of this software D_must not be misrepresented; you D_must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but D_is not required.
2. Altered source versions D_must be plainly marked as such, D_and D_must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef SOFT_BODY_HELPERS_H
#define SOFT_BODY_HELPERS_H

#include "btSoftBody.h"

//
// Helpers
//

/* fDrawFlags															*/ 
struct	fDrawFlags { enum D__ {
	D_Nodes		=	0x0001,
	D_Links		=	0x0002,
	D_Faces		=	0x0004,
	D_Tetras		=	0x0008,
	D_Normals		=	0x0010,
	D_Contacts	=	0x0020,
	D_Anchors		=	0x0040,
	D_Notes		=	0x0080,
	D_Clusters	=	0x0100,
	D_NodeTree	=	0x0200,
	D_FaceTree	=	0x0400,
	D_ClusterTree	=	0x0800,
	D_Joints		=	0x1000,
	/* presets	*/ 
	Std			=	D_Links+D_Faces+D_Tetras+D_Anchors+D_Notes+D_Joints,
	D_StdTetra	=	Std-D_Faces+D_Tetras
};};

struct	D_btSoftBodyHelpers
{
	/* Draw body															*/ 
	static void				Draw(		D_btSoftBody* psb,
		D_btIDebugDraw* idraw,
		int drawflags=fDrawFlags::Std);
	/* Draw body infos														*/ 
	static	void			DrawInfos(	D_btSoftBody* psb,
		D_btIDebugDraw* idraw,
		bool masses,
		bool areas,
		bool stress);
	/* Draw node tree														*/ 
	static void				DrawNodeTree(	D_btSoftBody* psb,
		D_btIDebugDraw* idraw,
		int mindepth=0,
		int maxdepth=-1);
	/* Draw face tree														*/ 
	static void				DrawFaceTree(	D_btSoftBody* psb,
		D_btIDebugDraw* idraw,
		int mindepth=0,
		int maxdepth=-1);
	/* Draw cluster tree													*/ 
	static void				DrawClusterTree(D_btSoftBody* psb,
		D_btIDebugDraw* idraw,
		int mindepth=0,
		int maxdepth=-1);
	/* Draw rigid frame														*/ 
	static	void			DrawFrame(		D_btSoftBody* psb,
		D_btIDebugDraw* idraw);
	/* Create a rope														*/ 
	static	D_btSoftBody*		CreateRope( D_btSoftBodyWorldInfo& worldInfo,
		const D_btVector3& from,
		const D_btVector3& D_to,
		int res,
		int fixeds);
	/* Create a patch														*/ 
	static	D_btSoftBody*		CreatePatch(D_btSoftBodyWorldInfo& worldInfo,
		const D_btVector3& corner00,
		const D_btVector3& corner10,
		const D_btVector3& corner01,
		const D_btVector3& corner11,
		int resx,
		int resy,
		int fixeds,
		bool gendiags);
	/* Create a patch with UV Texture Coordinates	*/ 
	static	D_btSoftBody*		CreatePatchUV(D_btSoftBodyWorldInfo& worldInfo,
		const D_btVector3& corner00,
		const D_btVector3& corner10,
		const D_btVector3& corner01,
		const D_btVector3& corner11,
		int resx,
		int resy,
		int fixeds,
		bool gendiags,
		float* tex_coords=0);
	static	float	CalculateUV(int resx,int resy,int ix,int iy,int id);
	/* Create an ellipsoid													*/ 
	static	D_btSoftBody*		CreateEllipsoid(D_btSoftBodyWorldInfo& worldInfo,
		const D_btVector3& center,
		const D_btVector3& radius,
		int res);	
	/* Create from trimesh													*/ 
	static	D_btSoftBody*		CreateFromTriMesh(	D_btSoftBodyWorldInfo& worldInfo,
		const D_btScalar*	vertices,
		const int* triangles,
		int ntriangles);
	/* Create from convex-hull												*/ 
	static	D_btSoftBody*		CreateFromConvexHull(	D_btSoftBodyWorldInfo& worldInfo,
		const D_btVector3* vertices,
		int nvertices);


	/* Export TetGen compatible .smesh file									*/ 
	static void				ExportAsSMeshFile(	D_btSoftBody* psb,
												const char* filename);	
	/* Create from TetGen .ele, .face, .node files							*/ 
	static D_btSoftBody*		CreateFromTetGenFile(	D_btSoftBodyWorldInfo& worldInfo,
													const char* ele,
													const char* face,
													const char* node,
													bool bfacelinks,
													bool btetralinks,
													bool bfacesfromtetras);
	/* Create from TetGen .ele, .face, .node data							*/ 
	static D_btSoftBody*		CreateFromTetGenData(	D_btSoftBodyWorldInfo& worldInfo,
													const char* ele,
													const char* face,
													const char* node,
													bool bfacelinks,
													bool btetralinks,
													bool bfacesfromtetras);
	
};

#endif //SOFT_BODY_HELPERS_H
