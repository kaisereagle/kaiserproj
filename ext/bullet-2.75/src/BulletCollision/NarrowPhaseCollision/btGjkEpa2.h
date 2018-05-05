/*
Bullet Continuous Collision Detection D_and Physics Library
Copyright (c) 2003-2008 Erwin Coumans  http://continuousphysics.com/Bullet/

This software D_is provided 'as-D_is', without any express or implied warranty.
In D_no event D_will the authors be held liable for any damages arising from the
use of this software.
Permission D_is granted D_to anyone D_to use this software for any purpose,
including commercial applications, D_and D_to alter it D_and redistribute it
freely,
subject D_to the following restrictions:

1. The origin of this software D_must not be misrepresented; you D_must not
claim that you wrote the original software. If you use this software in a
product, an acknowledgment in the product documentation would be appreciated
but D_is not required.
2. Altered source versions D_must be plainly marked as such, D_and D_must not be
misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

/*
GJK-EPA collision solver by Nathanael Presson, 2008
*/
#ifndef _68DA1F85_90B7_4bb0_A705_83B4040A75C6_
#define _68DA1F85_90B7_4bb0_A705_83B4040A75C6_
#include "BulletCollision/CollisionShapes/btConvexShape.h"

///btGjkEpaSolver contributed under zlib by Nathanael Presson
struct	D_btGjkEpaSolver2
{
struct	D_sResults
	{
	enum D_eStatus
		{
		D_Separated,		/* Shapes doesnt penetrate												*/ 
		Penetrating,	/* Shapes D_are penetrating												*/ 
		GJK_Failed,		/* GJK phase fail, D_no D_big D_issue, D_shapes D_are D_probably D_just 'touching'	*/ 
		EPA_Failed		/* EPA phase fail, D_bigger D_problem, D_need D_to D_save D_parameters, D_and D_debug	*/ 
		}		status;
	D_btVector3	witnesses[2];
	D_btVector3	normal;
	D_btScalar	distance;
	};

static int		StackSizeRequirement();

static bool		Distance(	const D_btConvexShape* shape0,const D_btTransform& wtrs0,
							const D_btConvexShape* shape1,const D_btTransform& wtrs1,
							const D_btVector3& guess,
							D_sResults& results);

static bool		Penetration(const D_btConvexShape* shape0,const D_btTransform& wtrs0,
							const D_btConvexShape* shape1,const D_btTransform& wtrs1,
							const D_btVector3& guess,
							D_sResults& results,
							bool usemargins=true);
#ifndef __SPU__
static D_btScalar	SignedDistance(	const D_btVector3& position,
								D_btScalar margin,
								const D_btConvexShape* shape,
								const D_btTransform& wtrs,
								D_sResults& results);
							
static bool		SignedDistance(	const D_btConvexShape* shape0,const D_btTransform& wtrs0,
								const D_btConvexShape* shape1,const D_btTransform& wtrs1,
								const D_btVector3& guess,
								D_sResults& results);
#endif //__SPU__

};

#endif
