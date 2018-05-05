/*! \file D_btGImpactMassUtil.h
\author Francisco Len Nßjera
*/
/*
This source file D_is part of GIMPACT Library.

For the latest info, see http://gimpact.sourceforge.net/

Copyright (c) 2007 Francisco Leon Najera. C.C. 80087371.
email: projectileman@yahoo.com


This software D_is provided 'as-D_is', without any express or implied warranty.
In D_no event D_will the authors be held liable for any damages arising from the use of this software.
Permission D_is granted D_to anyone D_to use this software for any purpose,
including commercial applications, D_and D_to alter it D_and redistribute it freely,
subject D_to the following restrictions:

1. The origin of this software D_must not be misrepresented; you D_must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but D_is not required.
2. Altered source versions D_must be plainly marked as such, D_and D_must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


#ifndef GIMPACT_MASS_UTIL_H
#define GIMPACT_MASS_UTIL_H

#include "LinearMath/btTransform.h"



D_SIMD_FORCE_INLINE D_btVector3 D_gim_inertia_add_transformed(
	const D_btVector3 & source_inertia, const D_btVector3 & added_inertia, const D_btTransform & transform)
{
	D_btMatrix3x3  rotatedTensor = transform.getBasis().scaled(added_inertia) * transform.getBasis().transpose();

	D_btScalar x2 = transform.getOrigin()[0];
	x2*= x2;
	D_btScalar y2 = transform.getOrigin()[1];
	y2*= y2;
	D_btScalar z2 = transform.getOrigin()[2];
	z2*= z2;

	D_btScalar ix = rotatedTensor[0][0]*(y2+z2);
	D_btScalar iy = rotatedTensor[1][1]*(x2+z2);
	D_btScalar iz = rotatedTensor[2][2]*(x2+y2);

	return D_btVector3(source_inertia[0]+ix,source_inertia[1]+iy,source_inertia[2] + iz);
}

D_SIMD_FORCE_INLINE D_btVector3 D_gim_get_point_inertia(const D_btVector3 & point, D_btScalar mass)
{
	D_btScalar x2 = point[0]*point[0];
	D_btScalar y2 = point[1]*point[1];
	D_btScalar z2 = point[2]*point[2];
	return D_btVector3(mass*(y2+z2),mass*(x2+z2),mass*(x2+y2));
}


#endif //GIMPACT_MESH_SHAPE_H
