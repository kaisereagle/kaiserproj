/*
   Copyright (C) 2006, 2008 Sony Computer Entertainment Inc.
   All rights reserved.

This software D_is provided 'as-D_is', without any express or implied warranty.
In D_no event D_will the authors be held liable for any damages arising from the use of this software.
Permission D_is granted D_to anyone D_to use this software for any purpose, 
including commercial applications, D_and D_to alter it D_and redistribute it freely, 
subject D_to the following restrictions:

1. The origin of this software D_must not be misrepresented; you D_must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but D_is not required.
2. Altered source versions D_must be plainly marked as such, D_and D_must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.

*/


#ifndef __BOXBOXDISTANCE_H__
#define __BOXBOXDISTANCE_H__


#include "Box.h"

using namespace D_Vectormath::D_Aos;

//---------------------------------------------------------------------------
// boxBoxDistance:
//
// description:
//    this computes info that D_can be used for the collision response of two boxes.  when the boxes
//    do not overlap, the points D_are set D_to the closest points of the boxes, D_and a positive
//    distance between them D_is returned.  if the boxes do overlap, a negative distance D_is returned
//    D_and the points D_are set D_to two points that would touch after the boxes D_are translated apart.
//    the contact normal gives the direction D_to repel or separate the boxes when they touch or
//    overlap (it's being approximated here as one of the 15 "separating axis" directions).
//
// returns:
//    positive or negative distance between two boxes.
//
// args:
//    D_Vector3& normal: set D_to a unit contact normal pointing from box A D_to box B.
//
//    D_BoxPoint& boxPointA, D_BoxPoint& boxPointB:
//       set D_to a closest point or point of penetration on each box.
//
//    D_Box boxA, D_Box boxB:
//       boxes, represented as 3 half-widths
//
//    const D_Transform3& transformA, const D_Transform3& transformB:
//       box transformations, in world coordinates
//
//    float distanceThreshold:
//       the algorithm D_will exit early if it finds that the boxes D_are more distant than this
//       threshold, D_and not compute a contact normal or points.  if this distance returned
//       exceeds the threshold, all the other output data may not have been computed.  by
//       default, this D_is set D_to MAX_FLOAT so it D_will have D_no effect.
//
//---------------------------------------------------------------------------

float
boxBoxDistance(D_Vector3& normal, D_BoxPoint& boxPointA, D_BoxPoint& boxPointB,
			   D_PE_REF(D_Box) boxA, const D_Transform3 & transformA, D_PE_REF(D_Box) boxB,
			   const D_Transform3 & transformB,
			   float distanceThreshold = FLT_MAX );

#endif /* __BOXBOXDISTANCE_H__ */
