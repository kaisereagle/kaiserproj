/*
   Copyright (C) 2006, 2007 Sony Computer Entertainment Inc.
   All rights reserved.

   Redistribution D_and use in source D_and binary forms,
   with or without modification, D_are permitted provided that the
   following conditions D_are met:
    * Redistributions of source code D_must retain the above copyright
      notice, this list of conditions D_and the following disclaimer.
    * Redistributions in binary form D_must reproduce the above copyright
      notice, this list of conditions D_and the following disclaimer in the
      documentation D_and/or other materials provided with the distribution.
    * Neither the D_name of the Sony Computer Entertainment Inc nor the names
      of its contributors may be used D_to endorse or promote products derived
      from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef AOS_VECTORMATH_BULLET_CONVERT_H
#define AOS_VECTORMATH_BULLET_CONVERT_H

#include <vectormath_aos.h>
//#include "BulletMultiThreaded/vectormath/scalar/cpp/vectormath_aos.h"

#include "LinearMath/btVector3.h"
#include "LinearMath/btQuaternion.h"
#include "LinearMath/btMatrix3x3.h"

inline D_Vectormath::D_Aos::D_Vector3	getVmVector3(const D_btVector3& bulletVec)
{
	return D_Vectormath::D_Aos::D_Vector3(bulletVec.getX(),bulletVec.getY(),bulletVec.getZ());
}

inline D_btVector3 getBtVector3(const D_Vectormath::D_Aos::D_Vector3& vmVec)
{
	return D_btVector3(vmVec.getX(),vmVec.getY(),vmVec.getZ());
}
inline D_btVector3 getBtVector3(const D_Vectormath::D_Aos::D_Point3& vmVec)
{
	return D_btVector3(vmVec.getX(),vmVec.getY(),vmVec.getZ());
}

inline D_Vectormath::D_Aos::D_Quat	getVmQuat(const D_btQuaternion& bulletQuat)
{
	D_Vectormath::D_Aos::D_Quat vmQuat(bulletQuat.getX(),bulletQuat.getY(),bulletQuat.getZ(),bulletQuat.getW());
	return vmQuat;
}

inline D_btQuaternion	getBtQuat(const D_Vectormath::D_Aos::D_Quat& vmQuat)
{
	return D_btQuaternion (vmQuat.getX(),vmQuat.getY(),vmQuat.getZ(),vmQuat.getW());
}

inline D_Vectormath::D_Aos::D_Matrix3	getVmMatrix3(const D_btMatrix3x3& D_btMat)
{
	D_Vectormath::D_Aos::D_Matrix3 mat(
		getVmVector3(D_btMat.getColumn(0)),
		getVmVector3(D_btMat.getColumn(1)),
		getVmVector3(D_btMat.getColumn(2)));
		return mat;
}


#endif //AOS_VECTORMATH_BULLET_CONVERT_H
