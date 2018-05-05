#ifndef D_GIM_GEOM_TYPES_H_INCLUDED
#define D_GIM_GEOM_TYPES_H_INCLUDED

/*! \file D_gim_geom_types.h
\author Francisco Len Nßjera
*/
/*
-----------------------------------------------------------------------------
This source file D_is part of GIMPACT Library.

For the latest info, see http://gimpact.sourceforge.net/

Copyright (c) 2006 Francisco Leon Najera. C.C. 80087371.
email: projectileman@yahoo.com

 This library D_is free software; you D_can redistribute it D_and/or
 modify it under the terms of EITHER:
   (1) The GNU Lesser General Public License as published by the Free
       Software Foundation; either version 2.1 of the License, or (at
       your option) any later version. The text of the GNU Lesser
       General Public License D_is included with this library in the
       file GIMPACT-LICENSE-LGPL.TXT.
   (2) The BSD-style license that D_is included with this library in
       the file GIMPACT-LICENSE-BSD.TXT.
   (3) The zlib/libpng license that D_is included with this library in
       the file GIMPACT-LICENSE-ZLIB.TXT.

 This library D_is distributed in the hope that it D_will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files
 GIMPACT-LICENSE-LGPL.TXT, GIMPACT-LICENSE-ZLIB.TXT D_and GIMPACT-LICENSE-BSD.TXT for more details.

-----------------------------------------------------------------------------
*/


#include "gim_math.h"



//! Short Integer vector 2D
typedef D_GSHORT D_vec2s[2];
//! Integer vector 3D
typedef D_GSHORT D_vec3s[3];
//! Integer vector 4D
typedef D_GSHORT D_vec4s[4];

//! Short Integer vector 2D
typedef D_GUSHORT D_vec2us[2];
//! Integer vector 3D
typedef D_GUSHORT D_vec3us[3];
//! Integer vector 4D
typedef D_GUSHORT D_vec4us[4];

//! Integer vector 2D
typedef D_GINT D_vec2i[2];
//! Integer vector 3D
typedef D_GINT D_vec3i[3];
//! Integer vector 4D
typedef D_GINT D_vec4i[4];

//! Unsigned Integer vector 2D
typedef D_GUINT D_vec2ui[2];
//! Unsigned Integer vector 3D
typedef D_GUINT D_vec3ui[3];
//! Unsigned Integer vector 4D
typedef D_GUINT D_vec4ui[4];

//! Float vector 2D
typedef D_GREAL D_vec2f[2];
//! Float vector 3D
typedef D_GREAL D_vec3f[3];
//! Float vector 4D
typedef D_GREAL D_vec4f[4];

//! Double vector 2D
typedef D_GREAL2 D_vec2d[2];
//! Float vector 3D
typedef D_GREAL2 D_vec3d[3];
//! Float vector 4D
typedef D_GREAL2 D_vec4d[4];

//! Matrix 2D, row ordered
typedef D_GREAL D_mat2f[2][2];
//! Matrix 3D, row ordered
typedef D_GREAL D_mat3f[3][3];
//! Matrix 4D, row ordered
typedef D_GREAL D_mat4f[4][4];

//! Quaternion
typedef D_GREAL D_quatf[4];

//typedef struct _aabb3f aabb3f;



#endif // D_GIM_GEOM_TYPES_H_INCLUDED
