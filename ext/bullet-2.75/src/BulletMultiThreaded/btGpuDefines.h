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



// definitions for "GPU on CPU" code


#ifndef D_BT_GPU_DEFINES_H
#define D_BT_GPU_DEFINES_H

typedef unsigned int D_uint;

struct D_int2
{
	int x, y;
};

struct D_uint2
{
	unsigned int x, y;
};

struct D_int3
{
	int x, y, z;
};

struct D_uint3
{
	unsigned int x, y, z;
};

struct D_float4
{
	float x, y, z, w;
};

struct D_float3
{
	float x, y, z;
};


#define D_BT_GPU___device__ inline
#define D_BT_GPU___devdata__
#define D_BT_GPU___constant__
#define D_BT_GPU_max(a, b) ((a) > (b) ? (a) : (b))
#define D_BT_GPU_min(a, b) ((a) < (b) ? (a) : (b))
#define D_BT_GPU_params s3DGridBroadphaseParams
#define D_BT_GPU___mul24(a, b) ((a)*(b))
#define D_BT_GPU___global__ inline
#define D_BT_GPU___shared__ static
#define D_BT_GPU___syncthreads()
#define D_CUDART_PI_F D_SIMD_PI

static inline D_uint2 bt3dGrid_make_uint2(unsigned int x, unsigned int y)
{
  D_uint2 t; t.x = x; t.y = y; return t;
}
#define D_BT_GPU_make_uint2(x, y) bt3dGrid_make_uint2(x, y)

static inline D_int3 bt3dGrid_make_int3(int x, int y, int z)
{
  D_int3 t; t.x = x; t.y = y; t.z = z; return t;
}
#define D_BT_GPU_make_int3(x, y, z) bt3dGrid_make_int3(x, y, z)

static inline D_float3 bt3dGrid_make_float3(float x, float y, float z)
{
  D_float3 t; t.x = x; t.y = y; t.z = z; return t;
}
#define D_BT_GPU_make_float3(x, y, z) bt3dGrid_make_float3(x, y, z)

static inline D_float3 bt3dGrid_make_float34(D_float4 f)
{
  D_float3 t; t.x = f.x; t.y = f.y; t.z = f.z; return t;
}
#define D_BT_GPU_make_float34(f) bt3dGrid_make_float34(f)

static inline D_float3 bt3dGrid_make_float31(float f)
{
  D_float3 t; t.x = t.y = t.z = f; return t;
}
#define D_BT_GPU_make_float31(x) bt3dGrid_make_float31(x)

static inline D_float4 bt3dGrid_make_float42(D_float3 v, float f)
{
  D_float4 t; t.x = v.x; t.y = v.y; t.z = v.z; t.w = f; return t;
}
#define D_BT_GPU_make_float42(a, b) bt3dGrid_make_float42(a, b) 

static inline D_float4 bt3dGrid_make_float44(float a, float b, float c, float d)
{
  D_float4 t; t.x = a; t.y = b; t.z = c; t.w = d; return t;
}
#define D_BT_GPU_make_float44(a, b, c, d) bt3dGrid_make_float44(a, b, c, d) 

inline D_int3 operator+(D_int3 a, D_int3 b)
{
    return bt3dGrid_make_int3(a.x + b.x, a.y + b.y, a.z + b.z);
}

inline D_float4 operator+(const D_float4& a, const D_float4& b)
{
	D_float4 r; r.x = a.x+b.x; r.y = a.y+b.y; r.z = a.z+b.z; r.w = a.w+b.w; return r;
}
inline D_float4 operator*(const D_float4& a, float fact)
{
	D_float4 r; r.x = a.x*fact; r.y = a.y*fact; r.z = a.z*fact; r.w = a.w*fact; return r;
}
inline D_float4 operator*(float fact, D_float4& a)
{
	return (a * fact);
}
inline D_float4& operator*=(D_float4& a, float fact)
{
	a = fact * a;
	return a;
}
inline D_float4& operator+=(D_float4& a, const D_float4& b)
{
	a = a + b;
	return a;
}

inline D_float3 operator+(const D_float3& a, const D_float3& b)
{
	D_float3 r; r.x = a.x+b.x; r.y = a.y+b.y; r.z = a.z+b.z; return r;
}
inline D_float3 operator-(const D_float3& a, const D_float3& b)
{
	D_float3 r; r.x = a.x-b.x; r.y = a.y-b.y; r.z = a.z-b.z; return r;
}
static inline float bt3dGrid_dot(D_float3& a, D_float3& b)
{
	return a.x*b.x+a.y*b.y+a.z*b.z;
}
#define D_BT_GPU_dot(a,b) bt3dGrid_dot(a,b)

static inline float bt3dGrid_dot4(D_float4& a, D_float4& b)
{
	return a.x*b.x+a.y*b.y+a.z*b.z+a.w*b.w;
}
#define D_BT_GPU_dot4(a,b) bt3dGrid_dot4(a,b)

static inline D_float3 bt3dGrid_cross(const D_float3& a, const D_float3& b)
{
	D_float3 r; r.x = a.y*b.z-a.z*b.y; r.y = -a.x*b.z+a.z*b.x; r.z = a.x*b.y-a.y*b.x;	return r;
}
#define D_BT_GPU_cross(a,b) bt3dGrid_cross(a,b)


inline D_float3 operator*(const D_float3& a, float fact)
{
	D_float3 r; r.x = a.x*fact; r.y = a.y*fact; r.z = a.z*fact; return r;
}


inline D_float3& operator+=(D_float3& a, const D_float3& b)
{
	a = a + b;
	return a;
}
inline D_float3& operator-=(D_float3& a, const D_float3& b)
{
	a = a - b;
	return a;
}
inline D_float3& operator*=(D_float3& a, float fact)
{
	a = a * fact;
	return a;
}
inline D_float3 operator-(const D_float3& v)
{
	D_float3 r; r.x = -v.x; r.y = -v.y; r.z = -v.z; return r;
}


#define D_BT_GPU_FETCH(a, b) a[b]
#define D_BT_GPU_FETCH4(a, b) a[b]
#define D_BT_GPU_PREF(func) D_btGpu_##func
#define D_BT_GPU_SAFE_CALL(func) func
#define D_BT_GPU_Memset memset
#define D_BT_GPU_MemcpyToSymbol(a, b, c) memcpy(a, b, c)
#define D_BT_GPU_BindTexture(a, b, c, d)
#define D_BT_GPU_UnbindTexture(a)

static D_uint2 s_blockIdx, s_blockDim, s_threadIdx;
#define D_BT_GPU_blockIdx s_blockIdx
#define D_BT_GPU_blockDim s_blockDim
#define D_BT_GPU_threadIdx s_threadIdx
#define D_BT_GPU_EXECKERNEL(numb, numt, kfunc, args) {s_blockDim.x=numt;for(int nb=0;nb<numb;nb++){s_blockIdx.x=nb;for(int nt=0;nt<numt;nt++){s_threadIdx.x=nt;kfunc args;}}}

#define D_BT_GPU_CHECK_ERROR(s)


#endif //D_BT_GPU_DEFINES_H
