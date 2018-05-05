#pragma once

typedef char zgS8;
typedef unsigned char zgU8;
typedef short zgS16;
typedef unsigned short zgU16;
typedef float zgF32;
typedef int zgS32;
typedef unsigned zgU32;

struct ZGVECTOR2
{
	ZGVECTOR2(){}
	ZGVECTOR2(zgF32 a, zgF32 b) : x(a), y(b){}
	union{
		struct{ zgF32 x,y;};
		zgF32 v[2];
	};
};

struct ZGVECTOR3
{
	ZGVECTOR3(){}
	ZGVECTOR3(zgF32 a, zgF32 b, zgF32 c) : x(a), y(b), z(c){}
	union{
		struct{zgF32 x,y,z;};
		zgF32 v[3];
	};
};

struct ZGVECTOR4
{
	ZGVECTOR4(){}
	ZGVECTOR4(zgF32 a, zgF32 b, zgF32 c, zgF32 d) : x(a), y(b), z(c), w(d){}
	union{
		struct{zgF32 x,y,z,w;};
		zgF32 v[4];
	};
};

struct ZGQUAT
{
	ZGQUAT(){}
	ZGQUAT(zgF32 a, zgF32 b, zgF32 c, zgF32 d) : x(a), y(b), z(c), w(d){}
	union{
		struct{zgF32 x,y,z,w;};
		zgF32 v[4];
	};
};

struct ZGMATRIX44
{
	ZGVECTOR4 m[4];
};

struct ZGMATRIX34
{
	ZGVECTOR3 m[4];
};

struct ZGMATRIX43
{
	ZGVECTOR4 m[3];
};