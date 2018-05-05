#ifndef __VERTEX_H__
#define __VERTEX_H__

// 頂点
//
//  ※もちろんサンプル用です
//

struct Vertex {
	float x, y, z;
	union {
		unsigned color;
		struct { unsigned char b, g, r, a; };
	};
};

#endif
