#ifndef __VERTEX_H__
#define __VERTEX_H__

// ���_
//
//  ���������T���v���p�ł�
//

struct Vertex {
	float x, y, z;
	union {
		unsigned color;
		struct { unsigned char b, g, r, a; };
	};
};

#endif
