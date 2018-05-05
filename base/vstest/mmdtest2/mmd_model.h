
#include <GL/gl.h>
#include <GL/glut.h>

#include <math.h>
#include <string>
#include <stdio.h>
#include <memory.h>


// �O���̃��C�u����
// http://www.syuhitu.org/other/bmp/BmpIoLib_h.html
#include "BmpIoLib.h"




// MMD�t�@�C���̃w�b�_��\������N���X
class MMD_Header {
private:
	char magic[3];
	float version;
	char model_name[20];
	char comment[256];

public:
	void read(FILE* fp);
	MMD_Header();
	virtual ~MMD_Header();
};


class MMD_face {
public:
	uint32_t    face_vert_count;
	GLuint      *face_index;

	void read(FILE* fp);
	MMD_face();
	virtual ~MMD_face();
	void draw();
};


class MMD_vertex {
public:
	float       x, y, z;
	float       nx, ny, nz;
	float       u, v;

	// ���f���̕\�������Ȃ炱���̃f�[�^�͎g��Ȃ��̂ŁA
	// �Ƃ肠�������u(read�����͂��Ƃ����ǂ��B)
	uint16_t    bone_num[2];
	uint8_t     bone_weight;
	uint8_t     edge_flag;

	MMD_vertex();
	virtual ~MMD_vertex();
	void read(FILE* fp);
};

// �v���~�e�B�u�Ɋւ���������ׂĕێ�����
// (���_, �@��, �e�N�X�`��)
class MMD_VertexArray {
public:
	uint32_t    count;

	// �t�@�C������ǂݍ��񂾃f�[�^���ێ�����
	MMD_vertex*     pVertex;

	//----------------------------------
	// OpenGL�Ń����_�����O���邽�߂̃o�b�t�@�̈���Ǘ�����
	//----------------------------------
	// vertex_buffer��index_buffer
	GLfloat         *p3dVerted;
	unsigned int    *pIndexBuffer;

	MMD_VertexArray();
	virtual ~MMD_VertexArray();
	void read(FILE* fp);
	void draw();
};


// �e�N�X�`�����Ǘ�����N���X
class Texture {
private:
	IBMP*   pBmp;
	GLuint  texture_id;

public:
	Texture(void);
	virtual ~Texture(void);

	void load(const char* filename, GLuint tex_id);
	unsigned int get_gl_texture_id(void);
};


// �}�e���A���f�[�^��\���N���X
class Material {
private:
	float       diffuse_color[3];       // dr, dg, db
	float       alpha;
	float       specular;
	float       specular_color[3];
	float       mirror_color[3];
	uint8_t     toon_index;
	uint8_t     edge_flag;
	uint32_t    face_vert_count;
	char        texture_name[20 + 1];     // +1��NULL�����p
	Texture     texture;
	std::string path;
	unsigned int    tex_index;

public:
	Material(void);
	virtual ~Material();
	uint32_t get_face_vert_count(void);
	void setpath(std::string pathname);

	void read(FILE* fp, GLuint tex_id);

	void draw(uint32_t start_face);
};

// �}�e���A���̔z��
class MaterialArray {
private:
	uint32_t    count;
	Material*   mat_array;
	std::string path;
	GLuint      *textureIds;

public:
	MaterialArray();
	virtual ~MaterialArray();
	void setpath(std::string pathname);
	void read(FILE* fp);

	void draw(void);
};


class MMD_File {
public:
	MMD_Header          m_header;
	MMD_VertexArray     m_vertics;
	MMD_face            m_face;
	MaterialArray       m_materials;
	std::string         path;

	void setpath(const char* pathname);
	void load(const char* iFilename);

	void draw(void);
};



