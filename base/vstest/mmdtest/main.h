#pragma once

#ifdef __cplusplus
extern "C" {
#endif

	// �F��ێ�����\����
	typedef struct tagInternalColor
	{
		unsigned char r;
		unsigned char g;
		unsigned char b;
	} ICOLOR;

	// �摜�f�[�^��ێ�����\����
	typedef struct tagInternalBMP
	{
		int width;
		int height;
		int BitPerPix;	// �P�s�N�Z��������̃r�b�g��
		ICOLOR *pColor;	// �J���[�e�[�u���������̓s�N�Z���̃f�[�^
		unsigned char *pPix;	// �s�N�Z���̃f�[�^
	} IBMP;

	// ���ʂ̃^�X�N
	IBMP* BmpIO_CreateBitmap(int width, int height, int BitPerPixcel);
	IBMP* BmpIO_Load(FILE *infile);
	int BmpIO_Save(FILE *outfile, const IBMP *pBmp);
	void BmpIO_DeleteBitmap(IBMP *pBmp);
	int BmpIO_GetWidth(const IBMP *pBmp);
	int BmpIO_GetHeight(const IBMP *pBmp);
	int BmpIO_GetBitPerPixcel(const IBMP *pBmp);
	unsigned char BmpIO_GetR(int x, int y, const IBMP *pBmp);
	unsigned char BmpIO_GetG(int x, int y, const IBMP *pBmp);
	unsigned char BmpIO_GetB(int x, int y, const IBMP *pBmp);

	// 24bit�r�b�g�}�b�v�p
	void BmpIO_SetR(int x, int y, IBMP *pBmp, unsigned char v);
	void BmpIO_SetG(int x, int y, IBMP *pBmp, unsigned char v);
	void BmpIO_SetB(int x, int y, IBMP *pBmp, unsigned char v);

	// 1,4,8bit�r�b�g�}�b�v�p
	unsigned char BmpIO_GetColorTableR(int idx, const IBMP *pBmp);
	unsigned char BmpIO_GetColorTableG(int idx, const IBMP *pBmp);
	unsigned char BmpIO_GetColorTableB(int idx, const IBMP *pBmp);
	void BmpIO_SetColorTableR(int idx, const IBMP *pBmp, unsigned char v);
	void BmpIO_SetColorTableG(int idx, const IBMP *pBmp, unsigned char v);
	void BmpIO_SetColorTableB(int idx, const IBMP *pBmp, unsigned char v);
	unsigned char BmpIO_GetPixcel(int x, int y, const IBMP *pBmp);
	void BmpIO_SetPixcel(int x, int y, const IBMP *pBmp, unsigned char v);
	int BmpIO_TranseTo24BitColor(IBMP *pBmp);

#ifdef __cplusplus
}
#endif

//�N�H�[�^�j�I���\����
struct Quaternion {
	float w;
	float x;
	float y;
	float z;
};


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
	GLfloat         *p3dNormal;
	GLfloat         *pTexuv;
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

public: // �b��
		// �`��p�̃p�����[�^�B�{���͂����ɋL�q����ׂ��ł͂Ȃ�
	uint32_t    start_face;

public:
	Material(void);
	virtual ~Material();
	uint32_t get_face_vert_count(void);
	void setpath(std::string pathname);

	void read(FILE* fp, GLuint tex_id);

	void draw();
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

// �{�[�����
class MMD_Bone {
public: // �b��
	char        bone_name[20];          // �{�[����
	uint16_t    parent_bone_index;      // �e�{�[���C���f�b�N�X
	uint16_t    tail_pos_bone_index;    // tail�ʒu�̃{�[���C���f�b�N�X
	uint8_t     bone_type;              // �{�[���̎��
	uint16_t    ik_parent_bone_index;   // IK�{�[���ԍ�(�Ȃ��ꍇ��0)
	float       bone_head_pos[3];       // �{�[���̃w�b�h�̈ʒu

										// �`��p�̏��
										// �t�@�C���t�H�[�}�b�g�ɂ͊܂܂�Ă��Ȃ��B
										// TODO : �t�@�C���t�H�[�}�b�g�Ƃ͕���������
	float       tx, ty, tz;
	Quaternion  quot;
	float       rot_mat[4 * 4];
public:
	MMD_Bone();
	~MMD_Bone();
	void read(FILE* fp);
	void draw();
	void set_translate(float tx, float ty, float tz);
	void set_rotation(float qx, float qy, float qz, float qw);
};

// �e�q�֌W�����ƂɃ{�[�����v�Z���邽�߂̃N���X
class MMD_BoneNode {
public:     // �b��
	MMD_BoneNode() { bone_index = 0; } // root-node���g����
	~MMD_BoneNode() {}

	uint16_t   bone_index;      // ���̃m�[�h�������{�[���̃C���f�b�N�X
	std::vector<MMD_BoneNode>   children;
};
// �q����ǉ����邽�߂̊֐�
void add_children(uint16_t parent, uint16_t child);


// �{�[�����̔z��
class MMD_BoneArray {
public: // �b��
	uint16_t    bone_count;             // �{�[����
	MMD_Bone    *bone_array;            // �{�[�����X�g(���я��ɈӖ�������̂Œ���)

	std::map<std::string, int>  bone_name_dict;     // �{�[�������L�[�ɂ��āA�C���f�b�N�X���������߂̎���

public:
	MMD_BoneArray();
	~MMD_BoneArray();

	void read(FILE* fp);

	// �f�o�b�O�p�@�\
	// �{�[���̃����_�����O
	void draw();
};


// �݌v����
//      ���[�V�����f�[�^�̓L�[�t���[���P��, �{�[���P�ʂɎw�肳���B
//      ���[�V�����f�[�^����A�{�[���ɑ΂��č��W�Ɖ�]�p�x���^���炦��
//      �{�[������A���_�ɑ΂��āA�e���x�Ȃǂ̏�񂪗^������  
//      ���_�͂����̔z��Ƃ��ĕێ�����Ă���B
//

class MMD_File {
public:
	MMD_Header          m_header;
	MMD_VertexArray     m_vertics;
	MMD_face            m_face;
	MaterialArray       m_materials;
	MMD_BoneArray       m_bones;
	std::string         path;

	void setpath(const char* pathname);
	void load(const char* iFilename);

	void draw(void);
};
// MMD�t�@�C���̃w�b�_��\������N���X
class VMD_Header {
public: // �Ƃ肠����
	char        VmdHeader[30];      // "Vocaloid Motion Data 0002"
	char        VmdModelName[20];   // ���O�������Ă���
	uint32_t    motion_count;

public:
	VMD_Header();
	virtual ~VMD_Header();
	void read(FILE* fp);

	// �K�v�Œ���̃A�N�Z�T���L�q���Ă���
	uint32_t    get_motion_count() { return motion_count; }
};

class VMD_Motion {
public:
	char        BoneName[15];           // �{�[����
	uint32_t    FrameNumber;            // �t���[���ԍ�
	float       px, py, pz;               // �ʒu
	Quaternion  rq;                     // ��]�p(�N�H�[�^�j�I��)
	uint8_t     Interpolation[4][4][4]; // �⊮
public:
	void read(FILE* fp);
	VMD_Motion();
	~VMD_Motion();
};

class VMD_File {
public:
	VMD_Header  vmd_header;             // �w�b�_
	VMD_Motion  *vmd_motion;            // ���[�V�����f�[�^
	int         motion_index;

public:
	VMD_File();
	~VMD_File();

	void setMmdMotion(MMD_File* pmd, int frame_number);
	void read(const char* filename);
};

