#include "windows.h"
#include "windowsx.h"
#include <winuser.h>
#include <GL/gl.h>
#include <GL/glut.h>
//#include <GL/glx.h>
#define GL_GLEXT_PROTOTYPES     
//#include <GL/glext.h>

#include <math.h>
#include <string>
#include <stdio.h>
#include <memory.h>


// �O���̃��C�u����
// http://www.syuhitu.org/other/bmp/BmpIoLib_h.html
#include "BmpIoLib.h"

#include "mmd_model.h"


// TODO : ��������B����͉��菑�������Ƃ��̎c�[
// �N���X�\���������ł����������͂�
extern MMD_File    mmdfile;
extern Texture     madoka_magic;


void MMD_Header::read(FILE* fp) {
	// �A���C�����g��������Ă��Ȃ��̂ŁA���O�œǂݍ���
	fread(magic, 1, 3, fp);
	fread(&version, 1, 4, fp);
	fread(model_name, 1, 20, fp);
	fread(comment, 1, 256, fp);
}

MMD_Header::MMD_Header() {
	memset(magic, 3, 0);
	version = 0.0f;
	memset(model_name, 0, 20);
	memset(comment, 0, 256);
}
MMD_Header::~MMD_Header() {}


void MMD_face::read(FILE* fp) {
	fread(&face_vert_count, 1, 4, fp);
	printf("face vert count = %d\n", face_vert_count);

	face_index = new GLuint[face_vert_count];
	for (int i = 0; i < face_vert_count; i++) {
		fread(&face_index[i], 1, 2, fp);
	}
}

MMD_face::MMD_face() {
}

MMD_face::~MMD_face() {
}

void MMD_face::draw()
{
}

MMD_vertex::MMD_vertex() {
}

MMD_vertex::~MMD_vertex() {
}

void MMD_vertex::read(FILE* fp)
{
	fread(&x, 1, 4, fp);
	fread(&y, 1, 4, fp);
	fread(&z, 1, 4, fp);
	fread(&nx, 1, 4, fp);
	fread(&ny, 1, 4, fp);
	fread(&nz, 1, 4, fp);
	fread(&u, 1, 4, fp);
	fread(&v, 1, 4, fp);

	fread(bone_num, 2, 2, fp);
	fread(&bone_weight, 1, 1, fp);
	fread(&edge_flag, 1, 1, fp);
}


MMD_VertexArray::MMD_VertexArray()
{
	p3dVerted = NULL;
}

MMD_VertexArray::~MMD_VertexArray()
{
	if (p3dVerted) {
		delete[]p3dVerted;
		p3dVerted = NULL;
	}
};

void MMD_VertexArray::read(FILE* fp)
{
	fread(&count, 4, 1, fp);
	printf("vertex count == %d\n", count);       // �f�o�b�O�p

	pVertex = new MMD_vertex[count];
	for (int i = 0; i < count; i++) {
		pVertex[i].read(fp);
	}
}

void MMD_VertexArray::draw()
{
	// ���_, �@��, �e�N�X�`�����W���Ƃ̔z��ɒu��������
	p3dVerted = new GLfloat[count * 3];
	for (int i = 0; i < count; i++) {
		p3dVerted[i * 3 + 0] = pVertex[i].x;
		p3dVerted[i * 3 + 1] = pVertex[i].y;
		p3dVerted[i * 3 + 2] = pVertex[i].z;
	}
	glVertexPointer(3, GL_FLOAT, 0, p3dVerted);   // ���_�z��
	printf("%s : vert array draw \n", __FUNCTION__);
	printf("%s : vertex count = %d \n", __FUNCTION__, count);
}

Texture::Texture(void)
{
}

Texture::~Texture()
{
	if (pBmp)
		glDeleteTextures(1, &texture_id);
};

void Texture::load(const char* filename, GLuint tex_id)
{
	pBmp = NULL;
	FILE* fp = fopen(filename, "rb");
	if (fp != NULL) {
		pBmp = BmpIO_Load(fp);
		if (pBmp != NULL) {
			// OpenGL�p�Ƀe�N�X�`������
			this->texture_id = tex_id;
			printf("load texture_id = %d\n", this->texture_id);
			printf("id: %d = %s\n", this->texture_id, filename);
			glBindTexture(GL_TEXTURE_2D, this->texture_id);
			glTexImage2D(
				GL_TEXTURE_2D, 0, GL_RGB, pBmp->width, pBmp->height,
				0, GL_RGB, GL_UNSIGNED_BYTE, pBmp->pColor);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
			glFlush();
			glBindTexture(GL_TEXTURE_2D, 0);
			//            glBindTexture( GL_TEXTURE_2D, 10 );   // ����Ȃ�����
		}
		else {
			printf(">>>>>>pBmp == NULL\n");
		}
		fclose(fp);
	}
}

unsigned int Texture::get_gl_texture_id(void)
{
	return this->texture_id;
}


// �}�e���A���f�[�^��\���N���X
Material::Material(void)
{
	tex_index = 0;
}
Material::~Material() {}

uint32_t Material::get_face_vert_count(void)
{
	return face_vert_count;
}

void Material::setpath(std::string pathname)
{
	path = pathname;
}

void Material::read(FILE* fp, GLuint texture_id)
{
	memset(texture_name, 0, 21);
	fread(diffuse_color, 4, 3, fp);
	fread(&alpha, 4, 1, fp);
	fread(&specular, 4, 1, fp);
	fread(specular_color, 4, 3, fp);
	fread(mirror_color, 4, 3, fp);
	fread(&toon_index, 1, 1, fp);
	fread(&edge_flag, 1, 1, fp);
	fread(&face_vert_count, 4, 1, fp);
	fread(texture_name, 1, 20, fp);
	std::string fullpath_texture = path + texture_name;
	texture.load(fullpath_texture.c_str(), texture_id);
}

// �}�e���A���̔z��
MaterialArray::MaterialArray() {};
MaterialArray::~MaterialArray() {};
void MaterialArray::setpath(std::string pathname)
{
	path = pathname;
}

void MaterialArray::read(FILE* fp)
{
	fread(&count, 1, 4, fp);
	mat_array = new Material[count];

	// �e�N�X�`���̏�����������Ŋm�肷��̂ŁA
	// ���̈ʒu�Ńe�N�X�`��ID��ێ�����z�������
	// TODO: �e�N�X�`�����g��Ȃ��}�e���A���̏ꍇ�͖��ʂ��o�Ă��܂��̂ŁA�Ή�������
	textureIds = new GLuint[count];
	glGenTextures(count, textureIds);
	printf("glGenTextures Array = %d\n", count);
	for (int i = 0; i < count; i++) {
		printf("[index:%d == id=%d]\n", i, textureIds[i]);
	}

	// �e�N�X�`�����ꖇ���ǂݍ���ł���
	// TODO: �قȂ�}�e���A���ł������t�@�C�������w���Ă��邱�Ƃ�����̂ŁA
	// �}�e���A���ƃe�N�X�`����1:1�̊֌W�ł͌����������B
	for (int i = 0; i < count; i++) {
		mat_array[i].setpath(path);
		mat_array[i].read(fp, textureIds[i]);
	}
}

void MaterialArray::draw(void)
{
	uint32_t effect_draw_count = 0;
	for (int i = 0; i < count; i++) {
		mat_array[i].draw(effect_draw_count);
		effect_draw_count += mat_array[i].get_face_vert_count();
	}
}


void Material::draw(uint32_t start_face)
{
	MMD_face*   face = &mmdfile.m_face;
	int face_count = this->face_vert_count;

	//    glBegin(GL_TRIANGLES);

	//    glBindTexture(GL_TEXTURE_2D, texture.get_gl_texture_id() );
	printf("texture_id = %d\n", texture.get_gl_texture_id());
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	//    glFrontFace(GL_CCW);        // GL_CW(���v��肪�\), GL_CCW(�����v��肪�\)
	//    glEnable(GL_CULL_FACE);     //�J�����OON 
	glBindTexture(GL_TEXTURE_2D, 0);


	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glRotatef(3.14159265358979323846f, 0.0f, 1.0f, 0.0f);

#if 1
	{
		GLsizei primitive_count = face_count * 3;
		printf("primitive_count=%d\n", primitive_count);
		printf("start_face=%d\n", start_face);

		glDrawElements(GL_TRIANGLES,                    // �v���~�e�B�u�̎��
			primitive_count,                // �����_�����O�v�f�̐�
			GL_UNSIGNED_INT,                // �C���f�b�N�X�z��̌^
			&face->face_index[start_face]); // �C���f�b�N�X�z��������|�C���^
	}

#else
	for (int i = 0; i < face_count; i += 3) {
		for (int j = 0; j < 3; j++) {
			int index = face->face_index[start_face + i + j];
			MMD_vertex* vert = &mmdfile.m_vertics.pVertex[index];
			// TODO : opengl ��directX�ō��W�n���قȂ�̂ŉ������Ȃ��Ƃ����Ȃ�
			//            glTexCoord2f( vert->u, fabsf(vert->v - 1.0f) ); // u=x, v=y

			// �o�O�̌�������
			// �e�N�X�`�����W�ȑO�̘b�ŁA
			// ���ׂẴ}�e���A���̕`��ŁA����̃e�N�X�`���n���h�����Q�Ƃ��Ă���l�q
			// ��̓I�ɂ́Amd_m_head.bmp �t�@�C���̓��e�݂̂����ׂẴ}�e���A���ɔ��f����Ă���B
			// (�摜���g�����ׂĐԂœh��Ԃ��Ȃǂ����Ċm�F�����B)


			// �������e�N�X�`�����W�n�̊m�F�p�̃R�[�h
			// �摜�̍������̃e�N�X�`���̂ݕ`�悷��B
			// (���ŕ\������ƁA���ʂ̉摜�{��)
			float v = fabsf(1.0f - vert->v);
			//            float v = vert->v;
			float u = vert->u;
			//            if(v > 0.1f && u > 0.1f){
			glTexCoord2f(u, v); // u=x, v=y
								//            }else{
								//                glTexCoord2f( 0, 0 ); // u=x, v=y
								//            }
								// �e�N�X�`�����W�n�̒�������
								// 
								// OpenGL�̃e�N�X�`�����W��T,S�ŕ\�������B
								// �摜���������_�Ƃ���
								// S ... �E������+�̍��W��
								// T ... �������+�̍��W��
								//
								// �Ȃ��AglTexCoord2F��(S,T)�Ŏw�肷��
								//
								// DirectX�̃e�N�X�`�����W��u,v�ŕ\�������
								// �摜��������_�Ƃ��āA
								// U ... �E������+�̍��W�n
								// V ... ��������+�̍��W�n
								// 

			glNormal3f(vert->nx, vert->ny, vert->nz);   // �@��
			glVertex3f(vert->x, vert->y, vert->z);      // ���_
		}
	}
#endif
	//    glEnd();

	glBindTexture(GL_TEXTURE_2D, 0);   // ����Ȃ�����
	glFlush();
}


// �e�N�X�`��������pmd�t�@�C���̔z���ɂ���̂Ŏd���Ȃ��B�B
void MMD_File::setpath(const char* pathname)
{
	path = pathname;
}

// �����Ŏw�肳�ꂽ�t�@�C�����J���āA���f���f�[�^��ǂݍ���
void MMD_File::load(const char* iFilename)
{
	std::string fullpath = path + iFilename;
	FILE* fp = fopen(fullpath.c_str(), "rb");

	m_header.read(fp);
	m_vertics.read(fp);
	m_face.read(fp);
	m_materials.setpath(path);
	m_materials.read(fp);

	fclose(fp);
};

void MMD_File::draw(void) {
	glEnableClientState(GL_VERTEX_ARRAY);
	m_vertics.draw();
	m_materials.draw();
	glDisableClientState(GL_VERTEX_ARRAY);
}


