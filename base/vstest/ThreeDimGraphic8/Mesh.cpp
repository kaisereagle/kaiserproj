#include "StdAfx.h"
#include "Mesh.h"

///// Mesh�N���X /////
Mesh::Mesh(LPDIRECT3DDEVICE9 pDev) : pDevice(pDev), pMesh(0), pMeshTextures(0), pMeshMaterials(0) {
}

Mesh::~Mesh() {
	SAFE_RELEASE(pMesh);
	if (pMeshTextures) for (DWORD i = 0; i < numMaterial; ++i) SAFE_RELEASE(pMeshTextures[i]);
	SAFE_DELETE_ARRAY(pMeshTextures);
	SAFE_DELETE_ARRAY(pMeshMaterials);
}

void Mesh::AddNormalVector(MeshData& meshData) {
	for (unsigned int i = 0; i < meshData.vertices.size(); ++i) meshData.vertices[i].normal = D3DXVECTOR3(0, 0, 0);
	for (unsigned int i = 0; i < meshData.faces.size(); ++i) {
		D3DXVECTOR3 p[3];
		for (unsigned int j = 0; j < 3; ++j) p[j] = meshData.vertices[meshData.faces[i].indices[j]].position;
		D3DXPLANE plane;
		D3DXPlaneFromPoints(&plane, &p[0], &p[1], &p[2]);	// �������Ŋm�F�������Aplane�͋K�i������Ă�����ۂ�
		for (unsigned int j = 0; j < 3; ++j) meshData.vertices[meshData.faces[i].indices[j]].normal += D3DXVECTOR3(plane.a, plane.b, plane.c);
		float l = D3DXVec3Length(&D3DXVECTOR3(plane.a, plane.b, plane.c));
	}
	for (unsigned int i = 0; i < meshData.vertices.size(); ++i) D3DXVec3Normalize(&meshData.vertices[i].normal, &meshData.vertices[i].normal);
}

void Mesh::SetMesh(MeshData meshData) {
	D3DXCreateMeshFVF(meshData.faces.size(), meshData.vertices.size(), D3DXMESH_MANAGED, D3DFVF_XYZ | D3DFVF_NORMAL | D3DFVF_TEX1, pDevice, &pMesh);
	Vertex* vertexBuffer;
	pMesh->LockVertexBuffer(0, (void**)&vertexBuffer);
	for (unsigned int i = 0; i < meshData.vertices.size(); ++i) {
		vertexBuffer[i].position = meshData.vertices[i].position;
		vertexBuffer[i].normal = meshData.vertices[i].normal;
		vertexBuffer[i].texture = meshData.vertices[i].texture;
	}
	pMesh->UnlockVertexBuffer();
	unsigned short* indexBuffer;
	pMesh->LockIndexBuffer(0, (void**)&indexBuffer);
	for (unsigned int i = 0; i < meshData.faces.size(); ++i) for (unsigned int j = 0; j < 3; ++j) indexBuffer[3*i + j] = meshData.faces[i].indices[j];
	pMesh->UnlockIndexBuffer();
	unsigned long* attributeBuffer;
	pMesh->LockAttributeBuffer(0, &attributeBuffer);
	for (unsigned int i = 0; i < meshData.faces.size(); ++i) attributeBuffer[i] = meshData.faces[i].material_number;
	pMesh->UnlockAttributeBuffer();
	numMaterial = meshData.material.size();
    pMeshMaterials = new D3DMATERIAL9[numMaterial];
    pMeshTextures  = new LPDIRECT3DTEXTURE9[numMaterial];
	for (unsigned long i = 0; i < numMaterial; ++i) pMeshTextures[i] = 0;
	for (unsigned long i = 0; i < numMaterial; ++i) { 
		pMeshMaterials[i] = meshData.material[i];
		char tex_filename[256] = {0};		// UNICODE���Ή��e�N�X�`���t�@�C����
		TCHAR textureFilename[256] = {0};	// UNICODE/�}���`�o�C�g���Ή��e�N�X�`���t�@�C����
		if (strcpy_s(tex_filename, meshData.texture_filename[i].c_str())) throw TEXT("�e�N�X�`���̓ǂݍ��݂Ɏ��s���܂���");
#ifdef UNICODE
		if (strlen(tex_filename) > 0) MultiByteToWideChar(CP_OEMCP, MB_PRECOMPOSED, tex_filename, strlen(tex_filename), textureFilename, (sizeof textureFilename)/2);
#else
		if (strlen(tex_filename) > 0) strcpy_s(textureFilename, tex_filename);
#endif
		if (lstrlen(textureFilename) > 0) // UNICODE/�}���`�o�C�g���Ή��e�N�X�`���t�@�C��������e�N�X�`�����쐬
			if(FAILED(D3DXCreateTextureFromFileEx(pDevice, textureFilename, 0, 0, 0, 0, D3DFMT_UNKNOWN, D3DPOOL_DEFAULT, D3DX_DEFAULT, D3DX_DEFAULT, 0xff000000, 0, 0, &pMeshTextures[i]))) throw TEXT("�e�N�X�`���̓ǂݍ��݂Ɏ��s���܂���");
	}
}

void Mesh::Draw(D3DXVECTOR3* position, D3DXMATRIX* rotation, D3DLIGHT9* light, Camera* camera) {
	D3DXMATRIX trans, world, view, proj;
	D3DXMatrixTranslation(&trans, position->x, position->y, position->z);
	world = (*rotation)*trans;
	camera->GetMatrix(&view, &proj);
	pDevice->SetLight(0, light );
	pDevice->LightEnable(0, TRUE );
    pDevice->SetTransform(D3DTS_WORLD, &world);
	pDevice->SetTransform(D3DTS_VIEW, &view);
	pDevice->SetTransform(D3DTS_PROJECTION, &proj);
	pDevice->SetTextureStageState(0, D3DTSS_ALPHAOP, D3DTOP_MODULATE);	// �|���S����Diffuse�F�̓����x���e�N�X�`���ɔ��f������
	pDevice->SetTextureStageState(0, D3DTSS_ALPHAARG2, D3DTA_DIFFUSE);
	for (unsigned long i = 0; i < numMaterial; ++i) {
		pDevice->SetMaterial(&pMeshMaterials[i]);
		pDevice->SetTexture(0, pMeshTextures[i]); 
		pMesh->DrawSubset(i);
    }
}

LPD3DXMESH Mesh::GetMesh() { return pMesh; }

int Mesh::GetNumMaterial() { return numMaterial; }



///// X�t�@�C�� ���b�V�� /////
XFileMesh::XFileMesh(LPCTSTR filename, LPDIRECT3DDEVICE9 pDev) : Mesh(pDev) {
	LPD3DXBUFFER pD3DXMtrlBuffer = NULL;
	if (FAILED( D3DXLoadMeshFromX(filename, D3DXMESH_SYSTEMMEM, pDevice, NULL, &pD3DXMtrlBuffer, NULL, &numMaterial, &pMesh))) throw TEXT("X�t�@�C���̓ǂݍ��݂Ɏ��s���܂���");
	D3DXMATERIAL* d3dxMaterials = (D3DXMATERIAL*)pD3DXMtrlBuffer->GetBufferPointer();
    pMeshMaterials = new D3DMATERIAL9[numMaterial];
    pMeshTextures  = new LPDIRECT3DTEXTURE9[numMaterial];
	for (DWORD i = 0; i < numMaterial; ++i) { 
		pMeshMaterials[i] = d3dxMaterials[i].MatD3D;
        pMeshTextures[i] = 0;
		TCHAR textureFilename[256] = {0};
#ifdef UNICODE
		if (d3dxMaterials[i].pTextureFilename) MultiByteToWideChar(CP_OEMCP,MB_PRECOMPOSED, d3dxMaterials[i].pTextureFilename, strlen(d3dxMaterials[i].pTextureFilename), textureFilename, (sizeof textureFilename)/2);
#else
		if (d3dxMaterials[i].pTextureFilename) strcpy_s(textureFilename, d3dxMaterials[i].pTextureFilename);
#endif
		if (textureFilename != NULL && lstrlen(textureFilename) > 0)
			if(FAILED(D3DXCreateTextureFromFileEx(pDevice, textureFilename, 0, 0, 0, 0, D3DFMT_UNKNOWN, D3DPOOL_DEFAULT, D3DX_DEFAULT, D3DX_DEFAULT, 0xff000000, 0, 0, &pMeshTextures[i]))) throw TEXT("�e�N�X�`���̓ǂݍ��݂Ɏ��s���܂���");
	}
	pD3DXMtrlBuffer->Release();
}



///// MqoMesh�N���X /////
MqoMesh::MqoMesh(LPCTSTR filename, LPDIRECT3DDEVICE9 pDev) : Mesh(pDev) {
	MeshData meshData = GetMeshDataFromMQO(filename);
	AddNormalVector(meshData);
	SetMesh(meshData);
}

/// �}�e���A���s�̕����񂩂�w�肵�����O�̒l�̔z����擾
/// @return �w�肵�����O�̒l�̔z��B�w�肵�����O���Ȃ���΋�z��B
/// @param str �}�e���A���s�̕�����
/// @param name �l�̖��O	shader(%d) vcol(%d) col(%.3f %.3f %.3f %.3f) dif(%.3f) amb(%.3f) emi(%.3f) spc(%.3f) power(%.2f) tex(%s) 
///							alpha(%s) bump(%s) proj_type(%d) proj_pos(%.3f %.3f %.3f) proj_scale(%.3f %.3f %.3f) proj_angle(%.3f %.3f %.3f)
/// @param n �l�̌�
template <typename T> vector<T> MqoMesh::MaterialPickOut(string str, char* name, int n) {
	vector<T> ts;
	T t;
	int offset = str.find("(", str.find(name, str.find("\"", str.find("\""))));// "�ގ���"�̌�́Aname�̌�́A���ʂ̈ʒu
	if (offset == string::npos) return ts;
	else ++offset;								// ���ʂ̎��̈ʒu
	int count = str.find(")", offset) - offset;	// offset��������ʂ܂ł̒���
	string s = str.substr(offset, count);		// �w�肵�����O�̃f�[�^�𒊏o
	stringstream ss(s);
	for (int i = 0; i < n; ++i) {
		ss >> t;
		ts.push_back(t);
	}
	return ts;
}

void MqoMesh::LoadMaterial(ifstream& ifs, MeshData& meshData){
	string str;
	while (!ifs.eof()) {
		getline(ifs, str);
		if (str.find("}") == 0) break;	// �s�̐擪��"}"������ƏI��
		vector<float> col = MaterialPickOut<float>(str, "col", 4);
		vector<float> dif = MaterialPickOut<float>(str, "dif", 1);
		vector<float> amb = MaterialPickOut<float>(str, "amb", 1);
		vector<float> emi = MaterialPickOut<float>(str, "emi", 1);
		vector<float> spc = MaterialPickOut<float>(str, "spc", 1);
		vector<float> power = MaterialPickOut<float>(str, "power", 1);
		vector<string> tex = MaterialPickOut<string>(str, "tex", 1);
		D3DMATERIAL9 material;
		material.Diffuse.r = dif[0]*col[0];
		material.Diffuse.g = dif[0]*col[1];
		material.Diffuse.b = dif[0]*col[2];
		material.Diffuse.a = dif[0]*col[3];
		material.Ambient.r = amb[0]*col[0];
		material.Ambient.g = amb[0]*col[1];
		material.Ambient.b = amb[0]*col[2];
		material.Ambient.a = amb[0]*col[3];
		material.Emissive.r = emi[0]*col[0];
		material.Emissive.g = emi[0]*col[1];
		material.Emissive.b = emi[0]*col[2];
		material.Emissive.a = emi[0]*col[3];
		material.Specular.r = spc[0]*col[0];
		material.Specular.g = spc[0]*col[1];
		material.Specular.b = spc[0]*col[2];
		material.Specular.a = spc[0]*col[3];
		material.Power = power[0];
		meshData.material.push_back(material);
		if (!tex.empty()) meshData.texture_filename.push_back(tex[0].substr(1, tex[0].length() - 2));	// ������̍ŏ��ƍŌ�́u"�v���J�b�g
		else meshData.texture_filename.push_back("");
	}
}

void MqoMesh::LoadObject(ifstream& ifs, MeshData& meshData){
	string str;
	int offset = meshData.vertices.size();
	while (!ifs.eof()) {
		getline(ifs, str);
		if (str.find("}") == 0) break;	// �s�̐擪��"}"������ƏI��
		if (str.find("visible ") != string::npos) {	// visible ���`�F�b�N
			stringstream ss(str);
			string s;
			int visible;
			ss >> s >> visible;
			if (!visible) break;	// visible = 0���ƏI��
		}
		if (str.find("vertex ") != string::npos) {	// ���_�f�[�^�ǂݍ��݊J�n
			while (!ifs.eof()) {
				getline(ifs, str);
				if (str.find("}") != string::npos) break;	// "}"������ƏI��
				stringstream ss(str);
				Vertex v;
				ss >> v.position.x >> v.position.y >> v.position.z;
				v.position *= 0.01f;			// �{��
				v.normal = D3DXVECTOR3(0, 0, 0);
				meshData.vertices.push_back(v);
			}
		}
		if (str.find("face ") != string::npos) {		// �|���S���f�[�^�ǂݍ��݊J�n
			while (!ifs.eof()) {
				getline(ifs, str);
				if (str.find("}") != string::npos) break;	// "}"������ƏI��
				if (str[str.find_first_not_of(" \t")] != '3') throw TEXT("MQO�t�@�C���t�H�[�}�b�g�G���[1"); // �󔒈ȊO�̐擪������3�ȊO���ƃG���[
				str = str.substr(str.find("(") + 1);		// �ŏ���"("�܂ł��J�b�g
				stringstream ss(str);
				Face f;
				ss >> f.indices[2] >> f.indices[1] >> f.indices[0];	// ���_�f�[�^�����A�Z�R�C�A�t�H�[�}�b�g�Ƃ͋t���Ɋi�[
				for (int i = 0; i < 3; ++i) f.indices[i] += offset;
				str = str.substr(str.find("(") + 1);	// ����"("�܂ł��J�b�g
				ss.str(str);
				ss >> f.material_number;
				meshData.faces.push_back(f);
				str = str.substr(str.find("(") + 1);	// ����"("�܂ł��J�b�g
				ss.str(str);
				ss	>> meshData.vertices[f.indices[2]].texture.x >> meshData.vertices[f.indices[2]].texture.y 
					>> meshData.vertices[f.indices[1]].texture.x >> meshData.vertices[f.indices[1]].texture.y
					>> meshData.vertices[f.indices[0]].texture.x >> meshData.vertices[f.indices[0]].texture.y;
			}
		}
	}
}

MeshData MqoMesh::GetMeshDataFromMQO(LPCTSTR filename) {
	MeshData meshData;
	ifstream ifs(filename);
	if (ifs.fail()) throw TEXT("�t�@�C��������܂���");
	string str;
	while(!ifs.eof()) {
		getline(ifs, str);
		if (str.find("Material") == 0) {
			if (*--str.end() != '{') throw TEXT("MQO�t�@�C���t�H�[�}�b�g�G���[2");
			LoadMaterial(ifs, meshData);
		}
		if (str.find("Object") == 0) {
			if (*--str.end() != '{') throw TEXT("MQO�t�@�C���t�H�[�}�b�g�G���[3");
			LoadObject(ifs, meshData);
		}
	}
	return meshData;
}



/// PmdMesh�N���X
PmdMesh::PmdMesh(LPCTSTR filename, LPDIRECT3DDEVICE9 pDev) : Mesh(pDev) {
	// PMD�t�@�C������PMD�f�[�^�𒊏o
    ifstream ifs(filename, ios::binary);
	if (ifs.fail()) throw TEXT("�t�@�C��������܂���");
	MmdStruct::PmdHeader pmdHeader;
	ifs.read((char*)&pmdHeader, sizeof(pmdHeader));
	unsigned long numPmdVertex;
	ifs.read((char*)&numPmdVertex, sizeof(numPmdVertex));
	MmdStruct::PmdVertex* pmdVertices = new MmdStruct::PmdVertex[numPmdVertex];
	ifs.read((char*)pmdVertices, sizeof(MmdStruct::PmdVertex)*numPmdVertex);
	unsigned long numPmdFace;
	ifs.read((char*)&numPmdFace, sizeof(numPmdFace));
	unsigned short *pmdFaces = new unsigned short[numPmdFace];
	ifs.read((char*)pmdFaces, sizeof(unsigned short)*numPmdFace);
	unsigned long numPmdMaterial;
	ifs.read((char*)&numPmdMaterial, sizeof(numPmdMaterial));
	MmdStruct::PmdMaterial* pmdMaterial = new MmdStruct::PmdMaterial[numPmdMaterial];
	ifs.read((char*)pmdMaterial, sizeof(MmdStruct::PmdMaterial)*numPmdMaterial);

	// PMD�f�[�^����MeshData�ɃR�s�[
	MeshData meshData;
	for (unsigned int i = 0; i < numPmdVertex; ++i) {
		Vertex v;
		v.position = D3DXVECTOR3(pmdVertices[i].pos[0], pmdVertices[i].pos[1], pmdVertices[i].pos[2]);
		v.position *= 0.1f;			// �{��
		v.normal= D3DXVECTOR3(pmdVertices[i].normal_vec[0], pmdVertices[i].normal_vec[1], pmdVertices[i].normal_vec[2]);
		v.texture = D3DXVECTOR2(pmdVertices[i].uv[0], pmdVertices[i].uv[1]);
		meshData.vertices.push_back(v);
	}
	delete pmdVertices;
	Face f;
	for (unsigned int i = 0; i < numPmdFace; ++i) {
		f.indices[i%3] = pmdFaces[i];
		if (i%3 == 2) meshData.faces.push_back(f);
	}
	delete pmdFaces;
	D3DMATERIAL9 material = {0};
	unsigned int j = 0, material_end = 0;
	for (unsigned int i = 0; i < numPmdMaterial; ++i) {
		CopyMaterial(material, pmdMaterial[i]);
		meshData.material.push_back(material);
		char tex[21] = {0};	// �t�@�C������20byte�̂Ƃ��̂��߂ɍŌ��0��ǉ�
		memcpy(tex, pmdMaterial[i].texture_file_name, 20);
		string s(tex);
		s = s.substr(0, s.find("*"));	// temp
		meshData.texture_filename.push_back(s);
		material_end += pmdMaterial[i].face_vert_count;
		for (; j < material_end; ++j) meshData.faces[j/3].material_number = i;
	}
	delete pmdMaterial;
	// MeshData���O���t�B�b�N�{�[�h�̃������ɃZ�b�g
	SetMesh(meshData);
}

void PmdMesh::CopyMaterial(D3DMATERIAL9& material, MmdStruct::PmdMaterial& pmdMaterial) {
	material.Ambient.a = pmdMaterial.alpha;
	material.Ambient.r = pmdMaterial.mirror_color[0];
	material.Ambient.g = pmdMaterial.mirror_color[1];
	material.Ambient.b = pmdMaterial.mirror_color[2];
	material.Diffuse.a = pmdMaterial.alpha;
	material.Diffuse.r = pmdMaterial.diffuse_color[0];
	material.Diffuse.g = pmdMaterial.diffuse_color[1];
	material.Diffuse.b = pmdMaterial.diffuse_color[2];
	material.Power = pmdMaterial.specularity;
	material.Specular.a = pmdMaterial.alpha;
	material.Specular.r = pmdMaterial.specular_color[0];
	material.Specular.g = pmdMaterial.specular_color[1];
	material.Specular.b = pmdMaterial.specular_color[2];
}
