#include "StdAfx.h"
#include "CollisionDetection.h"

void CollisionDetection::GetVertex(LPD3DXMESH pMesh, unsigned long index, D3DXVECTOR3* vertex) {
	unsigned long stride = pMesh->GetNumBytesPerVertex();
	unsigned short *indexBuffer = 0;									// ���_�ԍ��z��
	pMesh->LockIndexBuffer(D3DLOCK_READONLY, (void**)&indexBuffer);
	byte* vertices = 0;													// ���_�f�[�^�̐擪�A�h���X
	if (SUCCEEDED (pMesh->LockVertexBuffer(D3DLOCK_READONLY, (void**)&vertices))){
		vertex[0] = *((D3DXVECTOR3*)(vertices + stride*indexBuffer[index*3]));		// index�Ŏw�肵���|���S���̒��_�f�[�^���L���X�g�Ő؂�����vertex�ɃR�s�[
		vertex[1] = *((D3DXVECTOR3*)(vertices + stride*indexBuffer[index*3 + 1]));
		vertex[2] = *((D3DXVECTOR3*)(vertices + stride*indexBuffer[index*3 + 2]));
		pMesh->UnlockVertexBuffer();
	}
	pMesh->UnlockIndexBuffer();
}

bool CollisionDetection::GetIntersectionPoint(D3DXVECTOR3 rayStart, D3DXVECTOR3 rayDirection, LPD3DXMESH pMesh, D3DXVECTOR3* intersectionPoint, D3DXVECTOR3* normalVector) {
	// �������`�F�b�N
	unsigned long index;	// ��������|���S����index
	float U, V, d;		// �|���S����̌����_��UV���W, ���C�̎n�_�����_�܂ł̋���
	BOOL hit;			// �����������ۂ�
	D3DXIntersect(pMesh, &rayStart, &rayDirection, &hit, &index, &U, &V, &d, 0, 0);
	if (!hit) return false;

	// ��_���擾
	if (!intersectionPoint) return true;
	D3DXVECTOR3 vertex[3];	// ��������|���S���̒��_
	GetVertex(pMesh, index, vertex);
	*intersectionPoint = vertex[0] + U*(vertex[1] - vertex[0]) + V*(vertex[2] - vertex[0]);

	// �@���x�N�g�����擾
	if (!normalVector) return true;
	D3DXPLANE plane;		// ��������|���S���̍�镽��
	D3DXPlaneFromPoints(&plane, &vertex[0], &vertex[1], &vertex[2]);
	D3DXPlaneNormalize(&plane, &plane);
	*normalVector = D3DXVECTOR3(plane[0], plane[1], plane[2]);
	return true;
}

bool CollisionDetection::LayToMesh(D3DXVECTOR3 rayStart, D3DXVECTOR3 rayDirection, LPD3DXMESH pMesh, D3DXVECTOR3 meshPosition, D3DXMATRIX meshRotation, D3DXVECTOR3* intersectionPoint, D3DXVECTOR3* normalVector) {
	// ���[�J�����W�n�ɕϊ�
	D3DXMATRIX trans, world, invWorld, invRotation;
	D3DXMatrixTranslation(&trans, meshPosition.x, meshPosition.y, meshPosition.z);
	world = meshRotation*trans;
	D3DXMatrixInverse(&invWorld, 0, &world);
	D3DXMatrixInverse(&invRotation, 0, &meshRotation);
	D3DXVec3TransformCoord(&rayStart, &rayStart, &invWorld);
	D3DXVec3TransformCoord(&rayDirection, &rayDirection, &invRotation);

	// ���[�J�����W�n�Ō�_���擾
	if (!GetIntersectionPoint(rayStart, rayDirection, pMesh, intersectionPoint, normalVector)) return false;

	// ���[���h���W�n�ɕϊ�
	if (intersectionPoint) D3DXVec3TransformCoord(intersectionPoint, intersectionPoint, &world);
	if (normalVector) D3DXVec3TransformCoord(normalVector, normalVector, &meshRotation);
	return true;
}