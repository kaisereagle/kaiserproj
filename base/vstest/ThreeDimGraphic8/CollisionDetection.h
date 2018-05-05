#pragma once

/// �Փ˔���
class CollisionDetection {
private:
	/// �|���S���ԍ����璸�_���擾
	static void GetVertex(LPD3DXMESH pMesh, unsigned long index, D3DXVECTOR3* vertex);

	/// ���[�J�����W�Ń��C�ƃ��b�V���̌�_���擾
	/// ������LayToMesh()�Q��
	static bool GetIntersectionPoint(D3DXVECTOR3 rayStart, D3DXVECTOR3 rayDirection, LPD3DXMESH pMesh, D3DXVECTOR3* intersectionPoint, D3DXVECTOR3* normalVector);

public:
	/// ���C�ƃ��b�V���̌���
	/// @return					�����������ۂ�
	/// @param rayStart			���C�̊J�n�n�_
	/// @param rayDirection		���C�̕���
	/// @param pMesh			���b�V��
	/// @param meshPosition		���b�V���̈ʒu
	/// @param meshRotation		���b�V���̉�]
	/// @param intesectionPoint	�����_������A�h���X (0�̂Ƃ�����)
	/// @param normalVector		�����_�̖@���x�N�g��������A�h���X (0�̂Ƃ�����)
	static bool LayToMesh(D3DXVECTOR3 rayStart, D3DXVECTOR3 rayDirection, LPD3DXMESH pMesh, D3DXVECTOR3 meshPosition, D3DXMATRIX meshRotation, D3DXVECTOR3* intersectionPoint = 0, D3DXVECTOR3* normalVector = 0);
};