//***************
// VPD�|�[�Y
//  ������0�t���[���ڂ����̃��[�V�����Ƃ��ēǂݍ���
// 2009/08/10 Ru--en
//***************

#ifndef		_VPDPOSE_H_
#define		_VPDPOSE_H_

#include	"PMDTypes.h"

// �{�[�����̍��W�E��]
struct PoseDataList
{
	char	szBoneName[16];	// �{�[����

	Vector3	vecPosition;	// �{�[�����W
	Vector4 vecRotation;	// �{�[����]�N�H�[�^�j�I��

	PoseDataList	*pNext;
};

class cVPDPose
{
	private :
		PoseDataList	*m_pPoseDataList;
		int				m_unNumPoseBones;	// �{�[����

		static void trim( char *pStr );

	public :
		cVPDPose( void );
		~cVPDPose( void );

		bool initialize( FILE *pFile );
		void release( void );
		bool load( const wchar_t *wszFilePath );
		inline PoseDataList *getPotionDataList( void ){ return m_pPoseDataList; }
};

#endif	// _VPDPOSE_H_
