//***********
// PMD�{�[��
//***********

#ifndef	_PMDBONE_H_
#define	_PMDBONE_H_

#include	"PMDTypes.h"


class cPMDBone
{
	private :
		char			m_szName[21];

		bool			m_bIKLimitAngle;	// IK���Ɋp�x���������邩�ǂ���

		vec3			m_vec3OrgPosition;
		vec3			m_vec3Offset;

		mtrx			m_matInvTransform;	// �����l�̃{�[�������_�Ɉړ�������悤�ȍs��

		const cPMDBone	*m_pParentBone;
		cPMDBone		*m_pChildBone;

		// �ȉ��͌��݂̒l
		vec3			m_vec3Position;
		vec4			m_vec4Rotation;

		mtrx			m_matLocal;

		vec4			m_vec4LookRotation;

	public :
		mtrx			m_matSkinning;		// ���_�f�t�H�[���p�s��

		cPMDBone( void );
		~cPMDBone( void );

		void initialize( const PMD_Bone *pPMDBoneData, const cPMDBone pBoneArray[] );
		void recalcOffset( void );

		void reset( void );

		void updateMatrix( void );
		void lookAt( const vec3 *pvecTargetPos, float fLimitXD, float fLimitXU, float fLimitY  );
		void updateSkinningMat( void );

		void debugDraw( void );

		inline const char *getName( void ){ return m_szName; }
		inline void getOrgPos( vec3 *pvec3Out ){ *pvec3Out = m_vec3OrgPosition; }
		inline void getPos( vec3 *pvec3Out ){ pvec3Out->x = m_matLocal[3][0]; pvec3Out->y = m_matLocal[3][1]; pvec3Out->z = m_matLocal[3][2]; }
		inline mtrx &getLocalMatrix( void ){ return m_matLocal; };

	friend class cMotionPlayer;
	friend class cPMDIK;
	friend class cPMDRigidBody;
};

#endif	// _PMDMODEL_H_
