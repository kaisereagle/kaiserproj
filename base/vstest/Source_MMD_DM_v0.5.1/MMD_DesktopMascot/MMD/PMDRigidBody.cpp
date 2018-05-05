//*********
// PMD����
//*********

#include	<memory.h>
#include	"PMDRigidBody.h"

extern cBulletPhysics		g_clBulletPhysics;


//================
// �R���X�g���N�^
//================
cPMDRigidBody::cPMDRigidBody( void ) : m_pBone( NULL ), m_pbtColShape( NULL ), m_pbtRigidBody( NULL )
{
}

//==============
// �f�X�g���N�^
//==============
cPMDRigidBody::~cPMDRigidBody( void )
{
	release();
}

//========
// ������
//========
bool cPMDRigidBody::initialize( const PMD_RigidBody *pPMDRigidBody, cPMDBone *pBone )
{
	// �V�F�C�v�̍쐬
	switch( pPMDRigidBody->cbShapeType )
	{
		case 0 :	// ��
			m_pbtColShape = new btSphereShape( pPMDRigidBody->fWidth );
			break;

		case 1 :	// ��
			m_pbtColShape = new btBoxShape( btVector3( pPMDRigidBody->fWidth, pPMDRigidBody->fHeight, pPMDRigidBody->fDepth ) );
			break;

		case 2 :	// �J�v�Z��
			m_pbtColShape = new btCapsuleShape( pPMDRigidBody->fWidth, pPMDRigidBody->fHeight );
			break;
	}

	// ���ʂƊ����e���\���̐ݒ�
	btScalar	btsMass( 0.0f );
	btVector3	btv3LocalInertia( 0.0f, 0.0f ,0.0f );

	// �{�[���Ǐ]�łȂ��ꍇ�͎��ʂ�ݒ�
	if( pPMDRigidBody->cbRigidBodyType != 0 )	btsMass = pPMDRigidBody->fMass;

	// �����e���\���̌v�Z
	if( btsMass != 0.0f )	m_pbtColShape->calculateLocalInertia( btsMass, btv3LocalInertia );

	// �{�[���̈ʒu�擾
	Vector3		vec3BonePos;
	pBone->getPos( &vec3BonePos );

	// �{�[���I�t�Z�b�g�p�g�����X�t�H�[���쐬
	btMatrix3x3	btmRotationMat;
	btmRotationMat.setEulerZYX( pPMDRigidBody->vec3Rotation.x, pPMDRigidBody->vec3Rotation.y, pPMDRigidBody->vec3Rotation.z );

	btTransform		bttrBoneOffset;

	bttrBoneOffset.setIdentity();
	bttrBoneOffset.setOrigin( btVector3( pPMDRigidBody->vec3Position.x, pPMDRigidBody->vec3Position.y, pPMDRigidBody->vec3Position.z ) );
	bttrBoneOffset.setBasis( btmRotationMat );

	// ���̂̏����g�����X�t�H�[���쐬
	btTransform		bttrTransform;

	bttrTransform.setIdentity();
	bttrTransform.setOrigin( btVector3( vec3BonePos.x, vec3BonePos.y, vec3BonePos.z ) );
	bttrTransform = bttrTransform * bttrBoneOffset;

	// MotionState�̍쐬
	btMotionState	*pbtMotionState;

	switch( pPMDRigidBody->cbRigidBodyType )
	{
		case 0 : pbtMotionState = new btKinematicMotionState( bttrTransform, bttrBoneOffset, pBone );	break;
		case 1 : pbtMotionState = new btDefaultMotionState( bttrTransform );							break;
		case 2 : pbtMotionState = new btDefaultMotionState( bttrTransform );							break;
		//case 2 : pbtMotionState = new btKinematicMotionState( bttrTransform, bttrBoneOffset, pBone );	break;
	}

	// ���̂̃p�����[�^�̐ݒ�
	btRigidBody::btRigidBodyConstructionInfo	btRbInfo( btsMass, pbtMotionState, m_pbtColShape, btv3LocalInertia );
	btRbInfo.m_linearDamping  = pPMDRigidBody->fLinearDamping;	// �ړ���
	btRbInfo.m_angularDamping = pPMDRigidBody->fAngularDamping;	// ��]��
	btRbInfo.m_restitution    = pPMDRigidBody->fRestitution;	// ������
	btRbInfo.m_friction       = pPMDRigidBody->fFriction;		// ���C��
	btRbInfo.m_additionalDamping = true;

	// ���̂̍쐬
	m_pbtRigidBody = new btRigidBody( btRbInfo );

	// Kinematic�ݒ�
	if( pPMDRigidBody->cbRigidBodyType == 0 )
	{
		m_pbtRigidBody->setCollisionFlags( m_pbtRigidBody->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT );
		m_pbtRigidBody->setActivationState( DISABLE_DEACTIVATION );
	}
	m_pbtRigidBody->setSleepingThresholds( 0.0f, 0.0f );

	// ���̂��V�~�����[�V�������[���h�ɒǉ�
	g_clBulletPhysics.addToWorld( m_pbtRigidBody, 0x0001 << pPMDRigidBody->cbColGroupIndex, pPMDRigidBody->unColGroupMask );

	m_bttrBoneOffset = bttrBoneOffset;
	m_bttrInvBoneOffset = bttrBoneOffset.inverse();
	m_pBone = pBone;
	m_iType = pPMDRigidBody->cbRigidBodyType;

	if( strncmp( pBone->getName(), "�Z���^�[", 20 ) == 0 )	m_bNoCopyToBone = true;
	else													m_bNoCopyToBone = false;

	// 2009/08/07 Ru--en
	if (!m_bNoCopyToBone) pBone->m_cbRigidType = m_iType;

	return true;
}

//==================
// �{�[���ʒu���킹
//==================
void cPMDRigidBody::fixPosition( float fElapsedFrame )
{
	if( m_iType == 2 )
	{
		Vector3		vec3BonePos;
		m_pBone->getPos( &vec3BonePos );

		btTransform		bttrRbTransform = m_pbtRigidBody->getCenterOfMassTransform();

		bttrRbTransform.setOrigin( btVector3( 0.0f, 0.0f, 0.0f ) );
		bttrRbTransform = m_bttrBoneOffset * bttrRbTransform;

		bttrRbTransform.setOrigin( bttrRbTransform.getOrigin() + btVector3( vec3BonePos.x, vec3BonePos.y, vec3BonePos.z ) );
//		bttrRbTransform.setBasis( m_pbtRigidBody->getWorldTransform().getBasis() );

//		m_pbtRigidBody->setCenterOfMassTransform( bttrRbTransform );

		float	fRate = 0.2f * fElapsedFrame;
		if( fRate > 1.0f )	fRate = 1.0f;
		m_pbtRigidBody->translate( (bttrRbTransform.getOrigin() - m_pbtRigidBody->getCenterOfMassTransform().getOrigin()) * fRate );
/*
		m_pbtRigidBody->setLinearVelocity( btVector3( 0.0f, 0.0f, 0.0f ) );
		m_pbtRigidBody->setAngularVelocity( btVector3( 0.0f, 0.0f, 0.0f ) );
		m_pbtRigidBody->setInterpolationLinearVelocity( btVector3( 0.0f, 0.0f, 0.0f ) );
		m_pbtRigidBody->setInterpolationAngularVelocity( btVector3( 0.0f, 0.0f, 0.0f ) );
		m_pbtRigidBody->setInterpolationWorldTransform( m_pbtRigidBody->getCenterOfMassTransform() );
		m_pbtRigidBody->clearForces();
*/
	}
}

//==================================================================================
// �{�[���̎p�������̂̎p���ƈ�v������(���̃t���[���̃V�~�����[�V�����I����ɌĂ�)
//==================================================================================
void cPMDRigidBody::updateBoneTransform( void )
{
	if( m_iType != 0 && !m_bNoCopyToBone )
	{
		btTransform		bttrBoneTransform = m_pbtRigidBody->getCenterOfMassTransform() * m_bttrInvBoneOffset;

		bttrBoneTransform.getOpenGLMatrix( (float *)m_pBone->m_matLocal );
	}
}

//========================================
// ���̂��{�[���̈ʒu�֋����I�Ɉړ�������
//========================================
void cPMDRigidBody::moveToBonePos( void )
{
	if( m_iType != 0 )
	{
		Vector3		vec3BonePos;
		m_pBone->getPos( &vec3BonePos );

		btTransform		bttrRbTransform = m_pbtRigidBody->getCenterOfMassTransform();

		bttrRbTransform.setOrigin( btVector3( 0.0f, 0.0f, 0.0f ) );
		bttrRbTransform = m_bttrBoneOffset * bttrRbTransform;

		bttrRbTransform.setOrigin( bttrRbTransform.getOrigin() + btVector3( vec3BonePos.x, vec3BonePos.y, vec3BonePos.z ) );
		bttrRbTransform.setBasis( m_pbtRigidBody->getWorldTransform().getBasis() );

		m_pbtRigidBody->setCenterOfMassTransform( bttrRbTransform );

		m_pbtRigidBody->setLinearVelocity( btVector3( 0.0f, 0.0f, 0.0f ) );
		m_pbtRigidBody->setAngularVelocity( btVector3( 0.0f, 0.0f, 0.0f ) );
		m_pbtRigidBody->setInterpolationLinearVelocity( btVector3( 0.0f, 0.0f, 0.0f ) );
		m_pbtRigidBody->setInterpolationAngularVelocity( btVector3( 0.0f, 0.0f, 0.0f ) );
		m_pbtRigidBody->setInterpolationWorldTransform( m_pbtRigidBody->getCenterOfMassTransform() );
		m_pbtRigidBody->clearForces();
		m_pbtRigidBody->updateInertiaTensor();
	}
}

//======
// ���
//======
void cPMDRigidBody::release( void )
{
	if( m_pbtRigidBody )
	{
		if( m_pbtRigidBody->getMotionState() )	delete m_pbtRigidBody->getMotionState();
		g_clBulletPhysics.removeFromWorld( m_pbtRigidBody );

		delete m_pbtRigidBody;
		m_pbtRigidBody = NULL;
	}

	if( m_pbtColShape )
	{
		delete m_pbtColShape;
		m_pbtColShape = NULL;
	}
}
