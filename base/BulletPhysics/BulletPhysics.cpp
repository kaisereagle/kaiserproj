//****************
// Bullet���b�p�[
//****************
/*
(�Q�l) Monsho����́u���񂵂�̑����v  http://monsho.hp.infoseek.co.jp/
*/

#include	"BulletPhysics.h"


cBulletPhysics			g_clBulletPhysics;


//================
// �R���X�g���N�^
//================
cBulletPhysics::cBulletPhysics( void ) : m_pBtCollisionConfig( NULL ), m_pBtCollisionDispatcher( NULL ), m_pBtOverlappingPairCache( NULL ),
												m_pBtSolver( NULL ), m_pBtWorld( NULL ), m_pBtGroundShape( NULL )
{
}

//==============
// �f�X�g���N�^
//==============
cBulletPhysics::~cBulletPhysics( void )
{
	release();
}

//========
// ������
//========
bool cBulletPhysics::initialize( void )
{
	// �R���W�����R���t�B�O���쐬����
	m_pBtCollisionConfig = new btDefaultCollisionConfiguration();

	// �R���W�����f�B�X�p�b�`�����쐬����
	m_pBtCollisionDispatcher = new btCollisionDispatcher( m_pBtCollisionConfig );

	// �R���W�������[���h�̍ő�T�C�Y���w�肷��
	btVector3	btv3WorldAabbMin( -3000.0f, -3000.0f, -3000.0f );
	btVector3	btv3WorldAabbMax(  3000.0f,  3000.0f,  3000.0f );
	int			iMaxProxies = 1024;
	m_pBtOverlappingPairCache = new btAxisSweep3( btv3WorldAabbMin, btv3WorldAabbMax, iMaxProxies );

	// �̍S���v�Z�\���o���쐬����
	m_pBtSolver = new btSequentialImpulseConstraintSolver();

	// ���[���h�̍쐬
	m_pBtWorld = new btDiscreteDynamicsWorld( m_pBtCollisionDispatcher, m_pBtOverlappingPairCache, m_pBtSolver, m_pBtCollisionConfig );

	// �d�͐ݒ�
	m_pBtWorld->setGravity( btVector3( 0.0f, -9.8f * 2.0f, 0.0f ) );	// �l�N�^�C�Ȃǂ��ӂ�ӂ킷��̂łȂ�ƂȂ�2�{���Ă���

	//-----------------------------------------------------
	// ���p�Ƃ��Ė������ʂ��쐬
	m_pBtGroundShape = new btStaticPlaneShape( btVector3( 0.0f, 1.0f, 0.0f ), 0.0f );

	// ���̃g�����X�t�H�[����ݒ�
	btTransform		trGroundTransform;
	trGroundTransform.setIdentity();

	// MotionState���쐬����B���̂̎p��������������
	btMotionState	*pMotionState = new btDefaultMotionState( trGroundTransform );

	// ���̂��쐬����
	// ���� 0.0�A�����e���\�� 0.0 �Ȃ炱�̍��͓̂����Ȃ�
	btRigidBody::btRigidBodyConstructionInfo	rbInfo( 0.0f, pMotionState, m_pBtGroundShape, btVector3( 0.0f, 0.0f, 0.0f ) );
	btRigidBody		*pRigidBody = new btRigidBody( rbInfo );

	// �������[���h�ɏ���ǉ�
	m_pBtWorld->addRigidBody( pRigidBody );

	return true;
}

//======================
// ���̂����[���h�ɒǉ�
//======================
void cBulletPhysics::addToWorld( btRigidBody *pbtRB, unsigned short unGroupIdx, unsigned short unGroupMask )
{
	m_pBtWorld->addRigidBody( pbtRB, unGroupIdx, unGroupMask );
}

//==================================
// �R���X�g���C���g�����[���h�ɒǉ�
//==================================
void cBulletPhysics::addToWorld( btTypedConstraint *pctConstraint )
{
	m_pBtWorld->addConstraint( pctConstraint );
}

//========================
// ���̂����[���h����폜
//========================
void cBulletPhysics::removeFromWorld( btRigidBody *pbtRB )
{
	m_pBtWorld->removeCollisionObject( pbtRB );
}

//====================================
// �R���X�g���C���g�����[���h����폜
//====================================
void cBulletPhysics::removeFromWorld( btTypedConstraint *pctConstraint )
{
	m_pBtWorld->removeConstraint( pctConstraint );
}

//======
// �X�V
//======
void cBulletPhysics::update( float fElapsedFlame )
{
	float	fMilliSec = fElapsedFlame * (1000.0f / 30.0f);
//	float	fSubStep = fMilliSec * 0.06f;	// fMilliSec / (1000.0f / 60.0f)

//	m_pBtWorld->stepSimulation( fMilliSec, 1 * (int)fSubStep );
	m_pBtWorld->stepSimulation( fMilliSec, 1, fElapsedFlame / 30.0f );
}

//======
// ���
//======
void cBulletPhysics::release( void )
{
	if( !m_pBtWorld )	return;

	for( int i = m_pBtWorld->getNumCollisionObjects() - 1 ; i >= 0 ; i-- )
	{
		btCollisionObject	*pObj = m_pBtWorld->getCollisionObjectArray()[i];
		btRigidBody			*pRigidBody = btRigidBody::upcast( pObj );

		if( pRigidBody && pRigidBody->getMotionState() )	delete pRigidBody->getMotionState();

		m_pBtWorld->removeCollisionObject( pObj );

		delete pObj;
	}

	if( m_pBtGroundShape )
	{
		delete m_pBtGroundShape;
		m_pBtGroundShape = NULL;
	}

	if( m_pBtWorld )
	{
		delete m_pBtWorld;
		m_pBtWorld = NULL;
	}

	if( m_pBtSolver )
	{
		delete m_pBtSolver;
		m_pBtSolver = NULL;
	}

	if( m_pBtOverlappingPairCache )
	{
		delete m_pBtOverlappingPairCache;
		m_pBtOverlappingPairCache = NULL;
	}

	if( m_pBtCollisionDispatcher )
	{
		delete m_pBtCollisionDispatcher;
		m_pBtCollisionDispatcher = NULL;
	}

	if( m_pBtCollisionConfig )
	{
		delete m_pBtCollisionConfig;
		m_pBtCollisionConfig = NULL;
	}
}
