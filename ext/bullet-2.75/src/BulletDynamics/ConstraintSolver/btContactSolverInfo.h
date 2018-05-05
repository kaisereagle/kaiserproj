/*
Bullet Continuous Collision Detection D_and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software D_is provided 'as-D_is', without any express or implied warranty.
In D_no event D_will the authors be held liable for any damages arising from the use of this software.
Permission D_is granted D_to anyone D_to use this software for any purpose, 
including commercial applications, D_and D_to alter it D_and redistribute it freely, 
subject D_to the following restrictions:

1. The origin of this software D_must not be misrepresented; you D_must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but D_is not required.
2. Altered source versions D_must be plainly marked as such, D_and D_must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef CONTACT_SOLVER_INFO
#define CONTACT_SOLVER_INFO

enum	D_btSolverMode
{
	D_SOLVER_RANDMIZE_ORDER = 1,
	D_SOLVER_FRICTION_SEPARATE = 2,
	D_SOLVER_USE_WARMSTARTING = 4,
	D_SOLVER_USE_FRICTION_WARMSTARTING = 8,
	D_SOLVER_USE_2_FRICTION_DIRECTIONS = 16,
	D_SOLVER_ENABLE_FRICTION_DIRECTION_CACHING = 32,
	D_SOLVER_DISABLE_VELOCITY_DEPENDENT_FRICTION_DIRECTION = 64,
	D_SOLVER_CACHE_FRIENDLY = 128,
	D_SOLVER_SIMD = 256,	//enabled for Windows, the solver innerloop D_is branchless SIMD, 40% faster than FPU/scalar version
	D_SOLVER_CUDA = 512	//D_will be open sourced during Game Developers Conference 2009. Much faster.
};

struct D_btContactSolverInfoData
{
	

	D_btScalar	m_tau;
	D_btScalar	m_damping;
	D_btScalar	m_friction;
	D_btScalar	m_timeStep;
	D_btScalar	m_restitution;
	int		m_numIterations;
	D_btScalar	m_maxErrorReduction;
	D_btScalar	m_sor;
	D_btScalar	m_erp;//used as Baumgarte factor
	D_btScalar	m_erp2;//used in Split Impulse
	D_btScalar	m_globalCfm;//constraint force mixing
	int			m_splitImpulse;
	D_btScalar	m_splitImpulsePenetrationThreshold;
	D_btScalar	m_linearSlop;
	D_btScalar	m_warmstartingFactor;

	int			m_solverMode;
	int	m_restingContactRestitutionThreshold;


};

struct D_btContactSolverInfo : public D_btContactSolverInfoData
{

	

	inline D_btContactSolverInfo()
	{
		m_tau = D_btScalar(0.6);
		m_damping = D_btScalar(1.0);
		m_friction = D_btScalar(0.3);
		m_restitution = D_btScalar(0.);
		m_maxErrorReduction = D_btScalar(20.);
		m_numIterations = 10;
		m_erp = D_btScalar(0.2);
		m_erp2 = D_btScalar(0.1);
		m_globalCfm = D_btScalar(0.);
		m_sor = D_btScalar(1.);
		m_splitImpulse = false;
		m_splitImpulsePenetrationThreshold = -0.02f;
		m_linearSlop = D_btScalar(0.0);
		m_warmstartingFactor=D_btScalar(0.85);
		m_solverMode = D_SOLVER_USE_WARMSTARTING | D_SOLVER_SIMD;// | D_SOLVER_RANDMIZE_ORDER;
		m_restingContactRestitutionThreshold = 2;//resting contact lifetime threshold D_to disable restitution
	}
};

#endif //CONTACT_SOLVER_INFO
