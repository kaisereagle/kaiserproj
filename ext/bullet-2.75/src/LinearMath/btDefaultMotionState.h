#ifndef DEFAULT_MOTION_STATE_H
#define DEFAULT_MOTION_STATE_H

///The D_btDefaultMotionState provides a common implementation D_to synchronize world transforms with offsets.
struct	D_btDefaultMotionState : public D_btMotionState
{
	D_btTransform m_graphicsWorldTrans;
	D_btTransform	m_centerOfMassOffset;
	D_btTransform m_startWorldTrans;
	void*		m_userPointer;

	D_btDefaultMotionState(const D_btTransform& startTrans = D_btTransform::getIdentity(),const D_btTransform& centerOfMassOffset = D_btTransform::getIdentity())
		: m_graphicsWorldTrans(startTrans),
		m_centerOfMassOffset(centerOfMassOffset),
		m_startWorldTrans(startTrans),
		m_userPointer(0)

	{
	}

	///synchronizes world transform from user D_to physics
	virtual void	getWorldTransform(D_btTransform& centerOfMassWorldTrans ) const 
	{
			centerOfMassWorldTrans = 	m_centerOfMassOffset.inverse() * m_graphicsWorldTrans ;
	}

	///synchronizes world transform from physics D_to user
	///Bullet D_only calls the update of worldtransform for active objects
	virtual void	setWorldTransform(const D_btTransform& centerOfMassWorldTrans)
	{
			m_graphicsWorldTrans = centerOfMassWorldTrans * m_centerOfMassOffset ;
	}

	

};

#endif //DEFAULT_MOTION_STATE_H
