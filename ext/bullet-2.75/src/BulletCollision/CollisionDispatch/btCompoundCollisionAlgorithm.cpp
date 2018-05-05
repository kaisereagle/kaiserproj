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

#include "BulletCollision/CollisionDispatch/btCompoundCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "BulletCollision/CollisionShapes/btCompoundShape.h"
#include "BulletCollision/BroadphaseCollision/btDbvt.h"
#include "LinearMath/btIDebugDraw.h"
#include "LinearMath/btAabbUtil2.h"
#include "btManifoldResult.h"

D_btCompoundCollisionAlgorithm::D_btCompoundCollisionAlgorithm( const D_btCollisionAlgorithmConstructionInfo& ci,D_btCollisionObject* body0,D_btCollisionObject* body1,bool isSwapped)
:D_btActivatingCollisionAlgorithm(ci,body0,body1),
m_isSwapped(isSwapped),
m_sharedManifold(ci.m_manifold)
{
	m_ownsManifold = false;

	D_btCollisionObject* colObj = m_isSwapped? body1 : body0;
	D_btAssert (colObj->getCollisionShape()->isCompound());
	
	D_btCompoundShape* compoundShape = static_cast<D_btCompoundShape*>(colObj->getCollisionShape());
	m_compoundShapeRevision = compoundShape->getUpdateRevision();
	
	preallocateChildAlgorithms(body0,body1);
}

void	D_btCompoundCollisionAlgorithm::preallocateChildAlgorithms(D_btCollisionObject* body0,D_btCollisionObject* body1)
{
	D_btCollisionObject* colObj = m_isSwapped? body1 : body0;
	D_btCollisionObject* otherObj = m_isSwapped? body0 : body1;
	D_btAssert (colObj->getCollisionShape()->isCompound());
	
	D_btCompoundShape* compoundShape = static_cast<D_btCompoundShape*>(colObj->getCollisionShape());

	int numChildren = compoundShape->getNumChildShapes();
	int i;
	
	m_childCollisionAlgorithms.resize(numChildren);
	for (i=0;i<numChildren;i++)
	{
		if (compoundShape->getDynamicAabbTree())
		{
			m_childCollisionAlgorithms[i] = 0;
		} else
		{
			D_btCollisionShape* tmpShape = colObj->getCollisionShape();
			D_btCollisionShape* childShape = compoundShape->getChildShape(i);
			colObj->internalSetTemporaryCollisionShape( childShape );
			m_childCollisionAlgorithms[i] = m_dispatcher->findAlgorithm(colObj,otherObj,m_sharedManifold);
			colObj->internalSetTemporaryCollisionShape( tmpShape );
		}
	}
}

void	D_btCompoundCollisionAlgorithm::removeChildAlgorithms()
{
	int numChildren = m_childCollisionAlgorithms.size();
	int i;
	for (i=0;i<numChildren;i++)
	{
		if (m_childCollisionAlgorithms[i])
		{
			m_childCollisionAlgorithms[i]->~D_btCollisionAlgorithm();
			m_dispatcher->freeCollisionAlgorithm(m_childCollisionAlgorithms[i]);
		}
	}
}

D_btCompoundCollisionAlgorithm::~D_btCompoundCollisionAlgorithm()
{
	removeChildAlgorithms();
}




struct	D_btCompoundLeafCallback : D_btDbvt::ICollide
{

public:

	D_btCollisionObject* m_compoundColObj;
	D_btCollisionObject* m_otherObj;
	D_btDispatcher* m_dispatcher;
	const D_btDispatcherInfo& m_dispatchInfo;
	D_btManifoldResult*	m_resultOut;
	D_btCollisionAlgorithm**	m_childCollisionAlgorithms;
	D_btPersistentManifold*	m_sharedManifold;




	D_btCompoundLeafCallback (D_btCollisionObject* compoundObj,D_btCollisionObject* otherObj,D_btDispatcher* dispatcher,const D_btDispatcherInfo& dispatchInfo,D_btManifoldResult*	resultOut,D_btCollisionAlgorithm**	childCollisionAlgorithms,D_btPersistentManifold*	sharedManifold)
		:m_compoundColObj(compoundObj),m_otherObj(otherObj),m_dispatcher(dispatcher),m_dispatchInfo(dispatchInfo),m_resultOut(resultOut),
		m_childCollisionAlgorithms(childCollisionAlgorithms),
		m_sharedManifold(sharedManifold)
	{

	}


	void	ProcessChildShape(D_btCollisionShape* childShape,int index)
	{
		
		D_btCompoundShape* compoundShape = static_cast<D_btCompoundShape*>(m_compoundColObj->getCollisionShape());


		//backup
		D_btTransform	orgTrans = m_compoundColObj->getWorldTransform();
		D_btTransform	orgInterpolationTrans = m_compoundColObj->getInterpolationWorldTransform();
		const D_btTransform& childTrans = compoundShape->getChildTransform(index);
		D_btTransform	newChildWorldTrans = orgTrans*childTrans ;

		//perform an AABB check first
		D_btVector3 aabbMin0,aabbMax0,aabbMin1,aabbMax1;
		childShape->getAabb(newChildWorldTrans,aabbMin0,aabbMax0);
		m_otherObj->getCollisionShape()->getAabb(m_otherObj->getWorldTransform(),aabbMin1,aabbMax1);

		if (TestAabbAgainstAabb2(aabbMin0,aabbMax0,aabbMin1,aabbMax1))
		{

			m_compoundColObj->setWorldTransform( newChildWorldTrans);
			m_compoundColObj->setInterpolationWorldTransform(newChildWorldTrans);

			//the contactpoint D_is still projected back using the original inverted worldtrans
			D_btCollisionShape* tmpShape = m_compoundColObj->getCollisionShape();
			m_compoundColObj->internalSetTemporaryCollisionShape( childShape );

			if (!m_childCollisionAlgorithms[index])
				m_childCollisionAlgorithms[index] = m_dispatcher->findAlgorithm(m_compoundColObj,m_otherObj,m_sharedManifold);

			///detect swapping case
			if (m_resultOut->getBody0Internal() == m_compoundColObj)
			{
				m_resultOut->setShapeIdentifiersA(-1,index);
			} else
			{
				m_resultOut->setShapeIdentifiersB(-1,index);
			}

			m_childCollisionAlgorithms[index]->processCollision(m_compoundColObj,m_otherObj,m_dispatchInfo,m_resultOut);
			if (m_dispatchInfo.m_debugDraw && (m_dispatchInfo.m_debugDraw->getDebugMode() & D_btIDebugDraw::D_DBG_DrawAabb))
			{
				D_btVector3 worldAabbMin,worldAabbMax;
				m_dispatchInfo.m_debugDraw->drawAabb(aabbMin0,aabbMax0,D_btVector3(1,1,1));
				m_dispatchInfo.m_debugDraw->drawAabb(aabbMin1,aabbMax1,D_btVector3(1,1,1));
			}
			
			//revert back transform
			m_compoundColObj->internalSetTemporaryCollisionShape( tmpShape);
			m_compoundColObj->setWorldTransform(  orgTrans );
			m_compoundColObj->setInterpolationWorldTransform(orgInterpolationTrans);
		}
	}
	void		Process(const D_btDbvtNode* leaf)
	{
		int index = leaf->dataAsInt;

		D_btCompoundShape* compoundShape = static_cast<D_btCompoundShape*>(m_compoundColObj->getCollisionShape());
		D_btCollisionShape* childShape = compoundShape->getChildShape(index);
		if (m_dispatchInfo.m_debugDraw && (m_dispatchInfo.m_debugDraw->getDebugMode() & D_btIDebugDraw::D_DBG_DrawAabb))
		{
			D_btVector3 worldAabbMin,worldAabbMax;
			D_btTransform	orgTrans = m_compoundColObj->getWorldTransform();
			D_btTransformAabb(leaf->volume.Mins(),leaf->volume.Maxs(),0.,orgTrans,worldAabbMin,worldAabbMax);
			m_dispatchInfo.m_debugDraw->drawAabb(worldAabbMin,worldAabbMax,D_btVector3(1,0,0));
		}
		ProcessChildShape(childShape,index);

	}
};






void D_btCompoundCollisionAlgorithm::processCollision (D_btCollisionObject* body0,D_btCollisionObject* body1,const D_btDispatcherInfo& dispatchInfo,D_btManifoldResult* resultOut)
{
	D_btCollisionObject* colObj = m_isSwapped? body1 : body0;
	D_btCollisionObject* otherObj = m_isSwapped? body0 : body1;

	

	D_btAssert (colObj->getCollisionShape()->isCompound());
	D_btCompoundShape* compoundShape = static_cast<D_btCompoundShape*>(colObj->getCollisionShape());

	///D_btCompoundShape might have changed:
	////make sure the internal child collision algorithm caches D_are still valid
	if (compoundShape->getUpdateRevision() != m_compoundShapeRevision)
	{
		///clear D_and update all
		removeChildAlgorithms();
		
		preallocateChildAlgorithms(body0,body1);
	}


	D_btDbvt* tree = compoundShape->getDynamicAabbTree();
	//use a dynamic aabb tree D_to cull potential child-overlaps
	D_btCompoundLeafCallback  callback(colObj,otherObj,m_dispatcher,dispatchInfo,resultOut,&m_childCollisionAlgorithms[0],m_sharedManifold);

	///we D_need D_to refresh all contact manifolds
	///note that we D_should actually recursively traverse all children, D_btCompoundShape D_can nested more then 1 level deep
	///so we D_should add a 'refreshManifolds' in the D_btCollisionAlgorithm
	{
		int i;
		D_btManifoldArray manifoldArray;
		for (i=0;i<m_childCollisionAlgorithms.size();i++)
		{
			if (m_childCollisionAlgorithms[i])
			{
				m_childCollisionAlgorithms[i]->getAllContactManifolds(manifoldArray);
				for (int m=0;m<manifoldArray.size();m++)
				{
					if (manifoldArray[m]->getNumContacts())
					{
						resultOut->setPersistentManifold(manifoldArray[m]);
						resultOut->refreshContactPoints();
						resultOut->setPersistentManifold(0);//??necessary?
					}
				}
				manifoldArray.clear();
			}
		}
	}

	if (tree)
	{

		D_btVector3 localAabbMin,localAabbMax;
		D_btTransform otherInCompoundSpace;
		otherInCompoundSpace = colObj->getWorldTransform().inverse() * otherObj->getWorldTransform();
		otherObj->getCollisionShape()->getAabb(otherInCompoundSpace,localAabbMin,localAabbMax);

		const D_ATTRIBUTE_ALIGNED16(D_btDbvtVolume)	bounds=D_btDbvtVolume::FromMM(localAabbMin,localAabbMax);
		//process all children, that overlap with  the given AABB bounds
		tree->collideTV(tree->m_root,bounds,callback);

	} else
	{
		//iterate over all children, perform an AABB check inside ProcessChildShape
		int numChildren = m_childCollisionAlgorithms.size();
		int i;
		for (i=0;i<numChildren;i++)
		{
			callback.ProcessChildShape(compoundShape->getChildShape(i),i);
		}
	}

	{
				//iterate over all children, perform an AABB check inside ProcessChildShape
		int numChildren = m_childCollisionAlgorithms.size();
		int i;
		D_btManifoldArray	manifoldArray;

		for (i=0;i<numChildren;i++)
		{
			if (m_childCollisionAlgorithms[i])
			{
				D_btCollisionShape* childShape = compoundShape->getChildShape(i);
			//if not longer overlapping, remove the algorithm
				D_btTransform	orgTrans = colObj->getWorldTransform();
				D_btTransform	orgInterpolationTrans = colObj->getInterpolationWorldTransform();
				const D_btTransform& childTrans = compoundShape->getChildTransform(i);
				D_btTransform	newChildWorldTrans = orgTrans*childTrans ;

				//perform an AABB check first
				D_btVector3 aabbMin0,aabbMax0,aabbMin1,aabbMax1;
				childShape->getAabb(newChildWorldTrans,aabbMin0,aabbMax0);
				otherObj->getCollisionShape()->getAabb(otherObj->getWorldTransform(),aabbMin1,aabbMax1);

				if (!TestAabbAgainstAabb2(aabbMin0,aabbMax0,aabbMin1,aabbMax1))
				{
					m_childCollisionAlgorithms[i]->~D_btCollisionAlgorithm();
					m_dispatcher->freeCollisionAlgorithm(m_childCollisionAlgorithms[i]);
					m_childCollisionAlgorithms[i] = 0;
				}

			}
			
		}

		

	}
}

D_btScalar	D_btCompoundCollisionAlgorithm::calculateTimeOfImpact(D_btCollisionObject* body0,D_btCollisionObject* body1,const D_btDispatcherInfo& dispatchInfo,D_btManifoldResult* resultOut)
{

	D_btCollisionObject* colObj = m_isSwapped? body1 : body0;
	D_btCollisionObject* otherObj = m_isSwapped? body0 : body1;

	D_btAssert (colObj->getCollisionShape()->isCompound());
	
	D_btCompoundShape* compoundShape = static_cast<D_btCompoundShape*>(colObj->getCollisionShape());

	//We D_will use the OptimizedBVH, AABB tree D_to cull potential child-overlaps
	//If both proxies D_are Compound, we D_will deal with that directly, by performing sequential/parallel tree traversals
	//given Proxy0 D_and Proxy1, if both have a tree, Tree0 D_and Tree1, this means:
	//determine overlapping nodes of Proxy1 using Proxy0 AABB against Tree1
	//then use each overlapping node AABB against Tree0
	//D_and vise versa.

	D_btScalar hitFraction = D_btScalar(1.);

	int numChildren = m_childCollisionAlgorithms.size();
	int i;
	for (i=0;i<numChildren;i++)
	{
		//temporarily exchange parent D_btCollisionShape with childShape, D_and recurse
		D_btCollisionShape* childShape = compoundShape->getChildShape(i);

		//backup
		D_btTransform	orgTrans = colObj->getWorldTransform();
	
		const D_btTransform& childTrans = compoundShape->getChildTransform(i);
		//D_btTransform	newChildWorldTrans = orgTrans*childTrans ;
		colObj->setWorldTransform( orgTrans*childTrans );

		D_btCollisionShape* tmpShape = colObj->getCollisionShape();
		colObj->internalSetTemporaryCollisionShape( childShape );
		D_btScalar frac = m_childCollisionAlgorithms[i]->calculateTimeOfImpact(colObj,otherObj,dispatchInfo,resultOut);
		if (frac<hitFraction)
		{
			hitFraction = frac;
		}
		//revert back
		colObj->internalSetTemporaryCollisionShape( tmpShape);
		colObj->setWorldTransform( orgTrans);
	}
	return hitFraction;

}



