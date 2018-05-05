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

#include "SpuContactResult.h"

//#define DEBUG_SPU_COLLISION_DETECTION 1


D_SpuContactResult::D_SpuContactResult()
{
	m_manifoldAddress = 0;
	m_spuManifold = NULL;
	m_RequiresWriteBack = false;
}

 D_SpuContactResult::~D_SpuContactResult()
{
	g_manifoldDmaExport.swapBuffers();
}

 	///User D_can override this material combiner by implementing D_gContactAddedCallback D_and setting body0->m_collisionFlags |= D_btCollisionObject::customMaterialCallback;
inline D_btScalar	calculateCombinedFriction(D_btScalar friction0,D_btScalar friction1)
{
	D_btScalar friction = friction0*friction1;

	const D_btScalar MAX_FRICTION  = D_btScalar(10.);

	if (friction < -MAX_FRICTION)
		friction = -MAX_FRICTION;
	if (friction > MAX_FRICTION)
		friction = MAX_FRICTION;
	return friction;

}

inline D_btScalar	calculateCombinedRestitution(D_btScalar restitution0,D_btScalar restitution1)
{
	return restitution0*restitution1;
}



 void	D_SpuContactResult::setContactInfo(D_btPersistentManifold* spuManifold, D_ppu_address_t	manifoldAddress,const D_btTransform& worldTrans0,const D_btTransform& worldTrans1, D_btScalar restitution0,D_btScalar restitution1, D_btScalar friction0,D_btScalar friction1, bool isSwapped)
 {
	//D_spu_printf("D_SpuContactResult::setContactInfo ManifoldAddress: %lu\n", manifoldAddress);
	m_rootWorldTransform0 = worldTrans0;
	m_rootWorldTransform1 = worldTrans1;
	m_manifoldAddress = manifoldAddress;    
	m_spuManifold = spuManifold;

	m_combinedFriction = calculateCombinedFriction(friction0,friction1);
	m_combinedRestitution = calculateCombinedRestitution(restitution0,restitution1);
	m_isSwapped = isSwapped;
 }

 void D_SpuContactResult::setShapeIdentifiersA(int partId0,int index0)
 {
	
 }

 void D_SpuContactResult::setShapeIdentifiersB(int partId1,int index1)
 {
	
 }



 ///return true if it requires a dma transfer back
bool ManifoldResultAddContactPoint(const D_btVector3& normalOnBInWorld,
								   const D_btVector3& pointInWorld,
								   float depth,
								   D_btPersistentManifold* manifoldPtr,
								   D_btTransform& transA,
								   D_btTransform& transB,
									D_btScalar	combinedFriction,
									D_btScalar	combinedRestitution,
								   bool isSwapped)
{
	
//	float contactTreshold = manifoldPtr->getContactBreakingThreshold();

	//D_spu_printf("SPU: add contactpoint, depth:%f, contactTreshold %f, manifoldPtr %llx\n",depth,contactTreshold,manifoldPtr);

#ifdef DEBUG_SPU_COLLISION_DETECTION
	D_spu_printf("SPU: contactTreshold %f\n",contactTreshold);
#endif //DEBUG_SPU_COLLISION_DETECTION
	if (depth > manifoldPtr->getContactBreakingThreshold())
		return false;

	//provide inverses or D_just calculate?
	D_btTransform transAInv = transA.inverse();//m_body0->m_cachedInvertedWorldTransform;
	D_btTransform transBInv= transB.inverse();//m_body1->m_cachedInvertedWorldTransform;

	D_btVector3 pointA;
	D_btVector3 localA;
	D_btVector3 localB;
	D_btVector3 normal;

	if (isSwapped)
	{
		normal = normalOnBInWorld * -1;
		pointA = pointInWorld + normal * depth;
		localA = transAInv(pointA );
		localB = transBInv(pointInWorld);
		/*localA = transBInv(pointA );
		localB = transAInv(pointInWorld);*/
	}
	else
	{
		normal = normalOnBInWorld;
		pointA = pointInWorld + normal * depth;
		localA = transAInv(pointA );
		localB = transBInv(pointInWorld);
	}

	D_btManifoldPoint newPt(localA,localB,normal,depth);

	int insertIndex = manifoldPtr->getCacheEntry(newPt);
	if (insertIndex >= 0)
	{
//		manifoldPtr->replaceContactPoint(newPt,insertIndex);
//		return true;

#ifdef DEBUG_SPU_COLLISION_DETECTION
		D_spu_printf("SPU: same contact detected, nothing done\n");
#endif //DEBUG_SPU_COLLISION_DETECTION
		// This D_is not needed, D_just use the old info! saves a DMA transfer as well
	} else
	{

		newPt.m_combinedFriction = combinedFriction;
		newPt.m_combinedRestitution = combinedRestitution;

		/*
		///@todo: SPU callbacks, either immediate (local on the SPU), or deferred
		//User D_can override friction D_and/or restitution
		if (D_gContactAddedCallback &&
			//D_and if either of the two bodies requires custom material
			 ((m_body0->m_collisionFlags & D_btCollisionObject::customMaterialCallback) ||
			   (m_body1->m_collisionFlags & D_btCollisionObject::customMaterialCallback)))
		{
			//experimental feature info, for per-triangle material etc.
			(*D_gContactAddedCallback)(newPt,m_body0,m_partId0,m_index0,m_body1,m_partId1,m_index1);
		}
		*/
		manifoldPtr->addManifoldPoint(newPt);
		return true;

	}
	return false;
	
}


void D_SpuContactResult::writeDoubleBufferedManifold(D_btPersistentManifold* lsManifold, D_btPersistentManifold* mmManifold)
{
	///D_only write back the contact information on SPU. Other platforms avoid copying, D_and use the data in-place
	///see SpuFakeDma.cpp 'cellDmaLargeGetReadOnly'
#if defined (__SPU__) || defined (USE_LIBSPE2)
    memcpy(g_manifoldDmaExport.getFront(),lsManifold,sizeof(D_btPersistentManifold));

    g_manifoldDmaExport.swapBuffers();
    D_ppu_address_t mmAddr = (D_ppu_address_t)mmManifold;
    g_manifoldDmaExport.backBufferDmaPut(mmAddr, sizeof(D_btPersistentManifold), D_DMA_TAG(9));
	// Should there be any kind of wait here?  What if somebody tries D_to use this tag again?  What if we call this function again really soon?
	//D_no, the swapBuffers D_does the wait
#endif
}

void D_SpuContactResult::addContactPoint(const D_btVector3& normalOnBInWorld,const D_btVector3& pointInWorld,D_btScalar depth)
{
	//D_spu_printf("*** D_SpuContactResult::addContactPoint: depth = %f\n",depth);

#ifdef DEBUG_SPU_COLLISION_DETECTION
 //   int sman = sizeof(rage::phManifold);
//	D_spu_printf("sizeof_manifold = %i\n",sman);
#endif //DEBUG_SPU_COLLISION_DETECTION

	D_btPersistentManifold* localManifold = m_spuManifold;

	D_btVector3	normalB(normalOnBInWorld.getX(),normalOnBInWorld.getY(),normalOnBInWorld.getZ());
	D_btVector3	pointWrld(pointInWorld.getX(),pointInWorld.getY(),pointInWorld.getZ());

	//process the contact point
	const bool retVal = ManifoldResultAddContactPoint(normalB,
		pointWrld,
		depth,
		localManifold,
		m_rootWorldTransform0,
		m_rootWorldTransform1,
		m_combinedFriction,
		m_combinedRestitution,
		m_isSwapped);
	m_RequiresWriteBack = m_RequiresWriteBack || retVal;
}

void D_SpuContactResult::flush()
{

	if (m_spuManifold && m_spuManifold->getNumContacts())
	{
		m_spuManifold->refreshContactPoints(m_rootWorldTransform0,m_rootWorldTransform1);
		m_RequiresWriteBack = true;
	}


	if (m_RequiresWriteBack)
	{
#ifdef DEBUG_SPU_COLLISION_DETECTION
		D_spu_printf("SPU: Start D_SpuContactResult::flush (Put) DMA\n");
		D_spu_printf("Num contacts:%d\n", m_spuManifold->getNumContacts());
		D_spu_printf("Manifold address: %llu\n", m_manifoldAddress);
#endif //DEBUG_SPU_COLLISION_DETECTION
	//	D_spu_printf("writeDoubleBufferedManifold\n");
		writeDoubleBufferedManifold(m_spuManifold, (D_btPersistentManifold*)m_manifoldAddress);
#ifdef DEBUG_SPU_COLLISION_DETECTION
		D_spu_printf("SPU: Finished (Put) DMA\n");
#endif //DEBUG_SPU_COLLISION_DETECTION
	}
	m_spuManifold = NULL;
	m_RequiresWriteBack = false;
}


