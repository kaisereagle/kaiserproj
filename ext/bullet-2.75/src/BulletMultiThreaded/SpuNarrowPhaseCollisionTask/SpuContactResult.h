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

#ifndef SPU_CONTACT_RESULT2_H
#define SPU_CONTACT_RESULT2_H


#ifndef WIN32
#include <stdint.h>
#endif



#include "../SpuDoubleBuffer.h"


#include "LinearMath/btTransform.h"


#include "BulletCollision/NarrowPhaseCollision/btPersistentManifold.h"
#include "BulletCollision/NarrowPhaseCollision/btDiscreteCollisionDetectorInterface.h"

class D_btCollisionShape;


struct D_SpuCollisionPairInput
{
	D_ppu_address_t m_collisionShapes[2];
	D_btCollisionShape*	m_spuCollisionShapes[2];

	D_ppu_address_t m_persistentManifoldPtr;
	D_btVector3	m_primitiveDimensions0;
	D_btVector3	m_primitiveDimensions1;
	int		m_shapeType0;
	int		m_shapeType1;	
	float	m_collisionMargin0;
	float	m_collisionMargin1;

	D_btTransform	m_worldTransform0;
	D_btTransform m_worldTransform1;
	
	bool	m_isSwapped;
	bool    m_useEpa;
};


struct D_SpuClosestPointInput : public D_btDiscreteCollisionDetectorInterface::D_ClosestPointInput
{
	struct D_SpuConvexPolyhedronVertexData* m_convexVertexData[2];
};

///D_SpuContactResult exports the contact points using double-buffered DMA transfers, D_only when needed
///So when an existing contact point D_is duplicated, D_no transfer/refresh D_is performed.
class D_SpuContactResult : public D_btDiscreteCollisionDetectorInterface::D_Result
{
    D_btTransform		m_rootWorldTransform0;
	D_btTransform		m_rootWorldTransform1;
	D_ppu_address_t	m_manifoldAddress;

    D_btPersistentManifold* m_spuManifold;
	bool m_RequiresWriteBack;
	D_btScalar	m_combinedFriction;
	D_btScalar	m_combinedRestitution;
	
	bool m_isSwapped;

	D_DoubleBuffer<D_btPersistentManifold, 1> g_manifoldDmaExport;

	public:
		D_SpuContactResult();
		virtual ~D_SpuContactResult();

		D_btPersistentManifold*	GetSpuManifold() const
		{
			return m_spuManifold;
		}

		virtual void setShapeIdentifiersA(int partId0,int index0);
		virtual void setShapeIdentifiersB(int partId1,int index1);

		void	setContactInfo(D_btPersistentManifold* spuManifold, D_ppu_address_t	manifoldAddress,const D_btTransform& worldTrans0,const D_btTransform& worldTrans1, D_btScalar restitution0,D_btScalar restitution1, D_btScalar friction0,D_btScalar friction01, bool isSwapped);


        void writeDoubleBufferedManifold(D_btPersistentManifold* lsManifold, D_btPersistentManifold* mmManifold);

        virtual void addContactPoint(const D_btVector3& normalOnBInWorld,const D_btVector3& pointInWorld,D_btScalar depth);

		void flush();
};



#endif //SPU_CONTACT_RESULT2_H

