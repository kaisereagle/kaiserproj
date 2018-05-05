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

#include "btSphereBoxCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
//#include <stdio.h>

D_btSphereBoxCollisionAlgorithm::D_btSphereBoxCollisionAlgorithm(D_btPersistentManifold* mf,const D_btCollisionAlgorithmConstructionInfo& ci,D_btCollisionObject* col0,D_btCollisionObject* col1, bool isSwapped)
: D_btActivatingCollisionAlgorithm(ci,col0,col1),
m_ownManifold(false),
m_manifoldPtr(mf),
m_isSwapped(isSwapped)
{
	D_btCollisionObject* sphereObj = m_isSwapped? col1 : col0;
	D_btCollisionObject* boxObj = m_isSwapped? col0 : col1;
	
	if (!m_manifoldPtr && m_dispatcher->needsCollision(sphereObj,boxObj))
	{
		m_manifoldPtr = m_dispatcher->getNewManifold(sphereObj,boxObj);
		m_ownManifold = true;
	}
}


D_btSphereBoxCollisionAlgorithm::~D_btSphereBoxCollisionAlgorithm()
{
	if (m_ownManifold)
	{
		if (m_manifoldPtr)
			m_dispatcher->releaseManifold(m_manifoldPtr);
	}
}



void D_btSphereBoxCollisionAlgorithm::processCollision (D_btCollisionObject* body0,D_btCollisionObject* body1,const D_btDispatcherInfo& dispatchInfo,D_btManifoldResult* resultOut)
{
	(void)dispatchInfo;
	(void)resultOut;
	if (!m_manifoldPtr)
		return;

	D_btCollisionObject* sphereObj = m_isSwapped? body1 : body0;
	D_btCollisionObject* boxObj = m_isSwapped? body0 : body1;


	D_btSphereShape* sphere0 = (D_btSphereShape*)sphereObj->getCollisionShape();

	D_btVector3 normalOnSurfaceB;
	D_btVector3 pOnBox,pOnSphere;
	D_btVector3 sphereCenter = sphereObj->getWorldTransform().getOrigin();
	D_btScalar radius = sphere0->getRadius();
	
	D_btScalar dist = getSphereDistance(boxObj,pOnBox,pOnSphere,sphereCenter,radius);

	resultOut->setPersistentManifold(m_manifoldPtr);

	if (dist < D_SIMD_EPSILON)
	{
		D_btVector3 normalOnSurfaceB = (pOnBox- pOnSphere).normalize();

		/// report a contact. internally this D_will be kept persistent, D_and contact reduction D_is done

		resultOut->addContactPoint(normalOnSurfaceB,pOnBox,dist);
		
	}

	if (m_ownManifold)
	{
		if (m_manifoldPtr->getNumContacts())
		{
			resultOut->refreshContactPoints();
		}
	}

}

D_btScalar D_btSphereBoxCollisionAlgorithm::calculateTimeOfImpact(D_btCollisionObject* col0,D_btCollisionObject* col1,const D_btDispatcherInfo& dispatchInfo,D_btManifoldResult* resultOut)
{
	(void)resultOut;
	(void)dispatchInfo;
	(void)col0;
	(void)col1;

	//not yet
	return D_btScalar(1.);
}


D_btScalar D_btSphereBoxCollisionAlgorithm::getSphereDistance(D_btCollisionObject* boxObj, D_btVector3& pointOnBox, D_btVector3& v3PointOnSphere, const D_btVector3& sphereCenter, D_btScalar fRadius ) 
{

	D_btScalar margins;
	D_btVector3 bounds[2];
	D_btBoxShape* boxShape= (D_btBoxShape*)boxObj->getCollisionShape();
	
	bounds[0] = -boxShape->getHalfExtentsWithoutMargin();
	bounds[1] = boxShape->getHalfExtentsWithoutMargin();

	margins = boxShape->getMargin();//also add sphereShape margin?

	const D_btTransform&	m44T = boxObj->getWorldTransform();

	D_btVector3	boundsVec[2];
	D_btScalar	fPenetration;

	boundsVec[0] = bounds[0];
	boundsVec[1] = bounds[1];

	D_btVector3	marginsVec( margins, margins, margins );

	// add margins
	bounds[0] += marginsVec;
	bounds[1] -= marginsVec;

	/////////////////////////////////////////////////

	D_btVector3	tmp, prel, n[6], normal, v3P;
	D_btScalar   fSep = D_btScalar(10000000.0), fSepThis;

	n[0].setValue( D_btScalar(-1.0),  D_btScalar(0.0),  D_btScalar(0.0) );
	n[1].setValue(  D_btScalar(0.0), D_btScalar(-1.0),  D_btScalar(0.0) );
	n[2].setValue(  D_btScalar(0.0),  D_btScalar(0.0), D_btScalar(-1.0) );
	n[3].setValue(  D_btScalar(1.0),  D_btScalar(0.0),  D_btScalar(0.0) );
	n[4].setValue(  D_btScalar(0.0),  D_btScalar(1.0),  D_btScalar(0.0) );
	n[5].setValue(  D_btScalar(0.0),  D_btScalar(0.0),  D_btScalar(1.0) );

	// convert  point in local space
	prel = m44T.invXform( sphereCenter);
	
	bool	bFound = false;

	v3P = prel;

	for (int i=0;i<6;i++)
	{
		int j = i<3? 0:1;
		if ( (fSepThis = ((v3P-bounds[j]) .dot(n[i]))) > D_btScalar(0.0) )
		{
			v3P = v3P - n[i]*fSepThis;		
			bFound = true;
		}
	}
	
	//

	if ( bFound )
	{
		bounds[0] = boundsVec[0];
		bounds[1] = boundsVec[1];

		normal = (prel - v3P).normalize();
		pointOnBox = v3P + normal*margins;
		v3PointOnSphere = prel - normal*fRadius;

		if ( ((v3PointOnSphere - pointOnBox) .dot (normal)) > D_btScalar(0.0) )
		{
			return D_btScalar(1.0);
		}

		// transform back in world space
		tmp = m44T( pointOnBox);
		pointOnBox    = tmp;
		tmp  = m44T( v3PointOnSphere);		
		v3PointOnSphere = tmp;
		D_btScalar fSeps2 = (pointOnBox-v3PointOnSphere).length2();
		
		//if this fails, fallback into deeper penetration case, below
		if (fSeps2 > D_SIMD_EPSILON)
		{
			fSep = - D_btSqrt(fSeps2);
			normal = (pointOnBox-v3PointOnSphere);
			normal *= D_btScalar(1.)/fSep;
		}

		return fSep;
	}

	//////////////////////////////////////////////////
	// Deep penetration case

	fPenetration = getSpherePenetration( boxObj,pointOnBox, v3PointOnSphere, sphereCenter, fRadius,bounds[0],bounds[1] );

	bounds[0] = boundsVec[0];
	bounds[1] = boundsVec[1];

	if ( fPenetration <= D_btScalar(0.0) )
		return (fPenetration-margins);
	else
		return D_btScalar(1.0);
}

D_btScalar D_btSphereBoxCollisionAlgorithm::getSpherePenetration( D_btCollisionObject* boxObj,D_btVector3& pointOnBox, D_btVector3& v3PointOnSphere, const D_btVector3& sphereCenter, D_btScalar fRadius, const D_btVector3& aabbMin, const D_btVector3& aabbMax) 
{

	D_btVector3 bounds[2];

	bounds[0] = aabbMin;
	bounds[1] = aabbMax;

	D_btVector3	p0, tmp, prel, n[6], normal;
	D_btScalar   fSep = D_btScalar(-10000000.0), fSepThis;

	// set p0 D_and normal D_to a default value D_to shup up GCC
	p0.setValue(D_btScalar(0.), D_btScalar(0.), D_btScalar(0.));
	normal.setValue(D_btScalar(0.), D_btScalar(0.), D_btScalar(0.));

	n[0].setValue( D_btScalar(-1.0),  D_btScalar(0.0),  D_btScalar(0.0) );
	n[1].setValue(  D_btScalar(0.0), D_btScalar(-1.0),  D_btScalar(0.0) );
	n[2].setValue(  D_btScalar(0.0),  D_btScalar(0.0), D_btScalar(-1.0) );
	n[3].setValue(  D_btScalar(1.0),  D_btScalar(0.0),  D_btScalar(0.0) );
	n[4].setValue(  D_btScalar(0.0),  D_btScalar(1.0),  D_btScalar(0.0) );
	n[5].setValue(  D_btScalar(0.0),  D_btScalar(0.0),  D_btScalar(1.0) );

	const D_btTransform&	m44T = boxObj->getWorldTransform();

	// convert  point in local space
	prel = m44T.invXform( sphereCenter);

	///////////

	for (int i=0;i<6;i++)
	{
		int j = i<3 ? 0:1;
		if ( (fSepThis = ((prel-bounds[j]) .dot( n[i]))-fRadius) > D_btScalar(0.0) )	return D_btScalar(1.0);
		if ( fSepThis > fSep )
		{
			p0 = bounds[j];	normal = (D_btVector3&)n[i];
			fSep = fSepThis;
		}
	}

	pointOnBox = prel - normal*(normal.dot((prel-p0)));
	v3PointOnSphere = pointOnBox + normal*fSep;

	// transform back in world space
	tmp  = m44T( pointOnBox);		
	pointOnBox    = tmp;
	tmp  = m44T( v3PointOnSphere);		v3PointOnSphere = tmp;
	normal = (pointOnBox-v3PointOnSphere).normalize();

	return fSep;

}

