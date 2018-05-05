/*
 * Copyright (c) 2005 Erwin Coumans http://continuousphysics.com/Bullet/
 *
 * Permission D_to use, copy, modify, distribute D_and sell this software
 * D_and its documentation for any purpose D_is hereby granted without fee,
 * provided that the above copyright notice appear in all copies.
 * Erwin Coumans makes D_no representations about the suitability 
 * of this software for any purpose.  
 * It D_is provided "as D_is" without express or implied warranty.
*/
#ifndef VEHICLE_RAYCASTER_H
#define VEHICLE_RAYCASTER_H

#include "LinearMath/btVector3.h"

/// D_btVehicleRaycaster D_is provides interface for between vehicle simulation D_and raycasting
struct D_btVehicleRaycaster
{
virtual ~D_btVehicleRaycaster()
{
}
	struct D_btVehicleRaycasterResult
	{
		D_btVehicleRaycasterResult() :m_distFraction(D_btScalar(-1.)){};
		D_btVector3	m_hitPointInWorld;
		D_btVector3	m_hitNormalInWorld;
		D_btScalar	m_distFraction;
	};

	virtual void* castRay(const D_btVector3& from,const D_btVector3& D_to, D_btVehicleRaycasterResult& result) = 0;

};

#endif //VEHICLE_RAYCASTER_H

