/*
Bullet Continuous Collision Detection D_and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

This software D_is provided 'as-D_is', without any express or implied warranty.
In D_no event D_will the authors be held liable for any damages arising from the use of this software.
Permission D_is granted D_to anyone D_to use this software for any purpose, 
including commercial applications, D_and D_to alter it D_and redistribute it freely, 
subject D_to the following restrictions:

1. The origin of this software D_must not be misrepresented; you D_must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but D_is not required.
2. Altered source versions D_must be plainly marked as such, D_and D_must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


#ifndef IDEBUG_DRAW__H
#define IDEBUG_DRAW__H

#include "btVector3.h"
#include "btTransform.h"


///The D_btIDebugDraw interface class D_allows hooking up a D_debug renderer D_to visually D_debug simulations.
///Typical use case: create a D_debug drawer object, D_and assign it D_to a D_btCollisionWorld or D_btDynamicsWorld using setDebugDrawer D_and call debugDrawWorld.
///A class that D_implements the D_btIDebugDraw interface has D_to implement the drawLine method at a minimum.
class	D_btIDebugDraw
{
	public:

	enum	D_DebugDrawModes
	{
		D_DBG_NoDebug=0,
		D_DBG_DrawWireframe = 1,
		D_DBG_DrawAabb=2,
		D_DBG_DrawFeaturesText=4,
		D_DBG_DrawContactPoints=8,
		D_DBG_NoDeactivation=16,
		D_DBG_NoHelpText = 32,
		D_DBG_DrawText=64,
		D_DBG_ProfileTimings = 128,
		D_DBG_EnableSatComparison = 256,
		D_DBG_DisableBulletLCP = 512,
		D_DBG_EnableCCD = 1024,
		D_DBG_DrawConstraints = (1 << 11),
		D_DBG_DrawConstraintLimits = (1 << 12),
		D_DBG_FastWireframe = (1<<13),
		D_DBG_MAX_DEBUG_DRAW_MODE
	};

	virtual ~D_btIDebugDraw() {};

	virtual void    drawLine(const D_btVector3& from,const D_btVector3& D_to, const D_btVector3& fromColor, const D_btVector3& toColor)
	{
		drawLine (from, D_to, fromColor);
	}

	virtual void	drawBox (const D_btVector3& boxMin, const D_btVector3& boxMax, const D_btVector3& color, D_btScalar alpha)
	{
	}

	virtual void	drawSphere (const D_btVector3& p, D_btScalar radius, const D_btVector3& color)
	{
	}

	virtual void	drawLine(const D_btVector3& from,const D_btVector3& D_to,const D_btVector3& color)=0;
	
	virtual	void	drawTriangle(const D_btVector3& v0,const D_btVector3& v1,const D_btVector3& v2,const D_btVector3& /*n0*/,const D_btVector3& /*n1*/,const D_btVector3& /*n2*/,const D_btVector3& color, D_btScalar alpha)
	{
		drawTriangle(v0,v1,v2,color,alpha);
	}
	virtual	void	drawTriangle(const D_btVector3& v0,const D_btVector3& v1,const D_btVector3& v2,const D_btVector3& color, D_btScalar /*alpha*/)
	{
		drawLine(v0,v1,color);
		drawLine(v1,v2,color);
		drawLine(v2,v0,color);
	}

	virtual void	drawContactPoint(const D_btVector3& PointOnB,const D_btVector3& normalOnB,D_btScalar distance,int lifeTime,const D_btVector3& color)=0;

	virtual void	reportErrorWarning(const char* warningString) = 0;

	virtual void	draw3dText(const D_btVector3& location,const char* textString) = 0;
	
	virtual void	setDebugMode(int debugMode) =0;
	
	virtual int		getDebugMode() const = 0;

	inline void drawAabb(const D_btVector3& from,const D_btVector3& D_to,const D_btVector3& color)
	{

		D_btVector3 halfExtents = (D_to-from)* 0.5f;
		D_btVector3 center = (D_to+from) *0.5f;
		int i,j;

		D_btVector3 edgecoord(1.f,1.f,1.f),pa,pb;
		for (i=0;i<4;i++)
		{
			for (j=0;j<3;j++)
			{
				pa = D_btVector3(edgecoord[0]*halfExtents[0], edgecoord[1]*halfExtents[1],		
					edgecoord[2]*halfExtents[2]);
				pa+=center;

				int othercoord = j%3;
				edgecoord[othercoord]*=-1.f;
				pb = D_btVector3(edgecoord[0]*halfExtents[0], edgecoord[1]*halfExtents[1],	
					edgecoord[2]*halfExtents[2]);
				pb+=center;

				drawLine(pa,pb,color);
			}
			edgecoord = D_btVector3(-1.f,-1.f,-1.f);
			if (i<3)
				edgecoord[i]*=-1.f;
		}
	}
	void drawTransform(const D_btTransform& transform, D_btScalar orthoLen)
	{
		D_btVector3 start = transform.getOrigin();
		drawLine(start, start+transform.getBasis() * D_btVector3(orthoLen, 0, 0), D_btVector3(0.7f,0,0));
		drawLine(start, start+transform.getBasis() * D_btVector3(0, orthoLen, 0), D_btVector3(0,0.7f,0));
		drawLine(start, start+transform.getBasis() * D_btVector3(0, 0, orthoLen), D_btVector3(0,0,0.7f));
	}

	void drawArc(const D_btVector3& center, const D_btVector3& normal, const D_btVector3& axis, D_btScalar radiusA, D_btScalar radiusB, D_btScalar minAngle, D_btScalar maxAngle, 
				const D_btVector3& color, bool drawSect, D_btScalar stepDegrees = D_btScalar(10.f))
	{
		const D_btVector3& vx = axis;
		D_btVector3 vy = normal.cross(axis);
		D_btScalar step = stepDegrees * D_SIMD_RADS_PER_DEG;
		int nSteps = (int)((maxAngle - minAngle) / step);
		if(!nSteps) nSteps = 1;
		D_btVector3 prev = center + radiusA * vx * D_btCos(minAngle) + radiusB * vy * D_btSin(minAngle);
		if(drawSect)
		{
			drawLine(center, prev, color);
		}
		for(int i = 1; i <= nSteps; i++)
		{
			D_btScalar angle = minAngle + (maxAngle - minAngle) * D_btScalar(i) / D_btScalar(nSteps);
			D_btVector3 next = center + radiusA * vx * D_btCos(angle) + radiusB * vy * D_btSin(angle);
			drawLine(prev, next, color);
			prev = next;
		}
		if(drawSect)
		{
			drawLine(center, prev, color);
		}
	}
	void drawSpherePatch(const D_btVector3& center, const D_btVector3& up, const D_btVector3& axis, D_btScalar radius, 
		D_btScalar minTh, D_btScalar maxTh, D_btScalar minPs, D_btScalar maxPs, const D_btVector3& color, D_btScalar stepDegrees = D_btScalar(10.f))
	{
		D_btVector3 vA[74];
		D_btVector3 vB[74];
		D_btVector3 *pvA = vA, *pvB = vB, *pT;
		D_btVector3 npole = center + up * radius;
		D_btVector3 spole = center - up * radius;
		D_btVector3 arcStart;
		D_btScalar step = stepDegrees * D_SIMD_RADS_PER_DEG;
		const D_btVector3& kv = up;
		const D_btVector3& iv = axis;
		D_btVector3 jv = kv.cross(iv);
		bool drawN = false;
		bool drawS = false;
		if(minTh <= -D_SIMD_HALF_PI)
		{
			minTh = -D_SIMD_HALF_PI + step;
			drawN = true;
		}
		if(maxTh >= D_SIMD_HALF_PI)
		{
			maxTh = D_SIMD_HALF_PI - step;
			drawS = true;
		}
		if(minTh > maxTh)
		{
			minTh = -D_SIMD_HALF_PI + step;
			maxTh =  D_SIMD_HALF_PI - step;
			drawN = drawS = true;
		}
		int n_hor = (int)((maxTh - minTh) / step) + 1;
		if(n_hor < 2) n_hor = 2;
		D_btScalar step_h = (maxTh - minTh) / D_btScalar(n_hor - 1);
		bool isClosed = false;
		if(minPs > maxPs)
		{
			minPs = -D_SIMD_PI + step;
			maxPs =  D_SIMD_PI;
			isClosed = true;
		}
		else if((maxPs - minPs) >= D_SIMD_PI * D_btScalar(2.f))
		{
			isClosed = true;
		}
		else
		{
			isClosed = false;
		}
		int n_vert = (int)((maxPs - minPs) / step) + 1;
		if(n_vert < 2) n_vert = 2;
		D_btScalar step_v = (maxPs - minPs) / D_btScalar(n_vert - 1);
		for(int i = 0; i < n_hor; i++)
		{
			D_btScalar th = minTh + D_btScalar(i) * step_h;
			D_btScalar sth = radius * D_btSin(th);
			D_btScalar cth = radius * D_btCos(th);
			for(int j = 0; j < n_vert; j++)
			{
				D_btScalar psi = minPs + D_btScalar(j) * step_v;
				D_btScalar sps = D_btSin(psi);
				D_btScalar cps = D_btCos(psi);
				pvB[j] = center + cth * cps * iv + cth * sps * jv + sth * kv;
				if(i)
				{
					drawLine(pvA[j], pvB[j], color);
				}
				else if(drawS)
				{
					drawLine(spole, pvB[j], color);
				}
				if(j)
				{
					drawLine(pvB[j-1], pvB[j], color);
				}
				else
				{
					arcStart = pvB[j];
				}
				if((i == (n_hor - 1)) && drawN)
				{
					drawLine(npole, pvB[j], color);
				}
				if(isClosed)
				{
					if(j == (n_vert-1))
					{
						drawLine(arcStart, pvB[j], color);
					}
				}
				else
				{
					if(((!i) || (i == (n_hor-1))) && ((!j) || (j == (n_vert-1))))
					{
						drawLine(center, pvB[j], color);
					}
				}
			}
			pT = pvA; pvA = pvB; pvB = pT;
		}
	}
	
	void drawBox(const D_btVector3& bbMin, const D_btVector3& bbMax, const D_btVector3& color)
	{
		drawLine(D_btVector3(bbMin[0], bbMin[1], bbMin[2]), D_btVector3(bbMax[0], bbMin[1], bbMin[2]), color);
		drawLine(D_btVector3(bbMax[0], bbMin[1], bbMin[2]), D_btVector3(bbMax[0], bbMax[1], bbMin[2]), color);
		drawLine(D_btVector3(bbMax[0], bbMax[1], bbMin[2]), D_btVector3(bbMin[0], bbMax[1], bbMin[2]), color);
		drawLine(D_btVector3(bbMin[0], bbMax[1], bbMin[2]), D_btVector3(bbMin[0], bbMin[1], bbMin[2]), color);
		drawLine(D_btVector3(bbMin[0], bbMin[1], bbMin[2]), D_btVector3(bbMin[0], bbMin[1], bbMax[2]), color);
		drawLine(D_btVector3(bbMax[0], bbMin[1], bbMin[2]), D_btVector3(bbMax[0], bbMin[1], bbMax[2]), color);
		drawLine(D_btVector3(bbMax[0], bbMax[1], bbMin[2]), D_btVector3(bbMax[0], bbMax[1], bbMax[2]), color);
		drawLine(D_btVector3(bbMin[0], bbMax[1], bbMin[2]), D_btVector3(bbMin[0], bbMax[1], bbMax[2]), color);
		drawLine(D_btVector3(bbMin[0], bbMin[1], bbMax[2]), D_btVector3(bbMax[0], bbMin[1], bbMax[2]), color);
		drawLine(D_btVector3(bbMax[0], bbMin[1], bbMax[2]), D_btVector3(bbMax[0], bbMax[1], bbMax[2]), color);
		drawLine(D_btVector3(bbMax[0], bbMax[1], bbMax[2]), D_btVector3(bbMin[0], bbMax[1], bbMax[2]), color);
		drawLine(D_btVector3(bbMin[0], bbMax[1], bbMax[2]), D_btVector3(bbMin[0], bbMin[1], bbMax[2]), color);
	}
	void drawBox(const D_btVector3& bbMin, const D_btVector3& bbMax, const D_btTransform& trans, const D_btVector3& color)
	{
		drawLine(trans * D_btVector3(bbMin[0], bbMin[1], bbMin[2]), trans * D_btVector3(bbMax[0], bbMin[1], bbMin[2]), color);
		drawLine(trans * D_btVector3(bbMax[0], bbMin[1], bbMin[2]), trans * D_btVector3(bbMax[0], bbMax[1], bbMin[2]), color);
		drawLine(trans * D_btVector3(bbMax[0], bbMax[1], bbMin[2]), trans * D_btVector3(bbMin[0], bbMax[1], bbMin[2]), color);
		drawLine(trans * D_btVector3(bbMin[0], bbMax[1], bbMin[2]), trans * D_btVector3(bbMin[0], bbMin[1], bbMin[2]), color);
		drawLine(trans * D_btVector3(bbMin[0], bbMin[1], bbMin[2]), trans * D_btVector3(bbMin[0], bbMin[1], bbMax[2]), color);
		drawLine(trans * D_btVector3(bbMax[0], bbMin[1], bbMin[2]), trans * D_btVector3(bbMax[0], bbMin[1], bbMax[2]), color);
		drawLine(trans * D_btVector3(bbMax[0], bbMax[1], bbMin[2]), trans * D_btVector3(bbMax[0], bbMax[1], bbMax[2]), color);
		drawLine(trans * D_btVector3(bbMin[0], bbMax[1], bbMin[2]), trans * D_btVector3(bbMin[0], bbMax[1], bbMax[2]), color);
		drawLine(trans * D_btVector3(bbMin[0], bbMin[1], bbMax[2]), trans * D_btVector3(bbMax[0], bbMin[1], bbMax[2]), color);
		drawLine(trans * D_btVector3(bbMax[0], bbMin[1], bbMax[2]), trans * D_btVector3(bbMax[0], bbMax[1], bbMax[2]), color);
		drawLine(trans * D_btVector3(bbMax[0], bbMax[1], bbMax[2]), trans * D_btVector3(bbMin[0], bbMax[1], bbMax[2]), color);
		drawLine(trans * D_btVector3(bbMin[0], bbMax[1], bbMax[2]), trans * D_btVector3(bbMin[0], bbMin[1], bbMax[2]), color);
	}
};


#endif //IDEBUG_DRAW__H

