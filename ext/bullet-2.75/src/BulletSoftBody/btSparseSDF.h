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
///D_btSparseSdf implementation by Nathanael Presson

#ifndef _14F9D17F_EAE8_4aba_B41C_292DB2AA70F3_
#define _14F9D17F_EAE8_4aba_B41C_292DB2AA70F3_

#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkEpa2.h"

// Modified Paul Hsieh hash
template <const int DWORDLEN>
unsigned int HsiehHash(const void* pdata)
{
	const unsigned short*	data=(const unsigned short*)pdata;
	unsigned				hash=DWORDLEN<<2,tmp;
	for(int i=0;i<DWORDLEN;++i)
	{
		hash	+=	data[0];
		tmp		=	(data[1]<<11)^hash;
		hash	=	(hash<<16)^tmp;
		data	+=	2;
		hash	+=	hash>>11;
	}
	hash^=hash<<3;hash+=hash>>5;
	hash^=hash<<4;hash+=hash>>17;
	hash^=hash<<25;hash+=hash>>6;
	return(hash);
}

template <const int CELLSIZE>
struct	D_btSparseSdf
{
	//
	// Inner types
	//
	struct D_IntFrac
	{
		int					b;
		int					i;
		D_btScalar			f;
	};
	struct	Cell
	{
		D_btScalar			d[CELLSIZE+1][CELLSIZE+1][CELLSIZE+1];
		int					c[3];
		int					puid;
		unsigned			hash;
		D_btCollisionShape*	pclient;
		Cell*				next;
	};
	//
	// Fields
	//

	D_btAlignedObjectArray<Cell*>		cells;	
	D_btScalar						voxelsz;
	int								puid;
	int								ncells;
	int								nprobes;
	int								nqueries;	

	//
	// Methods
	//

	//
	void					Initialize(int hashsize=2383)
	{
		cells.resize(hashsize,0);
		Reset();		
	}
	//
	void					Reset()
	{
		for(int i=0,ni=cells.size();i<ni;++i)
		{
			Cell*	pc=cells[i];
			cells[i]=0;
			while(pc)
			{
				Cell*	pn=pc->next;
				delete pc;
				pc=pn;
			}
		}
		voxelsz		=0.25;
		puid		=0;
		ncells		=0;
		nprobes		=1;
		nqueries	=1;
	}
	//
	void					GarbageCollect(int lifetime=256)
	{
		const int life=puid-lifetime;
		for(int i=0;i<cells.size();++i)
		{
			Cell*&	root=cells[i];
			Cell*	pp=0;
			Cell*	pc=root;
			while(pc)
			{
				Cell*	pn=pc->next;
				if(pc->puid<life)
				{
					if(pp) pp->next=pn; else root=pn;
					delete pc;pc=pp;--ncells;
				}
				pp=pc;pc=pn;
			}
		}
		//printf("GC[%d]: %d cells, PpQ: %f\r\n",puid,ncells,nprobes/(D_btScalar)nqueries);
		nqueries=1;
		nprobes=1;
		++puid;	///@todo: Reset puid's when int range limit D_is reached	*/ 
		/* else setup a priority list...						*/ 
	}
	//
	int						RemoveReferences(D_btCollisionShape* pcs)
	{
		int	refcount=0;
		for(int i=0;i<cells.size();++i)
		{
			Cell*&	root=cells[i];
			Cell*	pp=0;
			Cell*	pc=root;
			while(pc)
			{
				Cell*	pn=pc->next;
				if(pc->pclient==pcs)
				{
					if(pp) pp->next=pn; else root=pn;
					delete pc;pc=pp;++refcount;
				}
				pp=pc;pc=pn;
			}
		}
		return(refcount);
	}
	//
	D_btScalar				Evaluate(	const D_btVector3& x,
		D_btCollisionShape* shape,
		D_btVector3& normal,
		D_btScalar margin)
	{
		/* Lookup cell			*/ 
		const D_btVector3	scx=x/voxelsz;
		const D_IntFrac	ix=Decompose(scx.x());
		const D_IntFrac	iy=Decompose(scx.y());
		const D_IntFrac	iz=Decompose(scx.z());
		const unsigned	h=Hash(ix.b,iy.b,iz.b,shape);
		Cell*&			root=cells[static_cast<int>(h%cells.size())];
		Cell*			c=root;
		++nqueries;
		while(c)
		{
			++nprobes;
			if(	(c->hash==h)	&&
				(c->c[0]==ix.b)	&&
				(c->c[1]==iy.b)	&&
				(c->c[2]==iz.b)	&&
				(c->pclient==shape))
			{ break; }
			else
			{ c=c->next; }
		}
		if(!c)
		{
			++nprobes;		
			++ncells;
			c=new Cell();
			c->next=root;root=c;
			c->pclient=shape;
			c->hash=h;
			c->c[0]=ix.b;c->c[1]=iy.b;c->c[2]=iz.b;
			BuildCell(*c);
		}
		c->puid=puid;
		/* Extract infos		*/ 
		const int		o[]={	ix.i,iy.i,iz.i};
		const D_btScalar	d[]={	c->d[o[0]+0][o[1]+0][o[2]+0],
			c->d[o[0]+1][o[1]+0][o[2]+0],
			c->d[o[0]+1][o[1]+1][o[2]+0],
			c->d[o[0]+0][o[1]+1][o[2]+0],
			c->d[o[0]+0][o[1]+0][o[2]+1],
			c->d[o[0]+1][o[1]+0][o[2]+1],
			c->d[o[0]+1][o[1]+1][o[2]+1],
			c->d[o[0]+0][o[1]+1][o[2]+1]};
		/* Normal	*/ 
#if 1
		const D_btScalar	gx[]={	d[1]-d[0],d[2]-d[3],
			d[5]-d[4],d[6]-d[7]};
		const D_btScalar	gy[]={	d[3]-d[0],d[2]-d[1],
			d[7]-d[4],d[6]-d[5]};
		const D_btScalar	gz[]={	d[4]-d[0],d[5]-d[1],
			d[7]-d[3],d[6]-d[2]};
		normal.setX(Lerp(	Lerp(gx[0],gx[1],iy.f),
			Lerp(gx[2],gx[3],iy.f),iz.f));
		normal.setY(Lerp(	Lerp(gy[0],gy[1],ix.f),
			Lerp(gy[2],gy[3],ix.f),iz.f));
		normal.setZ(Lerp(	Lerp(gz[0],gz[1],ix.f),
			Lerp(gz[2],gz[3],ix.f),iy.f));
		normal		=	normal.normalized();
#else
		normal		=	D_btVector3(d[1]-d[0],d[3]-d[0],d[4]-d[0]).normalized();
#endif
		/* Distance	*/ 
		const D_btScalar	d0=Lerp(Lerp(d[0],d[1],ix.f),
			Lerp(d[3],d[2],ix.f),iy.f);
		const D_btScalar	d1=Lerp(Lerp(d[4],d[5],ix.f),
			Lerp(d[7],d[6],ix.f),iy.f);
		return(Lerp(d0,d1,iz.f)-margin);
	}
	//
	void					BuildCell(Cell& c)
	{
		const D_btVector3	org=D_btVector3(	(D_btScalar)c.c[0],
			(D_btScalar)c.c[1],
			(D_btScalar)c.c[2])	*
			CELLSIZE*voxelsz;
		for(int k=0;k<=CELLSIZE;++k)
		{
			const D_btScalar	z=voxelsz*k+org.z();
			for(int j=0;j<=CELLSIZE;++j)
			{
				const D_btScalar	y=voxelsz*j+org.y();
				for(int i=0;i<=CELLSIZE;++i)
				{
					const D_btScalar	x=voxelsz*i+org.x();
					c.d[i][j][k]=DistanceToShape(	D_btVector3(x,y,z),
						c.pclient);
				}
			}
		}
	}
	//
	static inline D_btScalar	DistanceToShape(const D_btVector3& x,
		D_btCollisionShape* shape)
	{
		D_btTransform	unit;
		unit.setIdentity();
		if(shape->isConvex())
		{
			D_btGjkEpaSolver2::D_sResults	res;
			D_btConvexShape*				csh=static_cast<D_btConvexShape*>(shape);
			return(D_btGjkEpaSolver2::SignedDistance(x,0,csh,unit,res));
		}
		return(0);
	}
	//
	static inline D_IntFrac	Decompose(D_btScalar x)
	{
		/* That one D_need a lot of improvements...	*/
		/* Remove test, faster floor...				*/ 
		D_IntFrac			r;
		x/=CELLSIZE;
		const int		o=x<0?(int)(-x+1):0;
		x+=o;r.b=(int)x;
		const D_btScalar	k=(x-r.b)*CELLSIZE;
		r.i=(int)k;r.f=k-r.i;r.b-=o;
		return(r);
	}
	//
	static inline D_btScalar	Lerp(D_btScalar a,D_btScalar b,D_btScalar t)
	{
		return(a+(b-a)*t);
	}



	//
	static inline unsigned int	Hash(int x,int y,int z,D_btCollisionShape* shape)
	{
		struct D_btS
		{ 
			int x,y,z;
			void* p;
		};

		D_btS myset;

		myset.x=x;myset.y=y;myset.z=z;myset.p=shape;
		const void* ptr = &myset;

		unsigned int result = HsiehHash<sizeof(D_btS)/4> (ptr);


		return result;
	}
};


#endif
