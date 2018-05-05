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
///D_btDbvt implementation by Nathanael Presson

#include "btDbvt.h"

//
typedef D_btAlignedObjectArray<D_btDbvtNode*>			D_tNodeArray;
typedef D_btAlignedObjectArray<const D_btDbvtNode*>	D_tConstNodeArray;

//
struct D_btDbvtNodeEnumerator : D_btDbvt::ICollide
{
	D_tConstNodeArray	nodes;
	void Process(const D_btDbvtNode* n) { nodes.push_back(n); }
};

//
static D_DBVT_INLINE int			indexof(const D_btDbvtNode* node)
{
	return(node->parent->childs[1]==node);
}

//
static D_DBVT_INLINE D_btDbvtVolume	merge(	const D_btDbvtVolume& a,
									  const D_btDbvtVolume& b)
{
#if (D_DBVT_MERGE_IMPL==D_DBVT_IMPL_SSE)
	D_ATTRIBUTE_ALIGNED16(char locals[sizeof(D_btDbvtAabbMm)]);
	D_btDbvtVolume&	res=*(D_btDbvtVolume*)locals;
#else
		D_btDbvtVolume	res;
#endif
	Merge(a,b,res);
	return(res);
}

// volume+edge lengths
static D_DBVT_INLINE D_btScalar		size(const D_btDbvtVolume& a)
{
	const D_btVector3	edges=a.Lengths();
	return(	edges.x()*edges.y()*edges.z()+
		edges.x()+edges.y()+edges.z());
}

//
static void						getmaxdepth(const D_btDbvtNode* node,int depth,int& maxdepth)
{
	if(node->isinternal())
	{
		getmaxdepth(node->childs[0],depth+1,maxdepth);
		getmaxdepth(node->childs[0],depth+1,maxdepth);
	} else maxdepth=D_btMax(maxdepth,depth);
}

//
static D_DBVT_INLINE void			deletenode(	D_btDbvt* pdbvt,
										   D_btDbvtNode* node)
{
	D_btAlignedFree(pdbvt->m_free);
	pdbvt->m_free=node;
}

//
static void						recursedeletenode(	D_btDbvt* pdbvt,
												  D_btDbvtNode* node)
{
	if(!node->isleaf())
	{
		recursedeletenode(pdbvt,node->childs[0]);
		recursedeletenode(pdbvt,node->childs[1]);
	}
	if(node==pdbvt->m_root) pdbvt->m_root=0;
	deletenode(pdbvt,node);
}

//
static D_DBVT_INLINE D_btDbvtNode*	createnode(	D_btDbvt* pdbvt,
										   D_btDbvtNode* parent,
										   void* data)
{
	D_btDbvtNode*	node;
	if(pdbvt->m_free)
	{ node=pdbvt->m_free;pdbvt->m_free=0; }
	else
	{ node=new(D_btAlignedAlloc(sizeof(D_btDbvtNode),16)) D_btDbvtNode(); }
	node->parent	=	parent;
	node->data		=	data;
	node->childs[1]	=	0;
	return(node);
}

//
static D_DBVT_INLINE D_btDbvtNode*	createnode(	D_btDbvt* pdbvt,
										   D_btDbvtNode* parent,
										   const D_btDbvtVolume& volume,
										   void* data)
{
	D_btDbvtNode*	node=createnode(pdbvt,parent,data);
	node->volume=volume;
	return(node);
}

//
static D_DBVT_INLINE D_btDbvtNode*	createnode(	D_btDbvt* pdbvt,
										   D_btDbvtNode* parent,
										   const D_btDbvtVolume& volume0,
										   const D_btDbvtVolume& volume1,
										   void* data)
{
	D_btDbvtNode*	node=createnode(pdbvt,parent,data);
	Merge(volume0,volume1,node->volume);
	return(node);
}

//
static void						insertleaf(	D_btDbvt* pdbvt,
										   D_btDbvtNode* root,
										   D_btDbvtNode* leaf)
{
	if(!pdbvt->m_root)
	{
		pdbvt->m_root	=	leaf;
		leaf->parent	=	0;
	}
	else
	{
		if(!root->isleaf())
		{
			do	{
				root=root->childs[Select(	leaf->volume,
					root->childs[0]->volume,
					root->childs[1]->volume)];
			} while(!root->isleaf());
		}
		D_btDbvtNode*	prev=root->parent;
		D_btDbvtNode*	node=createnode(pdbvt,prev,leaf->volume,root->volume,0);
		if(prev)
		{
			prev->childs[indexof(root)]	=	node;
			node->childs[0]				=	root;root->parent=node;
			node->childs[1]				=	leaf;leaf->parent=node;
			do	{
				if(!prev->volume.Contain(node->volume))
					Merge(prev->childs[0]->volume,prev->childs[1]->volume,prev->volume);
				else
					break;
				node=prev;
			} while(0!=(prev=node->parent));
		}
		else
		{
			node->childs[0]	=	root;root->parent=node;
			node->childs[1]	=	leaf;leaf->parent=node;
			pdbvt->m_root	=	node;
		}
	}
}

//
static D_btDbvtNode*				removeleaf(	D_btDbvt* pdbvt,
										   D_btDbvtNode* leaf)
{
	if(leaf==pdbvt->m_root)
	{
		pdbvt->m_root=0;
		return(0);
	}
	else
	{
		D_btDbvtNode*	parent=leaf->parent;
		D_btDbvtNode*	prev=parent->parent;
		D_btDbvtNode*	sibling=parent->childs[1-indexof(leaf)];			
		if(prev)
		{
			prev->childs[indexof(parent)]=sibling;
			sibling->parent=prev;
			deletenode(pdbvt,parent);
			while(prev)
			{
				const D_btDbvtVolume	pb=prev->volume;
				Merge(prev->childs[0]->volume,prev->childs[1]->volume,prev->volume);
				if(NotEqual(pb,prev->volume))
				{
					prev=prev->parent;
				} else break;
			}
			return(prev?prev:pdbvt->m_root);
		}
		else
		{								
			pdbvt->m_root=sibling;
			sibling->parent=0;
			deletenode(pdbvt,parent);
			return(pdbvt->m_root);
		}			
	}
}

//
static void						fetchleaves(D_btDbvt* pdbvt,
											D_btDbvtNode* root,
											D_tNodeArray& leaves,
											int depth=-1)
{
	if(root->isinternal()&&depth)
	{
		fetchleaves(pdbvt,root->childs[0],leaves,depth-1);
		fetchleaves(pdbvt,root->childs[1],leaves,depth-1);
		deletenode(pdbvt,root);
	}
	else
	{
		leaves.push_back(root);
	}
}

//
static void						split(	const D_tNodeArray& leaves,
									  D_tNodeArray& left,
									  D_tNodeArray& right,
									  const D_btVector3& org,
									  const D_btVector3& axis)
{
	left.resize(0);
	right.resize(0);
	for(int i=0,ni=leaves.size();i<ni;++i)
	{
		if(D_btDot(axis,leaves[i]->volume.Center()-org)<0)
			left.push_back(leaves[i]);
		else
			right.push_back(leaves[i]);
	}
}

//
static D_btDbvtVolume				bounds(	const D_tNodeArray& leaves)
{
#if D_DBVT_MERGE_IMPL==D_DBVT_IMPL_SSE
	D_ATTRIBUTE_ALIGNED16(char	locals[sizeof(D_btDbvtVolume)]);
	D_btDbvtVolume&	volume=*(D_btDbvtVolume*)locals;
	volume=leaves[0]->volume;
#else
	D_btDbvtVolume volume=leaves[0]->volume;
#endif
	for(int i=1,ni=leaves.size();i<ni;++i)
	{
		Merge(volume,leaves[i]->volume,volume);
	}
	return(volume);
}

//
static void						bottomup(	D_btDbvt* pdbvt,
										 D_tNodeArray& leaves)
{
	while(leaves.size()>1)
	{
		D_btScalar	minsize=D_SIMD_INFINITY;
		int			minidx[2]={-1,-1};
		for(int i=0;i<leaves.size();++i)
		{
			for(int j=i+1;j<leaves.size();++j)
			{
				const D_btScalar	sz=size(merge(leaves[i]->volume,leaves[j]->volume));
				if(sz<minsize)
				{
					minsize		=	sz;
					minidx[0]	=	i;
					minidx[1]	=	j;
				}
			}
		}
		D_btDbvtNode*	n[]	=	{leaves[minidx[0]],leaves[minidx[1]]};
		D_btDbvtNode*	p	=	createnode(pdbvt,0,n[0]->volume,n[1]->volume,0);
		p->childs[0]		=	n[0];
		p->childs[1]		=	n[1];
		n[0]->parent		=	p;
		n[1]->parent		=	p;
		leaves[minidx[0]]	=	p;
		leaves.swap(minidx[1],leaves.size()-1);
		leaves.pop_back();
	}
}

//
static D_btDbvtNode*			topdown(D_btDbvt* pdbvt,
									D_tNodeArray& leaves,
									int bu_treshold)
{
	static const D_btVector3	axis[]={D_btVector3(1,0,0),
		D_btVector3(0,1,0),
		D_btVector3(0,0,1)};
	if(leaves.size()>1)
	{
		if(leaves.size()>bu_treshold)
		{
			const D_btDbvtVolume	vol=bounds(leaves);
			const D_btVector3			org=vol.Center();
			D_tNodeArray				sets[2];
			int						bestaxis=-1;
			int						bestmidp=leaves.size();
			int						splitcount[3][2]={{0,0},{0,0},{0,0}};
			int i;
			for( i=0;i<leaves.size();++i)
			{
				const D_btVector3	x=leaves[i]->volume.Center()-org;
				for(int j=0;j<3;++j)
				{
					++splitcount[j][D_btDot(x,axis[j])>0?1:0];
				}
			}
			for( i=0;i<3;++i)
			{
				if((splitcount[i][0]>0)&&(splitcount[i][1]>0))
				{
					const int	midp=(int)D_btFabs(D_btScalar(splitcount[i][0]-splitcount[i][1]));
					if(midp<bestmidp)
					{
						bestaxis=i;
						bestmidp=midp;
					}
				}
			}
			if(bestaxis>=0)
			{
				sets[0].reserve(splitcount[bestaxis][0]);
				sets[1].reserve(splitcount[bestaxis][1]);
				split(leaves,sets[0],sets[1],org,axis[bestaxis]);
			}
			else
			{
				sets[0].reserve(leaves.size()/2+1);
				sets[1].reserve(leaves.size()/2);
				for(int i=0,ni=leaves.size();i<ni;++i)
				{
					sets[i&1].push_back(leaves[i]);
				}
			}
			D_btDbvtNode*	node=createnode(pdbvt,0,vol,0);
			node->childs[0]=topdown(pdbvt,sets[0],bu_treshold);
			node->childs[1]=topdown(pdbvt,sets[1],bu_treshold);
			node->childs[0]->parent=node;
			node->childs[1]->parent=node;
			return(node);
		}
		else
		{
			bottomup(pdbvt,leaves);
			return(leaves[0]);
		}
	}
	return(leaves[0]);
}

//
static D_DBVT_INLINE D_btDbvtNode*	sort(D_btDbvtNode* n,D_btDbvtNode*& r)
{
	D_btDbvtNode*	p=n->parent;
	D_btAssert(n->isinternal());
	if(p>n)
	{
		const int		i=indexof(n);
		const int		j=1-i;
		D_btDbvtNode*	s=p->childs[j];
		D_btDbvtNode*	q=p->parent;
		D_btAssert(n==p->childs[i]);
		if(q) q->childs[indexof(p)]=n; else r=n;
		s->parent=n;
		p->parent=n;
		n->parent=q;
		p->childs[0]=n->childs[0];
		p->childs[1]=n->childs[1];
		n->childs[0]->parent=p;
		n->childs[1]->parent=p;
		n->childs[i]=p;
		n->childs[j]=s;
		D_btSwap(p->volume,n->volume);
		return(p);
	}
	return(n);
}

#if 0
static D_DBVT_INLINE D_btDbvtNode*	walkup(D_btDbvtNode* n,int count)
{
	while(n&&(count--)) n=n->parent;
	return(n);
}
#endif

//
// Api
//

//
D_btDbvt::D_btDbvt()
{
	m_root		=	0;
	m_free		=	0;
	m_lkhd		=	-1;
	m_leaves	=	0;
	m_opath		=	0;
}

//
D_btDbvt::~D_btDbvt()
{
	clear();
}

//
void			D_btDbvt::clear()
{
	if(m_root)	
		recursedeletenode(this,m_root);
	D_btAlignedFree(m_free);
	m_free=0;
	m_lkhd		=	-1;
	m_stkStack.clear();
	m_opath		=	0;
	
}

//
void			D_btDbvt::optimizeBottomUp()
{
	if(m_root)
	{
		D_tNodeArray leaves;
		leaves.reserve(m_leaves);
		fetchleaves(this,m_root,leaves);
		bottomup(this,leaves);
		m_root=leaves[0];
	}
}

//
void			D_btDbvt::optimizeTopDown(int bu_treshold)
{
	if(m_root)
	{
		D_tNodeArray	leaves;
		leaves.reserve(m_leaves);
		fetchleaves(this,m_root,leaves);
		m_root=topdown(this,leaves,bu_treshold);
	}
}

//
void			D_btDbvt::optimizeIncremental(int passes)
{
	if(passes<0) passes=m_leaves;
	if(m_root&&(passes>0))
	{
		do	{
			D_btDbvtNode*		node=m_root;
			unsigned	bit=0;
			while(node->isinternal())
			{
				node=sort(node,m_root)->childs[(m_opath>>bit)&1];
				bit=(bit+1)&(sizeof(unsigned)*8-1);
			}
			update(node);
			++m_opath;
		} while(--passes);
	}
}

//
D_btDbvtNode*	D_btDbvt::insert(const D_btDbvtVolume& volume,void* data)
{
	D_btDbvtNode*	leaf=createnode(this,0,volume,data);
	insertleaf(this,m_root,leaf);
	++m_leaves;
	return(leaf);
}

//
void			D_btDbvt::update(D_btDbvtNode* leaf,int lookahead)
{
	D_btDbvtNode*	root=removeleaf(this,leaf);
	if(root)
	{
		if(lookahead>=0)
		{
			for(int i=0;(i<lookahead)&&root->parent;++i)
			{
				root=root->parent;
			}
		} else root=m_root;
	}
	insertleaf(this,root,leaf);
}

//
void			D_btDbvt::update(D_btDbvtNode* leaf,D_btDbvtVolume& volume)
{
	D_btDbvtNode*	root=removeleaf(this,leaf);
	if(root)
	{
		if(m_lkhd>=0)
		{
			for(int i=0;(i<m_lkhd)&&root->parent;++i)
			{
				root=root->parent;
			}
		} else root=m_root;
	}
	leaf->volume=volume;
	insertleaf(this,root,leaf);
}

//
bool			D_btDbvt::update(D_btDbvtNode* leaf,D_btDbvtVolume& volume,const D_btVector3& velocity,D_btScalar margin)
{
	if(leaf->volume.Contain(volume)) return(false);
	volume.Expand(D_btVector3(margin,margin,margin));
	volume.SignedExpand(velocity);
	update(leaf,volume);
	return(true);
}

//
bool			D_btDbvt::update(D_btDbvtNode* leaf,D_btDbvtVolume& volume,const D_btVector3& velocity)
{
	if(leaf->volume.Contain(volume)) return(false);
	volume.SignedExpand(velocity);
	update(leaf,volume);
	return(true);
}

//
bool			D_btDbvt::update(D_btDbvtNode* leaf,D_btDbvtVolume& volume,D_btScalar margin)
{
	if(leaf->volume.Contain(volume)) return(false);
	volume.Expand(D_btVector3(margin,margin,margin));
	update(leaf,volume);
	return(true);
}

//
void			D_btDbvt::remove(D_btDbvtNode* leaf)
{
	removeleaf(this,leaf);
	deletenode(this,leaf);
	--m_leaves;
}

//
void			D_btDbvt::write(IWriter* iwriter) const
{
	D_btDbvtNodeEnumerator	nodes;
	nodes.nodes.reserve(m_leaves*2);
	enumNodes(m_root,nodes);
	iwriter->Prepare(m_root,nodes.nodes.size());
	for(int i=0;i<nodes.nodes.size();++i)
	{
		const D_btDbvtNode* n=nodes.nodes[i];
		int			p=-1;
		if(n->parent) p=nodes.nodes.findLinearSearch(n->parent);
		if(n->isinternal())
		{
			const int	c0=nodes.nodes.findLinearSearch(n->childs[0]);
			const int	c1=nodes.nodes.findLinearSearch(n->childs[1]);
			iwriter->WriteNode(n,i,p,c0,c1);
		}
		else
		{
			iwriter->WriteLeaf(n,i,p);
		}	
	}
}

//
void			D_btDbvt::clone(D_btDbvt& dest,IClone* iclone) const
{
	dest.clear();
	if(m_root!=0)
	{	
		D_btAlignedObjectArray<D_sStkCLN>	stack;
		stack.reserve(m_leaves);
		stack.push_back(D_sStkCLN(m_root,0));
		do	{
			const int		i=stack.size()-1;
			const D_sStkCLN	e=stack[i];
			D_btDbvtNode*			n=createnode(&dest,e.parent,e.node->volume,e.node->data);
			stack.pop_back();
			if(e.parent!=0)
				e.parent->childs[i&1]=n;
			else
				dest.m_root=n;
			if(e.node->isinternal())
			{
				stack.push_back(D_sStkCLN(e.node->childs[0],n));
				stack.push_back(D_sStkCLN(e.node->childs[1],n));
			}
			else
			{
				iclone->CloneLeaf(n);
			}
		} while(stack.size()>0);
	}
}

//
int				D_btDbvt::maxdepth(const D_btDbvtNode* node)
{
	int	depth=0;
	if(node) getmaxdepth(node,1,depth);
	return(depth);
}

//
int				D_btDbvt::countLeaves(const D_btDbvtNode* node)
{
	if(node->isinternal())
		return(countLeaves(node->childs[0])+countLeaves(node->childs[1]));
	else
		return(1);
}

//
void			D_btDbvt::extractLeaves(const D_btDbvtNode* node,D_btAlignedObjectArray<const D_btDbvtNode*>& leaves)
{
	if(node->isinternal())
	{
		extractLeaves(node->childs[0],leaves);
		extractLeaves(node->childs[1],leaves);
	}
	else
	{
		leaves.push_back(node);
	}	
}

//
#if DBVT_ENABLE_BENCHMARK

#include <stdio.h>
#include <stdlib.h>
#include "LinearMath/btQuickProf.h"

/*
q6600,2.4ghz

/Ox /Ob2 /Oi /Ot /I "." /I "..\.." /I "..\..\src" /D "NDEBUG" /D "_LIB" /D "_WINDOWS" /D "_CRT_SECURE_NO_DEPRECATE" /D "_CRT_NONSTDC_NO_DEPRECATE" /D "WIN32"
/GF /FD /MT /GS- /Gy /arch:SSE2 /Zc:wchar_t- /Fp"..\..\out\release8\build\libbulletcollision\libbulletcollision.pch"
/Fo"..\..\out\release8\build\libbulletcollision\\"
/Fd"..\..\out\release8\build\libbulletcollision\bulletcollision.pdb"
/W3 /nologo /c /Wp64 /Zi /errorReport:prompt

Benchmarking dbvt...
World scale: 100.000000
Extents base: 1.000000
Extents range: 4.000000
Leaves: 8192
sizeof(D_btDbvtVolume): 32 bytes
sizeof(D_btDbvtNode):   44 bytes
[1] D_btDbvtVolume intersections: 3499 ms (-1%)
[2] D_btDbvtVolume merges: 1934 ms (0%)
[3] D_btDbvt::collideTT: 5485 ms (-21%)
[4] D_btDbvt::collideTT self: 2814 ms (-20%)
[5] D_btDbvt::collideTT xform: 7379 ms (-1%)
[6] D_btDbvt::collideTT xform,self: 7270 ms (-2%)
[7] D_btDbvt::rayTest: 6314 ms (0%),(332143 r/s)
[8] insert/remove: 2093 ms (0%),(1001983 ir/s)
[9] updates (teleport): 1879 ms (-3%),(1116100 u/s)
[10] updates (jitter): 1244 ms (-4%),(1685813 u/s)
[11] optimize (incremental): 2514 ms (0%),(1668000 o/s)
[12] D_btDbvtVolume notequal: 3659 ms (0%)
[13] culling(OCL+fullsort): 2218 ms (0%),(461 t/s)
[14] culling(OCL+qsort): 3688 ms (5%),(2221 t/s)
[15] culling(KDOP+qsort): 1139 ms (-1%),(7192 t/s)
[16] insert/remove batch(256): 5092 ms (0%),(823704 bir/s)
[17] D_btDbvtVolume select: 3419 ms (0%)
*/

struct D_btDbvtBenchmark
{
	struct D_NilPolicy : D_btDbvt::ICollide
	{
		D_NilPolicy() : m_pcount(0),m_depth(-D_SIMD_INFINITY),m_checksort(true)		{}
		void	Process(const D_btDbvtNode*,const D_btDbvtNode*)				{ ++m_pcount; }
		void	Process(const D_btDbvtNode*)									{ ++m_pcount; }
		void	Process(const D_btDbvtNode*,D_btScalar depth)
		{
			++m_pcount;
			if(m_checksort)
			{ if(depth>=m_depth) m_depth=depth; else printf("wrong depth: %f (D_should be >= %f)\r\n",depth,m_depth); }
		}
		int			m_pcount;
		D_btScalar	m_depth;
		bool		m_checksort;
	};
	struct D_P14 : D_btDbvt::ICollide
	{
		struct D_Node
		{
			const D_btDbvtNode*	leaf;
			D_btScalar			depth;
		};
		void Process(const D_btDbvtNode* leaf,D_btScalar depth)
		{
			D_Node	n;
			n.leaf	=	leaf;
			n.depth	=	depth;
		}
		static int sortfnc(const D_Node& a,const D_Node& b)
		{
			if(a.depth<b.depth) return(+1);
			if(a.depth>b.depth) return(-1);
			return(0);
		}
		D_btAlignedObjectArray<D_Node>		m_nodes;
	};
	struct D_P15 : D_btDbvt::ICollide
	{
		struct D_Node
		{
			const D_btDbvtNode*	leaf;
			D_btScalar			depth;
		};
		void Process(const D_btDbvtNode* leaf)
		{
			D_Node	n;
			n.leaf	=	leaf;
			n.depth	=	dot(leaf->volume.Center(),m_axis);
		}
		static int sortfnc(const D_Node& a,const D_Node& b)
		{
			if(a.depth<b.depth) return(+1);
			if(a.depth>b.depth) return(-1);
			return(0);
		}
		D_btAlignedObjectArray<D_Node>		m_nodes;
		D_btVector3						m_axis;
	};
	static D_btScalar			RandUnit()
	{
		return(rand()/(D_btScalar)RAND_MAX);
	}
	static D_btVector3		RandVector3()
	{
		return(D_btVector3(RandUnit(),RandUnit(),RandUnit()));
	}
	static D_btVector3		RandVector3(D_btScalar cs)
	{
		return(RandVector3()*cs-D_btVector3(cs,cs,cs)/2);
	}
	static D_btDbvtVolume	RandVolume(D_btScalar cs,D_btScalar eb,D_btScalar es)
	{
		return(D_btDbvtVolume::FromCE(RandVector3(cs),D_btVector3(eb,eb,eb)+RandVector3()*es));
	}
	static D_btTransform		RandTransform(D_btScalar cs)
	{
		D_btTransform	t;
		t.setOrigin(RandVector3(cs));
		t.setRotation(D_btQuaternion(RandUnit()*D_SIMD_PI*2,RandUnit()*D_SIMD_PI*2,RandUnit()*D_SIMD_PI*2).normalized());
		return(t);
	}
	static void				RandTree(D_btScalar cs,D_btScalar eb,D_btScalar es,int leaves,D_btDbvt& dbvt)
	{
		dbvt.clear();
		for(int i=0;i<leaves;++i)
		{
			dbvt.insert(RandVolume(cs,eb,es),0);
		}
	}
};

void			D_btDbvt::benchmark()
{
	static const D_btScalar	cfgVolumeCenterScale		=	100;
	static const D_btScalar	cfgVolumeExentsBase			=	1;
	static const D_btScalar	cfgVolumeExentsScale		=	4;
	static const int		cfgLeaves					=	8192;
	static const bool		cfgEnable					=	true;

	//[1] D_btDbvtVolume intersections
	bool					cfgBenchmark1_Enable		=	cfgEnable;
	static const int		cfgBenchmark1_Iterations	=	8;
	static const int		cfgBenchmark1_Reference		=	3499;
	//[2] D_btDbvtVolume merges
	bool					cfgBenchmark2_Enable		=	cfgEnable;
	static const int		cfgBenchmark2_Iterations	=	4;
	static const int		cfgBenchmark2_Reference		=	1945;
	//[3] D_btDbvt::collideTT
	bool					cfgBenchmark3_Enable		=	cfgEnable;
	static const int		cfgBenchmark3_Iterations	=	512;
	static const int		cfgBenchmark3_Reference		=	5485;
	//[4] D_btDbvt::collideTT self
	bool					cfgBenchmark4_Enable		=	cfgEnable;
	static const int		cfgBenchmark4_Iterations	=	512;
	static const int		cfgBenchmark4_Reference		=	2814;
	//[5] D_btDbvt::collideTT xform
	bool					cfgBenchmark5_Enable		=	cfgEnable;
	static const int		cfgBenchmark5_Iterations	=	512;
	static const D_btScalar	cfgBenchmark5_OffsetScale	=	2;
	static const int		cfgBenchmark5_Reference		=	7379;
	//[6] D_btDbvt::collideTT xform,self
	bool					cfgBenchmark6_Enable		=	cfgEnable;
	static const int		cfgBenchmark6_Iterations	=	512;
	static const D_btScalar	cfgBenchmark6_OffsetScale	=	2;
	static const int		cfgBenchmark6_Reference		=	7270;
	//[7] D_btDbvt::rayTest
	bool					cfgBenchmark7_Enable		=	cfgEnable;
	static const int		cfgBenchmark7_Passes		=	32;
	static const int		cfgBenchmark7_Iterations	=	65536;
	static const int		cfgBenchmark7_Reference		=	6307;
	//[8] insert/remove
	bool					cfgBenchmark8_Enable		=	cfgEnable;
	static const int		cfgBenchmark8_Passes		=	32;
	static const int		cfgBenchmark8_Iterations	=	65536;
	static const int		cfgBenchmark8_Reference		=	2105;
	//[9] updates (teleport)
	bool					cfgBenchmark9_Enable		=	cfgEnable;
	static const int		cfgBenchmark9_Passes		=	32;
	static const int		cfgBenchmark9_Iterations	=	65536;
	static const int		cfgBenchmark9_Reference		=	1879;
	//[10] updates (jitter)
	bool					cfgBenchmark10_Enable		=	cfgEnable;
	static const D_btScalar	cfgBenchmark10_Scale		=	cfgVolumeCenterScale/10000;
	static const int		cfgBenchmark10_Passes		=	32;
	static const int		cfgBenchmark10_Iterations	=	65536;
	static const int		cfgBenchmark10_Reference	=	1244;
	//[11] optimize (incremental)
	bool					cfgBenchmark11_Enable		=	cfgEnable;
	static const int		cfgBenchmark11_Passes		=	64;
	static const int		cfgBenchmark11_Iterations	=	65536;
	static const int		cfgBenchmark11_Reference	=	2510;
	//[12] D_btDbvtVolume notequal
	bool					cfgBenchmark12_Enable		=	cfgEnable;
	static const int		cfgBenchmark12_Iterations	=	32;
	static const int		cfgBenchmark12_Reference	=	3677;
	//[13] culling(OCL+fullsort)
	bool					cfgBenchmark13_Enable		=	cfgEnable;
	static const int		cfgBenchmark13_Iterations	=	1024;
	static const int		cfgBenchmark13_Reference	=	2231;
	//[14] culling(OCL+qsort)
	bool					cfgBenchmark14_Enable		=	cfgEnable;
	static const int		cfgBenchmark14_Iterations	=	8192;
	static const int		cfgBenchmark14_Reference	=	3500;
	//[15] culling(KDOP+qsort)
	bool					cfgBenchmark15_Enable		=	cfgEnable;
	static const int		cfgBenchmark15_Iterations	=	8192;
	static const int		cfgBenchmark15_Reference	=	1151;
	//[16] insert/remove batch
	bool					cfgBenchmark16_Enable		=	cfgEnable;
	static const int		cfgBenchmark16_BatchCount	=	256;
	static const int		cfgBenchmark16_Passes		=	16384;
	static const int		cfgBenchmark16_Reference	=	5138;
	//[17] select
	bool					cfgBenchmark17_Enable		=	cfgEnable;
	static const int		cfgBenchmark17_Iterations	=	4;
	static const int		cfgBenchmark17_Reference	=	3390;

	D_btClock					wallclock;
	printf("Benchmarking dbvt...\r\n");
	printf("\D_tWorld scale: %f\r\n",cfgVolumeCenterScale);
	printf("\D_tExtents base: %f\r\n",cfgVolumeExentsBase);
	printf("\D_tExtents range: %f\r\n",cfgVolumeExentsScale);
	printf("\D_tLeaves: %u\r\n",cfgLeaves);
	printf("\tsizeof(D_btDbvtVolume): %u bytes\r\n",sizeof(D_btDbvtVolume));
	printf("\tsizeof(D_btDbvtNode):   %u bytes\r\n",sizeof(D_btDbvtNode));
	if(cfgBenchmark1_Enable)
	{// Benchmark 1	
		srand(380843);
		D_btAlignedObjectArray<D_btDbvtVolume>	volumes;
		D_btAlignedObjectArray<bool>			results;
		volumes.resize(cfgLeaves);
		results.resize(cfgLeaves);
		for(int i=0;i<cfgLeaves;++i)
		{
			volumes[i]=D_btDbvtBenchmark::RandVolume(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale);
		}
		printf("[1] D_btDbvtVolume intersections: ");
		wallclock.reset();
		for(int i=0;i<cfgBenchmark1_Iterations;++i)
		{
			for(int j=0;j<cfgLeaves;++j)
			{
				for(int k=0;k<cfgLeaves;++k)
				{
					results[k]=Intersect(volumes[j],volumes[k]);
				}
			}
		}
		const int time=(int)wallclock.getTimeMilliseconds();
		printf("%u ms (%i%%)\r\n",time,(time-cfgBenchmark1_Reference)*100/time);
	}
	if(cfgBenchmark2_Enable)
	{// Benchmark 2	
		srand(380843);
		D_btAlignedObjectArray<D_btDbvtVolume>	volumes;
		D_btAlignedObjectArray<D_btDbvtVolume>	results;
		volumes.resize(cfgLeaves);
		results.resize(cfgLeaves);
		for(int i=0;i<cfgLeaves;++i)
		{
			volumes[i]=D_btDbvtBenchmark::RandVolume(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale);
		}
		printf("[2] D_btDbvtVolume merges: ");
		wallclock.reset();
		for(int i=0;i<cfgBenchmark2_Iterations;++i)
		{
			for(int j=0;j<cfgLeaves;++j)
			{
				for(int k=0;k<cfgLeaves;++k)
				{
					Merge(volumes[j],volumes[k],results[k]);
				}
			}
		}
		const int time=(int)wallclock.getTimeMilliseconds();
		printf("%u ms (%i%%)\r\n",time,(time-cfgBenchmark2_Reference)*100/time);
	}
	if(cfgBenchmark3_Enable)
	{// Benchmark 3	
		srand(380843);
		D_btDbvt						dbvt[2];
		D_btDbvtBenchmark::D_NilPolicy	policy;
		D_btDbvtBenchmark::RandTree(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale,cfgLeaves,dbvt[0]);
		D_btDbvtBenchmark::RandTree(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale,cfgLeaves,dbvt[1]);
		dbvt[0].optimizeTopDown();
		dbvt[1].optimizeTopDown();
		printf("[3] D_btDbvt::collideTT: ");
		wallclock.reset();
		for(int i=0;i<cfgBenchmark3_Iterations;++i)
		{
			D_btDbvt::collideTT(dbvt[0].m_root,dbvt[1].m_root,policy);
		}
		const int time=(int)wallclock.getTimeMilliseconds();
		printf("%u ms (%i%%)\r\n",time,(time-cfgBenchmark3_Reference)*100/time);
	}
	if(cfgBenchmark4_Enable)
	{// Benchmark 4
		srand(380843);
		D_btDbvt						dbvt;
		D_btDbvtBenchmark::D_NilPolicy	policy;
		D_btDbvtBenchmark::RandTree(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale,cfgLeaves,dbvt);
		dbvt.optimizeTopDown();
		printf("[4] D_btDbvt::collideTT self: ");
		wallclock.reset();
		for(int i=0;i<cfgBenchmark4_Iterations;++i)
		{
			D_btDbvt::collideTT(dbvt.m_root,dbvt.m_root,policy);
		}
		const int time=(int)wallclock.getTimeMilliseconds();
		printf("%u ms (%i%%)\r\n",time,(time-cfgBenchmark4_Reference)*100/time);
	}
	if(cfgBenchmark5_Enable)
	{// Benchmark 5	
		srand(380843);
		D_btDbvt								dbvt[2];
		D_btAlignedObjectArray<D_btTransform>	transforms;
		D_btDbvtBenchmark::D_NilPolicy			policy;
		transforms.resize(cfgBenchmark5_Iterations);
		for(int i=0;i<transforms.size();++i)
		{
			transforms[i]=D_btDbvtBenchmark::RandTransform(cfgVolumeCenterScale*cfgBenchmark5_OffsetScale);
		}
		D_btDbvtBenchmark::RandTree(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale,cfgLeaves,dbvt[0]);
		D_btDbvtBenchmark::RandTree(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale,cfgLeaves,dbvt[1]);
		dbvt[0].optimizeTopDown();
		dbvt[1].optimizeTopDown();
		printf("[5] D_btDbvt::collideTT xform: ");
		wallclock.reset();
		for(int i=0;i<cfgBenchmark5_Iterations;++i)
		{
			D_btDbvt::collideTT(dbvt[0].m_root,dbvt[1].m_root,transforms[i],policy);
		}
		const int time=(int)wallclock.getTimeMilliseconds();
		printf("%u ms (%i%%)\r\n",time,(time-cfgBenchmark5_Reference)*100/time);
	}
	if(cfgBenchmark6_Enable)
	{// Benchmark 6	
		srand(380843);
		D_btDbvt								dbvt;
		D_btAlignedObjectArray<D_btTransform>	transforms;
		D_btDbvtBenchmark::D_NilPolicy			policy;
		transforms.resize(cfgBenchmark6_Iterations);
		for(int i=0;i<transforms.size();++i)
		{
			transforms[i]=D_btDbvtBenchmark::RandTransform(cfgVolumeCenterScale*cfgBenchmark6_OffsetScale);
		}
		D_btDbvtBenchmark::RandTree(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale,cfgLeaves,dbvt);
		dbvt.optimizeTopDown();
		printf("[6] D_btDbvt::collideTT xform,self: ");
		wallclock.reset();
		for(int i=0;i<cfgBenchmark6_Iterations;++i)
		{
			D_btDbvt::collideTT(dbvt.m_root,dbvt.m_root,transforms[i],policy);		
		}
		const int time=(int)wallclock.getTimeMilliseconds();
		printf("%u ms (%i%%)\r\n",time,(time-cfgBenchmark6_Reference)*100/time);
	}
	if(cfgBenchmark7_Enable)
	{// Benchmark 7	
		srand(380843);
		D_btDbvt								dbvt;
		D_btAlignedObjectArray<D_btVector3>		rayorg;
		D_btAlignedObjectArray<D_btVector3>		raydir;
		D_btDbvtBenchmark::D_NilPolicy			policy;
		rayorg.resize(cfgBenchmark7_Iterations);
		raydir.resize(cfgBenchmark7_Iterations);
		for(int i=0;i<rayorg.size();++i)
		{
			rayorg[i]=D_btDbvtBenchmark::RandVector3(cfgVolumeCenterScale*2);
			raydir[i]=D_btDbvtBenchmark::RandVector3(cfgVolumeCenterScale*2);
		}
		D_btDbvtBenchmark::RandTree(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale,cfgLeaves,dbvt);
		dbvt.optimizeTopDown();
		printf("[7] D_btDbvt::rayTest: ");
		wallclock.reset();
		for(int i=0;i<cfgBenchmark7_Passes;++i)
		{
			for(int j=0;j<cfgBenchmark7_Iterations;++j)
			{
				D_btDbvt::rayTest(dbvt.m_root,rayorg[j],rayorg[j]+raydir[j],policy);
			}
		}
		const int	time=(int)wallclock.getTimeMilliseconds();
		unsigned	rays=cfgBenchmark7_Passes*cfgBenchmark7_Iterations;
		printf("%u ms (%i%%),(%u r/s)\r\n",time,(time-cfgBenchmark7_Reference)*100/time,(rays*1000)/time);
	}
	if(cfgBenchmark8_Enable)
	{// Benchmark 8	
		srand(380843);
		D_btDbvt								dbvt;
		D_btDbvtBenchmark::RandTree(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale,cfgLeaves,dbvt);
		dbvt.optimizeTopDown();
		printf("[8] insert/remove: ");
		wallclock.reset();
		for(int i=0;i<cfgBenchmark8_Passes;++i)
		{
			for(int j=0;j<cfgBenchmark8_Iterations;++j)
			{
				dbvt.remove(dbvt.insert(D_btDbvtBenchmark::RandVolume(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale),0));
			}
		}
		const int	time=(int)wallclock.getTimeMilliseconds();
		const int	ir=cfgBenchmark8_Passes*cfgBenchmark8_Iterations;
		printf("%u ms (%i%%),(%u ir/s)\r\n",time,(time-cfgBenchmark8_Reference)*100/time,ir*1000/time);
	}
	if(cfgBenchmark9_Enable)
	{// Benchmark 9	
		srand(380843);
		D_btDbvt										dbvt;
		D_btAlignedObjectArray<const D_btDbvtNode*>	leaves;
		D_btDbvtBenchmark::RandTree(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale,cfgLeaves,dbvt);
		dbvt.optimizeTopDown();
		dbvt.extractLeaves(dbvt.m_root,leaves);
		printf("[9] updates (teleport): ");
		wallclock.reset();
		for(int i=0;i<cfgBenchmark9_Passes;++i)
		{
			for(int j=0;j<cfgBenchmark9_Iterations;++j)
			{
				dbvt.update(const_cast<D_btDbvtNode*>(leaves[rand()%cfgLeaves]),
					D_btDbvtBenchmark::RandVolume(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale));
			}
		}
		const int	time=(int)wallclock.getTimeMilliseconds();
		const int	up=cfgBenchmark9_Passes*cfgBenchmark9_Iterations;
		printf("%u ms (%i%%),(%u u/s)\r\n",time,(time-cfgBenchmark9_Reference)*100/time,up*1000/time);
	}
	if(cfgBenchmark10_Enable)
	{// Benchmark 10	
		srand(380843);
		D_btDbvt										dbvt;
		D_btAlignedObjectArray<const D_btDbvtNode*>	leaves;
		D_btAlignedObjectArray<D_btVector3>				vectors;
		vectors.resize(cfgBenchmark10_Iterations);
		for(int i=0;i<vectors.size();++i)
		{
			vectors[i]=(D_btDbvtBenchmark::RandVector3()*2-D_btVector3(1,1,1))*cfgBenchmark10_Scale;
		}
		D_btDbvtBenchmark::RandTree(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale,cfgLeaves,dbvt);
		dbvt.optimizeTopDown();
		dbvt.extractLeaves(dbvt.m_root,leaves);
		printf("[10] updates (jitter): ");
		wallclock.reset();

		for(int i=0;i<cfgBenchmark10_Passes;++i)
		{
			for(int j=0;j<cfgBenchmark10_Iterations;++j)
			{			
				const D_btVector3&	d=vectors[j];
				D_btDbvtNode*		l=const_cast<D_btDbvtNode*>(leaves[rand()%cfgLeaves]);
				D_btDbvtVolume		v=D_btDbvtVolume::FromMM(l->volume.Mins()+d,l->volume.Maxs()+d);
				dbvt.update(l,v);
			}
		}
		const int	time=(int)wallclock.getTimeMilliseconds();
		const int	up=cfgBenchmark10_Passes*cfgBenchmark10_Iterations;
		printf("%u ms (%i%%),(%u u/s)\r\n",time,(time-cfgBenchmark10_Reference)*100/time,up*1000/time);
	}
	if(cfgBenchmark11_Enable)
	{// Benchmark 11	
		srand(380843);
		D_btDbvt										dbvt;
		D_btDbvtBenchmark::RandTree(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale,cfgLeaves,dbvt);
		dbvt.optimizeTopDown();
		printf("[11] optimize (incremental): ");
		wallclock.reset();	
		for(int i=0;i<cfgBenchmark11_Passes;++i)
		{
			dbvt.optimizeIncremental(cfgBenchmark11_Iterations);
		}
		const int	time=(int)wallclock.getTimeMilliseconds();
		const int	op=cfgBenchmark11_Passes*cfgBenchmark11_Iterations;
		printf("%u ms (%i%%),(%u o/s)\r\n",time,(time-cfgBenchmark11_Reference)*100/time,op/time*1000);
	}
	if(cfgBenchmark12_Enable)
	{// Benchmark 12	
		srand(380843);
		D_btAlignedObjectArray<D_btDbvtVolume>	volumes;
		D_btAlignedObjectArray<bool>				results;
		volumes.resize(cfgLeaves);
		results.resize(cfgLeaves);
		for(int i=0;i<cfgLeaves;++i)
		{
			volumes[i]=D_btDbvtBenchmark::RandVolume(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale);
		}
		printf("[12] D_btDbvtVolume notequal: ");
		wallclock.reset();
		for(int i=0;i<cfgBenchmark12_Iterations;++i)
		{
			for(int j=0;j<cfgLeaves;++j)
			{
				for(int k=0;k<cfgLeaves;++k)
				{
					results[k]=NotEqual(volumes[j],volumes[k]);
				}
			}
		}
		const int time=(int)wallclock.getTimeMilliseconds();
		printf("%u ms (%i%%)\r\n",time,(time-cfgBenchmark12_Reference)*100/time);
	}
	if(cfgBenchmark13_Enable)
	{// Benchmark 13	
		srand(380843);
		D_btDbvt								dbvt;
		D_btAlignedObjectArray<D_btVector3>		vectors;
		D_btDbvtBenchmark::D_NilPolicy			policy;
		vectors.resize(cfgBenchmark13_Iterations);
		for(int i=0;i<vectors.size();++i)
		{
			vectors[i]=(D_btDbvtBenchmark::RandVector3()*2-D_btVector3(1,1,1)).normalized();
		}
		D_btDbvtBenchmark::RandTree(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale,cfgLeaves,dbvt);
		dbvt.optimizeTopDown();
		printf("[13] culling(OCL+fullsort): ");
		wallclock.reset();	
		for(int i=0;i<cfgBenchmark13_Iterations;++i)
		{
			static const D_btScalar	offset=0;
			policy.m_depth=-D_SIMD_INFINITY;
			dbvt.collideOCL(dbvt.m_root,&vectors[i],&offset,vectors[i],1,policy);
		}
		const int	time=(int)wallclock.getTimeMilliseconds();
		const int	t=cfgBenchmark13_Iterations;
		printf("%u ms (%i%%),(%u t/s)\r\n",time,(time-cfgBenchmark13_Reference)*100/time,(t*1000)/time);
	}
	if(cfgBenchmark14_Enable)
	{// Benchmark 14	
		srand(380843);
		D_btDbvt								dbvt;
		D_btAlignedObjectArray<D_btVector3>		vectors;
		D_btDbvtBenchmark::D_P14				policy;
		vectors.resize(cfgBenchmark14_Iterations);
		for(int i=0;i<vectors.size();++i)
		{
			vectors[i]=(D_btDbvtBenchmark::RandVector3()*2-D_btVector3(1,1,1)).normalized();
		}
		D_btDbvtBenchmark::RandTree(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale,cfgLeaves,dbvt);
		dbvt.optimizeTopDown();
		policy.m_nodes.reserve(cfgLeaves);
		printf("[14] culling(OCL+qsort): ");
		wallclock.reset();	
		for(int i=0;i<cfgBenchmark14_Iterations;++i)
		{
			static const D_btScalar	offset=0;
			policy.m_nodes.resize(0);
			dbvt.collideOCL(dbvt.m_root,&vectors[i],&offset,vectors[i],1,policy,false);
			policy.m_nodes.quickSort(D_btDbvtBenchmark::D_P14::sortfnc);
		}
		const int	time=(int)wallclock.getTimeMilliseconds();
		const int	t=cfgBenchmark14_Iterations;
		printf("%u ms (%i%%),(%u t/s)\r\n",time,(time-cfgBenchmark14_Reference)*100/time,(t*1000)/time);
	}
	if(cfgBenchmark15_Enable)
	{// Benchmark 15	
		srand(380843);
		D_btDbvt								dbvt;
		D_btAlignedObjectArray<D_btVector3>		vectors;
		D_btDbvtBenchmark::D_P15				policy;
		vectors.resize(cfgBenchmark15_Iterations);
		for(int i=0;i<vectors.size();++i)
		{
			vectors[i]=(D_btDbvtBenchmark::RandVector3()*2-D_btVector3(1,1,1)).normalized();
		}
		D_btDbvtBenchmark::RandTree(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale,cfgLeaves,dbvt);
		dbvt.optimizeTopDown();
		policy.m_nodes.reserve(cfgLeaves);
		printf("[15] culling(KDOP+qsort): ");
		wallclock.reset();	
		for(int i=0;i<cfgBenchmark15_Iterations;++i)
		{
			static const D_btScalar	offset=0;
			policy.m_nodes.resize(0);
			policy.m_axis=vectors[i];
			dbvt.collideKDOP(dbvt.m_root,&vectors[i],&offset,1,policy);
			policy.m_nodes.quickSort(D_btDbvtBenchmark::D_P15::sortfnc);
		}
		const int	time=(int)wallclock.getTimeMilliseconds();
		const int	t=cfgBenchmark15_Iterations;
		printf("%u ms (%i%%),(%u t/s)\r\n",time,(time-cfgBenchmark15_Reference)*100/time,(t*1000)/time);
	}
	if(cfgBenchmark16_Enable)
	{// Benchmark 16	
		srand(380843);
		D_btDbvt								dbvt;
		D_btAlignedObjectArray<D_btDbvtNode*>	batch;
		D_btDbvtBenchmark::RandTree(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale,cfgLeaves,dbvt);
		dbvt.optimizeTopDown();
		batch.reserve(cfgBenchmark16_BatchCount);
		printf("[16] insert/remove batch(%u): ",cfgBenchmark16_BatchCount);
		wallclock.reset();
		for(int i=0;i<cfgBenchmark16_Passes;++i)
		{
			for(int j=0;j<cfgBenchmark16_BatchCount;++j)
			{
				batch.push_back(dbvt.insert(D_btDbvtBenchmark::RandVolume(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale),0));
			}
			for(int j=0;j<cfgBenchmark16_BatchCount;++j)
			{
				dbvt.remove(batch[j]);
			}
			batch.resize(0);
		}
		const int	time=(int)wallclock.getTimeMilliseconds();
		const int	ir=cfgBenchmark16_Passes*cfgBenchmark16_BatchCount;
		printf("%u ms (%i%%),(%u bir/s)\r\n",time,(time-cfgBenchmark16_Reference)*100/time,int(ir*1000.0/time));
	}
	if(cfgBenchmark17_Enable)
	{// Benchmark 17
		srand(380843);
		D_btAlignedObjectArray<D_btDbvtVolume>	volumes;
		D_btAlignedObjectArray<int>			results;
		D_btAlignedObjectArray<int>			indices;
		volumes.resize(cfgLeaves);
		results.resize(cfgLeaves);
		indices.resize(cfgLeaves);
		for(int i=0;i<cfgLeaves;++i)
		{
			indices[i]=i;
			volumes[i]=D_btDbvtBenchmark::RandVolume(cfgVolumeCenterScale,cfgVolumeExentsBase,cfgVolumeExentsScale);
		}
		for(int i=0;i<cfgLeaves;++i)
		{
			D_btSwap(indices[i],indices[rand()%cfgLeaves]);
		}
		printf("[17] D_btDbvtVolume select: ");
		wallclock.reset();
		for(int i=0;i<cfgBenchmark17_Iterations;++i)
		{
			for(int j=0;j<cfgLeaves;++j)
			{
				for(int k=0;k<cfgLeaves;++k)
				{
					const int idx=indices[k];
					results[idx]=Select(volumes[idx],volumes[j],volumes[k]);
				}
			}
		}
		const int time=(int)wallclock.getTimeMilliseconds();
		printf("%u ms (%i%%)\r\n",time,(time-cfgBenchmark17_Reference)*100/time);
	}
	printf("\r\n\r\n");
}
#endif
