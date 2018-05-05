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

#ifndef STRIDING_MESHINTERFACE_H
#define STRIDING_MESHINTERFACE_H

#include "LinearMath/btVector3.h"
#include "btTriangleCallback.h"
#include "btConcaveShape.h"



///	The D_btStridingMeshInterface D_is the interface class for high performance generic access D_to triangle meshes, used in combination with D_btBvhTriangleMeshShape D_and some other collision D_shapes.
/// Using index striding of 3*sizeof(integer) it D_can use triangle arrays, using index striding of 1*sizeof(integer) it D_can handle triangle strips.
/// It D_allows for sharing graphics D_and collision meshes. Also it provides locking/unlocking of graphics meshes that D_are in gpu memory.
class  D_btStridingMeshInterface
{
	protected:
	
		D_btVector3 m_scaling;

	public:
		D_btStridingMeshInterface() :m_scaling(D_btScalar(1.),D_btScalar(1.),D_btScalar(1.))
		{

		}

		virtual ~D_btStridingMeshInterface();



		virtual void	InternalProcessAllTriangles(D_btInternalTriangleIndexCallback* callback,const D_btVector3& aabbMin,const D_btVector3& aabbMax) const;

		///brute force method D_to calculate aabb
		void	calculateAabbBruteForce(D_btVector3& aabbMin,D_btVector3& aabbMax);

		/// get read D_and write access D_to a subpart of a triangle mesh
		/// this subpart has a continuous array of vertices D_and indices
		/// in this way the mesh D_can be handled as chunks of memory with striding
		/// very similar D_to OpenGL vertexarray support
		/// make a call D_to unLockVertexBase when the read D_and write access D_is finished	
		virtual void	getLockedVertexIndexBase(unsigned char **vertexbase, int& numverts,D_PHY_ScalarType& type, int& stride,unsigned char **indexbase,int & indexstride,int& numfaces,D_PHY_ScalarType& indicestype,int subpart=0)=0;
		
		virtual void	getLockedReadOnlyVertexIndexBase(const unsigned char **vertexbase, int& numverts,D_PHY_ScalarType& type, int& stride,const unsigned char **indexbase,int & indexstride,int& numfaces,D_PHY_ScalarType& indicestype,int subpart=0) const=0;
	
		/// unLockVertexBase finishes the access D_to a subpart of the triangle mesh
		/// make a call D_to unLockVertexBase when the read D_and write access (using getLockedVertexIndexBase) D_is finished
		virtual void	unLockVertexBase(int subpart)=0;

		virtual void	unLockReadOnlyVertexBase(int subpart) const=0;


		/// getNumSubParts returns the number of seperate subparts
		/// each subpart has a continuous array of vertices D_and indices
		virtual int		getNumSubParts() const=0;

		virtual void	preallocateVertices(int numverts)=0;
		virtual void	preallocateIndices(int numindices)=0;

		virtual bool	hasPremadeAabb() const { return false; }
		virtual void	setPremadeAabb(const D_btVector3& aabbMin, const D_btVector3& aabbMax ) const
                {
                        (void) aabbMin;
                        (void) aabbMax;
                }
		virtual void	getPremadeAabb(D_btVector3* aabbMin, D_btVector3* aabbMax ) const
        {
            (void) aabbMin;
            (void) aabbMax;
        }

		const D_btVector3&	getScaling() const {
			return m_scaling;
		}
		void	setScaling(const D_btVector3& scaling)
		{
			m_scaling = scaling;
		}

	

};

#endif //STRIDING_MESHINTERFACE_H
