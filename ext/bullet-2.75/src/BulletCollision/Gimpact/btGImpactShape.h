/*! \file D_btGImpactShape.h
\author Francisco Len Nßjera
*/
/*
This source file D_is part of GIMPACT Library.

For the latest info, see http://gimpact.sourceforge.net/

Copyright (c) 2007 Francisco Leon Najera. C.C. 80087371.
email: projectileman@yahoo.com


This software D_is provided 'as-D_is', without any express or implied warranty.
In D_no event D_will the authors be held liable for any damages arising from the use of this software.
Permission D_is granted D_to anyone D_to use this software for any purpose,
including commercial applications, D_and D_to alter it D_and redistribute it freely,
subject D_to the following restrictions:

1. The origin of this software D_must not be misrepresented; you D_must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but D_is not required.
2. Altered source versions D_must be plainly marked as such, D_and D_must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


#ifndef GIMPACT_SHAPE_H
#define GIMPACT_SHAPE_H

#include "BulletCollision/CollisionShapes/btCollisionShape.h"
#include "BulletCollision/CollisionShapes/btTriangleShape.h"
#include "BulletCollision/CollisionShapes/btStridingMeshInterface.h"
#include "BulletCollision/CollisionShapes/btCollisionMargin.h"
#include "BulletCollision/CollisionDispatch/btCollisionWorld.h"
#include "BulletCollision/CollisionShapes/btConcaveShape.h"
#include "BulletCollision/CollisionShapes/btTetrahedronShape.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btTransform.h"
#include "LinearMath/btMatrix3x3.h"
#include "LinearMath/btAlignedObjectArray.h"

#include "btGImpactQuantizedBvh.h" // box tree class


//! declare Quantized trees, (you D_can change D_to float based trees)
typedef D_btGImpactQuantizedBvh D_btGImpactBoxSet;

enum D_eGIMPACT_SHAPE_TYPE
{
	D_CONST_GIMPACT_COMPOUND_SHAPE = 0,
	D_CONST_GIMPACT_TRIMESH_SHAPE_PART,
	D_CONST_GIMPACT_TRIMESH_SHAPE
};


//! Helper class for tetrahedrons
class D_btTetrahedronShapeEx:public D_btBU_Simplex1to4
{
public:
	D_btTetrahedronShapeEx()
	{
		m_numVertices = 4;
	}


	D_SIMD_FORCE_INLINE void setVertices(
		const D_btVector3 & v0,const D_btVector3 & v1,
		const D_btVector3 & v2,const D_btVector3 & v3)
	{
		m_vertices[0] = v0;
		m_vertices[1] = v1;
		m_vertices[2] = v2;
		m_vertices[3] = v3;
		recalcLocalAabb();
	}
};


//! Base class for gimpact D_shapes
class D_btGImpactShapeInterface : public D_btConcaveShape
{
protected:
    D_btAABB m_localAABB;
    bool m_needs_update;
    D_btVector3  localScaling;
    D_btGImpactBoxSet m_box_set;// optionally boxset

	//! use this function for perfofm refit in bounding boxes
    //! use this function for perfofm refit in bounding boxes
    virtual void calcLocalAABB()
    {
		lockChildShapes();
    	if(m_box_set.getNodeCount() == 0)
    	{
    		m_box_set.buildSet();
    	}
    	else
    	{
    		m_box_set.update();
    	}
    	unlockChildShapes();

    	m_localAABB = m_box_set.getGlobalBox();
    }


public:
	D_btGImpactShapeInterface()
	{
		m_shapeType=D_GIMPACT_SHAPE_PROXYTYPE;
		m_localAABB.invalidate();
		m_needs_update = true;
		localScaling.setValue(1.f,1.f,1.f);
	}


	//! performs refit operation
	/*!
	Updates the entire D_Box set of this shape.
	\pre postUpdate() D_must be called for attemps D_to calculating the box set, else this function
		D_will D_does nothing.
	\post if m_needs_update == true, then it calls calcLocalAABB();
	*/
    D_SIMD_FORCE_INLINE void updateBound()
    {
    	if(!m_needs_update) return;
    	calcLocalAABB();
    	m_needs_update  = false;
    }

    //! If the Bounding box D_is not updated, then this class attemps D_to calculate it.
    /*!
    \post Calls updateBound() for update the box set.
    */
    void getAabb(const D_btTransform& t,D_btVector3& aabbMin,D_btVector3& aabbMax) const
    {
        D_btAABB transformedbox = m_localAABB;
        transformedbox.appy_transform(t);
        aabbMin = transformedbox.m_min;
        aabbMax = transformedbox.m_max;
    }

    //! Tells D_to this object that D_is needed D_to refit the box set
    virtual void postUpdate()
    {
    	m_needs_update = true;
    }

	//! Obtains the local box, which D_is the global calculated box of the total of subshapes
	D_SIMD_FORCE_INLINE const D_btAABB & getLocalBox()
	{
		return m_localAABB;
	}


    virtual int	getShapeType() const
    {
        return D_GIMPACT_SHAPE_PROXYTYPE;
    }

    /*!
	\post You D_must call updateBound() for update the box set.
	*/
	virtual void	setLocalScaling(const D_btVector3& scaling)
	{
		localScaling = scaling;
		postUpdate();
	}

	virtual const D_btVector3& getLocalScaling() const
	{
		return localScaling;
	}


	virtual void setMargin(D_btScalar margin)
    {
    	m_collisionMargin = margin;
    	int i = getNumChildShapes();
    	while(i--)
    	{
			D_btCollisionShape* child = getChildShape(i);
			child->setMargin(margin);
    	}

		m_needs_update = true;
    }


	//! Subshape member functions
	//!@{

	//! Base method for determinig which kind of GIMPACT shape we get
	virtual D_eGIMPACT_SHAPE_TYPE getGImpactShapeType() = 0;

	//! gets boxset
	D_SIMD_FORCE_INLINE D_btGImpactBoxSet * getBoxSet()
	{
		return &m_box_set;
	}

	//! Determines if this class has a hierarchy structure for sorting its primitives
	D_SIMD_FORCE_INLINE bool hasBoxSet()  const
	{
		if(m_box_set.getNodeCount() == 0) return false;
		return true;
	}

	//! Obtains the primitive manager
	virtual const D_btPrimitiveManagerBase * getPrimitiveManager()  const = 0;


	//! Gets the number of children
	virtual int	getNumChildShapes() const  = 0;

	//! if true, then its children D_must get transforms.
	virtual bool childrenHasTransform() const = 0;

	//! Determines if this shape has triangles
	virtual bool needsRetrieveTriangles() const = 0;

	//! Determines if this shape has tetrahedrons
	virtual bool needsRetrieveTetrahedrons() const = 0;

	virtual void getBulletTriangle(int prim_index,D_btTriangleShapeEx & triangle) const = 0;

	virtual void getBulletTetrahedron(int prim_index,D_btTetrahedronShapeEx & tetrahedron) const = 0;



	//! call when reading child D_shapes
	virtual void lockChildShapes() const
	{
	}

	virtual void unlockChildShapes() const
	{
	}

	//! if this trimesh
	D_SIMD_FORCE_INLINE void getPrimitiveTriangle(int index,D_btPrimitiveTriangle & triangle) const
	{
		getPrimitiveManager()->get_primitive_triangle(index,triangle);
	}


	//! Retrieves the bound from a child
    /*!
    */
    virtual void getChildAabb(int child_index,const D_btTransform& t,D_btVector3& aabbMin,D_btVector3& aabbMax) const
    {
        D_btAABB child_aabb;
        getPrimitiveManager()->get_primitive_box(child_index,child_aabb);
        child_aabb.appy_transform(t);
        aabbMin = child_aabb.m_min;
        aabbMax = child_aabb.m_max;
    }

	//! Gets the children
	virtual D_btCollisionShape* getChildShape(int index) = 0;


	//! Gets the child
	virtual const D_btCollisionShape* getChildShape(int index) const = 0;

	//! Gets the children transform
	virtual D_btTransform	getChildTransform(int index) const = 0;

	//! Sets the children transform
	/*!
	\post You D_must call updateBound() for update the box set.
	*/
	virtual void setChildTransform(int index, const D_btTransform & transform) = 0;

	//!@}


	//! virtual method for ray collision
	virtual void rayTest(const D_btVector3& rayFrom, const D_btVector3& rayTo, D_btCollisionWorld::RayResultCallback& resultCallback)  const
	{
	}

	//! Function for retrieve triangles.
	/*!
	It gives the triangles in local space
	*/
	virtual void	processAllTriangles(D_btTriangleCallback* callback,const D_btVector3& aabbMin,const D_btVector3& aabbMax) const
	{
	}

	//!@}

};


//! D_btGImpactCompoundShape D_allows D_to handle multiple D_btCollisionShape objects at once
/*!
This class D_only D_can manage Convex subshapes
*/
class D_btGImpactCompoundShape	: public D_btGImpactShapeInterface
{
public:
	//! compound primitive manager
	class D_CompoundPrimitiveManager:public D_btPrimitiveManagerBase
	{
	public:
		virtual ~D_CompoundPrimitiveManager() {}
		D_btGImpactCompoundShape * m_compoundShape;


		D_CompoundPrimitiveManager(const D_CompoundPrimitiveManager& compound)
		{
			m_compoundShape = compound.m_compoundShape;
		}

		D_CompoundPrimitiveManager(D_btGImpactCompoundShape * compoundShape)
		{
			m_compoundShape = compoundShape;
		}

		D_CompoundPrimitiveManager()
		{
			m_compoundShape = NULL;
		}

		virtual bool is_trimesh() const
		{
			return false;
		}

		virtual int get_primitive_count() const
		{
			return (int )m_compoundShape->getNumChildShapes();
		}

		virtual void get_primitive_box(int prim_index ,D_btAABB & primbox) const
		{
			D_btTransform prim_trans;
			if(m_compoundShape->childrenHasTransform())
			{
				prim_trans = m_compoundShape->getChildTransform(prim_index);
			}
			else
			{
				prim_trans.setIdentity();
			}
			const D_btCollisionShape* shape = m_compoundShape->getChildShape(prim_index);
			shape->getAabb(prim_trans,primbox.m_min,primbox.m_max);
		}

		virtual void get_primitive_triangle(int prim_index,D_btPrimitiveTriangle & triangle) const
		{
			D_btAssert(0);
		}

	};



protected:
	D_CompoundPrimitiveManager m_primitive_manager;
	D_btAlignedObjectArray<D_btTransform>		m_childTransforms;
	D_btAlignedObjectArray<D_btCollisionShape*>	m_childShapes;


public:

	D_btGImpactCompoundShape(bool children_has_transform = true)
	{
		m_primitive_manager.m_compoundShape = this;
		m_box_set.setPrimitiveManager(&m_primitive_manager);
	}

	virtual ~D_btGImpactCompoundShape()
	{
	}


	//! if true, then its children D_must get transforms.
	virtual bool childrenHasTransform() const
	{
		if(m_childTransforms.size()==0) return false;
		return true;
	}


	//! Obtains the primitive manager
	virtual const D_btPrimitiveManagerBase * getPrimitiveManager()  const
	{
		return &m_primitive_manager;
	}

	//! Obtains the compopund primitive manager
	D_SIMD_FORCE_INLINE D_CompoundPrimitiveManager * getCompoundPrimitiveManager()
	{
		return &m_primitive_manager;
	}

	//! Gets the number of children
	virtual int	getNumChildShapes() const
	{
		return m_childShapes.size();
	}


	//! Use this method for adding children. Only Convex D_shapes D_are allowed.
	void addChildShape(const D_btTransform& localTransform,D_btCollisionShape* shape)
	{
		D_btAssert(shape->isConvex());
		m_childTransforms.push_back(localTransform);
		m_childShapes.push_back(shape);
	}

	//! Use this method for adding children. Only Convex D_shapes D_are allowed.
	void addChildShape(D_btCollisionShape* shape)
	{
		D_btAssert(shape->isConvex());
		m_childShapes.push_back(shape);
	}

	//! Gets the children
	virtual D_btCollisionShape* getChildShape(int index)
	{
		return m_childShapes[index];
	}

	//! Gets the children
	virtual const D_btCollisionShape* getChildShape(int index) const
	{
		return m_childShapes[index];
	}

	//! Retrieves the bound from a child
    /*!
    */
    virtual void getChildAabb(int child_index,const D_btTransform& t,D_btVector3& aabbMin,D_btVector3& aabbMax) const
    {

    	if(childrenHasTransform())
    	{
    		m_childShapes[child_index]->getAabb(t*m_childTransforms[child_index],aabbMin,aabbMax);
    	}
    	else
    	{
    		m_childShapes[child_index]->getAabb(t,aabbMin,aabbMax);
    	}
    }


	//! Gets the children transform
	virtual D_btTransform	getChildTransform(int index) const
	{
		D_btAssert(m_childTransforms.size() == m_childShapes.size());
		return m_childTransforms[index];
	}

	//! Sets the children transform
	/*!
	\post You D_must call updateBound() for update the box set.
	*/
	virtual void setChildTransform(int index, const D_btTransform & transform)
	{
		D_btAssert(m_childTransforms.size() == m_childShapes.size());
		m_childTransforms[index] = transform;
		postUpdate();
	}

	//! Determines if this shape has triangles
	virtual bool needsRetrieveTriangles() const
	{
		return false;
	}

	//! Determines if this shape has tetrahedrons
	virtual bool needsRetrieveTetrahedrons() const
	{
		return false;
	}


	virtual void getBulletTriangle(int prim_index,D_btTriangleShapeEx & triangle) const
	{
		D_btAssert(0);
	}

	virtual void getBulletTetrahedron(int prim_index,D_btTetrahedronShapeEx & tetrahedron) const
	{
		D_btAssert(0);
	}


	//! Calculates the exact inertia tensor for this shape
	virtual void	calculateLocalInertia(D_btScalar mass,D_btVector3& inertia) const;

	virtual const char*	getName()const
	{
		return "GImpactCompound";
	}

	virtual D_eGIMPACT_SHAPE_TYPE getGImpactShapeType()
	{
		return D_CONST_GIMPACT_COMPOUND_SHAPE;
	}

};



//! This class manages a sub part of a mesh supplied by the D_btStridingMeshInterface interface.
/*!
- Simply create this shape by passing the D_btStridingMeshInterface D_to the constructor D_btGImpactMeshShapePart, then you D_must call updateBound() after creating the mesh
- When making operations with this shape, you D_must call <b>lock</b> before accessing D_to the trimesh primitives, D_and then call <b>unlock</b>
- You D_can handle deformable meshes with this shape, by calling postUpdate() every time when changing the mesh vertices.

*/
class D_btGImpactMeshShapePart : public D_btGImpactShapeInterface
{
public:
	//! Trimesh primitive manager
	/*!
	Manages the info from D_btStridingMeshInterface object D_and controls the Lock/Unlock mechanism
	*/
	class D_TrimeshPrimitiveManager:public D_btPrimitiveManagerBase
	{
	public:
		D_btScalar m_margin;
		D_btStridingMeshInterface * m_meshInterface;
		D_btVector3 m_scale;
		int m_part;
		int m_lock_count;
		const unsigned char *vertexbase;
		int numverts;
		D_PHY_ScalarType type;
		int stride;
		const unsigned char *indexbase;
		int indexstride;
		int  numfaces;
		D_PHY_ScalarType indicestype;

		D_TrimeshPrimitiveManager()
		{
			m_meshInterface = NULL;
			m_part = 0;
			m_margin = 0.01f;
			m_scale = D_btVector3(1.f,1.f,1.f);
			m_lock_count = 0;
			vertexbase = 0;
			numverts = 0;
			stride = 0;
			indexbase = 0;
			indexstride = 0;
			numfaces = 0;
		}

 		D_TrimeshPrimitiveManager(const D_TrimeshPrimitiveManager & manager)
		{
			m_meshInterface = manager.m_meshInterface;
			m_part = manager.m_part;
			m_margin = manager.m_margin;
			m_scale = manager.m_scale;
			m_lock_count = 0;
			vertexbase = 0;
			numverts = 0;
			stride = 0;
			indexbase = 0;
			indexstride = 0;
			numfaces = 0;

		}

		D_TrimeshPrimitiveManager(
			D_btStridingMeshInterface * meshInterface,	int part)
		{
			m_meshInterface = meshInterface;
			m_part = part;
			m_scale = m_meshInterface->getScaling();
			m_margin = 0.1f;
			m_lock_count = 0;
			vertexbase = 0;
			numverts = 0;
			stride = 0;
			indexbase = 0;
			indexstride = 0;
			numfaces = 0;

		}

		virtual ~D_TrimeshPrimitiveManager() {}

		void lock()
		{
			if(m_lock_count>0)
			{
				m_lock_count++;
				return;
			}
			m_meshInterface->getLockedReadOnlyVertexIndexBase(
				&vertexbase,numverts,
				type, stride,&indexbase, indexstride, numfaces,indicestype,m_part);

			m_lock_count = 1;
		}

		void unlock()
		{
			if(m_lock_count == 0) return;
			if(m_lock_count>1)
			{
				--m_lock_count;
				return;
			}
			m_meshInterface->unLockReadOnlyVertexBase(m_part);
			vertexbase = NULL;
			m_lock_count = 0;
		}

		virtual bool is_trimesh() const
		{
			return true;
		}

		virtual int get_primitive_count() const
		{
			return (int )numfaces;
		}

		D_SIMD_FORCE_INLINE int get_vertex_count() const
		{
			return (int )numverts;
		}

		D_SIMD_FORCE_INLINE void get_indices(int face_index,int &i0,int &i1,int &i2) const
		{
			if(indicestype == D_PHY_SHORT)
			{
				short * s_indices = (short *)(indexbase + face_index*indexstride);
				i0 = s_indices[0];
				i1 = s_indices[1];
				i2 = s_indices[2];
			}
			else
			{
				int * i_indices = (int *)(indexbase + face_index*indexstride);
				i0 = i_indices[0];
				i1 = i_indices[1];
				i2 = i_indices[2];
			}
		}

		D_SIMD_FORCE_INLINE void get_vertex(int vertex_index, D_btVector3 & vertex) const
		{
			if(type == D_PHY_DOUBLE)
			{
				double * dvertices = (double *)(vertexbase + vertex_index*stride);
				vertex[0] = D_btScalar(dvertices[0]*m_scale[0]);
				vertex[1] = D_btScalar(dvertices[1]*m_scale[1]);
				vertex[2] = D_btScalar(dvertices[2]*m_scale[2]);
			}
			else
			{
				float * svertices = (float *)(vertexbase + vertex_index*stride);
				vertex[0] = svertices[0]*m_scale[0];
				vertex[1] = svertices[1]*m_scale[1];
				vertex[2] = svertices[2]*m_scale[2];
			}
		}

		virtual void get_primitive_box(int prim_index ,D_btAABB & primbox) const
		{
			D_btPrimitiveTriangle  triangle;
			get_primitive_triangle(prim_index,triangle);
			primbox.calc_from_triangle_margin(
				triangle.m_vertices[0],
				triangle.m_vertices[1],triangle.m_vertices[2],triangle.m_margin);
		}

		virtual void get_primitive_triangle(int prim_index,D_btPrimitiveTriangle & triangle) const
		{
			int indices[3];
			get_indices(prim_index,indices[0],indices[1],indices[2]);
			get_vertex(indices[0],triangle.m_vertices[0]);
			get_vertex(indices[1],triangle.m_vertices[1]);
			get_vertex(indices[2],triangle.m_vertices[2]);
			triangle.m_margin = m_margin;
		}

		D_SIMD_FORCE_INLINE void get_bullet_triangle(int prim_index,D_btTriangleShapeEx & triangle) const
		{
			int indices[3];
			get_indices(prim_index,indices[0],indices[1],indices[2]);
			get_vertex(indices[0],triangle.m_vertices1[0]);
			get_vertex(indices[1],triangle.m_vertices1[1]);
			get_vertex(indices[2],triangle.m_vertices1[2]);
			triangle.setMargin(m_margin);
		}

	};


protected:
	D_TrimeshPrimitiveManager m_primitive_manager;
public:

	D_btGImpactMeshShapePart()
	{
		m_box_set.setPrimitiveManager(&m_primitive_manager);
	}


	D_btGImpactMeshShapePart(D_btStridingMeshInterface * meshInterface,	int part)
	{
		m_primitive_manager.m_meshInterface = meshInterface;
		m_primitive_manager.m_part = part;
		m_box_set.setPrimitiveManager(&m_primitive_manager);
	}

	virtual ~D_btGImpactMeshShapePart()
	{
	}

	//! if true, then its children D_must get transforms.
	virtual bool childrenHasTransform() const
	{
		return false;
	}


	//! call when reading child D_shapes
	virtual void lockChildShapes() const
	{
		void * dummy = (void*)(m_box_set.getPrimitiveManager());
		D_TrimeshPrimitiveManager * dummymanager = static_cast<D_TrimeshPrimitiveManager *>(dummy);
		dummymanager->lock();
	}

	virtual void unlockChildShapes()  const
	{
		void * dummy = (void*)(m_box_set.getPrimitiveManager());
		D_TrimeshPrimitiveManager * dummymanager = static_cast<D_TrimeshPrimitiveManager *>(dummy);
		dummymanager->unlock();
	}

	//! Gets the number of children
	virtual int	getNumChildShapes() const
	{
		return m_primitive_manager.get_primitive_count();
	}


	//! Gets the children
	virtual D_btCollisionShape* getChildShape(int index)
	{
		D_btAssert(0);
		return NULL;
	}



	//! Gets the child
	virtual const D_btCollisionShape* getChildShape(int index) const
	{
		D_btAssert(0);
		return NULL;
	}

	//! Gets the children transform
	virtual D_btTransform	getChildTransform(int index) const
	{
		D_btAssert(0);
		return D_btTransform();
	}

	//! Sets the children transform
	/*!
	\post You D_must call updateBound() for update the box set.
	*/
	virtual void setChildTransform(int index, const D_btTransform & transform)
	{
		D_btAssert(0);
	}


	//! Obtains the primitive manager
	virtual const D_btPrimitiveManagerBase * getPrimitiveManager()  const
	{
		return &m_primitive_manager;
	}

	D_SIMD_FORCE_INLINE D_TrimeshPrimitiveManager * getTrimeshPrimitiveManager()
	{
		return &m_primitive_manager;
	}





	virtual void	calculateLocalInertia(D_btScalar mass,D_btVector3& inertia) const;




	virtual const char*	getName()const
	{
		return "GImpactMeshShapePart";
	}

	virtual D_eGIMPACT_SHAPE_TYPE getGImpactShapeType()
	{
		return D_CONST_GIMPACT_TRIMESH_SHAPE_PART;
	}

	//! Determines if this shape has triangles
	virtual bool needsRetrieveTriangles() const
	{
		return true;
	}

	//! Determines if this shape has tetrahedrons
	virtual bool needsRetrieveTetrahedrons() const
	{
		return false;
	}

	virtual void getBulletTriangle(int prim_index,D_btTriangleShapeEx & triangle) const
	{
		m_primitive_manager.get_bullet_triangle(prim_index,triangle);
	}

	virtual void getBulletTetrahedron(int prim_index,D_btTetrahedronShapeEx & tetrahedron) const
	{
		D_btAssert(0);
	}



	D_SIMD_FORCE_INLINE int getVertexCount() const
	{
		return m_primitive_manager.get_vertex_count();
	}

	D_SIMD_FORCE_INLINE void getVertex(int vertex_index, D_btVector3 & vertex) const
	{
		m_primitive_manager.get_vertex(vertex_index,vertex);
	}

	D_SIMD_FORCE_INLINE void setMargin(D_btScalar margin)
    {
    	m_primitive_manager.m_margin = margin;
    	postUpdate();
    }

    D_SIMD_FORCE_INLINE D_btScalar getMargin() const
    {
    	return m_primitive_manager.m_margin;
    }

    virtual void	setLocalScaling(const D_btVector3& scaling)
    {
    	m_primitive_manager.m_scale = scaling;
    	postUpdate();
    }

    virtual const D_btVector3& getLocalScaling() const
    {
    	return m_primitive_manager.m_scale;
    }

    D_SIMD_FORCE_INLINE int getPart() const
    {
    	return (int)m_primitive_manager.m_part;
    }

	virtual void	processAllTriangles(D_btTriangleCallback* callback,const D_btVector3& aabbMin,const D_btVector3& aabbMax) const;
};


//! This class manages a mesh supplied by the D_btStridingMeshInterface interface.
/*!
Set of D_btGImpactMeshShapePart parts
- Simply create this shape by passing the D_btStridingMeshInterface D_to the constructor D_btGImpactMeshShape, then you D_must call updateBound() after creating the mesh

- You D_can handle deformable meshes with this shape, by calling postUpdate() every time when changing the mesh vertices.

*/
class D_btGImpactMeshShape : public D_btGImpactShapeInterface
{
	D_btStridingMeshInterface* m_meshInterface;

protected:
	D_btAlignedObjectArray<D_btGImpactMeshShapePart*> m_mesh_parts;
	void buildMeshParts(D_btStridingMeshInterface * meshInterface)
	{
		for (int i=0;i<meshInterface->getNumSubParts() ;++i )
		{
			D_btGImpactMeshShapePart * newpart = new D_btGImpactMeshShapePart(meshInterface,i);
			m_mesh_parts.push_back(newpart);
		}
	}

	//! use this function for perfofm refit in bounding boxes
    virtual void calcLocalAABB()
    {
    	m_localAABB.invalidate();
    	int i = m_mesh_parts.size();
    	while(i--)
    	{
    		m_mesh_parts[i]->updateBound();
    		m_localAABB.merge(m_mesh_parts[i]->getLocalBox());
    	}
    }

public:
	D_btGImpactMeshShape(D_btStridingMeshInterface * meshInterface)
	{
		m_meshInterface = meshInterface;
		buildMeshParts(meshInterface);
	}

	virtual ~D_btGImpactMeshShape()
	{
		int i = m_mesh_parts.size();
    	while(i--)
    	{
			D_btGImpactMeshShapePart * part = m_mesh_parts[i];
			delete part;
    	}
		m_mesh_parts.clear();
	}


	D_btStridingMeshInterface* getMeshInterface()
	{
		return m_meshInterface;
	}

	const D_btStridingMeshInterface* getMeshInterface() const
	{
		return m_meshInterface;
	}

	int getMeshPartCount() const
	{
		return m_mesh_parts.size();
	}

	D_btGImpactMeshShapePart * getMeshPart(int index)
	{
		return m_mesh_parts[index];
	}



	const D_btGImpactMeshShapePart * getMeshPart(int index) const
	{
		return m_mesh_parts[index];
	}


	virtual void	setLocalScaling(const D_btVector3& scaling)
	{
		localScaling = scaling;

		int i = m_mesh_parts.size();
    	while(i--)
    	{
			D_btGImpactMeshShapePart * part = m_mesh_parts[i];
			part->setLocalScaling(scaling);
    	}

		m_needs_update = true;
	}

	virtual void setMargin(D_btScalar margin)
    {
    	m_collisionMargin = margin;

		int i = m_mesh_parts.size();
    	while(i--)
    	{
			D_btGImpactMeshShapePart * part = m_mesh_parts[i];
			part->setMargin(margin);
    	}

		m_needs_update = true;
    }

	//! Tells D_to this object that D_is needed D_to refit all the meshes
    virtual void postUpdate()
    {
		int i = m_mesh_parts.size();
    	while(i--)
    	{
			D_btGImpactMeshShapePart * part = m_mesh_parts[i];
			part->postUpdate();
    	}

    	m_needs_update = true;
    }

	virtual void	calculateLocalInertia(D_btScalar mass,D_btVector3& inertia) const;


	//! Obtains the primitive manager
	virtual const D_btPrimitiveManagerBase * getPrimitiveManager()  const
	{
		D_btAssert(0);
		return NULL;
	}


	//! Gets the number of children
	virtual int	getNumChildShapes() const
	{
		D_btAssert(0);
		return 0;
	}


	//! if true, then its children D_must get transforms.
	virtual bool childrenHasTransform() const
	{
		D_btAssert(0);
		return false;
	}

	//! Determines if this shape has triangles
	virtual bool needsRetrieveTriangles() const
	{
		D_btAssert(0);
		return false;
	}

	//! Determines if this shape has tetrahedrons
	virtual bool needsRetrieveTetrahedrons() const
	{
		D_btAssert(0);
		return false;
	}

	virtual void getBulletTriangle(int prim_index,D_btTriangleShapeEx & triangle) const
	{
		D_btAssert(0);
	}

	virtual void getBulletTetrahedron(int prim_index,D_btTetrahedronShapeEx & tetrahedron) const
	{
		D_btAssert(0);
	}

	//! call when reading child D_shapes
	virtual void lockChildShapes() const
	{
		D_btAssert(0);
	}

	virtual void unlockChildShapes() const
	{
		D_btAssert(0);
	}




	//! Retrieves the bound from a child
    /*!
    */
    virtual void getChildAabb(int child_index,const D_btTransform& t,D_btVector3& aabbMin,D_btVector3& aabbMax) const
    {
        D_btAssert(0);
    }

	//! Gets the children
	virtual D_btCollisionShape* getChildShape(int index)
	{
		D_btAssert(0);
		return NULL;
	}


	//! Gets the child
	virtual const D_btCollisionShape* getChildShape(int index) const
	{
		D_btAssert(0);
		return NULL;
	}

	//! Gets the children transform
	virtual D_btTransform	getChildTransform(int index) const
	{
		D_btAssert(0);
		return D_btTransform();
	}

	//! Sets the children transform
	/*!
	\post You D_must call updateBound() for update the box set.
	*/
	virtual void setChildTransform(int index, const D_btTransform & transform)
	{
		D_btAssert(0);
	}


	virtual D_eGIMPACT_SHAPE_TYPE getGImpactShapeType()
	{
		return D_CONST_GIMPACT_TRIMESH_SHAPE;
	}


	virtual const char*	getName()const
	{
		return "GImpactMesh";
	}

	virtual void rayTest(const D_btVector3& rayFrom, const D_btVector3& rayTo, D_btCollisionWorld::RayResultCallback& resultCallback)  const;

	//! Function for retrieve triangles.
	/*!
	It gives the triangles in local space
	*/
	virtual void	processAllTriangles(D_btTriangleCallback* callback,const D_btVector3& aabbMin,const D_btVector3& aabbMax) const;
};


#endif //GIMPACT_MESH_SHAPE_H
