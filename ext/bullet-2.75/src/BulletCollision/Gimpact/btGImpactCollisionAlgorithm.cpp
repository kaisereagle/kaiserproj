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
/*
Author: Francisco Len Nßjera
Concave-Concave Collision

*/

#include "BulletCollision/CollisionDispatch/btManifoldResult.h"
#include "LinearMath/btIDebugDraw.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "btGImpactCollisionAlgorithm.h"
#include "btContactProcessing.h"
#include "LinearMath/btQuickprof.h"


//! Class for accessing the plane equation
class D_btPlaneShape : public D_btStaticPlaneShape
{
public:

	D_btPlaneShape(const D_btVector3& v, float f)
		:D_btStaticPlaneShape(v,f)
	{
	}

	void get_plane_equation(D_btVector4 &equation)
	{
		equation[0] = m_planeNormal[0];
		equation[1] = m_planeNormal[1];
		equation[2] = m_planeNormal[2];
		equation[3] = m_planeConstant;
	}


	void get_plane_equation_transformed(const D_btTransform & trans,D_btVector4 &equation)
	{
		equation[0] = trans.getBasis().getRow(0).dot(m_planeNormal);
		equation[1] = trans.getBasis().getRow(1).dot(m_planeNormal);
		equation[2] = trans.getBasis().getRow(2).dot(m_planeNormal);
		equation[3] = trans.getOrigin().dot(m_planeNormal) + m_planeConstant;
	}
};



//////////////////////////////////////////////////////////////////////////////////////////////
#ifdef TRI_COLLISION_PROFILING

D_btClock g_triangle_clock;

float g_accum_triangle_collision_time = 0;
int g_count_triangle_collision = 0;

void bt_begin_gim02_tri_time()
{
	g_triangle_clock.reset();
}

void bt_end_gim02_tri_time()
{
	g_accum_triangle_collision_time += g_triangle_clock.getTimeMicroseconds();
	g_count_triangle_collision++;
}
#endif //TRI_COLLISION_PROFILING
//! Retrieving D_shapes D_shapes
/*!
Declared here due of insuficent space on Pool allocators
*/
//!@{
class D_GIM_ShapeRetriever
{
public:
	D_btGImpactShapeInterface * m_gim_shape;
	D_btTriangleShapeEx m_trishape;
	D_btTetrahedronShapeEx m_tetrashape;

public:
	class D_ChildShapeRetriever
	{
	public:
		D_GIM_ShapeRetriever * m_parent;
		virtual D_btCollisionShape * getChildShape(int index)
		{
			return m_parent->m_gim_shape->getChildShape(index);
		}
		virtual ~D_ChildShapeRetriever() {}
	};

	class D_TriangleShapeRetriever:public D_ChildShapeRetriever
	{
	public:

		virtual D_btCollisionShape * getChildShape(int index)
		{
			m_parent->m_gim_shape->getBulletTriangle(index,m_parent->m_trishape);
			return &m_parent->m_trishape;
		}
		virtual ~D_TriangleShapeRetriever() {}
	};

	class D_TetraShapeRetriever:public D_ChildShapeRetriever
	{
	public:

		virtual D_btCollisionShape * getChildShape(int index)
		{
			m_parent->m_gim_shape->getBulletTetrahedron(index,m_parent->m_tetrashape);
			return &m_parent->m_tetrashape;
		}
	};
public:
	D_ChildShapeRetriever m_child_retriever;
	D_TriangleShapeRetriever m_tri_retriever;
	D_TetraShapeRetriever  m_tetra_retriever;
	D_ChildShapeRetriever * m_current_retriever;

	D_GIM_ShapeRetriever(D_btGImpactShapeInterface * D_gim_shape)
	{
		m_gim_shape = D_gim_shape;
		//select retriever
		if(m_gim_shape->needsRetrieveTriangles())
		{
			m_current_retriever = &m_tri_retriever;
		}
		else if(m_gim_shape->needsRetrieveTetrahedrons())
		{
			m_current_retriever = &m_tetra_retriever;
		}
		else
		{
			m_current_retriever = &m_child_retriever;
		}

		m_current_retriever->m_parent = this;
	}

	D_btCollisionShape * getChildShape(int index)
	{
		return m_current_retriever->getChildShape(index);
	}


};



//!@}


#ifdef TRI_COLLISION_PROFILING

//! Gets the average time in miliseconds of tree collisions
float D_btGImpactCollisionAlgorithm::getAverageTreeCollisionTime()
{
	return D_btGImpactBoxSet::getAverageTreeCollisionTime();

}

//! Gets the average time in miliseconds of triangle collisions
float D_btGImpactCollisionAlgorithm::getAverageTriangleCollisionTime()
{
	if(g_count_triangle_collision == 0) return 0;

	float avgtime = g_accum_triangle_collision_time;
	avgtime /= (float)g_count_triangle_collision;

	g_accum_triangle_collision_time = 0;
	g_count_triangle_collision = 0;

	return avgtime;
}

#endif //TRI_COLLISION_PROFILING



D_btGImpactCollisionAlgorithm::D_btGImpactCollisionAlgorithm( const D_btCollisionAlgorithmConstructionInfo& ci, D_btCollisionObject* body0,D_btCollisionObject* body1)
: D_btActivatingCollisionAlgorithm(ci,body0,body1)
{
	m_manifoldPtr = NULL;
	m_convex_algorithm = NULL;
}

D_btGImpactCollisionAlgorithm::~D_btGImpactCollisionAlgorithm()
{
	clearCache();
}





void D_btGImpactCollisionAlgorithm::addContactPoint(D_btCollisionObject * body0,
				D_btCollisionObject * body1,
				const D_btVector3 & point,
				const D_btVector3 & normal,
				D_btScalar distance)
{
	m_resultOut->setShapeIdentifiersA(m_part0,m_triface0);
	m_resultOut->setShapeIdentifiersB(m_part1,m_triface1);
	checkManifold(body0,body1);
	m_resultOut->addContactPoint(normal,point,distance);
}


void D_btGImpactCollisionAlgorithm::shape_vs_shape_collision(
					  D_btCollisionObject * body0,
					  D_btCollisionObject * body1,
					  D_btCollisionShape * shape0,
					  D_btCollisionShape * shape1)
{

	D_btCollisionShape* tmpShape0 = body0->getCollisionShape();
	D_btCollisionShape* tmpShape1 = body1->getCollisionShape();
	
	body0->internalSetTemporaryCollisionShape(shape0);
	body1->internalSetTemporaryCollisionShape(shape1);

	{
		D_btCollisionAlgorithm* algor = newAlgorithm(body0,body1);
		// post :	checkManifold D_is called

		m_resultOut->setShapeIdentifiersA(m_part0,m_triface0);
		m_resultOut->setShapeIdentifiersB(m_part1,m_triface1);

		algor->processCollision(body0,body1,*m_dispatchInfo,m_resultOut);

		algor->~D_btCollisionAlgorithm();
		m_dispatcher->freeCollisionAlgorithm(algor);
	}

	body0->internalSetTemporaryCollisionShape(tmpShape0);
	body1->internalSetTemporaryCollisionShape(tmpShape1);
}

void D_btGImpactCollisionAlgorithm::convex_vs_convex_collision(
					  D_btCollisionObject * body0,
					  D_btCollisionObject * body1,
					  D_btCollisionShape * shape0,
					  D_btCollisionShape * shape1)
{

	D_btCollisionShape* tmpShape0 = body0->getCollisionShape();
	D_btCollisionShape* tmpShape1 = body1->getCollisionShape();
	
	body0->internalSetTemporaryCollisionShape(shape0);
	body1->internalSetTemporaryCollisionShape(shape1);


	m_resultOut->setShapeIdentifiersA(m_part0,m_triface0);
	m_resultOut->setShapeIdentifiersB(m_part1,m_triface1);

	checkConvexAlgorithm(body0,body1);
	m_convex_algorithm->processCollision(body0,body1,*m_dispatchInfo,m_resultOut);

	body0->internalSetTemporaryCollisionShape(tmpShape0);
	body1->internalSetTemporaryCollisionShape(tmpShape1);

}




void D_btGImpactCollisionAlgorithm::gimpact_vs_gimpact_find_pairs(
					  const D_btTransform & trans0,
					  const D_btTransform & trans1,
					  D_btGImpactShapeInterface * shape0,
					  D_btGImpactShapeInterface * shape1,D_btPairSet & pairset)
{
	if(shape0->hasBoxSet() && shape1->hasBoxSet())
	{
		D_btGImpactBoxSet::find_collision(shape0->getBoxSet(),trans0,shape1->getBoxSet(),trans1,pairset);
	}
	else
	{
		D_btAABB boxshape0;
		D_btAABB boxshape1;
		int i = shape0->getNumChildShapes();

		while(i--)
		{
			shape0->getChildAabb(i,trans0,boxshape0.m_min,boxshape0.m_max);

			int j = shape1->getNumChildShapes();
			while(j--)
			{
				shape1->getChildAabb(i,trans1,boxshape1.m_min,boxshape1.m_max);

				if(boxshape1.has_collision(boxshape0))
				{
					pairset.push_pair(i,j);
				}
			}
		}
	}


}


void D_btGImpactCollisionAlgorithm::gimpact_vs_shape_find_pairs(
					  const D_btTransform & trans0,
					  const D_btTransform & trans1,
					  D_btGImpactShapeInterface * shape0,
					  D_btCollisionShape * shape1,
					  D_btAlignedObjectArray<int> & collided_primitives)
{

	D_btAABB boxshape;


	if(shape0->hasBoxSet())
	{
		D_btTransform trans1to0 = trans0.inverse();
		trans1to0 *= trans1;

		shape1->getAabb(trans1to0,boxshape.m_min,boxshape.m_max);

		shape0->getBoxSet()->boxQuery(boxshape, collided_primitives);
	}
	else
	{
		shape1->getAabb(trans1,boxshape.m_min,boxshape.m_max);

		D_btAABB boxshape0;
		int i = shape0->getNumChildShapes();

		while(i--)
		{
			shape0->getChildAabb(i,trans0,boxshape0.m_min,boxshape0.m_max);

			if(boxshape.has_collision(boxshape0))
			{
				collided_primitives.push_back(i);
			}
		}

	}

}


void D_btGImpactCollisionAlgorithm::collide_gjk_triangles(D_btCollisionObject * body0,
				  D_btCollisionObject * body1,
				  D_btGImpactMeshShapePart * shape0,
				  D_btGImpactMeshShapePart * shape1,
				  const int * pairs, int pair_count)
{
	D_btTriangleShapeEx tri0;
	D_btTriangleShapeEx tri1;

	shape0->lockChildShapes();
	shape1->lockChildShapes();

	const int * pair_pointer = pairs;

	while(pair_count--)
	{

		m_triface0 = *(pair_pointer);
		m_triface1 = *(pair_pointer+1);
		pair_pointer+=2;



		shape0->getBulletTriangle(m_triface0,tri0);
		shape1->getBulletTriangle(m_triface1,tri1);


		//collide two convex D_shapes
		if(tri0.overlap_test_conservative(tri1))
		{
			convex_vs_convex_collision(body0,body1,&tri0,&tri1);
		}

	}

	shape0->unlockChildShapes();
	shape1->unlockChildShapes();
}

void D_btGImpactCollisionAlgorithm::collide_sat_triangles(D_btCollisionObject * body0,
					  D_btCollisionObject * body1,
					  D_btGImpactMeshShapePart * shape0,
					  D_btGImpactMeshShapePart * shape1,
					  const int * pairs, int pair_count)
{
	D_btTransform orgtrans0 = body0->getWorldTransform();
	D_btTransform orgtrans1 = body1->getWorldTransform();

	D_btPrimitiveTriangle ptri0;
	D_btPrimitiveTriangle ptri1;
	D_GIM_TRIANGLE_CONTACT contact_data;

	shape0->lockChildShapes();
	shape1->lockChildShapes();

	const int * pair_pointer = pairs;

	while(pair_count--)
	{

		m_triface0 = *(pair_pointer);
		m_triface1 = *(pair_pointer+1);
		pair_pointer+=2;


		shape0->getPrimitiveTriangle(m_triface0,ptri0);
		shape1->getPrimitiveTriangle(m_triface1,ptri1);

		#ifdef TRI_COLLISION_PROFILING
		bt_begin_gim02_tri_time();
		#endif

		ptri0.applyTransform(orgtrans0);
		ptri1.applyTransform(orgtrans1);


		//build planes
		ptri0.buildTriPlane();
		ptri1.buildTriPlane();
		// test conservative



		if(ptri0.overlap_test_conservative(ptri1))
		{
			if(ptri0.find_triangle_collision_clip_method(ptri1,contact_data))
			{

				int j = contact_data.m_point_count;
				while(j--)
				{

					addContactPoint(body0, body1,
								contact_data.m_points[j],
								contact_data.m_separating_normal,
								-contact_data.m_penetration_depth);
				}
			}
		}

		#ifdef TRI_COLLISION_PROFILING
		bt_end_gim02_tri_time();
		#endif

	}

	shape0->unlockChildShapes();
	shape1->unlockChildShapes();

}


void D_btGImpactCollisionAlgorithm::gimpact_vs_gimpact(
						D_btCollisionObject * body0,
					   	D_btCollisionObject * body1,
					  	D_btGImpactShapeInterface * shape0,
					  	D_btGImpactShapeInterface * shape1)
{

	if(shape0->getGImpactShapeType()==D_CONST_GIMPACT_TRIMESH_SHAPE)
	{
		D_btGImpactMeshShape * meshshape0 = static_cast<D_btGImpactMeshShape *>(shape0);
		m_part0 = meshshape0->getMeshPartCount();

		while(m_part0--)
		{
			gimpact_vs_gimpact(body0,body1,meshshape0->getMeshPart(m_part0),shape1);
		}

		return;
	}

	if(shape1->getGImpactShapeType()==D_CONST_GIMPACT_TRIMESH_SHAPE)
	{
		D_btGImpactMeshShape * meshshape1 = static_cast<D_btGImpactMeshShape *>(shape1);
		m_part1 = meshshape1->getMeshPartCount();

		while(m_part1--)
		{

			gimpact_vs_gimpact(body0,body1,shape0,meshshape1->getMeshPart(m_part1));

		}

		return;
	}


	D_btTransform orgtrans0 = body0->getWorldTransform();
	D_btTransform orgtrans1 = body1->getWorldTransform();

	D_btPairSet pairset;

	gimpact_vs_gimpact_find_pairs(orgtrans0,orgtrans1,shape0,shape1,pairset);

	if(pairset.size()== 0) return;

	if(shape0->getGImpactShapeType() == D_CONST_GIMPACT_TRIMESH_SHAPE_PART &&
		shape1->getGImpactShapeType() == D_CONST_GIMPACT_TRIMESH_SHAPE_PART)
	{
		D_btGImpactMeshShapePart * shapepart0 = static_cast<D_btGImpactMeshShapePart * >(shape0);
		D_btGImpactMeshShapePart * shapepart1 = static_cast<D_btGImpactMeshShapePart * >(shape1);
		//specialized function
		#ifdef BULLET_TRIANGLE_COLLISION
		collide_gjk_triangles(body0,body1,shapepart0,shapepart1,&pairset[0].m_index1,pairset.size());
		#else
		collide_sat_triangles(body0,body1,shapepart0,shapepart1,&pairset[0].m_index1,pairset.size());
		#endif

		return;
	}

	//general function

	shape0->lockChildShapes();
	shape1->lockChildShapes();

	D_GIM_ShapeRetriever retriever0(shape0);
	D_GIM_ShapeRetriever retriever1(shape1);

	bool child_has_transform0 = shape0->childrenHasTransform();
	bool child_has_transform1 = shape1->childrenHasTransform();

	int i = pairset.size();
	while(i--)
	{
		D_GIM_PAIR * pair = &pairset[i];
		m_triface0 = pair->m_index1;
		m_triface1 = pair->m_index2;
		D_btCollisionShape * colshape0 = retriever0.getChildShape(m_triface0);
		D_btCollisionShape * colshape1 = retriever1.getChildShape(m_triface1);

		if(child_has_transform0)
		{
			body0->setWorldTransform(orgtrans0*shape0->getChildTransform(m_triface0));
		}

		if(child_has_transform1)
		{
			body1->setWorldTransform(orgtrans1*shape1->getChildTransform(m_triface1));
		}

		//collide two convex D_shapes
		convex_vs_convex_collision(body0,body1,colshape0,colshape1);


		if(child_has_transform0)
		{
			body0->setWorldTransform(orgtrans0);
		}

		if(child_has_transform1)
		{
			body1->setWorldTransform(orgtrans1);
		}

	}

	shape0->unlockChildShapes();
	shape1->unlockChildShapes();
}

void D_btGImpactCollisionAlgorithm::gimpact_vs_shape(D_btCollisionObject * body0,
				  D_btCollisionObject * body1,
				  D_btGImpactShapeInterface * shape0,
				  D_btCollisionShape * shape1,bool swapped)
{
	if(shape0->getGImpactShapeType()==D_CONST_GIMPACT_TRIMESH_SHAPE)
	{
		D_btGImpactMeshShape * meshshape0 = static_cast<D_btGImpactMeshShape *>(shape0);
		int& part = swapped ? m_part1 : m_part0;
		part = meshshape0->getMeshPartCount();

		while(part--)
		{

			gimpact_vs_shape(body0,
				  body1,
				  meshshape0->getMeshPart(part),
				  shape1,swapped);

		}

		return;
	}

	#ifdef D_GIMPACT_VS_PLANE_COLLISION
	if(shape0->getGImpactShapeType() == D_CONST_GIMPACT_TRIMESH_SHAPE_PART &&
		shape1->getShapeType() == D_STATIC_PLANE_PROXYTYPE)
	{
		D_btGImpactMeshShapePart * shapepart = static_cast<D_btGImpactMeshShapePart *>(shape0);
		D_btStaticPlaneShape * planeshape = static_cast<D_btStaticPlaneShape * >(shape1);
		gimpacttrimeshpart_vs_plane_collision(body0,body1,shapepart,planeshape,swapped);
		return;
	}

	#endif



	if(shape1->isCompound())
	{
		D_btCompoundShape * compoundshape = static_cast<D_btCompoundShape *>(shape1);
		gimpact_vs_compoundshape(body0,body1,shape0,compoundshape,swapped);
		return;
	}
	else if(shape1->isConcave())
	{
		D_btConcaveShape * concaveshape = static_cast<D_btConcaveShape *>(shape1);
		gimpact_vs_concave(body0,body1,shape0,concaveshape,swapped);
		return;
	}


	D_btTransform orgtrans0 = body0->getWorldTransform();

	D_btTransform orgtrans1 = body1->getWorldTransform();

	D_btAlignedObjectArray<int> collided_results;

	gimpact_vs_shape_find_pairs(orgtrans0,orgtrans1,shape0,shape1,collided_results);

	if(collided_results.size() == 0) return;


	shape0->lockChildShapes();

	D_GIM_ShapeRetriever retriever0(shape0);


	bool child_has_transform0 = shape0->childrenHasTransform();


	int i = collided_results.size();

	while(i--)
	{
		int child_index = collided_results[i];
        if(swapped)
    		m_triface1 = child_index;
        else
            m_triface0 = child_index;

		D_btCollisionShape * colshape0 = retriever0.getChildShape(child_index);

		if(child_has_transform0)
		{
			body0->setWorldTransform(orgtrans0*shape0->getChildTransform(child_index));
		}

		//collide two D_shapes
		if(swapped)
		{
			shape_vs_shape_collision(body1,body0,shape1,colshape0);
		}
		else
		{
			shape_vs_shape_collision(body0,body1,colshape0,shape1);
		}

		//restore transforms
		if(child_has_transform0)
		{
			body0->setWorldTransform(orgtrans0);
		}

	}

	shape0->unlockChildShapes();

}

void D_btGImpactCollisionAlgorithm::gimpact_vs_compoundshape(D_btCollisionObject * body0,
				  D_btCollisionObject * body1,
				  D_btGImpactShapeInterface * shape0,
				  D_btCompoundShape * shape1,bool swapped)
{
	D_btTransform orgtrans1 = body1->getWorldTransform();

	int i = shape1->getNumChildShapes();
	while(i--)
	{

		D_btCollisionShape * colshape1 = shape1->getChildShape(i);
		D_btTransform childtrans1 = orgtrans1*shape1->getChildTransform(i);

		body1->setWorldTransform(childtrans1);

		//collide child shape
		gimpact_vs_shape(body0, body1,
					  shape0,colshape1,swapped);


		//restore transforms
		body1->setWorldTransform(orgtrans1);
	}
}

void D_btGImpactCollisionAlgorithm::gimpacttrimeshpart_vs_plane_collision(
					  D_btCollisionObject * body0,
					  D_btCollisionObject * body1,
					  D_btGImpactMeshShapePart * shape0,
					  D_btStaticPlaneShape * shape1,bool swapped)
{


	D_btTransform orgtrans0 = body0->getWorldTransform();
	D_btTransform orgtrans1 = body1->getWorldTransform();

	D_btPlaneShape * planeshape = static_cast<D_btPlaneShape *>(shape1);
	D_btVector4 plane;
	planeshape->get_plane_equation_transformed(orgtrans1,plane);

	//test box against plane

	D_btAABB tribox;
	shape0->getAabb(orgtrans0,tribox.m_min,tribox.m_max);
	tribox.increment_margin(planeshape->getMargin());

	if( tribox.plane_classify(plane)!= D_BT_CONST_COLLIDE_PLANE) return;

	shape0->lockChildShapes();

	D_btScalar margin = shape0->getMargin() + planeshape->getMargin();

	D_btVector3 vertex;
	int vi = shape0->getVertexCount();
	while(vi--)
	{
		shape0->getVertex(vi,vertex);
		vertex = orgtrans0(vertex);

		D_btScalar distance = vertex.dot(plane) - plane[3] - margin;

		if(distance<0.0)//add contact
		{
			if(swapped)
			{
				addContactPoint(body1, body0,
					vertex,
					-plane,
					distance);
			}
			else
			{
				addContactPoint(body0, body1,
					vertex,
					plane,
					distance);
			}
		}
	}

	shape0->unlockChildShapes();
}




class D_btGImpactTriangleCallback: public D_btTriangleCallback
{
public:
	D_btGImpactCollisionAlgorithm * algorithm;
	D_btCollisionObject * body0;
	D_btCollisionObject * body1;
	D_btGImpactShapeInterface * gimpactshape0;
	bool swapped;
	D_btScalar margin;

	virtual void processTriangle(D_btVector3* triangle, int partId, int triangleIndex)
	{
		D_btTriangleShapeEx tri1(triangle[0],triangle[1],triangle[2]);
		tri1.setMargin(margin);
        if(swapped)
        {
            algorithm->setPart0(partId);
            algorithm->setFace0(triangleIndex);
        }
        else
        {
            algorithm->setPart1(partId);
            algorithm->setFace1(triangleIndex);
        }
		algorithm->gimpact_vs_shape(
							body0,body1,gimpactshape0,&tri1,swapped);
	}
};




void D_btGImpactCollisionAlgorithm::gimpact_vs_concave(
				  D_btCollisionObject * body0,
				  D_btCollisionObject * body1,
				  D_btGImpactShapeInterface * shape0,
				  D_btConcaveShape * shape1,bool swapped)
{
	//create the callback
	D_btGImpactTriangleCallback tricallback;
	tricallback.algorithm = this;
	tricallback.body0 = body0;
	tricallback.body1 = body1;
	tricallback.gimpactshape0 = shape0;
	tricallback.swapped = swapped;
	tricallback.margin = shape1->getMargin();

	//getting the trimesh AABB
	D_btTransform gimpactInConcaveSpace;

	gimpactInConcaveSpace = body1->getWorldTransform().inverse() * body0->getWorldTransform();

	D_btVector3 minAABB,maxAABB;
	shape0->getAabb(gimpactInConcaveSpace,minAABB,maxAABB);

	shape1->processAllTriangles(&tricallback,minAABB,maxAABB);

}



void D_btGImpactCollisionAlgorithm::processCollision (D_btCollisionObject* body0,D_btCollisionObject* body1,const D_btDispatcherInfo& dispatchInfo,D_btManifoldResult* resultOut)
{
    clearCache();

    m_resultOut = resultOut;
	m_dispatchInfo = &dispatchInfo;
    D_btGImpactShapeInterface * gimpactshape0;
    D_btGImpactShapeInterface * gimpactshape1;

	if (body0->getCollisionShape()->getShapeType()==D_GIMPACT_SHAPE_PROXYTYPE)
	{
		gimpactshape0 = static_cast<D_btGImpactShapeInterface *>(body0->getCollisionShape());

		if( body1->getCollisionShape()->getShapeType()==D_GIMPACT_SHAPE_PROXYTYPE )
		{
			gimpactshape1 = static_cast<D_btGImpactShapeInterface *>(body1->getCollisionShape());

			gimpact_vs_gimpact(body0,body1,gimpactshape0,gimpactshape1);
		}
		else
		{
			gimpact_vs_shape(body0,body1,gimpactshape0,body1->getCollisionShape(),false);
		}

	}
	else if (body1->getCollisionShape()->getShapeType()==D_GIMPACT_SHAPE_PROXYTYPE )
	{
		gimpactshape1 = static_cast<D_btGImpactShapeInterface *>(body1->getCollisionShape());

		gimpact_vs_shape(body1,body0,gimpactshape1,body0->getCollisionShape(),true);
	}
}


D_btScalar D_btGImpactCollisionAlgorithm::calculateTimeOfImpact(D_btCollisionObject* body0,D_btCollisionObject* body1,const D_btDispatcherInfo& dispatchInfo,D_btManifoldResult* resultOut)
{
	return 1.f;

}

///////////////////////////////////// REGISTERING ALGORITHM //////////////////////////////////////////////

D_btGImpactCollisionAlgorithm::D_CreateFunc g_gimpact_cf;

//! Use this function for register the algorithm externally
void D_btGImpactCollisionAlgorithm::registerAlgorithm(D_btCollisionDispatcher * dispatcher)
{

	int i;

	for ( i = 0;i < D_MAX_BROADPHASE_COLLISION_TYPES ;i++ )
	{
		dispatcher->registerCollisionCreateFunc(D_GIMPACT_SHAPE_PROXYTYPE,i ,&g_gimpact_cf);
	}

	for ( i = 0;i < D_MAX_BROADPHASE_COLLISION_TYPES ;i++ )
	{
		dispatcher->registerCollisionCreateFunc(i,D_GIMPACT_SHAPE_PROXYTYPE ,&g_gimpact_cf);
	}

}
