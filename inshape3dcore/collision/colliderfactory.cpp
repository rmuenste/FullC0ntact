#include "colliderfactory.h"
#include <stdlib.h>
#include <iostream>
#include <colliderspheresphere.h>
#include <collidersphereboxboundary.h>
#include <colliderboxboxboundary.h>
#include <colliderboxbox.h>
#include <colliderboxsphere.h>
#include <colliderconvexconvexgjk.h>
#include <collidercylinderboundarybox.h>
#include <contactgenerator2cylinder.h>
#include <contactgeneratorboxcylinder.h>
#include <contactgeneratorcylindersphere.h>
#include <collidersphereplane.h>
#include <collidermeshboundarybox.h>
#include <collidermeshsphere.h>
#include <collidermeshmesh.h>
#include <colliderspherecompound.h>
#include <colliderspheresubdomain.h>
#include <colliderspherecylindricalboundary.h>
#include <collidermeshmesh.h>

namespace i3d {

ColliderFactory::ColliderFactory(void)
{
}

ColliderFactory::~ColliderFactory(void)
{
}

Collider* ColliderFactory::ProduceCollider(RigidBody *pBody0, RigidBody *pBody1)
{
  if(pBody0->getShape() == RigidBody::SPHERE)
  {
    return CreateColliderSphereX(pBody0, pBody1);
  }
  else if(pBody0->getShape() == RigidBody::BOX)
  {
    //save that body1 is a box
    return CreateColliderBoxX(pBody0, pBody1);
  }
  else if(pBody0->getShape() == RigidBody::CYLINDER)
  {
    //save that body1 is a cylinder
    return CreateColliderCylinderX(pBody0,pBody1);
  }
  else if(pBody0->getShape() == RigidBody::MESH)
  {
    return CreateColliderMeshX(pBody0,pBody1);
  }
  else if(pBody0->getShape() == RigidBody::BOUNDARYBOX)
  {
    //body0 is a boundary
    return CreateColliderBoundaryX(pBody0,pBody1);
  }
  else if (pBody0->getShape() == RigidBody::CYLINDERBDRY || pBody0->getShape() == RigidBody::HALLOWCYLBDRY)
  {
    //body0 is a boundary
    return CreateColliderCylinderBoundaryX(pBody0, pBody1);
  }
  else if(pBody0->getShape() == RigidBody::COMPOUND)
  {
    //body0 is a compound rigid body
    return CreateColliderCompoundX(pBody0,pBody1);
  }  
  else if(pBody0->getShape() == RigidBody::SUBDOMAIN)
  {
    //body0 is a compound rigid body
    return CreateColliderSubDomainX(pBody0,pBody1);
  }  
  else
  {
    std::cerr<<"Error in ProduceCollider: unknown collider type..."<<std::endl;
    return new Collider();
  }
}

Collider* ColliderFactory::CreateColliderBoundaryX(RigidBody *pBody0, RigidBody *pBody1)
{

  if(pBody1->getShape() == RigidBody::SPHERE)
  {
    //body1 is a boundary
    Collider *collider = new ColliderSphereBoxBoundary();
    collider->setBody0(pBody1);
    collider->setBody1(pBody0);
    return collider;
  }
  else if(pBody1->getShape() == RigidBody::BOX)
  {
    //body1 is a box
    Collider *collider = new ColliderBoxBoxBoundary();
    collider->setBody0(pBody1);
    collider->setBody1(pBody0);
    return collider;
  }
  else if(pBody1->getShape() == RigidBody::CYLINDER)
  {
    //body1 is a sphere
    Collider *collider = new ColliderCylinderBoundaryBox();
    collider->setBody0(pBody1);
    collider->setBody1(pBody0);
    return collider;
  }
  else if(pBody1->getShape() == RigidBody::MESH)
  {
    //body1 is a boundary
    Collider *collider = new ColliderMeshBoundaryBox();
    collider->setBody0(pBody1);
    collider->setBody1(pBody0);
    return collider;
  }
  else if(pBody1->getShape() == RigidBody::SUBDOMAIN)
  {
    //body0 is a compound rigid body
    Collider *collider = new Collider();
    collider->setBody0(pBody1);
    collider->setBody1(pBody0);
    return collider;
  }  
  else
  {
    std::cerr<<"Error in CreateColliderBoundaryX: unknown collider type..."<<std::endl;
    exit(0);
  }
}

Collider* ColliderFactory::CreateColliderCylinderBoundaryX(RigidBody *pBody0, RigidBody *pBody1)
{
  if(pBody1->getShape() == RigidBody::SPHERE)
  {
    //body1 is a boundary
    Collider *collider = new ColliderSphereCylindricalBoundary();
    collider->setBody0(pBody1);
    collider->setBody1(pBody0);
    return collider;
  }
  else
  {
    std::cerr<<"Error in CreateColliderCylinderBoundaryX: unknown collider type..."<<std::endl;
    exit(0);
  }
}

Collider* ColliderFactory::CreateColliderSphereX(RigidBody *pBody0, RigidBody *pBody1)
{
	if(pBody1->getShape() == RigidBody::SPHERE)
  {
    //body1 is a sphere
    Collider *collider = new ColliderSphereSphere();
    collider->setBody0(pBody0);
    collider->setBody1(pBody1);
		return collider;
  }
  else if (pBody1->getShape() == RigidBody::CYLINDERBDRY || pBody1->getShape() == RigidBody::HALLOWCYLBDRY)
  {
    //body1 is a boundary
    Collider *collider = new ColliderSphereCylindricalBoundary();
    collider->setBody0(pBody0);
    collider->setBody1(pBody1);
    return collider;
  }
	else if(pBody1->getShape() == RigidBody::BOUNDARYBOX)
  {
    //std::cout<<"CylinderBoundaryCollider Created SphereX..."<<std::endl;             
    Collider *collider = new ColliderSphereBoxBoundary();
    collider->setBody0(pBody0);
    collider->setBody1(pBody1);
		return collider;      
  }
  else if(pBody1->getShape() == RigidBody::BOX)
  {
    //body1 is a box
    Collider *collider = new ColliderBoxSphere();
    collider->setBody0(pBody0);
    collider->setBody1(pBody1);
		return collider;
  }
  else if(pBody1->getShape() == RigidBody::CYLINDER)
  {
    //body1 is a cylinder
    Collider *collider = new ColliderConvexConvexGjk();
    collider->generator_ = new CContactGeneratorCylinderSphere<Real>();
    collider->setBody0(pBody1);
    collider->setBody1(pBody0);
    collider->setShape0(RigidBody::CYLINDER);
    collider->setShape1(RigidBody::SPHERE);
    return collider;
  }
  else if(pBody1->getShape() == RigidBody::PLANE)
  {
    //body1 is a plane
    Collider *collider = new ColliderSpherePlane();
    collider->setBody0(pBody0);
    collider->setBody1(pBody1);
    collider->setShape0(RigidBody::SPHERE);
    collider->setShape1(RigidBody::PLANE);
    return collider;
  }
  else if(pBody1->getShape() == RigidBody::MESH)  
  {
    //body1 is a mesh
    Collider *collider = new ColliderMeshSphere();
    collider->setBody0(pBody1);
    collider->setBody1(pBody0);
    return collider;
  }
  else if(pBody1->getShape() == RigidBody::COMPOUND)
  {
    //body1 is a compound object
    Collider *collider = new ColliderSphereCompound();
    collider->setBody0(pBody0);
    collider->setBody1(pBody1);
    return collider;
  }
  else if(pBody1->getShape() == RigidBody::SUBDOMAIN)
  {
    //body1 is a sphere
    Collider *collider = new ColliderSphereSubdomain();
    collider->setBody0(pBody0);
    collider->setBody1(pBody1);
    return collider;
  }  
  else
  {
    std::cerr<<"Error in CreateColliderSphereX: unknown collider type..."<<std::endl;
    exit(0);
  }
}

Collider* ColliderFactory::CreateColliderPlaneX(RigidBody *pBody0, RigidBody *pBody1)
{
	if(pBody1->getShape() == RigidBody::SPHERE)
  {
    //body1 is a sphere
    Collider *collider = new ColliderSpherePlane();
    collider->setBody0(pBody1);
    collider->setBody1(pBody0);
		return collider;
  }
	else if(pBody1->getShape() == RigidBody::BOUNDARYBOX)
  {
    //body1 is a boundary
    Collider *collider = new Collider();
    collider->setBody0(pBody0);
    collider->setBody1(pBody1);
		return collider;
  }
  else if(pBody1->getShape() == RigidBody::BOX)
  {
    //body1 is a box
    Collider *collider = new Collider();
    collider->setBody0(pBody1);
    collider->setBody1(pBody0);
		return collider;
  }
  else if(pBody1->getShape() == RigidBody::CYLINDER)
  {
    //body1 is a sphere
    Collider *collider = new Collider();
    collider->setBody0(pBody1);
    collider->setBody1(pBody0);
    collider->setShape0(RigidBody::CYLINDER);
    collider->setShape1(RigidBody::SPHERE);
    return collider;
  }
  else if(pBody1->getShape() == RigidBody::COMPOUND)
  {
    std::cerr<<"Error in ProduceCollider: COMPOUND collider not implemented..."<<std::endl;
    exit(0);
  }      
	else
	{
		std::cerr<<"Error in CreateColliderPlaneX: unknown collider type..."<<std::endl;
		exit(0);
	}
}

Collider* ColliderFactory::CreateColliderBoxX(RigidBody *pBody0, RigidBody *pBody1)
{
	if(pBody1->getShape() == RigidBody::BOUNDARYBOX)
  {
    //body1 is a boundary
    Collider *collider = new ColliderBoxBoxBoundary();
    collider->setBody0(pBody0);
    collider->setBody1(pBody1);
		return collider;
  }
  else if(pBody1->getShape() == RigidBody::COMPOUND)
  {
    std::cerr<<"Error in ProduceCollider: COMPOUND collider not implemented..."<<std::endl;
    exit(0);
  }      
	else if(pBody1->getShape() == RigidBody::BOX)
  {
    //body1 is a box
    Collider *collider = new ColliderBoxBox();
    collider->setBody0(pBody0);
    collider->setBody1(pBody1);
		return collider;
  }
  else if(pBody1->getShape() == RigidBody::SPHERE)  
  {
    //body1 is a sphere
    Collider *collider = new ColliderBoxSphere();
    collider->setBody0(pBody1);
    collider->setBody1(pBody0);
		return collider;
  }
  else if(pBody1->getShape() == RigidBody::CYLINDER)  
  {
    //body1 is a sphere
    Collider *collider = new ColliderConvexConvexGjk();
    collider->generator_ = new CContactGeneratorBoxCylinder<Real>();
    collider->setBody0(pBody1);
    collider->setBody1(pBody0);
    collider->setShape0(RigidBody::CYLINDER);
    collider->setShape1(RigidBody::BOX);
    return collider;
  }
	else if(pBody1->getShape() == RigidBody::MESH)
  {
    Collider *collider = new ColliderMeshMesh();
    collider->setBody0(pBody1);
    collider->setBody1(pBody0);
		return collider;
  }
	else
	{
		std::cerr<<"Error in CreateColliderBoxX: unknown collider type..."<<std::endl;
		exit(0);
	}
}

Collider* ColliderFactory::CreateColliderCylinderX(RigidBody *pBody0, RigidBody *pBody1)
{
	if(pBody1->getShape() == RigidBody::BOUNDARYBOX)
  {
    //body1 is a boundary
    Collider *collider = new ColliderCylinderBoundaryBox();
    collider->setBody0(pBody0);
    collider->setBody1(pBody1);
		return collider;
  }
  else if(pBody1->getShape() == RigidBody::COMPOUND)
  {
    std::cerr<<"Error in ProduceCollider: COMPOUND collider not implemented..."<<std::endl;
    exit(0);
  }      
	else if(pBody1->getShape() == RigidBody::BOX)
  {
    //body1 is a box
    Collider *collider    = new ColliderConvexConvexGjk();
    collider->generator_ = new CContactGeneratorBoxCylinder<Real>();
    collider->setBody0(pBody0);
    collider->setBody1(pBody1);
    collider->setShape0(RigidBody::CYLINDER);
    collider->setShape1(RigidBody::BOX);
		return collider;
  }
  else if(pBody1->getShape() == RigidBody::SPHERE)  
  {
    //body1 is a sphere
    Collider *collider = new ColliderConvexConvexGjk();
    collider->generator_ = new CContactGeneratorCylinderSphere<Real>();
    collider->setBody0(pBody0);
    collider->setBody1(pBody1);
    collider->setShape0(RigidBody::CYLINDER);
    collider->setShape1(RigidBody::SPHERE);
		return collider;
  }
  else if(pBody1->getShape() == RigidBody::CYLINDER)  
  {
    //body1 is a sphere
    Collider *collider = new ColliderConvexConvexGjk();
    collider->generator_ = new CContactGenerator2Cylinder<Real>();
    collider->setBody0(pBody0);
    collider->setBody1(pBody1);
    collider->setShape0(RigidBody::CYLINDER);
    collider->setShape1(RigidBody::CYLINDER);
		return collider;
  }
	else
	{
		std::cerr<<"Error in CreateColliderCylinderX: unknown collider type..."<<std::endl;
		exit(0);
	}

}

Collider* ColliderFactory::CreateColliderMeshX(RigidBody *pBody0, RigidBody *pBody1)
{
	if(pBody1->getShape() == RigidBody::BOUNDARYBOX)
  {
    //body1 is a boundary
    Collider *collider = new ColliderMeshBoundaryBox();
    collider->setBody0(pBody0);
    collider->setBody1(pBody1);
		return collider;
  }
  else if(pBody1->getShape() == RigidBody::COMPOUND)
  {
    std::cerr<<"Error in ProduceCollider: COMPOUND collider not implemented..."<<std::endl;
    exit(0);
  }      
	else if(pBody1->getShape() == RigidBody::BOX)
  {
    //body1 is a box
    //convertBox2Mesh and call collidermeshmesh
    Collider *collider = new ColliderMeshMesh();
    collider->setBody0(pBody0);
    collider->setBody1(pBody1);
    return collider;
  }
  else if(pBody1->getShape() == RigidBody::SPHERE)  
  {
    //body1 is a sphere
    Collider *collider = new ColliderMeshSphere();
    collider->setBody0(pBody0);
    collider->setBody1(pBody1);
    return collider;
  }
  else if(pBody1->getShape() == RigidBody::MESH)  
  {
    //body1 is a sphere
    Collider *collider = new ColliderMeshMesh();
    collider->setBody0(pBody0);
    collider->setBody1(pBody1);
    return collider;
  }  
  else if(pBody1->getShape() == RigidBody::CYLINDER)  
  {
    //body1 is a sphere
		std::cerr<<"Error in CreateColliderMeshX: unknown collider type..."<<std::endl;
		exit(0);
  }
	else
	{
		std::cerr<<"Error in CreateColliderMeshX: unknown collider type..."<<std::endl;
		exit(0);
	}

}

Collider* ColliderFactory::CreateColliderCompoundX(RigidBody *pBody0, RigidBody *pBody1)
{
  if(pBody1->getShape() == RigidBody::SPHERE)  
  {
    //body1 is a sphere
    Collider *collider = new ColliderSphereCompound();
    collider->setBody0(pBody1);
    collider->setBody1(pBody0);
    return collider;
  }
  else if(pBody1->getShape() == RigidBody::COMPOUND)
  {
    Collider *collider = new Collider();
    return collider;
  }
	else
	{
		std::cerr<<"Error in CreateColliderCompoundX: unknown collider type..."<<std::endl;
		exit(0);
	}
}

Collider* ColliderFactory::CreateColliderSubDomainX(RigidBody *pBody0, RigidBody *pBody1)
{
  if(pBody1->getShape() == RigidBody::SPHERE)  
  {
    Collider *collider = new ColliderSphereSubdomain();
    collider->setBody0(pBody1);
    collider->setBody1(pBody0);
    return collider;
  }
  else if(pBody1->getShape() == RigidBody::COMPOUND)
  {
    Collider *collider = new Collider();
    return collider;
  }
  else if(pBody1->getShape() == RigidBody::BOUNDARYBOX)
  {
    Collider *collider = new Collider();
    return collider;
  }
  else
  {
    std::cerr<<"Error in CreateColliderSubDomainX: unknown collider type..."<<std::endl;
    exit(0);
  }
}

}
