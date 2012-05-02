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

namespace i3d {

CColliderFactory::CColliderFactory(void)
{
}

CColliderFactory::~CColliderFactory(void)
{
}

CCollider* CColliderFactory::ProduceCollider(CRigidBody *pBody0, CRigidBody *pBody1)
{
  if(pBody0->GetShape() == CRigidBody::SPHERE)
  {
    return CreateColliderSphereX(pBody0, pBody1);
  }
  else if(pBody0->GetShape() == CRigidBody::BOX)
  {
    //save that body1 is a box
    return CreateColliderBoxX(pBody0, pBody1);
  }
  else if(pBody0->GetShape() == CRigidBody::CYLINDER)
  {
    //save that body1 is a cylinder
    return CreateColliderCylinderX(pBody0,pBody1);
  }
  else if(pBody0->GetShape() == CRigidBody::MESH)
  {
    return CreateColliderMeshX(pBody0,pBody1);
  }
  else if(pBody0->GetShape() == CRigidBody::BOUNDARYBOX)
  {
    //body0 is a boundary
    return CreateColliderBoundaryX(pBody0,pBody1);
  }
  else if(pBody0->GetShape() == CRigidBody::COMPOUND)
  {
    //body0 is a compound rigid body
    return CreateColliderCompoundX(pBody0,pBody1);
  }  
  else if(pBody0->GetShape() == CRigidBody::SUBDOMAIN)
  {
    //body0 is a compound rigid body
    return CreateColliderSubDomainX(pBody0,pBody1);
  }  
  else
  {
    std::cerr<<"Error in ProduceCollider: unknown collider type..."<<std::endl;
    return new CCollider();
  }
}

CCollider* CColliderFactory::CreateColliderBoundaryX(CRigidBody *pBody0, CRigidBody *pBody1)
{
  if(pBody1->GetShape() == CRigidBody::SPHERE)
  {
    //body1 is a boundary
    CCollider *collider = new CColliderSphereBoxBoundary();
    collider->SetBody0(pBody1);
    collider->SetBody1(pBody0);
    return collider;
  }
  else if(pBody1->GetShape() == CRigidBody::BOX)
  {
    //body1 is a box
    CCollider *collider = new CColliderBoxBoxBoundary();
    collider->SetBody0(pBody1);
    collider->SetBody1(pBody0);
    return collider;
  }
  else if(pBody1->GetShape() == CRigidBody::CYLINDER)
  {
    //body1 is a sphere
    CCollider *collider = new CColliderCylinderBoundaryBox();
    collider->SetBody0(pBody1);
    collider->SetBody1(pBody0);
    return collider;
  }
  else if(pBody1->GetShape() == CRigidBody::MESH)
  {
    //body1 is a boundary
    CCollider *collider = new CColliderMeshBoundaryBox();
    collider->SetBody0(pBody1);
    collider->SetBody1(pBody0);
    return collider;
  }
  else if(pBody1->GetShape() == CRigidBody::SUBDOMAIN)
  {
    //body0 is a compound rigid body
    CCollider *collider = new CCollider();
    collider->SetBody0(pBody1);
    collider->SetBody1(pBody0);
    return collider;
  }  
  else
  {
    std::cerr<<"Error in CreateColliderBoundaryX: unknown collider type..."<<std::endl;
    exit(0);
  }
}

CCollider* CColliderFactory::CreateColliderSphereX(CRigidBody *pBody0, CRigidBody *pBody1)
{
	if(pBody1->GetShape() == CRigidBody::SPHERE)
  {
    //body1 is a sphere
    CCollider *collider = new CColliderSphereSphere();
    collider->SetBody0(pBody0);
    collider->SetBody1(pBody1);
		return collider;
  }
	else if(pBody1->GetShape() == CRigidBody::BOUNDARYBOX)
  {
    //body1 is a boundary
    CCollider *collider = new CColliderSphereBoxBoundary();
    collider->SetBody0(pBody0);
    collider->SetBody1(pBody1);
		return collider;
  }
  else if(pBody1->GetShape() == CRigidBody::BOX)
  {
    //body1 is a box
    CCollider *collider = new CColliderBoxSphere();
    collider->SetBody0(pBody0);
    collider->SetBody1(pBody1);
		return collider;
  }
  else if(pBody1->GetShape() == CRigidBody::CYLINDER)
  {
    //body1 is a cylinder
    CCollider *collider = new CColliderConvexConvexGjk();
    collider->m_pGenerator = new CContactGeneratorCylinderSphere<Real>();
    collider->SetBody0(pBody1);
    collider->SetBody1(pBody0);
    collider->SetShape0(CRigidBody::CYLINDER);
    collider->SetShape1(CRigidBody::SPHERE);
    return collider;
  }
  else if(pBody1->GetShape() == CRigidBody::PLANE)
  {
    //body1 is a plane
    CCollider *collider = new CColliderSpherePlane();
    collider->SetBody0(pBody0);
    collider->SetBody1(pBody1);
    collider->SetShape0(CRigidBody::SPHERE);
    collider->SetShape1(CRigidBody::PLANE);
    return collider;
  }
  else if(pBody1->GetShape() == CRigidBody::MESH)  
  {
    //body1 is a mesh
    CCollider *collider = new CColliderMeshSphere();
    collider->SetBody0(pBody1);
    collider->SetBody1(pBody0);
    return collider;
  }
  else if(pBody1->GetShape() == CRigidBody::COMPOUND)
  {
    //body1 is a sphere
    CCollider *collider = new CColliderSphereCompound();
    collider->SetBody0(pBody0);
    collider->SetBody1(pBody1);
    return collider;
  }
  else if(pBody1->GetShape() == CRigidBody::SUBDOMAIN)
  {
    //body0 is a compound rigid body
    std::cerr<<"Error in CreateColliderSphereX: unknown collider type..."<<std::endl;
    exit(0);
  }  
  else
  {
    std::cerr<<"Error in CreateColliderSphereX: unknown collider type..."<<std::endl;
    exit(0);
  }
}

CCollider* CColliderFactory::CreateColliderPlaneX(CRigidBody *pBody0, CRigidBody *pBody1)
{
	if(pBody1->GetShape() == CRigidBody::SPHERE)
  {
    //body1 is a sphere
    CCollider *collider = new CColliderSpherePlane();
    collider->SetBody0(pBody1);
    collider->SetBody1(pBody0);
		return collider;
  }
	else if(pBody1->GetShape() == CRigidBody::BOUNDARYBOX)
  {
    //body1 is a boundary
    CCollider *collider = new CCollider();
    collider->SetBody0(pBody0);
    collider->SetBody1(pBody1);
		return collider;
  }
  else if(pBody1->GetShape() == CRigidBody::BOX)
  {
    //body1 is a box
    CCollider *collider = new CCollider();
    collider->SetBody0(pBody1);
    collider->SetBody1(pBody0);
		return collider;
  }
  else if(pBody1->GetShape() == CRigidBody::CYLINDER)
  {
    //body1 is a sphere
    CCollider *collider = new CCollider();
    collider->SetBody0(pBody1);
    collider->SetBody1(pBody0);
    collider->SetShape0(CRigidBody::CYLINDER);
    collider->SetShape1(CRigidBody::SPHERE);
    return collider;
  }
  else if(pBody1->GetShape() == CRigidBody::COMPOUND)
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

CCollider* CColliderFactory::CreateColliderBoxX(CRigidBody *pBody0, CRigidBody *pBody1)
{
	if(pBody1->GetShape() == CRigidBody::BOUNDARYBOX)
  {
    //body1 is a boundary
    CCollider *collider = new CColliderBoxBoxBoundary();
    collider->SetBody0(pBody0);
    collider->SetBody1(pBody1);
		return collider;
  }
  else if(pBody1->GetShape() == CRigidBody::COMPOUND)
  {
    std::cerr<<"Error in ProduceCollider: COMPOUND collider not implemented..."<<std::endl;
    exit(0);
  }      
	else if(pBody1->GetShape() == CRigidBody::BOX)
  {
    //body1 is a box
    CCollider *collider = new CColliderBoxBox();
    collider->SetBody0(pBody0);
    collider->SetBody1(pBody1);
		return collider;
  }
  else if(pBody1->GetShape() == CRigidBody::SPHERE)  
  {
    //body1 is a sphere
    CCollider *collider = new CColliderBoxSphere();
    collider->SetBody0(pBody1);
    collider->SetBody1(pBody0);
		return collider;
  }
  else if(pBody1->GetShape() == CRigidBody::CYLINDER)  
  {
    //body1 is a sphere
    CCollider *collider = new CColliderConvexConvexGjk();
    collider->m_pGenerator = new CContactGeneratorBoxCylinder<Real>();
    collider->SetBody0(pBody1);
    collider->SetBody1(pBody0);
    collider->SetShape0(CRigidBody::CYLINDER);
    collider->SetShape1(CRigidBody::BOX);
    return collider;
  }
	else if(pBody1->GetShape() == CRigidBody::MESH)
  {
    CCollider *collider = new CColliderMeshMesh();
    collider->SetBody0(pBody1);
    collider->SetBody1(pBody0);
		return collider;
  }
	else
	{
		std::cerr<<"Error in CreateColliderBoxX: unknown collider type..."<<std::endl;
		exit(0);
	}
}

CCollider* CColliderFactory::CreateColliderCylinderX(CRigidBody *pBody0, CRigidBody *pBody1)
{
	if(pBody1->GetShape() == CRigidBody::BOUNDARYBOX)
  {
    //body1 is a boundary
    CCollider *collider = new CColliderCylinderBoundaryBox();
    collider->SetBody0(pBody0);
    collider->SetBody1(pBody1);
		return collider;
  }
  else if(pBody1->GetShape() == CRigidBody::COMPOUND)
  {
    std::cerr<<"Error in ProduceCollider: COMPOUND collider not implemented..."<<std::endl;
    exit(0);
  }      
	else if(pBody1->GetShape() == CRigidBody::BOX)
  {
    //body1 is a box
    CCollider *collider    = new CColliderConvexConvexGjk();
    collider->m_pGenerator = new CContactGeneratorBoxCylinder<Real>();
    collider->SetBody0(pBody0);
    collider->SetBody1(pBody1);
    collider->SetShape0(CRigidBody::CYLINDER);
    collider->SetShape1(CRigidBody::BOX);
		return collider;
  }
  else if(pBody1->GetShape() == CRigidBody::SPHERE)  
  {
    //body1 is a sphere
    CCollider *collider = new CColliderConvexConvexGjk();
    collider->m_pGenerator = new CContactGeneratorCylinderSphere<Real>();
    collider->SetBody0(pBody0);
    collider->SetBody1(pBody1);
    collider->SetShape0(CRigidBody::CYLINDER);
    collider->SetShape1(CRigidBody::SPHERE);
		return collider;
  }
  else if(pBody1->GetShape() == CRigidBody::CYLINDER)  
  {
    //body1 is a sphere
    CCollider *collider = new CColliderConvexConvexGjk();
    collider->m_pGenerator = new CContactGenerator2Cylinder<Real>();
    collider->SetBody0(pBody0);
    collider->SetBody1(pBody1);
    collider->SetShape0(CRigidBody::CYLINDER);
    collider->SetShape1(CRigidBody::CYLINDER);
		return collider;
  }
	else
	{
		std::cerr<<"Error in CreateColliderCylinderX: unknown collider type..."<<std::endl;
		exit(0);
	}

}

CCollider* CColliderFactory::CreateColliderMeshX(CRigidBody *pBody0, CRigidBody *pBody1)
{
	if(pBody1->GetShape() == CRigidBody::BOUNDARYBOX)
  {
    //body1 is a boundary
    CCollider *collider = new CColliderMeshBoundaryBox();
    collider->SetBody0(pBody0);
    collider->SetBody1(pBody1);
		return collider;
  }
  else if(pBody1->GetShape() == CRigidBody::COMPOUND)
  {
    std::cerr<<"Error in ProduceCollider: COMPOUND collider not implemented..."<<std::endl;
    exit(0);
  }      
	else if(pBody1->GetShape() == CRigidBody::BOX)
  {
    //body1 is a box
    //convertBox2Mesh and call collidermeshmesh
    CCollider *collider = new CColliderMeshMesh();
    collider->SetBody0(pBody0);
    collider->SetBody1(pBody1);
    return collider;
  }
  else if(pBody1->GetShape() == CRigidBody::SPHERE)  
  {
    //body1 is a sphere
    CCollider *collider = new CColliderMeshSphere();
    collider->SetBody0(pBody0);
    collider->SetBody1(pBody1);
    return collider;
  }
  else if(pBody1->GetShape() == CRigidBody::CYLINDER)  
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

CCollider* CColliderFactory::CreateColliderCompoundX(CRigidBody *pBody0, CRigidBody *pBody1)
{
  if(pBody1->GetShape() == CRigidBody::SPHERE)  
  {
    //body1 is a sphere
    CCollider *collider = new CColliderSphereCompound();
    collider->SetBody0(pBody1);
    collider->SetBody1(pBody0);
    return collider;
  }
  else if(pBody1->GetShape() == CRigidBody::COMPOUND)
  {
    CCollider *collider = new CCollider();
    return collider;
  }
	else
	{
		std::cerr<<"Error in CreateColliderCompoundX: unknown collider type..."<<std::endl;
		exit(0);
	}
}

CCollider* CColliderFactory::CreateColliderSubDomainX(CRigidBody *pBody0, CRigidBody *pBody1)
{
  if(pBody1->GetShape() == CRigidBody::SPHERE)  
  {
    //body1 is a sphere
    CCollider *collider = new CCollider();
    collider->SetBody0(pBody1);
    collider->SetBody1(pBody0);
    return collider;
  }
  else if(pBody1->GetShape() == CRigidBody::COMPOUND)
  {
    CCollider *collider = new CCollider();
    return collider;
  }
  else if(pBody1->GetShape() == CRigidBody::BOUNDARYBOX)
  {
    CCollider *collider = new CCollider();
    return collider;
  }
	else
	{
		std::cerr<<"Error in CreateColliderSubDomainX: unknown collider type..."<<std::endl;
		exit(0);
	}
}

}
