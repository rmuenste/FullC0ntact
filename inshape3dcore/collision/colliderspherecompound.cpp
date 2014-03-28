#include "colliderspherecompound.h"
#include <sphere.h>
#include <collisioninfo.h>
#include <colliderfactory.h>
#include <world.h>

namespace i3d {

CColliderSphereCompound::CColliderSphereCompound(void)
{
}

CColliderSphereCompound::~CColliderSphereCompound(void)
{
}

void CColliderSphereCompound::Collide(std::vector<Contact> &vContacts)
{

  //produce a collider for every body of
  //the compound and concatenate the vector
  //of contact points
  CompoundBody *body1 = dynamic_cast<CompoundBody*>(m_pBody1);
  RigidBody       *p0 = m_pBody0;

  for(int i=0;i<body1->GetNumComponents();i++)
  {
    
    RigidBody *p1 = body1->GetComponent(i);

    //Check every pair
    CColliderFactory colliderFactory;

    //get a collider
    CCollider *collider = colliderFactory.ProduceCollider(p0,p1);

    //attach the world object
    collider->SetWorld(m_pWorld);

    //compute the potential contact points
    collider->Collide(vContacts);

    delete collider;
  }

}

}
