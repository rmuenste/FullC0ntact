#include "colliderspherecompound.h"
#include <sphere.h>
#include <collisioninfo.h>
#include <colliderfactory.h>
#include <world.h>

namespace i3d {

ColliderSphereCompound::ColliderSphereCompound(void)
{
}

ColliderSphereCompound::~ColliderSphereCompound(void)
{
}

void ColliderSphereCompound::collide(std::vector<Contact> &vContacts)
{

  //produce a collider for every body of
  //the compound and concatenate the vector
  //of contact points
  CompoundBody *body1 = dynamic_cast<CompoundBody*>(body1_);
  RigidBody       *p0 = body0_;

  for(int i=0;i<body1->getNumComponents();i++)
  {
    
    RigidBody *p1 = body1->getComponent(i);

    //Check every pair
    ColliderFactory colliderFactory;

    //get a collider
    Collider *collider = colliderFactory.ProduceCollider(p0,p1);

    //attach the world object
    collider->setWorld(world_);

    //compute the potential contact points
    collider->collide(vContacts);

    delete collider;
  }

}

}
