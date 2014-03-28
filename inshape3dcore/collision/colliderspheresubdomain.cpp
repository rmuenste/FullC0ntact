#include "colliderspheresubdomain.h"
#include <sphere.h>
#include <collisioninfo.h>
#include <colliderfactory.h>
#include <world.h>
#include <subdomainboundary.h>

namespace i3d {

ColliderSphereSubdomain::ColliderSphereSubdomain(void)
{
}

ColliderSphereSubdomain::~ColliderSphereSubdomain(void)
{
}

void ColliderSphereSubdomain::collide(std::vector<Contact> &vContacts)
{

  //produce a collider for every body of
  //the compound and concatenate the vector
  //of contact points
  CompoundBody *body1 = dynamic_cast<CompoundBody*>(body1_);
  Spherer     *sphere = dynamic_cast<Spherer *>(body0_->shape_);
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
    
    //examine the contacts
    if(!vContacts.empty())
    {
      for(unsigned int i = 0;i<vContacts.size();i++)
      {
        //examine the signed distance of the center
        //of mass to the subdomain boundary
        
        //check whether the body is still in the subdomain
        if(vContacts[i].m_dDistance >= 0.0)
        {
          //if the distance is less than the radius
          //the body is part of the neighboring subdomain
          //and we make it a remote body in the other subdomain
          if(vContacts[i].m_dDistance <= sphere->getRadius())
          {
            //send the new_remote_body signal
            world_->sendList_.push_back(std::pair<int,int>(body1_->iID_,world_->parInfo_.getId()));
            break;
          }
        }
        else
        {
          //the body's center of mass is in the other domain
          //the body is no longer a local body in this domain
          
          //if in the previous time step it was a local body
          //in this domain and a remote body in the other domain 
          //the roles now switch
        }        
      }
    }
    delete collider;
  }
}

void ColliderSphereSubdomain::Collide()
{

  //produce a collider for every body of
  //the compound and concatenate the vector
  //of contact points
  SubdomainBoundary *body1 = dynamic_cast<SubdomainBoundary*>(body1_);
  Spherer     *sphere = dynamic_cast<Spherer *>(body0_->shape_);
  RigidBody       *p0 = body0_;

  for(int j=0;j<body1->GetNumComponents();j++)
  {
    
    RigidBody *p1 = body1->GetComponent(j);

    //Check every pair
    ColliderFactory colliderFactory;

    //get a collider
    Collider *collider = colliderFactory.ProduceCollider(p0,p1);

    //attach the world object
    collider->setWorld(world_);

    std::vector<Contact> vContacts;

    //compute the potential contact points
    collider->collide(vContacts);
    
    //examine the contacts
    if(!vContacts.empty())
    {
      for(unsigned int i = 0;i<vContacts.size();i++)
      {
        //examine the signed distance of the center
        //of mass to the subdomain boundary
        
        //check whether the body is still in the subdomain
        if(vContacts[i].m_dDistance >= 0.0)
        {
          //if the distance is less than the radius
          //the body is part of the neighboring subdomain
          //and we make it a remote body in the other subdomain
          if(vContacts[i].m_dDistance <= sphere->getRadius())
          {
            //send the new_remote_body signal
            int iNeighbor = body1->GetNeighbor(j);
            SubdomainContact scontact;
            scontact.m_iNeighbor = iNeighbor;
            scontact.m_dDistance = vContacts[i].m_dDistance;
            m_vContacts.push_back(scontact);
          }
        }
        else
        {
          //the body's center of mass is in the other domain
          //the body is no longer a local body in this domain
         
          //if in the previous time step it was a local body
          //in this domain and a remote body in the other domain 
          //the roles now switch
        }        
      }
    }
    delete collider;
  }
}

}
