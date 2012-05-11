#include "colliderspheresubdomain.h"
#include <sphere.h>
#include <collisioninfo.h>
#include <colliderfactory.h>
#include <world.h>

namespace i3d {

CColliderSphereSubdomain::CColliderSphereSubdomain(void)
{
}

CColliderSphereSubdomain::~CColliderSphereSubdomain(void)
{
}

void CColliderSphereSubdomain::Collide(std::vector<CContact> &vContacts)
{

  //produce a collider for every body of
  //the compound and concatenate the vector
  //of contact points
  CCompoundBody *body1 = dynamic_cast<CCompoundBody*>(m_pBody1);
  CSpherer     *sphere = dynamic_cast<CSpherer *>(m_pBody0->m_pShape);
  CRigidBody       *p0 = m_pBody0;

  for(int i=0;i<body1->GetNumComponents();i++)
  {
    
    CRigidBody *p1 = body1->GetComponent(i);

    //Check every pair
    CColliderFactory colliderFactory;

    //get a collider
    CCollider *collider = colliderFactory.ProduceCollider(p0,p1);

    //attach the world object
    collider->SetWorld(m_pWorld);

    //compute the potential contact points
    collider->Collide(vContacts);
    
    //examine the contacts
    if(!vContacts.empty())
    {
      std::cout<<m_pBody0->m_iID<<","<<m_pBody1->m_iID<<std::endl;      
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
          if(vContacts[i].m_dDistance <= sphere->Radius())
          {
            //send the new_remote_body signal
            std::cout<<"new_remote_body signal "<<m_pBody0->m_iID<<std::endl;         
            m_pWorld->m_lSendList.push_back(std::pair<int,int>(m_pBody1->m_iID,m_pWorld->m_myParInfo.GetID()));
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

}
