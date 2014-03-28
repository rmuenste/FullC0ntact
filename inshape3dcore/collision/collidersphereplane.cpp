#include "collidersphereplane.h"
#include <sphere.h>
#include "collisioninfo.h"
#include <world.h>

namespace i3d {

CColliderSpherePlane::CColliderSpherePlane(void)
{
}

CColliderSpherePlane::~CColliderSpherePlane(void)
{
}

void CColliderSpherePlane::Collide(std::vector<Contact> &vContacts)
{

  //calculate the distance
  Spherer *sphere = dynamic_cast<Spherer *>(m_pBody0->shape_);
  Planer *pPlane  = dynamic_cast<Planer *>(m_pBody1->shape_);
  Real rad1        = sphere->getRadius();
  VECTOR3 vOP      = m_pBody0->com_ - pPlane->m_vOrigin;
  Real signeddist  = (pPlane->m_vNormal * vOP);
  Real dist        = fabs(signeddist);
  dist             = dist - rad1;
  VECTOR3 position = m_pBody0->com_ - (dist*0.5) * pPlane->m_vNormal;
  Real relVel      = (m_pBody0->velocity_+m_pWorld->getGravityEffect(m_pBody0) * 
                      m_pWorld->timeControl_->GetDeltaT()) * pPlane->m_vNormal;  
  //if the bodies are on collision course
  if(relVel < 0.0)
  {
    Real distpertime = -relVel*m_pWorld->timeControl_->GetDeltaT();
    //check whether there will be a collision next time step
    if(dist <= distpertime)
    {
      Contact contact;
      contact.m_dDistance  = signeddist;
      contact.m_vNormal    = pPlane->m_vNormal;
      contact.m_vPosition0 = position;
      contact.m_vPosition1 = position;
      contact.m_pBody0     = m_pBody0;
      contact.m_pBody1     = m_pBody1;
      contact.id0          = contact.m_pBody0->iID_;
      contact.id1          = contact.m_pBody1->iID_;
      contact.vn           = relVel;
      contact.m_iState     = CollisionInfo::TOUCHING;      
      vContacts.push_back(contact);
    }//end if(dist <= distpertime)
  }
  else if(relVel < 0.00001 && dist < 0.005)
  {
    Contact contact;
    contact.m_dDistance  = signeddist;
    contact.m_vNormal    = pPlane->m_vNormal;
    contact.m_vPosition0 = position;
    contact.m_vPosition1 = position;
    contact.m_pBody0     = m_pBody0;
    contact.m_pBody1     = m_pBody1;
    contact.id0          = contact.m_pBody0->iID_;
    contact.id1          = contact.m_pBody1->iID_;
    contact.vn           = relVel;
    contact.m_iState     = CollisionInfo::TOUCHING;      
    vContacts.push_back(contact);
  }
  else if(dist < 0.1*rad1)
  {
    Contact contact;
    contact.m_dDistance  = signeddist;
    contact.m_vNormal    = pPlane->m_vNormal;
    contact.m_vPosition0 = position;
    contact.m_vPosition1 = position;
    contact.m_pBody0     = m_pBody0;
    contact.m_pBody1     = m_pBody1;
    contact.id0 = contact.m_pBody0->iID_;
    contact.id1 = contact.m_pBody1->iID_;
    contact.vn           = relVel;
    contact.m_iState     = CollisionInfo::TOUCHING;
    vContacts.push_back(contact);
  }
  else
  {
    
  }
  
}

}
