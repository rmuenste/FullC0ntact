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

void CColliderSpherePlane::Collide(std::vector<CContact> &vContacts)
{

  //calculate the distance
  CSpherer *sphere = dynamic_cast<CSpherer *>(m_pBody0->m_pShape);
  CPlaner *pPlane  = dynamic_cast<CPlaner *>(m_pBody1->m_pShape);
  Real rad1        = sphere->Radius();
  VECTOR3 vOP      = m_pBody0->m_vCOM - pPlane->m_vOrigin;
  Real signeddist  = (pPlane->m_vNormal * vOP);
  Real dist        = fabs(signeddist);
  dist             = dist - rad1;
  VECTOR3 position = m_pBody0->m_vCOM - (dist*0.5) * pPlane->m_vNormal;
  Real relVel      = (m_pBody0->m_vVelocity+m_pWorld->GetGravityEffect(m_pBody0) * 
                      m_pWorld->m_pTimeControl->GetDeltaT()) * pPlane->m_vNormal;  
  //if the bodies are on collision course
  if(relVel < 0.0)
  {
    Real distpertime = -relVel*m_pWorld->m_pTimeControl->GetDeltaT();
    //check whether there will be a collision next time step
    if(dist <= distpertime)
    {
      CContact contact;
      contact.m_dDistance  = signeddist;
      contact.m_vNormal    = pPlane->m_vNormal;
      contact.m_vPosition0 = position;
      contact.m_vPosition1 = position;
      contact.m_pBody0     = m_pBody0;
      contact.m_pBody1     = m_pBody1;
      contact.id0          = contact.m_pBody0->m_iID;
      contact.id1          = contact.m_pBody1->m_iID;
      contact.vn           = relVel;
      contact.m_iState     = CCollisionInfo::TOUCHING;      
      vContacts.push_back(contact);
    }//end if(dist <= distpertime)
  }
  else if(relVel < 0.00001 && dist < 0.005)
  {
    CContact contact;
    contact.m_dDistance  = signeddist;
    contact.m_vNormal    = pPlane->m_vNormal;
    contact.m_vPosition0 = position;
    contact.m_vPosition1 = position;
    contact.m_pBody0     = m_pBody0;
    contact.m_pBody1     = m_pBody1;
    contact.id0          = contact.m_pBody0->m_iID;
    contact.id1          = contact.m_pBody1->m_iID;
    contact.vn           = relVel;
    contact.m_iState     = CCollisionInfo::TOUCHING;      
    vContacts.push_back(contact);
  }
  else if(dist < 0.1*rad1)
  {
    CContact contact;
    contact.m_dDistance  = signeddist;
    contact.m_vNormal    = pPlane->m_vNormal;
    contact.m_vPosition0 = position;
    contact.m_vPosition1 = position;
    contact.m_pBody0     = m_pBody0;
    contact.m_pBody1     = m_pBody1;
    contact.id0 = contact.m_pBody0->m_iID;
    contact.id1 = contact.m_pBody1->m_iID;
    contact.vn           = relVel;
    contact.m_iState     = CCollisionInfo::TOUCHING;
    vContacts.push_back(contact);
  }
  else
  {
    
  }
  
}

}
