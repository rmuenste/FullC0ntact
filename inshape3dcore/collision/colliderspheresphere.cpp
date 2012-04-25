#include "colliderspheresphere.h"
#include <sphere.h>
#include <mymath.h>
#include "collisioninfo.h"
#include <world.h>

namespace i3d {

CColliderSphereSphere::CColliderSphereSphere(void)
{

}

CColliderSphereSphere::~CColliderSphereSphere(void)
{

}

void CColliderSphereSphere::Collide(std::vector<CContact> &vContacts)
{
  const Real contactTolerance = 0.00005;
	VECTOR3 &vel1 = m_pBody0->m_vVelocity;
	VECTOR3 &pos1 = m_pBody0->m_vCOM;

	CSphere<Real> *pSphere = dynamic_cast<CSphere<Real>* >(m_pBody0->m_pShape);
	Real rad1 = pSphere->Radius();

	VECTOR3 &vel2 = m_pBody1->m_vVelocity;
	VECTOR3 &pos2 = m_pBody1->m_vCOM;

	pSphere = dynamic_cast<CSphere<Real>* >(m_pBody1->m_pShape);
	Real rad2 = pSphere->Radius();

	Real dist = std::numeric_limits<Real>::max();

	//calc distance and relative orientation
	//we first need to calculate the relative velocity and
	//the velocity along the normal
	VECTOR3 vn = pos1 - pos2;
	vn.Normalize();

	//calculate the relative velocity
	VECTOR3 v12 = vel1 - vel2;
	
	//at last calculate the velocity along the normal
	Real velalongnormal = vn * v12;

  //calculate the distance
  dist = (pos2-pos1).mag() - rad1 - rad2;
  Real dist1 = fabs(vn*vel1);
  Real dist2 = fabs(vn*vel2);
  Real distpertime = (dist1+dist2)*m_pWorld->m_pTimeControl->GetDeltaT();  
  
  if(velalongnormal < -0.005 && distpertime >= dist)
  //if(relativeNormalVelocity < -0.005)
  { 
      CContact contact;
      contact.m_dDistance  = dist;
      contact.m_vNormal    = vn;
      contact.m_vPosition0 = pos1;
      contact.m_vPosition1 = pos2;
      contact.m_pBody0     = m_pBody0;
      contact.m_pBody1     = m_pBody1;
      contact.id0 = contact.m_pBody0->m_iID;
      contact.id1 = contact.m_pBody1->m_iID;
      contact.vn           = velalongnormal;
      contact.m_iState     = CCollisionInfo::TOUCHING;      
      //std::cout<<"Pre-contact normal velocity: "<<velalongnormal<<" colliding contact"<<std::endl;
      //std::cout<<"Pre-contact angular velocity0: "<<contact.m_pBody0->GetAngVel();
      //std::cout<<"Pre-contact angular velocity1: "<<contact.m_pBody1->GetAngVel();
      //std::cout<<"Pre-contact  velocity0: "<<contact.m_pBody0->m_vVelocity;
      //std::cout<<"Pre-contact  velocity1: "<<contact.m_pBody1->m_vVelocity;
      vContacts.push_back(contact);
  }
  else if(velalongnormal < 0.00001 && dist < contactTolerance)
  {
    CContact contact;
    contact.m_dDistance  = dist;
    contact.m_vNormal    = vn;
    contact.m_vPosition0 = pos1;
    contact.m_vPosition1 = pos2;
    contact.m_pBody0     = m_pBody0;
    contact.m_pBody1     = m_pBody1;    
    contact.id0          = contact.m_pBody0->m_iID;
    contact.id1          = contact.m_pBody1->m_iID;
    contact.vn           = velalongnormal;
    contact.m_iState     = CCollisionInfo::TOUCHING;
    vContacts.push_back(contact);
  }
  else if(dist < 0.1*rad1)
  {
    CContact contact;
    contact.m_dDistance  = dist;
    contact.m_vNormal    = vn;
    contact.m_vPosition0 = pos1;
    contact.m_vPosition1 = pos2;
    contact.m_pBody0     = m_pBody0;
    contact.m_pBody1     = m_pBody1;    
    contact.id0          = contact.m_pBody0->m_iID;
    contact.id1          = contact.m_pBody1->m_iID;
    contact.vn           = velalongnormal;
    contact.m_iState     = CCollisionInfo::TOUCHING;
    vContacts.push_back(contact);
  }
  else
  {
    return;
    CContact contact;
    contact.m_dDistance  = dist;
    contact.m_vNormal    = vn;
    contact.m_vPosition0 = pos1;
    contact.m_vPosition1 = pos2;
    contact.m_pBody0     = m_pBody0;
    contact.m_pBody1     = m_pBody1;
    contact.id0 = contact.m_pBody0->m_iID;
    contact.id1 = contact.m_pBody1->m_iID;
    contact.vn           = velalongnormal;
    contact.m_iState     = CCollisionInfo::VANISHING_CLOSEPROXIMITY;      
    vContacts.push_back(contact);    
  }

}


}