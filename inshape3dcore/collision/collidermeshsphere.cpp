#include "collidermeshsphere.h"
#include <meshobject.h>
#include <sphere.h>
#include <distancemeshsphere.h>
#include "collisioninfo.h"

namespace i3d {

CColliderMeshSphere::CColliderMeshSphere(void)
{
}

CColliderMeshSphere::~CColliderMeshSphere(void)
{
}

void CColliderMeshSphere::Collide(std::vector<Contact> &vContacts)
{
	//calculate the distance
  CMeshObjectr *pMeshObjectOrig = dynamic_cast<CMeshObjectr*>(m_pBody0->shape_);
	CSpherer *pSphere = dynamic_cast<CSpherer *>(m_pBody1->shape_);
  CSpherer sphere(m_pBody1->com_,pSphere->Radius());

  //distance to bounding box greater than eps
  CDistanceMeshSphere<Real> distMeshSphere(&pMeshObjectOrig->m_BVH,sphere);
  Real dist = distMeshSphere.ComputeDistanceEpsNaive( 0.1 * pSphere->Radius());

  std::vector<VECTOR3>::iterator viter = distMeshSphere.m_vPoint.begin();
  int j=0;
  //std::cout<<"Contact point: "<<distMeshSphere.m_vPoint.size()<<std::endl;  
  for(;viter!=distMeshSphere.m_vPoint.end();viter++,j++)
  {
    VECTOR3 vPoint = *viter;
    //VECTOR3 position = sphere->Center() - (dist*0.5) * pPlane->m_vNormal;
    VECTOR3 vR0 = vPoint-m_pBody0->com_;
    VECTOR3 vR1 = vPoint-m_pBody1->com_;

    VECTOR3 relVel = 
      (m_pBody0->velocity_ + (VECTOR3::Cross(m_pBody0->getAngVel(),vR0))
      - m_pBody1->velocity_ - (VECTOR3::Cross(m_pBody1->getAngVel(),vR1)));

    Real relativeNormalVelocity = relVel * distMeshSphere.m_vNormals[j];
    //if the bodies are on collision course
    if(relativeNormalVelocity < 0.0)
    {
      //std::cout<<"Pre-contact normal velocity: "<<relVel<<" colliding contact"<<std::endl;
      Contact contact;
      contact.m_dDistance  = dist;
      contact.m_vNormal    = distMeshSphere.m_vNormals[j];
      contact.m_vPosition0 = vPoint;
      contact.m_vPosition1 = vPoint;
      contact.m_pBody0     = m_pBody0;
      contact.m_pBody1     = m_pBody1;
      contact.id0          = contact.m_pBody0->iID_;
      contact.id1          = contact.m_pBody1->iID_;
      contact.vn           = relativeNormalVelocity;
      contact.m_iState     = CollisionInfo::TOUCHING;
      vContacts.push_back(contact);
    }
    else if(dist < 0.1 * pSphere->Radius())
    {
      //std::cout<<"Pre-contact normal velocity: "<<relVel<<" resting contact"<<std::endl;
      Contact contact;
      contact.m_dDistance  = dist;
      contact.m_vNormal    = distMeshSphere.m_vNormals[j];
      contact.m_vPosition0 = vPoint;
      contact.m_vPosition1 = vPoint;
      contact.m_pBody0     = m_pBody0;
      contact.m_pBody1     = m_pBody1;
      contact.id0          = contact.m_pBody0->iID_;
      contact.id1          = contact.m_pBody1->iID_;
      contact.vn           = relativeNormalVelocity;
      contact.m_iState     = CollisionInfo::TOUCHING;
      vContacts.push_back(contact);
    }
    else
    {
      return;
      Contact contact;
      contact.m_dDistance  = dist;
      contact.m_vNormal    = distMeshSphere.m_vNormals[j];
      contact.m_vPosition0 = vPoint;
      contact.m_vPosition1 = vPoint;
      contact.m_pBody0     = m_pBody0;
      contact.m_pBody1     = m_pBody1;
      contact.id0          = contact.m_pBody0->iID_;
      contact.id1          = contact.m_pBody1->iID_;
      contact.vn           = relativeNormalVelocity;
      contact.m_iState     = CollisionInfo::VANISHING_CLOSEPROXIMITY;
      vContacts.push_back(contact);      
    }
  }//end for
}

}
