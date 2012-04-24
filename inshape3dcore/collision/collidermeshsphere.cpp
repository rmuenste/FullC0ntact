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

void CColliderMeshSphere::Collide(CRigidBody *pBody0, CRigidBody *pBody1, std::vector<CContact> &vContacts, Real dDeltaT)
{

}

void CColliderMeshSphere::Collide(std::vector<CContact> &vContacts, Real dDeltaT)
{
  return;
	//calculate the distance
  CMeshObjectr *pMeshObjectOrig = dynamic_cast<CMeshObjectr*>(m_pBody0->m_pShape);
	CSpherer *pSphere = dynamic_cast<CSpherer *>(m_pBody1->m_pShape);
  CSpherer sphere(m_pBody1->m_vCOM,pSphere->Radius());

  //distance to bounding box greater than eps
  CDistanceMeshSphere<Real> distMeshSphere(&pMeshObjectOrig->m_BVH,sphere);
  Real dist = distMeshSphere.ComputeDistanceEps(0.0004);
  //Real dist = distMeshSphere.ComputeDistanceEpsNaive(0.004);

  std::vector<VECTOR3>::iterator viter = distMeshSphere.m_vPoint.begin();
  int j=0;
  //std::cout<<"Contact point: "<<distMeshSphere.m_vPoint.size()<<std::endl;  
  for(;viter!=distMeshSphere.m_vPoint.end();viter++,j++)
  {
    VECTOR3 vPoint = *viter;
    //VECTOR3 position = sphere->Center() - (dist*0.5) * pPlane->m_vNormal;
    VECTOR3 vR0 = vPoint-m_pBody0->m_vCOM;
    VECTOR3 vR1 = vPoint-m_pBody1->m_vCOM;

    VECTOR3 relVel = 
      (m_pBody0->m_vVelocity + (VECTOR3::Cross(m_pBody0->GetAngVel(),vR0))
      - m_pBody1->m_vVelocity - (VECTOR3::Cross(m_pBody1->GetAngVel(),vR1)));

    Real relativeNormalVelocity = relVel * distMeshSphere.m_vNormals[j];
    //if the bodies are on collision course
    if(relativeNormalVelocity < 0.0)
    {
      //std::cout<<"Pre-contact normal velocity: "<<relVel<<" colliding contact"<<std::endl;
      CContact contact;
      contact.m_dDistance  = dist;
      contact.m_vNormal    = distMeshSphere.m_vNormals[j];
      contact.m_vPosition0 = vPoint;
      contact.m_vPosition1 = vPoint;
      contact.m_pBody0     = m_pBody0;
      contact.m_pBody1     = m_pBody1;
      contact.id0          = contact.m_pBody0->m_iID;
      contact.id1          = contact.m_pBody1->m_iID;
      contact.vn           = relativeNormalVelocity;
      contact.m_iState     = CCollisionInfo::TOUCHING;
      vContacts.push_back(contact);
    }
    else if(relativeNormalVelocity < 0.00001 && dist < 0.005)
    {
      //std::cout<<"Pre-contact normal velocity: "<<relVel<<" resting contact"<<std::endl;
      CContact contact;
      contact.m_dDistance  = dist;
      contact.m_vNormal    = distMeshSphere.m_vNormals[j];
      contact.m_vPosition0 = vPoint;
      contact.m_vPosition1 = vPoint;
      contact.m_pBody0     = m_pBody0;
      contact.m_pBody1     = m_pBody1;
      contact.id0          = contact.m_pBody0->m_iID;
      contact.id1          = contact.m_pBody1->m_iID;
      contact.vn           = relativeNormalVelocity;
      contact.m_iState     = CCollisionInfo::TOUCHING;
      vContacts.push_back(contact);
    }
    else
    {
      CContact contact;
      contact.m_dDistance  = dist;
      contact.m_vNormal    = distMeshSphere.m_vNormals[j];
      contact.m_vPosition0 = vPoint;
      contact.m_vPosition1 = vPoint;
      contact.m_pBody0     = m_pBody0;
      contact.m_pBody1     = m_pBody1;
      contact.id0          = contact.m_pBody0->m_iID;
      contact.id1          = contact.m_pBody1->m_iID;
      contact.vn           = relativeNormalVelocity;
      contact.m_iState     = CCollisionInfo::VANISHING_CLOSEPROXIMITY;
      vContacts.push_back(contact);      
    }
  }//end for
}

}
