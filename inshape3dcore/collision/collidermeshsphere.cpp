#include "collidermeshsphere.h"
#include <meshobject.h>
#include <sphere.h>
#include <distancemeshsphere.h>
#include "collisioninfo.h"

namespace i3d {

ColliderMeshSphere::ColliderMeshSphere(void)
{
}

ColliderMeshSphere::~ColliderMeshSphere(void)
{
}

void ColliderMeshSphere::collide(std::vector<Contact> &vContacts)
{
	//calculate the distance
  CMeshObjectr *pMeshObjectOrig = dynamic_cast<CMeshObjectr*>(body0_->shape_);
	Spherer *pSphere = dynamic_cast<Spherer *>(body1_->shape_);
  VECTOR3 trans = body1_->getTransformedPosition();
  Spherer sphere(trans,pSphere->getRadius());

  //distance to bounding box greater than eps
  CDistanceMeshSphere<Real> distMeshSphere(&pMeshObjectOrig->m_BVH,sphere);
  Real dist = distMeshSphere.ComputeDistanceEpsNaive( 0.1 * pSphere->getRadius());
  std::cout<<"Distance to mesh: "<<dist<<std::endl;
  std::cout<<"Number of Contact points: "<<distMeshSphere.m_vPoint.size()<<std::endl;

  std::vector<VECTOR3>::iterator viter = distMeshSphere.m_vPoint.begin();
  int j=0;

  for(;viter!=distMeshSphere.m_vPoint.end();viter++,j++)
  {

    VECTOR3 vPoint = *viter;
    //VECTOR3 position = sphere->Center() - (dist*0.5) * pPlane->m_vNormal;
    VECTOR3 vR0 = vPoint-body0_->com_;
    VECTOR3 vR1 = vPoint-body1_->com_;

    VECTOR3 relVel = 
      (body0_->velocity_ + (VECTOR3::Cross(body0_->getAngVel(),vR0))
      - body1_->velocity_ - (VECTOR3::Cross(body1_->getAngVel(),vR1)));

    Real relativeNormalVelocity = relVel * distMeshSphere.m_vNormals[j];
    //if the bodies are on collision course
    if(relativeNormalVelocity < 0.0 && dist < 0.1 * pSphere->getRadius())
    {
      //std::cout<<"Pre-contact normal velocity: "<<relVel<<" colliding contact"<<std::endl;
      Contact contact;
      contact.m_dDistance  = dist+sphere.getRadius();
      contact.m_vNormal    = distMeshSphere.m_vNormals[j];
      contact.m_vPosition0 = vPoint;
      contact.m_vPosition1 = vPoint;
      contact.m_pBody0     = body0_;
      contact.m_pBody1     = body1_;
      contact.id0          = contact.m_pBody0->iID_;
      contact.id1          = contact.m_pBody1->iID_;
      contact.vn           = relativeNormalVelocity;
      contact.m_iState     = CollisionInfo::TOUCHING;
      vContacts.push_back(contact);
    }
    else if(dist < 0.1 * pSphere->getRadius())
    {
      //std::cout<<"Pre-contact normal velocity: "<<relVel<<" resting contact"<<std::endl;
      Contact contact;
      contact.m_dDistance  = dist+sphere.getRadius();
      contact.m_vNormal    = distMeshSphere.m_vNormals[j];
      contact.m_vPosition0 = vPoint;
      contact.m_vPosition1 = vPoint;
      contact.m_pBody0     = body0_;
      contact.m_pBody1     = body1_;
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
      contact.m_pBody0     = body0_;
      contact.m_pBody1     = body1_;
      contact.id0          = contact.m_pBody0->iID_;
      contact.id1          = contact.m_pBody1->iID_;
      contact.vn           = relativeNormalVelocity;
      contact.m_iState     = CollisionInfo::VANISHING_CLOSEPROXIMITY;
      vContacts.push_back(contact);      
    }
  }//end for
}

}
