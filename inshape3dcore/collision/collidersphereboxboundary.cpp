/*
   This library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Library General Public
   License version 2 as published by the Free Software Foundation.

   This library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Library General Public License for more details.

   You should have received a copy of the GNU Library General Public License
   along with this library; see the file COPYING.LIB.  If not, write to
   the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
   Boston, MA 02110-1301, USA.
*/

#include "collidersphereboxboundary.h"
#include <sphere.h>
#include <collisioninfo.h>
#include <distancepointrec.h>
#include <world.h>
#include <cylinder.h>
#include <quaternion.h>

namespace i3d {

ColliderSphereBoxBoundary::ColliderSphereBoxBoundary(void)
{
}

ColliderSphereBoxBoundary::~ColliderSphereBoxBoundary(void)
{
}

void ColliderSphereBoxBoundary::collide(std::vector<Contact> &vContacts)
{

#ifdef FC_MPI_SUPPORT  
  if(!m_pBody0->IsLocal())
    return;
#endif
  
  //now check for all walls
  for(int k=0;k<6;k++)
  {
    //calculate the distance
    int indexOrigin = k/2;


    Spherer *sphere         = dynamic_cast<Spherer *>(body0_->shape_);
    BoundaryBoxr *pBoundary = dynamic_cast<BoundaryBoxr *>(body1_->shape_);
	  VECTOR3 trans = body0_->getTransformedPosition();
    Real rad1 = sphere->getRadius();
    Real position = trans.m_dCoords[indexOrigin];
    Real distcenter = fabs(pBoundary->extents_[k] - position);
    Real dist = distcenter -rad1;
    Real relVel = (body0_->velocity_+world_->getGravityEffect(body0_)*world_->timeControl_->GetDeltaT()) * pBoundary->normals_[k];

    //if the bodies are on collision course
    if(relVel < 0.0)
    {
      Real distpertime = -relVel*world_->timeControl_->GetDeltaT();
      //check whether there will be a collision next time step
      if(dist <= distpertime)
      {
        //std::cout<<"Pre-contact normal velocity: "<<relVel<<" colliding contact"<<std::endl;
 
        Contact contact;
        contact.m_dDistance  = distcenter;
        contact.m_vNormal    = pBoundary->normals_[k];
        //pos = center - (dist*Normal)
		    VECTOR3 pos = trans - (dist*pBoundary->normals_[k]);
        contact.m_vPosition0 = pos;
        contact.m_vPosition1 = pos;
        contact.m_pBody0     = body0_;
        contact.m_pBody1     = body1_;
        contact.id0 = contact.m_pBody0->iID_;
        contact.id1 = contact.m_pBody1->iID_;
        contact.vn           = relVel;
        contact.m_iState     = CollisionInfo::TOUCHING;
        vContacts.push_back(contact);
      }//end if(dist <= distpertime)
    }
    else if(relVel < 0.00001 && dist < 0.005)
    {

      //std::cout<<"Pre-contact normal velocity: "<<relVel<<" resting contact"<<std::endl;
      Contact contact;
	  contact.m_dDistance = distcenter;
      contact.m_vNormal    = pBoundary->normals_[k];
      //pos = center - (dist*Normal)
	  VECTOR3 pos = trans - (dist*pBoundary->normals_[k]);
      contact.m_vPosition0 = pos;
      contact.m_vPosition1 = pos;
      contact.m_pBody0     = body0_;
      contact.m_pBody1     = body1_;
      contact.id0 = contact.m_pBody0->iID_;
      contact.id1 = contact.m_pBody1->iID_;
      contact.vn           = relVel;
      contact.m_iState     = CollisionInfo::TOUCHING;
      vContacts.push_back(contact);
    }
    else if(dist < 0.1*rad1)
    {
      Contact contact;
	  contact.m_dDistance = distcenter;
      contact.m_vNormal    = pBoundary->normals_[k];
      //pos = center - (dist*Normal)
	  VECTOR3 pos = trans - (dist*pBoundary->normals_[k]);
      contact.m_vPosition0 = pos;
      contact.m_vPosition1 = pos;
      contact.m_pBody0     = body0_;
      contact.m_pBody1     = body1_;
      contact.id0 = contact.m_pBody0->iID_;
      contact.id1 = contact.m_pBody1->iID_;
      contact.vn           = relVel;
      contact.m_iState     = CollisionInfo::TOUCHING;
      vContacts.push_back(contact);
    }
    else
    {
      //std::cout<<"Pre-contact normal velocity: "<<relVel<<" resting contact"<<std::endl;
      continue;
      Contact contact;
      contact.m_dDistance  = dist;
      contact.m_vNormal    = pBoundary->normals_[k];
      //pos = center - (dist*Normal)
	  VECTOR3 pos = trans - (dist*pBoundary->normals_[k]);
      contact.m_vPosition0 = pos;
      contact.m_vPosition1 = pos;
      contact.m_pBody0     = body0_;
      contact.m_pBody1     = body1_;
      contact.id0 = contact.m_pBody0->iID_;
      contact.id1 = contact.m_pBody1->iID_;
      contact.vn           = relVel;
      contact.m_iState     = CollisionInfo::VANISHING_CLOSEPROXIMITY;
      vContacts.push_back(contact);      
    }
       
  }//end for all walls
 
}

// void CColliderSphereBoxBoundary::Collide(std::vector<CContact> &vContacts)
// {
//  
//   //calculate the distance
//   Spherer *sphere         = dynamic_cast<Spherer *>(m_pBody0->m_pShape);
//   BoundaryBoxr *pBoundary = dynamic_cast<BoundaryBoxr *>(m_pBody1->m_pShape);
//   Real rad1 = sphere->Radius();
//   VECTOR3 boundaryAxis(0,0,-1);
//   Real boundaryRad    = 7.0;
//   Real regionBigCylinder = -13.0 - rad1;
//   Real coneHeight     = 18.2;
//   Real coneBaseZ      = -13.0;
//   Real endConeZ       = 0.0;
//   Real apexZ          = 5.2;
//   Real endCylSmall    = 60;
//   VECTOR3 vApex(0,0,apexZ);
//   
//   VECTOR3 sphereCenter = m_pBody0->m_vCOM;
// 
//   //rotated by degree around x
//   CCylinderr cylinder = CCylinderr(VECTOR3(0,0,0),VECTOR3(0,0,1),1.0,5.0);
//   
//   Quaternionr q;
//   Quaternionr p;
//   q.CreateFromEulerAngles(0,0,-0.785398163);
//   p.CreateFromEulerAngles(0,0,0.785398163);  
//   //q.CreateFromEulerAngles(0,0,-1.570796327);
//   //p.CreateFromEulerAngles(0,0,1.570796327);  
//   
//   MATRIX3X3 mat  = q.GetMatrix();
//   MATRIX3X3 modelWorld = p.GetMatrix();    
//   VECTOR3 sTrans = sphereCenter - VECTOR3(0,9,-18);
//   sTrans         = mat * sTrans;
// 
//   Real dist = 0.0;    
//   if(cylinder.PointInside(sTrans))
//   {    
//     dist = sqrtf((sTrans.x*sTrans.x) + (sTrans.y*sTrans.y));    
//     dist = fabs(1.0-(dist+rad1));        
//     VECTOR3 vNormal = sTrans - VECTOR3(0,0,sTrans.z);
//     vNormal.Normalize();
//     VECTOR3 vContact = VECTOR3(0,0,sTrans.z) + dist * vNormal;
//     vNormal = -vNormal;
//     vNormal = modelWorld * vNormal;
//     Real relVel = ((m_pBody0->m_vVelocity+m_pWorld->GetGravityEffect(m_pBody0)*m_pWorld->m_pTimeControl->GetDeltaT()) * vNormal);
//     if(relVel < 0.0)
//     {
//       Real distpertime = -relVel*m_pWorld->m_pTimeControl->GetDeltaT();
//       //check whether there will be a collision next time step
//       if(dist <= distpertime || dist < 0.1*rad1)
//       {
//         CContact contact;
//         contact.m_vNormal= vNormal;
//         contact.m_vPosition0 = (modelWorld * vContact) + VECTOR3(0,9,-18);
//         contact.m_dDistance  = dist;
//         contact.m_vPosition1 = contact.m_vPosition0;
//         contact.m_pBody0     = m_pBody0;
//         contact.m_pBody1     = m_pBody1;
//         contact.id0 = contact.m_pBody0->m_iID;
//         contact.id1 = contact.m_pBody1->m_iID;
//         contact.m_iState     = CCollisionInfo::TOUCHING;
//         vContacts.push_back(contact);
//       }
//     }
//     return;
//   }      
//   // determine the region of the particle
//   if(sphereCenter.z < regionBigCylinder)
//   {
//     //cylindrical region
//     dist = sqrtf((sphereCenter.x*sphereCenter.x) + (sphereCenter.y*sphereCenter.y));
//     dist = fabs(boundaryRad-(dist+rad1));
//     VECTOR3 vNormal = sphereCenter - VECTOR3(0,0,sphereCenter.z);
//     vNormal.Normalize();
//     VECTOR3 vContact = VECTOR3(0,0,sphereCenter.z) + dist * vNormal;
//     vNormal = -vNormal;
//     Real relVel = ((m_pBody0->m_vVelocity+m_pWorld->GetGravityEffect(m_pBody0)*m_pWorld->m_pTimeControl->GetDeltaT()) * vNormal);
//     //if the bodies are on collision course
//     if(relVel < 0.0)
//     {
//       Real distpertime = -relVel*m_pWorld->m_pTimeControl->GetDeltaT();
//       //check whether there will be a collision next time step
//       if(dist <= distpertime || dist < 0.1*rad1)
//       {
//         CContact contact;
//         contact.m_vNormal= vNormal;
//         contact.m_vPosition0 = vContact;
//         contact.m_dDistance  = dist;
//         contact.m_vPosition1 = contact.m_vPosition0;
//         contact.m_pBody0     = m_pBody0;
//         contact.m_pBody1     = m_pBody1;
//         contact.id0 = contact.m_pBody0->m_iID;
//         contact.id1 = contact.m_pBody1->m_iID;
//         contact.m_iState     = CCollisionInfo::TOUCHING;
//         vContacts.push_back(contact);
//       }
//     }
//   }
//   else
//   {
//     // conical region
//     if((sphereCenter.z >= coneBaseZ)&&(sphereCenter.z <= endConeZ))
//     {
//       //If the sphere center is inside the cone
//       //Get the radius at height sphereCenter.z
//       //calculate height for sphere
//       Real h1   = fabs(sphereCenter.z-vApex.z);
//       Real radH =  h1*(boundaryRad/coneHeight);
//       // project the sphere center onto the circle
//       VECTOR3 b0 = sphereCenter - VECTOR3(0,0,sphereCenter.z);
//       b0.Normalize();
//       VECTOR3 c0 = VECTOR3(0,0,sphereCenter.z) + radH * b0;
//       VECTOR3 vDir = vApex - c0;
//       vDir.Normalize();
//       VECTOR3 d0 = sphereCenter-c0;
//       Real alpha = d0 * vDir;
//       VECTOR3 vContact = c0 + (alpha * vDir);
//       VECTOR3 vNormal  = sphereCenter - vContact;
//       dist = vNormal.mag()-rad1;
//       vNormal.Normalize();
//       Real relVel = ((m_pBody0->m_vVelocity+m_pWorld->GetGravityEffect(m_pBody0)*m_pWorld->m_pTimeControl->GetDeltaT()) * vNormal);
//       //if the bodies are on collision course
//       if(relVel < 0.0)
//       {
//         Real distpertime = -relVel*m_pWorld->m_pTimeControl->GetDeltaT();
//         //check whether there will be a collision next time step
//         if(dist <= distpertime || dist < 0.1*rad1)
//         {
//           CContact contact;          
//           contact.m_vNormal    = vNormal;
//           contact.m_vPosition0 = vContact;
//           contact.m_dDistance  = dist;
//           contact.m_vPosition1 = contact.m_vPosition0;
//           contact.m_pBody0     = m_pBody0;
//           contact.m_pBody1     = m_pBody1;
//           contact.id0          = contact.m_pBody0->m_iID;
//           contact.id1          = contact.m_pBody1->m_iID;
//           contact.m_iState     = CCollisionInfo::TOUCHING;
//           vContacts.push_back(contact);
//         }
//       }      
//     }
//     else if(sphereCenter.z < coneBaseZ)
//     {
//       //If the sphere center is inside the cone
//       // Get the radius at height sphereCenter.z
//       // calculate height for sphere
//       Real h1   = fabs(sphereCenter.z-vApex.z);
//       Real radH =  h1*(boundaryRad/coneHeight);
//       // project the sphere center onto the circle
//       VECTOR3 b0 = sphereCenter - VECTOR3(0,0,sphereCenter.z);
//       b0.Normalize();
//       VECTOR3 c0 = VECTOR3(0,0,sphereCenter.z) + radH * b0;
//       VECTOR3 vDir = vApex - c0;
//       vDir.Normalize();
//       VECTOR3 d0 = sphereCenter-c0;
//       Real alpha = d0 * vDir;
//       VECTOR3 vContact = c0 + (alpha * vDir);
//       VECTOR3 vNormal  = sphereCenter - vContact;
//       dist = vNormal.mag()-rad1;
//       vNormal.Normalize();
//       Real relVel = ((m_pBody0->m_vVelocity+m_pWorld->GetGravityEffect(m_pBody0)*m_pWorld->m_pTimeControl->GetDeltaT()) * vNormal);
//       //if the bodies are on collision course
//       if(relVel < 0.0)
//       {
//         Real distpertime = -relVel*m_pWorld->m_pTimeControl->GetDeltaT();
//         //check whether there will be a collision next time step
//         if(dist <= distpertime || dist < 0.1*rad1)
//         {
//           CContact contact;          
//           contact.m_vNormal    = vNormal;
//           contact.m_vPosition0 = vContact;
//           contact.m_dDistance  = dist;
//           contact.m_vPosition1 = contact.m_vPosition0;
//           contact.m_pBody0     = m_pBody0;
//           contact.m_pBody1     = m_pBody1;
//           contact.id0          = contact.m_pBody0->m_iID;
//           contact.id1          = contact.m_pBody1->m_iID;
//           contact.m_iState     = CCollisionInfo::TOUCHING;
//           vContacts.push_back(contact);
//         }
//       }
//       //------------------------------------------------------------------------------
//       dist = sqrtf((sphereCenter.x*sphereCenter.x) + (sphereCenter.y*sphereCenter.y));
//       dist = fabs(boundaryRad-(dist+rad1));
//       vNormal = sphereCenter - VECTOR3(0,0,sphereCenter.z);
//       vNormal.Normalize();
//       vContact = VECTOR3(0,0,sphereCenter.z) + dist * vNormal;
//       vNormal = -vNormal;
//       relVel = ((m_pBody0->m_vVelocity+m_pWorld->GetGravityEffect(m_pBody0)*m_pWorld->m_pTimeControl->GetDeltaT()) * vNormal);
//       //if the bodies are on collision course
//       if(relVel < 0.0)
//       {
//         Real distpertime = -relVel*m_pWorld->m_pTimeControl->GetDeltaT();
//         //check whether there will be a collision next time step
//         if(dist <= distpertime)
//         {
//           CContact contact;
//           contact.m_vNormal= vNormal;
//           contact.m_vPosition0 = vContact;
//           contact.m_dDistance  = dist;
//           contact.m_vPosition1 = contact.m_vPosition0;
//           contact.m_pBody0     = m_pBody0;
//           contact.m_pBody1     = m_pBody1;
//           contact.id0 = contact.m_pBody0->m_iID;
//           contact.id1 = contact.m_pBody1->m_iID;
//           contact.m_iState     = CCollisionInfo::TOUCHING;
//           vContacts.push_back(contact);
//         }
//       }
//       //------------------------------------------------------------------------------      
//     }
//     else if((sphereCenter.z > endConeZ)&&(sphereCenter.z < endCylSmall))
//     {
//       //cylindrical region
//       dist = sqrtf((sphereCenter.x*sphereCenter.x) + (sphereCenter.y*sphereCenter.y));
//       dist = fabs(2.0-(dist+rad1));
//       VECTOR3 vNormal = sphereCenter - VECTOR3(0,0,sphereCenter.z);
//       vNormal.Normalize();
//       VECTOR3 vContact = VECTOR3(0,0,sphereCenter.z) + dist * vNormal;
//       vNormal = -vNormal;
//       Real relVel = ((m_pBody0->m_vVelocity+m_pWorld->GetGravityEffect(m_pBody0)*m_pWorld->m_pTimeControl->GetDeltaT()) * vNormal);
//       //if the bodies are on collision course
//       if(relVel < 0.0)
//       {
//         Real distpertime = -relVel*m_pWorld->m_pTimeControl->GetDeltaT();
//         //check whether there will be a collision next time step
//         if(dist <= distpertime || dist < 0.1*rad1)
//         {
//           CContact contact;
//           contact.m_vNormal= vNormal;
//           contact.m_vPosition0 = vContact;
//           contact.m_dDistance  = dist;
//           contact.m_vPosition1 = contact.m_vPosition0;
//           contact.m_pBody0     = m_pBody0;
//           contact.m_pBody1     = m_pBody1;
//           contact.id0 = contact.m_pBody0->m_iID;
//           contact.id1 = contact.m_pBody1->m_iID;
//           contact.m_iState     = CCollisionInfo::TOUCHING;
//           vContacts.push_back(contact);
//         }
//       }      
//     }
//   }
// }

}
