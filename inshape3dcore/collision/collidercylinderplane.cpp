/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) <2011>  <Raphael Muenster>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with this program; if not, write to the Free Software Foundation, Inc.,
    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
*/ 


//===================================================
//                     INCLUDES
//===================================================
#include "collidercylinderboundarybox.h"
#include <cylinder.h>
#include <boundarybox.h>

namespace i3d {

ColliderCylinderBoundaryBox::CColliderCylinderBoundaryBox() 
{

}

ColliderCylinderBoundaryBox::~CColliderCylinderBoundaryBox() 
{

}

void ColliderCylinderBoundaryBox::Collide(RigidBody *pBody0, RigidBody *pBody1, std::vector<Contact> &vContacts, Real dDeltaT)
{

}

void ColliderCylinderBoundaryBox::Collide(std::vector<Contact> &vContacts, Real dDeltaT)
{
  int k;
  VECTOR3 directions[][2]={{VECTOR3(0,1,0),VECTOR3(0,0,1)},{VECTOR3(1,0,0),VECTOR3(0,0,1)},{VECTOR3(1,0,0),VECTOR3(0,1,0)}};    
  VECTOR3 invNorm[] ={VECTOR3(0,1,1),VECTOR3(1,0,1),VECTOR3(1,1,0)};
  const Real parallelTolerance = Real(1.0)-0.0002;
  const Real perpendicularTolerance = 0.005;
  const Real PROXIMITYCOLLISION = 0.01; 
  const Real COLLIDINGTOLERANCE = 0.01;

  Cylinderr *cylinder     = dynamic_cast<Cylinderr *>(body0_->shape_);
  Cylinderr *pCylinder    = dynamic_cast<Cylinderr *>(body0_->getWorldTransformedShape());
  CBoundaryBoxr *pBoundary = dynamic_cast<CBoundaryBoxr *>(body1_->shape_);
  
  //now check for all walls
  for(k=0;k<=4;k+=2)
  {
    //calculate the distance
    int indexOrigin = k/2;

    VECTOR3 vClosestPoint =pCylinder->GetSupport(-pBoundary->m_vNormals[k]);
    int iContacts = 0;
    Real dist = vClosestPoint.m_dCoords[indexOrigin] - pBoundary->m_Values[k];
    if(fabs(dist) < COLLIDINGTOLERANCE)
    {
      //there are three cases the cylinder is parallel
      //to the plane normal, perpendicular or
      //none of the above
      Real dotUN =  pCylinder->GetU() * pBoundary->m_vNormals[k];
      if(fabs(dotUN) > parallelTolerance)
      {
        //4 contact points
        //projectpointonplane
        iContacts = 4;
        VECTOR3 center = pCylinder->GetCenter();
        center -=(pCylinder->GetHalfLength()+dist/2.0)*pBoundary->m_vNormals[k];
        VECTOR3 arr[4];
        arr[0] = center + pCylinder->GetRadius() * directions[indexOrigin][0];
        arr[1] = center - pCylinder->GetRadius() * directions[indexOrigin][0];
        arr[2] = center + pCylinder->GetRadius() * directions[indexOrigin][1];
        arr[3] = center - pCylinder->GetRadius() * directions[indexOrigin][1];              

        for(int j=0;j<4;j++)
        {

          VECTOR3 angPart = (VECTOR3::Cross(body0_->getAngVel(),arr[j]-body0_->com_));
          VECTOR3 relativeVelocity = (body0_->velocity_ + angPart);

          //relative velocity along the normal
          Real normalVelocity = relativeVelocity * pBoundary->m_vNormals[k];

          if(normalVelocity < -0.005)
          { 
            Contact contact;
            contact.m_dDistance  = dist;
            contact.m_vNormal    = pBoundary->m_vNormals[k];
            contact.m_vPosition0 = arr[j];
            contact.m_vPosition1 = arr[j];
            contact.m_pBody0     = body0_;
            contact.m_pBody1     = body1_;
            contact.vn           = normalVelocity;
            vContacts.push_back(contact);
          }
          else if(normalVelocity < 0.00001)
          { 
            Contact contact;
            contact.m_dDistance  = dist;
            contact.m_vNormal    = pBoundary->m_vNormals[k];
            contact.m_vPosition0 = arr[j];
            contact.m_vPosition1 = arr[j];
            contact.m_pBody0     = body0_;
            contact.m_pBody1     = body1_;
            contact.vn           = normalVelocity;
            vContacts.push_back(contact);
          }
        }//end for
      }
      else if(fabs(dotUN) < perpendicularTolerance)   
      {
       //2 contact points
       VECTOR3 center = pCylinder->GetCenter();
       center -=(pCylinder->GetRadius()+dist/2.0)*pBoundary->m_vNormals[k];
       VECTOR3 arr[2];
       arr[0] = center + pCylinder->GetHalfLength() * pCylinder->GetU();
       arr[1] = center - pCylinder->GetHalfLength() * pCylinder->GetU();
       for(int j=0;j<2;j++)
       {
         
          VECTOR3 angPart = (VECTOR3::Cross(body0_->getAngVel(),arr[j]-body0_->com_));
          VECTOR3 relativeVelocity = (body0_->velocity_ + angPart);

          //relative velocity along the normal
          Real normalVelocity = relativeVelocity * pBoundary->m_vNormals[k];

          if(normalVelocity < -0.005)
          { 
            Contact contact;
            contact.m_dDistance  = dist;
            contact.m_vNormal    = pBoundary->m_vNormals[k];
            contact.m_vPosition0 = arr[j];
            contact.m_vPosition1 = arr[j];
            contact.m_pBody0     = body0_;
            contact.m_pBody1     = body1_;
            contact.vn           = normalVelocity;
            vContacts.push_back(contact);
          }
          else if(normalVelocity < 0.00001)
          { 
            Contact contact;
            contact.m_dDistance  = dist;
            contact.m_vNormal    = pBoundary->m_vNormals[k];
            contact.m_vPosition0 = arr[j];
            contact.m_vPosition1 = arr[j];
            contact.m_pBody0     = body0_;
            contact.m_pBody1     = body1_;
            contact.vn           = normalVelocity;
            vContacts.push_back(contact);
          }
       }      
      }
      else
      {
        vClosestPoint-=(dist/2.0)*pBoundary->m_vNormals[k];
        VECTOR3 angPart = (VECTOR3::Cross(body0_->getAngVel(),vClosestPoint-body0_->com_));
        VECTOR3 relativeVelocity = (body0_->velocity_ + angPart);

        //relative velocity along the normal
        Real normalVelocity = relativeVelocity * pBoundary->m_vNormals[k];
        
        if(normalVelocity < -0.005)
        { 
          Contact contact;
          contact.m_dDistance  = dist;
          contact.m_vNormal    = pBoundary->m_vNormals[k];
          contact.m_vPosition0 = vClosestPoint;
          contact.m_vPosition1 = vClosestPoint;
          contact.m_pBody0     = body0_;
          contact.m_pBody1     = body1_;
          contact.vn           = normalVelocity;
          vContacts.push_back(contact);
        }
        else if(normalVelocity < 0.00001)
        { 
          Contact contact;
          contact.m_dDistance  = dist;
          contact.m_vNormal    = pBoundary->m_vNormals[k];
          contact.m_vPosition0 = vClosestPoint;
          contact.m_vPosition1 = vClosestPoint;
          contact.m_pBody0     = body0_;
          contact.m_pBody1     = body1_;
          contact.vn           = normalVelocity;
          vContacts.push_back(contact);
        }
      }
    }
  }//end for all walls
  
  //now check for all walls
  for(k=1;k<=5;k+=2)
  {
    //calculate the distance
    int indexOrigin = k/2;

    VECTOR3 vClosestPoint =pCylinder->GetSupport(-pBoundary->m_vNormals[k]);
    int iContacts = 0;
    Real dist = pBoundary->m_Values[k] - vClosestPoint.m_dCoords[indexOrigin];
    if(fabs(dist) < COLLIDINGTOLERANCE)
    {
      //there are three cases the cylinder is parallel
      //to the plane normal, perpendicular or
      //none of the above
      Real dotUN =  pCylinder->GetU() * pBoundary->m_vNormals[k];
      if(fabs(dotUN) > parallelTolerance)
      {
        //4 contact points
        //projectpointonplane
        iContacts = 4;
        VECTOR3 center = pCylinder->GetCenter();
        center -=(pCylinder->GetHalfLength()+dist/2.0)*pBoundary->m_vNormals[k];
        VECTOR3 arr[4];
        arr[0] = center + pCylinder->GetRadius() * directions[indexOrigin][0];
        arr[1] = center - pCylinder->GetRadius() * directions[indexOrigin][0];
        arr[2] = center + pCylinder->GetRadius() * directions[indexOrigin][1];
        arr[3] = center - pCylinder->GetRadius() * directions[indexOrigin][1];              

        for(int j=0;j<4;j++)
        {

          VECTOR3 angPart = (VECTOR3::Cross(body0_->getAngVel(),arr[j]-body0_->com_));
          VECTOR3 relativeVelocity = (body0_->velocity_ + angPart);

          //relative velocity along the normal
          Real normalVelocity = relativeVelocity * pBoundary->m_vNormals[k];

          if(normalVelocity < -0.005)
          { 
            Contact contact;
            contact.m_dDistance  = dist;
            contact.m_vNormal    = pBoundary->m_vNormals[k];
            contact.m_vPosition0 = arr[j];
            contact.m_vPosition1 = arr[j];
            contact.m_pBody0     = body0_;
            contact.m_pBody1     = body1_;
            contact.vn           = normalVelocity;
            vContacts.push_back(contact);
          }
          else if(normalVelocity < 0.00001)
          { 
            Contact contact;
            contact.m_dDistance  = dist;
            contact.m_vNormal    = pBoundary->m_vNormals[k];
            contact.m_vPosition0 = arr[j];
            contact.m_vPosition1 = arr[j];
            contact.m_pBody0     = body0_;
            contact.m_pBody1     = body1_;
            contact.vn           = normalVelocity;
            vContacts.push_back(contact);
          }
        }//end for
      }
      else if(fabs(dotUN) < perpendicularTolerance)   
      {
       //2 contact points
       VECTOR3 center = pCylinder->GetCenter();
       center -=(pCylinder->GetRadius()+dist/2.0)*pBoundary->m_vNormals[k];
       VECTOR3 arr[2];
       arr[0] = center + pCylinder->GetHalfLength() * pCylinder->GetU();
       arr[1] = center - pCylinder->GetHalfLength() * pCylinder->GetU();
       for(int j=0;j<2;j++)
       {
         
          VECTOR3 angPart = (VECTOR3::Cross(body0_->getAngVel(),arr[j]-body0_->com_));
          VECTOR3 relativeVelocity = (body0_->velocity_ + angPart);

          //relative velocity along the normal
          Real normalVelocity = relativeVelocity * pBoundary->m_vNormals[k];

          if(normalVelocity < -0.005)
          { 
            Contact contact;
            contact.m_dDistance  = dist;
            contact.m_vNormal    = pBoundary->m_vNormals[k];
            contact.m_vPosition0 = arr[j];
            contact.m_vPosition1 = arr[j];
            contact.m_pBody0     = body0_;
            contact.m_pBody1     = body1_;
            contact.vn           = normalVelocity;
            vContacts.push_back(contact);
          }
          else if(normalVelocity < 0.00001)
          { 
            Contact contact;
            contact.m_dDistance  = dist;
            contact.m_vNormal    = pBoundary->m_vNormals[k];
            contact.m_vPosition0 = arr[j];
            contact.m_vPosition1 = arr[j];
            contact.m_pBody0     = body0_;
            contact.m_pBody1     = body1_;
            contact.vn           = normalVelocity;
            vContacts.push_back(contact);
          }
       }      
      }
      else
      {
        vClosestPoint-=(dist/2.0)*pBoundary->m_vNormals[k];
        VECTOR3 angPart = (VECTOR3::Cross(body0_->getAngVel(),vClosestPoint-body0_->com_));
        VECTOR3 relativeVelocity = (body0_->velocity_ + angPart);

        //relative velocity along the normal
        Real normalVelocity = relativeVelocity * pBoundary->m_vNormals[k];
        
        if(normalVelocity < -0.005)
        { 
          Contact contact;
          contact.m_dDistance  = dist;
          contact.m_vNormal    = pBoundary->m_vNormals[k];
          contact.m_vPosition0 = vClosestPoint;
          contact.m_vPosition1 = vClosestPoint;
          contact.m_pBody0     = body0_;
          contact.m_pBody1     = body1_;
          contact.vn           = normalVelocity;
          vContacts.push_back(contact);
        }
        else if(normalVelocity < 0.00001)
        { 
          Contact contact;
          contact.m_dDistance  = dist;
          contact.m_vNormal    = pBoundary->m_vNormals[k];
          contact.m_vPosition0 = vClosestPoint;
          contact.m_vPosition1 = vClosestPoint;
          contact.m_pBody0     = body0_;
          contact.m_pBody1     = body1_;
          contact.vn           = normalVelocity;
          vContacts.push_back(contact);
        }
      }
    }
  }//end for all walls  
  
  delete pCylinder;
}

}