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
#include "collisioninfo.h"

namespace i3d {

ColliderCylinderBoundaryBox::ColliderCylinderBoundaryBox() 
{

}

ColliderCylinderBoundaryBox::~ColliderCylinderBoundaryBox() 
{

}

void ColliderCylinderBoundaryBox::collide(std::vector<Contact> &vContacts)
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
  BoundaryBoxr *pBoundary = dynamic_cast<BoundaryBoxr *>(body1_->shape_);
  
  //now check for all walls
  for(k=0;k<=4;k+=2)
  {
    //calculate the distance
    int indexOrigin = k/2;

    VECTOR3 vClosestPoint =pCylinder->getSupport(-pBoundary->normals_[k]);
    int iContacts = 0;
    Real dist = vClosestPoint.m_dCoords[indexOrigin] - pBoundary->extents_[k];
    if(fabs(dist) < COLLIDINGTOLERANCE)
    {
      //there are three cases the cylinder is parallel
      //to the plane normal, perpendicular or
      //none of the above
      Real dotUN =  pCylinder->getU() * pBoundary->normals_[k];
      if(fabs(dotUN) > parallelTolerance)
      {
        //4 contact points
        //projectpointonplane
        iContacts = 4;
        VECTOR3 center = pCylinder->getCenter();
        center -=Real(pCylinder->getHalfLength()+dist/2.0)*pBoundary->normals_[k];
        VECTOR3 arr[4];
        arr[0] = center + pCylinder->getRadius() * directions[indexOrigin][0];
        arr[1] = center - pCylinder->getRadius() * directions[indexOrigin][0];
        arr[2] = center + pCylinder->getRadius() * directions[indexOrigin][1];
        arr[3] = center - pCylinder->getRadius() * directions[indexOrigin][1];              

        for(int j=0;j<4;j++)
        {

          VECTOR3 angPart = (VECTOR3::Cross(body0_->getAngVel(),arr[j]-body0_->com_));
          VECTOR3 relativeVelocity = (body0_->velocity_ + angPart);

          //relative velocity along the normal
          Real normalVelocity = relativeVelocity * pBoundary->normals_[k];

          if(fabs(dist) < COLLIDINGTOLERANCE)
          { 
            Contact contact;
            contact.m_dDistance  = dist;
            contact.m_vNormal    = pBoundary->normals_[k];
            contact.m_vPosition0 = arr[j];
            contact.m_vPosition1 = arr[j];
            contact.m_pBody0     = body0_;
            contact.m_pBody1     = body1_;
            contact.id0          = contact.m_pBody0->iID_;
            contact.id1          = contact.m_pBody1->iID_;
            contact.vn           = normalVelocity;
            contact.m_iState     = CollisionInfo::TOUCHING;
            vContacts.push_back(contact);
          }
        }//end for
      }
      else if(fabs(dotUN) < perpendicularTolerance)   
      {
       //2 contact points
       VECTOR3 center = pCylinder->getCenter();
       center -=Real((pCylinder->getRadius()+dist/2.0))*pBoundary->normals_[k];
       VECTOR3 arr[2];
       arr[0] = center + pCylinder->getHalfLength() * pCylinder->getU();
       arr[1] = center - pCylinder->getHalfLength() * pCylinder->getU();
       for(int j=0;j<2;j++)
       {
         
          VECTOR3 angPart = (VECTOR3::Cross(body0_->getAngVel(),arr[j]-body0_->com_));
          VECTOR3 relativeVelocity = (body0_->velocity_ + angPart);

          //relative velocity along the normal
          Real normalVelocity = relativeVelocity * pBoundary->normals_[k];

          if(fabs(dist) < COLLIDINGTOLERANCE)
          { 
            Contact contact;
            contact.m_dDistance  = dist;
            contact.m_vNormal    = pBoundary->normals_[k];
            contact.m_vPosition0 = arr[j];
            contact.m_vPosition1 = arr[j];
            contact.m_pBody0     = body0_;
            contact.m_pBody1     = body1_;
            contact.id0          = contact.m_pBody0->iID_;
            contact.id1          = contact.m_pBody1->iID_;
            contact.vn           = normalVelocity;
            contact.m_iState     = CollisionInfo::TOUCHING;
            vContacts.push_back(contact);
          }
       }      
      }
      else
      {
        vClosestPoint-=Real((dist/2.0))*pBoundary->normals_[k];
        VECTOR3 angPart = (VECTOR3::Cross(body0_->getAngVel(),vClosestPoint-body0_->com_));
        VECTOR3 relativeVelocity = (body0_->velocity_ + angPart);

        //relative velocity along the normal
        Real normalVelocity = relativeVelocity * pBoundary->normals_[k];
        
        if(normalVelocity < -0.005)
        { 
          Contact contact;
          contact.m_dDistance  = dist;
          contact.m_vNormal    = pBoundary->normals_[k];
          contact.m_vPosition0 = vClosestPoint;
          contact.m_vPosition1 = vClosestPoint;
          contact.m_pBody0     = body0_;
          contact.m_pBody1     = body1_;
          contact.vn           = normalVelocity;
          contact.m_iState     = CollisionInfo::TOUCHING;                      
          vContacts.push_back(contact);
        }
        else if(normalVelocity < 0.00001)
        { 
          Contact contact;
          contact.m_dDistance  = dist;
          contact.m_vNormal    = pBoundary->normals_[k];
          contact.m_vPosition0 = vClosestPoint;
          contact.m_vPosition1 = vClosestPoint;
          contact.m_pBody0     = body0_;
          contact.m_pBody1     = body1_;
          contact.id0 = contact.m_pBody0->iID_;
          contact.id1 = contact.m_pBody1->iID_;
          contact.vn           = normalVelocity;
          contact.m_iState     = CollisionInfo::TOUCHING;                                
          vContacts.push_back(contact);
        }
        else
        {
          return;
          Contact contact;
          contact.m_dDistance  = dist;
          contact.m_vNormal    = pBoundary->normals_[k];
          contact.m_vPosition0 = vClosestPoint;
          contact.m_vPosition1 = vClosestPoint;
          contact.m_pBody0     = body0_;
          contact.m_pBody1     = body1_;
          contact.id0 = contact.m_pBody0->iID_;
          contact.id1 = contact.m_pBody1->iID_;
          contact.vn           = normalVelocity;
          contact.m_iState     = CollisionInfo::VANISHING_CLOSEPROXIMITY;                                
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

    VECTOR3 vClosestPoint =pCylinder->getSupport(-pBoundary->normals_[k]);
    int iContacts = 0;
    Real dist = pBoundary->extents_[k] - vClosestPoint.m_dCoords[indexOrigin];
    if(fabs(dist) < COLLIDINGTOLERANCE)
    {
      //there are three cases the cylinder is parallel
      //to the plane normal, perpendicular or
      //none of the above
      Real dotUN =  pCylinder->getU() * pBoundary->normals_[k];
      if(fabs(dotUN) > parallelTolerance)
      {
        //4 contact points
        //projectpointonplane
        iContacts = 4;
        VECTOR3 center = pCylinder->getCenter();
        center -=Real((pCylinder->getHalfLength()+dist/2.0))*pBoundary->normals_[k];
        VECTOR3 arr[4];
        arr[0] = center + pCylinder->getRadius() * directions[indexOrigin][0];
        arr[1] = center - pCylinder->getRadius() * directions[indexOrigin][0];
        arr[2] = center + pCylinder->getRadius() * directions[indexOrigin][1];
        arr[3] = center - pCylinder->getRadius() * directions[indexOrigin][1];              

        for(int j=0;j<4;j++)
        {

          VECTOR3 angPart = (VECTOR3::Cross(body0_->getAngVel(),arr[j]-body0_->com_));
          VECTOR3 relativeVelocity = (body0_->velocity_ + angPart);

          //relative velocity along the normal
          Real normalVelocity = relativeVelocity * pBoundary->normals_[k];

          if(normalVelocity < -0.005)
          { 
            Contact contact;
            contact.m_dDistance  = dist;
            contact.m_vNormal    = pBoundary->normals_[k];
            contact.m_vPosition0 = arr[j];
            contact.m_vPosition1 = arr[j];
            contact.m_pBody0     = body0_;
            contact.m_pBody1     = body1_;
            contact.id0          = contact.m_pBody0->iID_;
            contact.id1          = contact.m_pBody1->iID_;
            contact.vn           = normalVelocity;
            contact.m_iState     = CollisionInfo::COLLIDING;
            vContacts.push_back(contact);
          }
          else if(normalVelocity < 0.00001)
          { 
            Contact contact;
            contact.m_dDistance  = dist;
            contact.m_vNormal    = pBoundary->normals_[k];
            contact.m_vPosition0 = arr[j];
            contact.m_vPosition1 = arr[j];
            contact.m_pBody0     = body0_;
            contact.m_pBody1     = body1_;
            contact.id0          = contact.m_pBody0->iID_;
            contact.id1          = contact.m_pBody1->iID_;
            contact.vn           = normalVelocity;
            contact.m_iState     = CollisionInfo::TOUCHING;
            vContacts.push_back(contact);
          }
          else
          {
            Contact contact;
            contact.m_dDistance  = dist;
            contact.m_vNormal    = pBoundary->normals_[k];
            contact.m_vPosition0 = arr[j];
            contact.m_vPosition1 = arr[j];
            contact.m_pBody0     = body0_;
            contact.m_pBody1     = body1_;
            contact.id0          = contact.m_pBody0->iID_;
            contact.id1          = contact.m_pBody1->iID_;
            contact.vn           = normalVelocity;
            contact.m_iState     = CollisionInfo::VANISHING_CLOSEPROXIMITY;
            vContacts.push_back(contact);            
          }
        }//end for
      }
      else if(fabs(dotUN) < perpendicularTolerance)   
      {
       //2 contact points
       VECTOR3 center = pCylinder->getCenter();
       center -=Real((pCylinder->getRadius()+dist/2.0))*pBoundary->normals_[k];
       VECTOR3 arr[2];
       arr[0] = center + pCylinder->getHalfLength() * pCylinder->getU();
       arr[1] = center - pCylinder->getHalfLength() * pCylinder->getU();
       for(int j=0;j<2;j++)
       {
         
          VECTOR3 angPart = (VECTOR3::Cross(body0_->getAngVel(),arr[j]-body0_->com_));
          VECTOR3 relativeVelocity = (body0_->velocity_ + angPart);

          //relative velocity along the normal
          Real normalVelocity = relativeVelocity * pBoundary->normals_[k];

          if(normalVelocity < -0.005)
          { 
            Contact contact;
            contact.m_dDistance  = dist;
            contact.m_vNormal    = pBoundary->normals_[k];
            contact.m_vPosition0 = arr[j];
            contact.m_vPosition1 = arr[j];
            contact.m_pBody0     = body0_;
            contact.m_pBody1     = body1_;
            contact.id0          = contact.m_pBody0->iID_;
            contact.id1          = contact.m_pBody1->iID_;
            contact.vn           = normalVelocity;
            contact.m_iState     = CollisionInfo::TOUCHING;
            vContacts.push_back(contact);
          }
          else if(normalVelocity < 0.00001)
          { 
            Contact contact;
            contact.m_dDistance  = dist;
            contact.m_vNormal    = pBoundary->normals_[k];
            contact.m_vPosition0 = arr[j];
            contact.m_vPosition1 = arr[j];
            contact.m_pBody0     = body0_;
            contact.m_pBody1     = body1_;
            contact.id0          = contact.m_pBody0->iID_;
            contact.id1          = contact.m_pBody1->iID_;
            contact.vn           = normalVelocity;
            contact.m_iState     = CollisionInfo::TOUCHING;
            vContacts.push_back(contact);
          }
          else
          {
            return;
            Contact contact;
            contact.m_dDistance  = dist;
            contact.m_vNormal    = pBoundary->normals_[k];
            contact.m_vPosition0 = arr[j];
            contact.m_vPosition1 = arr[j];
            contact.m_pBody0     = body0_;
            contact.m_pBody1     = body1_;
            contact.id0          = contact.m_pBody0->iID_;
            contact.id1          = contact.m_pBody1->iID_;
            contact.vn           = normalVelocity;
            contact.m_iState     = CollisionInfo::VANISHING_CLOSEPROXIMITY;
            vContacts.push_back(contact);            
          }
       }      
      }
      else
      {
        vClosestPoint-=Real((dist/2.0))*pBoundary->normals_[k];
        VECTOR3 angPart = (VECTOR3::Cross(body0_->getAngVel(),vClosestPoint-body0_->com_));
        VECTOR3 relativeVelocity = (body0_->velocity_ + angPart);

        //relative velocity along the normal
        Real normalVelocity = relativeVelocity * pBoundary->normals_[k];
        
        if(normalVelocity < -0.005)
        { 
          Contact contact;
          contact.m_dDistance  = dist;
          contact.m_vNormal    = pBoundary->normals_[k];
          contact.m_vPosition0 = vClosestPoint;
          contact.m_vPosition1 = vClosestPoint;
          contact.m_pBody0     = body0_;
          contact.m_pBody1     = body1_;
          contact.vn           = normalVelocity;
          contact.m_iState     = CollisionInfo::TOUCHING;                      
          vContacts.push_back(contact);
        }
        else if(normalVelocity < 0.00001)
        { 
          Contact contact;
          contact.m_dDistance  = dist;
          contact.m_vNormal    = pBoundary->normals_[k];
          contact.m_vPosition0 = vClosestPoint;
          contact.m_vPosition1 = vClosestPoint;
          contact.m_pBody0     = body0_;
          contact.m_pBody1     = body1_;
          contact.vn           = normalVelocity;
          contact.m_iState     = CollisionInfo::TOUCHING;                      
          vContacts.push_back(contact);
        }
        else
        {
          return;
          Contact contact;
          contact.m_dDistance  = dist;
          contact.m_vNormal    = pBoundary->normals_[k];
          contact.m_vPosition0 = vClosestPoint;
          contact.m_vPosition1 = vClosestPoint;
          contact.m_pBody0     = body0_;
          contact.m_pBody1     = body1_;
          contact.vn           = normalVelocity;
          contact.m_iState     = CollisionInfo::TOUCHING;                      
          vContacts.push_back(contact);          
        }
      }
    }
  }//end for all walls  
  
  delete pCylinder;
}

}
