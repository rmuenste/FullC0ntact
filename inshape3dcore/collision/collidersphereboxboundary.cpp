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

namespace i3d {

CColliderSphereBoxBoundary::CColliderSphereBoxBoundary(void)
{
}

CColliderSphereBoxBoundary::~CColliderSphereBoxBoundary(void)
{
}

void CColliderSphereBoxBoundary::Collide(CRigidBody *pBody0, CRigidBody *pBody1, std::vector<CContact> &vContacts, Real dDeltaT)
{

	//now check for all walls
	for(int k=0;k<6;k++)
	{
		//calculate the distance
		int indexOrigin = k/2;
		CSpherer *sphere         = dynamic_cast<CSpherer *>(pBody0->m_pShape);
		CBoundaryBoxr *pBoundary = dynamic_cast<CBoundaryBoxr *>(pBody1->m_pShape);
		Real rad1 = sphere->Radius();
		Real position = pBody0->m_vCOM.m_dCoords[indexOrigin];
		Real dist = fabs(pBoundary->m_Values[k] - position)-rad1;
		Real relVel = (pBody0->m_vVelocity * pBoundary->m_vNormals[k]);
		//if the bodies are on collision course
		if(relVel < 0.0)
		{
			Real distpertime = -relVel*dDeltaT;
			//check whether there will be a collision next time step
			if(dist <= distpertime)
			{
        //std::cout<<"Pre-contact normal velocity: "<<relVel<<" colliding contact"<<std::endl;

				CContact contact;
				contact.m_dDistance  = dist;
				contact.m_vNormal    = pBoundary->m_vNormals[k];
        //pos = center - (dist*Normal)
        VECTOR3 pos = m_pBody0->m_vCOM - (dist*pBoundary->m_vNormals[k]);
				contact.m_vPosition0 = pos;
				contact.m_vPosition1 = pos;
				contact.m_pBody0     = pBody0;
				contact.m_pBody1     = pBody1;
        contact.id0 = contact.m_pBody0->m_iID;
			  contact.id1 = contact.m_pBody1->m_iID;
				contact.vn           = relVel;
				vContacts.push_back(contact);
			}//end if(dist <= distpertime)
		}
      else if(relVel < 0.00001 && dist < 0.005)
      {
        //std::cout<<"Pre-contact normal velocity: "<<relVel<<" resting contact"<<std::endl;
        CContact contact;
        contact.m_dDistance  = dist;
        contact.m_vNormal    = pBoundary->m_vNormals[k];
        //pos = center - (dist*Normal)
        VECTOR3 pos = m_pBody0->m_vCOM - (dist*pBoundary->m_vNormals[k]);
        contact.m_vPosition0 = pos;
        contact.m_vPosition1 = pos;
        contact.m_pBody0     = pBody0;
        contact.m_pBody1     = pBody1;
        contact.id0 = contact.m_pBody0->m_iID;
			  contact.id1 = contact.m_pBody1->m_iID;
        contact.vn           = relVel;
        contact.m_pBody0->m_bTouchesGround = true;
        contact.m_pBody1->m_bTouchesGround = true;
        contact.m_bResting   = true;        
        vContacts.push_back(contact);
      }
	}//end for all walls
}

void CColliderSphereBoxBoundary::Collide(std::vector<CContact> &vContacts, Real dDeltaT)
{
  //calculate the distance
  CSpherer *sphere         = dynamic_cast<CSpherer *>(m_pBody0->m_pShape);
  CBoundaryBoxr *pBoundary = dynamic_cast<CBoundaryBoxr *>(m_pBody1->m_pShape);
  Real rad1 = sphere->Radius();
  VECTOR3 boundaryAxis(0,0,-1);
  Real boundaryRad    = 0.5f;
  Real regionCylinder = 2.5 + rad1;
  Real coneHeight     = 10.0/6.0;
  Real coneBaseZ      = 2.5;
  Real endConeZ       = 5.0/3.0;
  Real apexZ          = 5.0/6.0;
  VECTOR3 vApex(0,0,apexZ);
  
  VECTOR3 sphereCenter = m_pBody0->m_vCOM;

  Real dist = 0.0;  
  // determine the region of the particle
  if(sphereCenter.z > regionCylinder)
  {
    //cylindrical region
    dist = sqrtf((sphereCenter.x*sphereCenter.x) + (sphereCenter.y*sphereCenter.y));
    dist = fabs(boundaryRad-(dist+rad1));
    VECTOR3 vNormal = sphereCenter - VECTOR3(0,0,sphereCenter.z);
    vNormal.Normalize();
    VECTOR3 vContact = VECTOR3(0,0,sphereCenter.z) + dist * vNormal;
    vNormal = -vNormal;
    Real relVel = (m_pBody0->m_vVelocity * vNormal);
    //if the bodies are on collision course
    if(relVel < 0.0)
    {
      Real distpertime = -relVel*dDeltaT;
      //check whether there will be a collision next time step
      if(dist <= distpertime)
      {
        CContact contact;
        contact.m_vNormal= vNormal;
        contact.m_vPosition0 = vContact;
        contact.m_dDistance  = dist;
        contact.m_vPosition1 = contact.m_vPosition0;
        contact.m_pBody0     = m_pBody0;
        contact.m_pBody1     = m_pBody1;
        contact.id0 = contact.m_pBody0->m_iID;
        contact.id1 = contact.m_pBody1->m_iID;
        contact.m_bResting   = false;
        contact.m_iState     = CCollisionInfo::TOUCHING;
        vContacts.push_back(contact);
      }
    }
  }
  else
  {
    // conical region
    if((sphereCenter.z <= coneBaseZ)&&(sphereCenter.z >= endConeZ))
    {
      //If the sphere center is inside the cone
      // Get the radius at height sphereCenter.z
      // calculate height for sphere
      Real h1   = fabs(sphereCenter.z-vApex.z);
      Real radH =  h1*(boundaryRad/coneHeight);
      // project the sphere center onto the circle
      VECTOR3 b0 = sphereCenter - VECTOR3(0,0,sphereCenter.z);
      b0.Normalize();
      VECTOR3 c0 = VECTOR3(0,0,sphereCenter.z) + radH * b0;
      VECTOR3 vDir = vApex - c0;
      vDir.Normalize();
      VECTOR3 d0 = sphereCenter-c0;
      Real alpha = d0 * vDir;
      VECTOR3 vContact = c0 + (alpha * vDir);
      VECTOR3 vNormal  = sphereCenter - vContact;
      dist = vNormal.mag()-rad1;
      vNormal.Normalize();
      Real relVel = (m_pBody0->m_vVelocity * vNormal);
      //if the bodies are on collision course
      if(relVel < 0.0)
      {
        Real distpertime = -relVel*dDeltaT;
        //check whether there will be a collision next time step
        if(dist <= distpertime)
        {
          CContact contact;          
          contact.m_vNormal    = vNormal;
          contact.m_vPosition0 = vContact;
          contact.m_dDistance  = dist;
          contact.m_vPosition1 = contact.m_vPosition0;
          contact.m_pBody0     = m_pBody0;
          contact.m_pBody1     = m_pBody1;
          contact.id0          = contact.m_pBody0->m_iID;
          contact.id1          = contact.m_pBody1->m_iID;
          contact.m_bResting   = false;
          contact.m_iState     = CCollisionInfo::TOUCHING;
          vContacts.push_back(contact);
        }
      }      
    }
    else if(sphereCenter.z > coneBaseZ)
    {
      //If the sphere center is inside the cone
      // Get the radius at height sphereCenter.z
      // calculate height for sphere
      Real h1   = fabs(sphereCenter.z-vApex.z);
      Real radH =  h1*(boundaryRad/coneHeight);
      // project the sphere center onto the circle
      VECTOR3 b0 = sphereCenter - VECTOR3(0,0,sphereCenter.z);
      b0.Normalize();
      VECTOR3 c0 = VECTOR3(0,0,sphereCenter.z) + radH * b0;
      VECTOR3 vDir = vApex - c0;
      vDir.Normalize();
      VECTOR3 d0 = sphereCenter-c0;
      Real alpha = d0 * vDir;
      VECTOR3 vContact = c0 + (alpha * vDir);
      VECTOR3 vNormal  = sphereCenter - vContact;
      dist = vNormal.mag()-rad1;
      vNormal.Normalize();
      Real relVel = (m_pBody0->m_vVelocity * vNormal);
      //if the bodies are on collision course
      if(relVel < 0.0)
      {
        Real distpertime = -relVel*dDeltaT;
        //check whether there will be a collision next time step
        if(dist <= distpertime)
        {
          CContact contact;          
          contact.m_vNormal    = vNormal;
          contact.m_vPosition0 = vContact;
          contact.m_dDistance  = dist;
          contact.m_vPosition1 = contact.m_vPosition0;
          contact.m_pBody0     = m_pBody0;
          contact.m_pBody1     = m_pBody1;
          contact.id0          = contact.m_pBody0->m_iID;
          contact.id1          = contact.m_pBody1->m_iID;
          contact.m_bResting   = false;
          contact.m_iState     = CCollisionInfo::TOUCHING;
          vContacts.push_back(contact);
        }
      }
      //------------------------------------------------------------------------------
      dist = sqrtf((sphereCenter.x*sphereCenter.x) + (sphereCenter.y*sphereCenter.y));
      dist = fabs(boundaryRad-(dist+rad1));
      vNormal = sphereCenter - VECTOR3(0,0,sphereCenter.z);
      vNormal.Normalize();
      vContact = VECTOR3(0,0,sphereCenter.z) + dist * vNormal;
      vNormal = -vNormal;
      relVel = (m_pBody0->m_vVelocity * vNormal);
      //if the bodies are on collision course
      if(relVel < 0.0)
      {
        Real distpertime = -relVel*dDeltaT;
        //check whether there will be a collision next time step
        if(dist <= distpertime)
        {
          CContact contact;
          contact.m_vNormal= vNormal;
          contact.m_vPosition0 = vContact;
          contact.m_dDistance  = dist;
          contact.m_vPosition1 = contact.m_vPosition0;
          contact.m_pBody0     = m_pBody0;
          contact.m_pBody1     = m_pBody1;
          contact.id0 = contact.m_pBody0->m_iID;
          contact.id1 = contact.m_pBody1->m_iID;
          contact.m_bResting   = false;
          contact.m_iState     = CCollisionInfo::TOUCHING;
          vContacts.push_back(contact);
        }
      }
      //------------------------------------------------------------------------------      
    }
    else
    {
      //through the nozzle
    }
  }
}


/*void CColliderSphereBoxBoundary::Collide(std::vector<CContact> &vContacts, Real dDeltaT)
{

  CBoundaryBoxr *pBoundary = dynamic_cast<CBoundaryBoxr *>(m_pBody1->m_pShape);

  //now check for all walls
  for(int k=0;k<pBoundary->m_vBorders.size();k++)
  {
    //calculate the distance
    int indexOrigin = k/2;
    CSpherer *sphere         = dynamic_cast<CSpherer *>(m_pBody0->m_pShape);
    Real rad1 = sphere->Radius();
    CDistancePointRec<Real> distRecPoint(m_pBody0->m_vCOM,pBoundary->m_vBorders[k]);
    Real dist = distRecPoint.ComputeDistance();
    dist = dist -rad1;
    VECTOR3 vNormal = pBoundary->m_vBorders[k].GetNormal();
    VECTOR3 position = 0.5 * (distRecPoint.m_vClosestPoint0-rad1*vNormal + distRecPoint.m_vClosestPoint1);
    Real relVel = (m_pBody0->m_vVelocity * vNormal);
    //if the bodies are on collision course
    if(relVel < 0.0)
    {
      Real distpertime = -relVel*dDeltaT;
      //check whether there will be a collision next time step
      if(dist <= distpertime)
      {
        //std::cout<<"Pre-contact normal velocity: "<<relVel<<" colliding contact"<<std::endl;
        CContact contact;
        contact.m_dDistance  = dist;
        contact.m_vNormal    = vNormal;
        //pos = center - (dist*Normal)
        contact.m_vPosition0 = position;
        contact.m_vPosition1 = position;
        contact.m_pBody0     = m_pBody0;
        contact.m_pBody1     = m_pBody1;
        contact.id0 = contact.m_pBody0->m_iID;
        contact.id1 = contact.m_pBody1->m_iID;
        contact.vn           = relVel;
        contact.m_bResting   = false;
        contact.m_iState     = CCollisionInfo::TOUCHING;
        vContacts.push_back(contact);
      }//end if(dist <= distpertime)
    }
    else if(relVel < 0.00001 && dist < 0.005)
    {
      //std::cout<<"Pre-contact normal velocity: "<<relVel<<" resting contact"<<std::endl;
      CContact contact;
      contact.m_dDistance  = dist;
      contact.m_vNormal    = vNormal;
      contact.m_vPosition0 = position;
      contact.m_vPosition1 = position;
      contact.m_pBody0     = m_pBody0;
      contact.m_pBody1     = m_pBody1;
      contact.id0 = contact.m_pBody0->m_iID;
      contact.id1 = contact.m_pBody1->m_iID;
      contact.vn           = relVel;
      contact.m_bResting   = true;
      contact.m_iState     = CCollisionInfo::TOUCHING;
      vContacts.push_back(contact);
    }
    else
    {
      continue;
      //std::cout<<"Pre-contact normal velocity: "<<relVel<<" close proximity"<<std::endl;
      CContact contact;
      contact.m_dDistance  = dist;
      contact.m_vNormal    = vNormal;
      contact.m_vPosition0 = position;
      contact.m_vPosition1 = position;
      contact.m_pBody0     = m_pBody0;
      contact.m_pBody1     = m_pBody1;
      contact.id0 = contact.m_pBody0->m_iID;
      contact.id1 = contact.m_pBody1->m_iID;
      contact.vn           = relVel;
      contact.m_bResting   = false;
      contact.m_iState     = CCollisionInfo::VANISHING_CLOSEPROXIMITY;
      vContacts.push_back(contact);      
    }
    
  }//end for k

  //
  VECTOR3 origin(0.1225,0.2,0.0);
  Real length = 0.02;
  Real y = 0.2;
  //transform point to local coordinates
  CSpherer *sphere         = dynamic_cast<CSpherer *>(m_pBody0->m_pShape);
  Real rad1 = sphere->Radius();
  VECTOR3 center = m_pBody0->m_vCOM;
  VECTOR3 ctrans = center - origin;
  int section = int(ctrans.x/0.02);
  if(section < 0 || section > 13)
    return;

  ctrans.x-=Real(section)*length;

  if(ctrans.y < -1.75 * rad1)
  {
    return;
  }

  //determine the region the point is in
  CContact contact;
  //region 0
  if(ctrans.x <=0.0025)
  {
    //determine contact time
    contact.m_vNormal=VECTOR3(0,-1,0);
    Real distpertime = m_pBody0->m_vVelocity.y * dDeltaT;
    Real dist =  fabs(ctrans.y) - rad1;
    if(dist < distpertime || ctrans.y >= 0)
    {
      //generate 1 contact point
      contact.m_vPosition0.x = ctrans.x;
      contact.m_vPosition0.y = 0.0;
      contact.m_vPosition0.z = ctrans.z;
      //transform contact point

      contact.m_vPosition0 += origin;
      contact.m_vPosition0.x+=Real(section)*length;
      contact.m_dDistance  = dist;
      //pos = center - (dist*Normal)
      contact.m_vPosition1 = contact.m_vPosition0;
      contact.m_pBody0     = m_pBody0;
      contact.m_pBody1     = m_pBody1;
      contact.id0 = contact.m_pBody0->m_iID;
      contact.id1 = contact.m_pBody1->m_iID;
      contact.m_bResting   = false;
      contact.m_iState     = CCollisionInfo::TOUCHING;
      vContacts.push_back(contact);
    }
  }
  //region 1
  else if(ctrans.x <= 0.0125)
  {

    Real distpertime = m_pBody0->m_vVelocity.y * dDeltaT;
    Real dist =  fabs(ctrans.y) - rad1;
    if(ctrans.y < 0 && dist < distpertime)
    {
      //look at the x coordinate
      Real mid = 0.0075;
      if(ctrans.x < mid)
      {
        contact.m_vNormal= ctrans - VECTOR3(0.0025,0.0,ctrans.z);
        contact.m_vNormal.Normalize();

        contact.m_vPosition0 = ctrans - rad1*contact.m_vNormal;

        contact.m_vPosition0 += origin;
        contact.m_vPosition0.x+=Real(section)*length;
        contact.m_dDistance  = dist;
        //pos = center - (dist*Normal)
        contact.m_vPosition1 = contact.m_vPosition0;
        contact.m_pBody0     = m_pBody0;
        contact.m_pBody1     = m_pBody1;
        contact.id0 = contact.m_pBody0->m_iID;
        contact.id1 = contact.m_pBody1->m_iID;
        contact.m_bResting   = false;
        contact.m_iState     = CCollisionInfo::TOUCHING;
        vContacts.push_back(contact);

      }
      else
      {
        contact.m_vNormal= ctrans - VECTOR3(0.0125,0.0,ctrans.z);
        contact.m_vNormal.Normalize();
        contact.m_vPosition0 = ctrans - rad1*contact.m_vNormal;

        contact.m_vPosition0 += origin;
        contact.m_vPosition0.x+=Real(section)*length;
        contact.m_dDistance  = dist;
        //pos = center - (dist*Normal)
        contact.m_vPosition1 = contact.m_vPosition0;
        contact.m_pBody0     = m_pBody0;
        contact.m_pBody1     = m_pBody1;
        contact.id0 = contact.m_pBody0->m_iID;
        contact.id1 = contact.m_pBody1->m_iID;
        contact.m_bResting   = false;
        contact.m_iState     = CCollisionInfo::TOUCHING;
        vContacts.push_back(contact);

      }
    }
    else if(ctrans.y >= 0)
    {
      //look at the x coordinate
      Real mid = 0.0075;
      if(fabs(ctrans.x - mid) < rad1/10.0)
      {
        contact.m_vNormal= VECTOR3(1,0,0);

        contact.m_vPosition0 = ctrans - rad1*contact.m_vNormal;

        contact.m_vPosition0 += origin;
        contact.m_vPosition0.x+=Real(section)*length;
        contact.m_dDistance  = dist;
        //pos = center - (dist*Normal)
        contact.m_vPosition1 = contact.m_vPosition0;
        contact.m_pBody0     = m_pBody0;
        contact.m_pBody1     = m_pBody1;
        contact.id0 = contact.m_pBody0->m_iID;
        contact.id1 = contact.m_pBody1->m_iID;
        contact.m_bResting   = false;
        contact.m_iState     = CCollisionInfo::TOUCHING;
        vContacts.push_back(contact);

        contact.m_vNormal= VECTOR3(-1,0,0);

        contact.m_vPosition0 = ctrans - rad1*contact.m_vNormal;

        contact.m_vPosition0 += origin;
        contact.m_vPosition0.x+=Real(section)*length;
        contact.m_dDistance  = dist;
        //pos = center - (dist*Normal)
        contact.m_vPosition1 = contact.m_vPosition0;
        contact.m_pBody0     = m_pBody0;
        contact.m_pBody1     = m_pBody1;
        contact.id0 = contact.m_pBody0->m_iID;
        contact.id1 = contact.m_pBody1->m_iID;
        contact.m_bResting   = false;
        contact.m_iState     = CCollisionInfo::TOUCHING;
        vContacts.push_back(contact);

        dist = 0.005 - ctrans.y - rad1;

        //add another contact point
        if(dist < distpertime || dist <= rad1/10.0)
        {
          contact.m_vNormal = VECTOR3(0,-1,0);
          contact.m_vPosition0 = ctrans - rad1*contact.m_vNormal;
          contact.m_vPosition0 += origin;
          contact.m_vPosition0.x+=Real(section)*length;
          contact.m_dDistance  = dist;
          //pos = center - (dist*Normal)
          contact.m_vPosition1 = contact.m_vPosition0;
          contact.m_pBody0     = m_pBody0;
          contact.m_pBody1     = m_pBody1;
          contact.id0 = contact.m_pBody0->m_iID;
          contact.id1 = contact.m_pBody1->m_iID;
          contact.m_bResting   = false;
          contact.m_iState     = CCollisionInfo::TOUCHING;
          vContacts.push_back(contact);
        }
      }
      else if(ctrans.x < mid)
      {
        contact.m_vNormal= VECTOR3(1,0,0);

        contact.m_vPosition0 = ctrans - rad1*contact.m_vNormal;

        contact.m_vPosition0 += origin;
        contact.m_vPosition0.x+=Real(section)*length;
        contact.m_dDistance  = dist;
        //pos = center - (dist*Normal)
        contact.m_vPosition1 = contact.m_vPosition0;
        contact.m_pBody0     = m_pBody0;
        contact.m_pBody1     = m_pBody1;
        contact.id0 = contact.m_pBody0->m_iID;
        contact.id1 = contact.m_pBody1->m_iID;
        contact.m_bResting   = false;
        contact.m_iState     = CCollisionInfo::TOUCHING;
        vContacts.push_back(contact);

        dist = 0.005 - ctrans.y - rad1;

        //add another contact point
        if(dist < distpertime || dist < 0)
        {
          contact.m_vNormal = VECTOR3(0,-1,0);
          contact.m_vPosition0 = ctrans - rad1*contact.m_vNormal;
          contact.m_vPosition0 += origin;
          contact.m_vPosition0.x+=Real(section)*length;
          contact.m_dDistance  = dist;
          //pos = center - (dist*Normal)
          contact.m_vPosition1 = contact.m_vPosition0;
          contact.m_pBody0     = m_pBody0;
          contact.m_pBody1     = m_pBody1;
          contact.id0 = contact.m_pBody0->m_iID;
          contact.id1 = contact.m_pBody1->m_iID;
          contact.m_bResting   = false;
          contact.m_iState     = CCollisionInfo::TOUCHING;
          vContacts.push_back(contact);
        }
      }
      else if(ctrans.x > mid)
      {
        contact.m_vNormal= VECTOR3(-1,0,0);

        contact.m_vPosition0 = ctrans - rad1*contact.m_vNormal;

        contact.m_vPosition0 += origin;
        contact.m_vPosition0.x+=Real(section)*length;
        contact.m_dDistance  = dist;
        //pos = center - (dist*Normal)
        contact.m_vPosition1 = contact.m_vPosition0;
        contact.m_pBody0     = m_pBody0;
        contact.m_pBody1     = m_pBody1;
        contact.id0 = contact.m_pBody0->m_iID;
        contact.id1 = contact.m_pBody1->m_iID;
        contact.m_bResting   = false;
        contact.m_iState     = CCollisionInfo::TOUCHING;
        vContacts.push_back(contact);

        dist = 0.005 - ctrans.y - rad1;

        //add another contact point
        if(dist < distpertime || dist < 0)
        {
          contact.m_vNormal = VECTOR3(0,-1,0);
          contact.m_vPosition0 = ctrans - rad1*contact.m_vNormal;
          contact.m_vPosition0 += origin;
          contact.m_vPosition0.x+=Real(section)*length;
          contact.m_dDistance  = dist;
          //pos = center - (dist*Normal)
          contact.m_vPosition1 = contact.m_vPosition0;
          contact.m_pBody0     = m_pBody0;
          contact.m_pBody1     = m_pBody1;
          contact.id0 = contact.m_pBody0->m_iID;
          contact.id1 = contact.m_pBody1->m_iID;
          contact.m_bResting   = false;
          contact.m_iState     = CCollisionInfo::TOUCHING;
          vContacts.push_back(contact);
        }

      }
    }
  }
  //region 2
  else if(ctrans.x <= 0.015)
  {
    //determine contact time
    contact.m_vNormal=VECTOR3(0,-1,0);
    Real distpertime = m_pBody0->m_vVelocity.y * dDeltaT;
    Real dist =  fabs(ctrans.y) - rad1;
    if(dist < distpertime || ctrans.y >= 0)
    {
      //generate 1 contact point
      contact.m_vPosition0.x = ctrans.x;
      contact.m_vPosition0.y = 0.0;
      contact.m_vPosition0.z = ctrans.z;
      //transform contact point

      contact.m_vPosition0 += origin;
      contact.m_vPosition0.x+=Real(section)*length;
      contact.m_dDistance  = dist;
      //pos = center - (dist*Normal)
      contact.m_vPosition1 = contact.m_vPosition0;
      contact.m_pBody0     = m_pBody0;
      contact.m_pBody1     = m_pBody1;
      contact.id0 = contact.m_pBody0->m_iID;
      contact.id1 = contact.m_pBody1->m_iID;
      contact.m_bResting   = false;
      contact.m_iState     = CCollisionInfo::TOUCHING;
      vContacts.push_back(contact);
    }
  }
  //region 3
  else if(ctrans.x <= 0.02)
  {
    contact.m_vNormal= ctrans - VECTOR3(0.015,0.0,ctrans.z);
    contact.m_vNormal.Normalize();
    Real distpertime = m_pBody0->m_vVelocity.y * dDeltaT;
    Real dist =  fabs(ctrans.y) - rad1;
    if(dist < distpertime || ctrans.y > -0.005)
    {
      //generate 2 contact points
      contact.m_vPosition0 = ctrans - rad1 *contact.m_vNormal;
      //transform contact point

      contact.m_vPosition0 += origin;
      contact.m_vPosition0.x+=Real(section)*length;
      contact.m_dDistance  = dist;
      //pos = center - (dist*Normal)
      contact.m_vPosition1 = contact.m_vPosition0;
      contact.m_pBody0     = m_pBody0;
      contact.m_pBody1     = m_pBody1;
      contact.id0 = contact.m_pBody0->m_iID;
      contact.id1 = contact.m_pBody1->m_iID;
      contact.m_bResting   = false;
      contact.m_iState     = CCollisionInfo::TOUCHING;
      vContacts.push_back(contact);
      
      //add another contact point
      contact.m_vNormal= ctrans - VECTOR3(0.02,0.0,ctrans.z);
      contact.m_vNormal.Normalize();

      contact.m_vPosition0 = ctrans - rad1 *contact.m_vNormal;
      //transform contact point

      contact.m_vPosition0 += origin;
      contact.m_vPosition0.x+=Real(section)*length;
      contact.m_dDistance  = dist;
      //pos = center - (dist*Normal)
      contact.m_vPosition1 = contact.m_vPosition0;
      contact.m_pBody0     = m_pBody0;
      contact.m_pBody1     = m_pBody1;
      contact.id0 = contact.m_pBody0->m_iID;
      contact.id1 = contact.m_pBody1->m_iID;
      contact.m_bResting   = false;
      contact.m_iState     = CCollisionInfo::TOUCHING;
      vContacts.push_back(contact);
    }
  }



}*/
  
}
