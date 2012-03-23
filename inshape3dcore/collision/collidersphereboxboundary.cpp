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
 
   //now check for all walls
   for(int k=0;k<6;k++)
   {
     //calculate the distance
     int indexOrigin = k/2;
     CSpherer *sphere         = dynamic_cast<CSpherer *>(m_pBody0->m_pShape);
     CBoundaryBoxr *pBoundary = dynamic_cast<CBoundaryBoxr *>(m_pBody1->m_pShape);
     Real rad1 = sphere->Radius();
     Real position = m_pBody0->m_vCOM.m_dCoords[indexOrigin];
     Real distcenter = fabs(pBoundary->m_Values[k] - position);
     Real dist = distcenter -rad1;
     Real relVel = (m_pBody0->m_vVelocity * pBoundary->m_vNormals[k]);
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
       contact.m_vNormal    = pBoundary->m_vNormals[k];
       //pos = center - (dist*Normal)
       VECTOR3 pos = m_pBody0->m_vCOM - (dist*pBoundary->m_vNormals[k]);
       contact.m_vPosition0 = pos;
       contact.m_vPosition1 = pos;
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
       //std::cout<<"Pre-contact normal velocity: "<<relVel<<" resting contact"<<std::endl;
       continue;
       CContact contact;
       contact.m_dDistance  = dist;
       contact.m_vNormal    = pBoundary->m_vNormals[k];
       //pos = center - (dist*Normal)
       VECTOR3 pos = m_pBody0->m_vCOM - (dist*pBoundary->m_vNormals[k]);
       contact.m_vPosition0 = pos;
       contact.m_vPosition1 = pos;
       contact.m_pBody0     = m_pBody0;
       contact.m_pBody1     = m_pBody1;
       contact.id0 = contact.m_pBody0->m_iID;
       contact.id1 = contact.m_pBody1->m_iID;
       contact.vn           = relVel;
       contact.m_bResting   = false;
       contact.m_iState     = CCollisionInfo::VANISHING_CLOSEPROXIMITY;
       vContacts.push_back(contact);      
     }
       
   }//end for all walls
 
 }

}
