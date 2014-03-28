#include "colliderboxboxboundary.h"
#include <obb3.h>
#include <iostream>
#include <globalheader.h>
#include <transform.h>
#include <stdio.h>
#include "collisioninfo.h"
#include <world.h>

namespace i3d {

CColliderBoxBoxBoundary::CColliderBoxBoxBoundary(void)
{
}

CColliderBoxBoxBoundary::~CColliderBoxBoxBoundary(void)
{
}

void CColliderBoxBoxBoundary::Collide(std::vector<Contact> &vContacts)
{

  int i=0;
  VECTOR3 newvertices[8];
  VECTOR3 vertices[8];
  OBB3r *pBox0         = dynamic_cast<OBB3r *>(m_pBody0->getWorldTransformedShape());
  pBox0->computeVertices(vertices);

  delete pBox0;
  const OBB3r &origBox0 = dynamic_cast<const OBB3r& >(m_pBody0->getOriginalShape());

  if(!m_pBody0->affectedByGravity_)
    return;

  CPredictionTransform<Real,OBB3r> Transform;
  OBB3r newbox = Transform.PredictMotion(origBox0,
                                          m_pBody0->velocity_,
                                          m_pBody0->getTransformation(),
                                          m_pBody0->getAngVel(),m_pWorld->timeControl_->GetDeltaT());

  //get the vertices
  newbox.computeVertices(newvertices);

  //get the bounding box
	CBoundaryBoxr *pBoundary = dynamic_cast<CBoundaryBoxr *>(m_pBody1->shape_);

  Real radius = newbox.getBoundingSphereRadius();

	//now check for all walls
	for(int k=0;k<6;k++)
	{
		//calculate the distance
		int indexOrigin = k/2;

    //center of the plane
		VECTOR3 planeCenter = pBoundary->m_vPoints[k];

    //calculate the distance to the plane
		Real dist2Center = (newbox.center_ - planeCenter) * pBoundary->m_vNormals[k];
		if(dist2Center > radius)
			continue;

    Real dist;
		//for all vertices distance to plane
    for(i=0;i<8;i++)
    {
			Real newdist = (newvertices[i]-planeCenter) * pBoundary->m_vNormals[k];
      dist = (vertices[i]-planeCenter) * pBoundary->m_vNormals[k];
      //better: if the distance in the next time step is smaller than tolerance

      //compute the relative velocity
      VECTOR3 angPart = (VECTOR3::Cross(m_pBody0->getAngVel(),vertices[i]-m_pBody0->com_));
      VECTOR3 relativeVelocity = (m_pBody0->velocity_ + angPart);

      //relative velocity along the normal
      Real normalVelocity = relativeVelocity * pBoundary->m_vNormals[k];
      Real distpertime = normalVelocity * m_pWorld->timeControl_->GetDeltaT();
      
      if(dist < 0.1 * radius)
      {
        //g_Log.Write("Pre-contact normal velocity: %lf colliding contact",normalVelocity);
        //printf("Pre-contact normal velocity: %lf (%d,%d) colliding contact\n",normalVelocity,m_pBody0->m_iID,m_pBody1->m_iID);
        Contact contact;
        contact.m_dDistance  = dist;
        contact.m_vNormal    = pBoundary->m_vNormals[k];
        contact.m_vPosition0 = vertices[i];
        contact.m_vPosition1 = vertices[i];
        contact.m_pBody0     = m_pBody0;
        contact.m_pBody1     = m_pBody1;
        contact.id0          = m_pBody0->iID_;
        contact.id1          = m_pBody1->iID_;          
        contact.vn           = normalVelocity;
        contact.m_iState     = CollisionInfo::TOUCHING;
        vContacts.push_back(contact);
      }

    }//end for

  }//end for all walls
}

}
