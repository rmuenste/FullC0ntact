#include "colliderboxboxboundary.h"
#include <obb3.h>
#include <iostream>
#include <globalheader.h>
#include <transform.h>
#include <stdio.h>
#include "collisioninfo.h"
#include <world.h>

namespace i3d {

ColliderBoxBoxBoundary::ColliderBoxBoxBoundary(void)
{
}

ColliderBoxBoxBoundary::~ColliderBoxBoxBoundary(void)
{
}

void ColliderBoxBoxBoundary::collide(std::vector<Contact> &vContacts)
{

  int i=0;
  VECTOR3 newvertices[8];
  VECTOR3 vertices[8];
  OBB3r *pBox0         = dynamic_cast<OBB3r *>(body0_->getWorldTransformedShape());
  pBox0->computeVertices(vertices);

  delete pBox0;
  const OBB3r &origBox0 = dynamic_cast<const OBB3r& >(body0_->getOriginalShape());

  if(!body0_->affectedByGravity_)
    return;

  CPredictionTransform<Real,OBB3r> Transform;
  OBB3r newbox = Transform.PredictMotion(origBox0,
                                          body0_->velocity_,
                                          body0_->getTransformation(),
                                          body0_->getAngVel(),world_->timeControl_->GetDeltaT());

  //get the vertices
  newbox.computeVertices(newvertices);

  //get the bounding box
	BoundaryBoxr *pBoundary = dynamic_cast<BoundaryBoxr *>(body1_->shape_);

  Real radius = newbox.getBoundingSphereRadius();

	//now check for all walls
	for(int k=0;k<6;k++)
	{
		//calculate the distance
		int indexOrigin = k/2;

    //center of the plane
		VECTOR3 planeCenter = pBoundary->points_[k];

    //calculate the distance to the plane
		Real dist2Center = (newbox.center_ - planeCenter) * pBoundary->normals_[k];
		if(dist2Center > radius)
			continue;

    Real dist;
		//for all vertices distance to plane
    for(i=0;i<8;i++)
    {
			Real newdist = (newvertices[i]-planeCenter) * pBoundary->normals_[k];
      dist = (vertices[i]-planeCenter) * pBoundary->normals_[k];
      //better: if the distance in the next time step is smaller than tolerance

      //compute the relative velocity
      VECTOR3 angPart = (VECTOR3::Cross(body0_->getAngVel(),vertices[i]-body0_->com_));
      VECTOR3 relativeVelocity = (body0_->velocity_ + angPart);

      //relative velocity along the normal
      Real normalVelocity = relativeVelocity * pBoundary->normals_[k];
      Real distpertime = normalVelocity * world_->timeControl_->GetDeltaT();
      
      if(dist < 0.1 * radius)
      {
        //g_Log.Write("Pre-contact normal velocity: %lf colliding contact",normalVelocity);
        //printf("Pre-contact normal velocity: %lf (%d,%d) colliding contact\n",normalVelocity,m_pBody0->m_iID,m_pBody1->m_iID);
        Contact contact;
        contact.m_dDistance  = dist;
        contact.m_vNormal    = pBoundary->normals_[k];
        contact.m_vPosition0 = vertices[i];
        contact.m_vPosition1 = vertices[i];
        contact.m_pBody0     = body0_;
        contact.m_pBody1     = body1_;
        contact.id0          = body0_->iID_;
        contact.id1          = body1_->iID_;          
        contact.vn           = normalVelocity;
        contact.m_iState     = CollisionInfo::TOUCHING;
        vContacts.push_back(contact);
      }

    }//end for

  }//end for all walls
}

}
