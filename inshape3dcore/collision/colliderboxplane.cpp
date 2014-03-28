#include "colliderboxplane.h"
#include <obb3.h>
#include <iostream>
#include <globalheader.h>
#include <transform.h>

namespace i3d {

CColliderBoxPlane::CColliderBoxPlane(void)
{
}

CColliderBoxPlane::~CColliderBoxPlane(void)
{
}

void CColliderBoxPlane::Collide(std::vector<Contact> &vContacts)
{

 // int i=0;
 // VECTOR3 newvertices[8];
 // VECTOR3 vertices[8];
 // COBB3r *pBox0         = dynamic_cast<COBB3r *>(m_pBody0->GetWorldTransformedShape());
 // pBox0->ComputeVertices(vertices);

 // delete pBox0;
 // const COBB3r &origBox0 = dynamic_cast<const COBB3r& >(m_pBody0->GetOriginalShape());

 // if(!m_pBody0->m_bAffectedByGravity)
 //   return;

 // CPredictionTransform<Real,COBB3r> Transform;
 // COBB3r newbox = Transform.PredictMotion(origBox0,m_pBody0->m_vVelocity,m_pBody0->GetTransformation(),m_pBody0->GetAngVel(),dDeltaT);

 // //get the vertices
 // newbox.ComputeVertices(newvertices);

 // //get the bounding box
	//CBoundaryBoxr *pBoundary = dynamic_cast<CBoundaryBoxr *>(m_pBody1->m_pShape);

 // Real radius = newbox.GetBoundingSphereRadius();

	////now check for all walls
	//for(int k=0;k<6;k++)
	//{
	//	//calculate the distance
	//	int indexOrigin = k/2;

 //   //center of the plane
	//	VECTOR3 planeCenter = pBoundary->m_vPoints[k];

 //   //calculate the distance to the plane
	//	Real dist2Center = (newbox.m_vCenter - planeCenter) * pBoundary->m_vNormals[k];
	//	if(dist2Center > radius)
	//		continue;

 //   Real dist;
	//	//for all vertices distance to plane
 //   for(i=0;i<8;i++)
 //   {
	//		Real newdist = (newvertices[i]-planeCenter) * pBoundary->m_vNormals[k];
 //     dist = (vertices[i]-planeCenter) * pBoundary->m_vNormals[k];
 //     //better: if the distance in the next time step is smaller than tolerance

 //     //compute the relative velocity
 //     VECTOR3 angPart = (VECTOR3::Cross(m_pBody0->GetAngVel(),vertices[i]-m_pBody0->m_vCOM));
 //     VECTOR3 relativeVelocity = (m_pBody0->m_vVelocity + angPart);

 //     //relative velocity along the normal
 //     Real normalVelocity = relativeVelocity * pBoundary->m_vNormals[k];
 //     Real distpertime = normalVelocity * dDeltaT;

 //     if(normalVelocity < -0.005)
 //     { 
 //       
 //       if(fabs(distpertime) >= dist)
 //       {
 //         //g_Log.Write("Pre-contact normal velocity: %lf colliding contact",normalVelocity);
 //         //printf("Pre-contact normal velocity: %lf colliding contact\n",normalVelocity);
 //         CContact contact;
 //         contact.m_dDistance  = dist;
 //         contact.m_vNormal    = pBoundary->m_vNormals[k];
 //         contact.m_vPosition0 = vertices[i];
 //         contact.m_vPosition1 = vertices[i];
 //         contact.m_pBody0     = m_pBody0;
 //         contact.m_pBody1     = m_pBody1;
 //         contact.vn           = normalVelocity;
 //         vContacts.push_back(contact);
 //       }
 //     }
 //     else if(normalVelocity < 0.00001)
 //     { 
 //       if(dist < 0.00005)
 //       {
 //         //printf("Pre-contact normal velocity: %lf resting contact\n",normalVelocity);
 //         CContact contact;
 //         contact.m_dDistance  = dist;
 //         contact.m_vNormal    = pBoundary->m_vNormals[k];
 //         contact.m_vPosition0 = vertices[i];
 //         contact.m_vPosition1 = vertices[i];
 //         contact.m_pBody0     = m_pBody0;
 //         contact.m_pBody1     = m_pBody1;
 //         contact.vn           = normalVelocity;
 //         vContacts.push_back(contact);
 //        }
 //     }

 //     
 //   }//end for

	//}//end for all walls
}

}