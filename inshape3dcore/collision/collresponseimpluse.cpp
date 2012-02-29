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

#include "collresponseimpluse.h"
#include <3dmodel.h>
#include <3dmesh.h>
#include <rigidbody.h>

namespace i3d {

CCollResponseImpluse::CCollResponseImpluse(void)
{
}

CCollResponseImpluse::~CCollResponseImpluse(void)
{
}

CCollResponseImpluse::CCollResponseImpluse(std::list<CCollisionInfo> *CollInfo, CWorld *pWorld) : CCollResponse(CollInfo,pWorld)
{

}

void CCollResponseImpluse::ResolveCollisions()
{
	VECTOR3 FW1;
	VECTOR3 FW2;
	double eps1=0.4;
	Real e = 0.4;
	std::list<CCollisionInfo>::iterator Iter;
	std::vector<CContact>::iterator vIter;
	//We loop over all pairs
	for(Iter=m_CollInfo->begin();Iter!=m_CollInfo->end();Iter++)
	{
		CCollisionInfo Info = *Iter;

		for(vIter=Info.m_vContacts.begin();vIter!=Info.m_vContacts.end();vIter++)
		{

			CContact &contact = *vIter;

			//the difference between the 2rad and the distance
			//if this value is negative it is the overlap
			//if the value is positive then it is the distance that
			//is missing to the desired eps distance
			Real distout =  contact.m_dDistance;

			//calculate the collision normal and normalize it
			//compute collision normal
			VECTOR3 vNormal = contact.m_vNormal;

			//the massinverse is needed
			Real masssminv = Info.m_pBody1->m_dInvMass + Info.m_pBody2->m_dInvMass;

			//compute relative velocity
			VECTOR3 vAB = Info.m_pBody1->m_vVelocity - Info.m_pBody2->m_vVelocity;

			//velocity along the normal
			Real rNab = vAB * vNormal;

			//calculate the impulse
			Real n2 = vNormal * vNormal;
			Real rImpulse = (rNab * (-1.0-eps1))/(masssminv*n2);

			//create a vector that applies the impulse to the object
			//if the vector is added onto the velocity
			FW1 = vNormal *  rImpulse * Info.m_pBody1->m_dInvMass;
			FW2 = vNormal * -rImpulse * Info.m_pBody2->m_dInvMass;

			//m_Log.Write("----------------------------------------\n");
			//m_Log.Write("Detected a SPHERE-SPHERE collision ($i,$i) \n",Info.iID1,Info.iID2);
			//m_Log.Write("Simulation time: %f",Info.m_dTime);
			//m_Log.Write("relative normal Velocity before : %f",rNab);
			//m_Log.Write("relative normal Velocity post-condition : %f",(m_pParams->m_vVelocities[Info.iID1]+FW1-m_pParams->m_vVelocities[Info.iID2]-FW2) * vNormal);
			//m_Log.Write("----------------------------------------\n");

			//Add a response to solve the collision
		}
	}//end for

}//end ResolveCollisions

}