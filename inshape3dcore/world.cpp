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

#include "world.h"
#include <particlefactory.h>
#include <iostream>
#include <sstream>

namespace i3d {

CWorld::CWorld()
{
  m_dDensityMedium = 0.0;
  m_bExtGraph = false;
}

CWorld::CWorld(const CWorld &copy)
{
	m_vRigidBodies = copy.m_vRigidBodies;
        m_bExtGraph = copy.m_bExtGraph;
}

CWorld::~CWorld()
{
	//output a message

}


void CWorld::Init()
{

}

Real CWorld::GetTotalEnergy()
{
	std::vector<CRigidBody*>::iterator rIter;
	Real totalEnergy=0.0;
	for(rIter=m_vRigidBodies.begin();rIter!=m_vRigidBodies.end();rIter++)
	{
		CRigidBody &body = *(*rIter);
		if(body.m_iShape == CRigidBody::BOUNDARYBOX)
			continue;
		totalEnergy += totalEnergy + body.GetEnergy();
	}
	return totalEnergy;
}

std::string CWorld::ToString()
{
	
	std::string out;
	std::stringstream ss;
	//iterators for models and submeshes
	std::vector<CRigidBody*>::iterator rIter;
	for(rIter=m_vRigidBodies.begin();rIter!=m_vRigidBodies.end();rIter++)
	{
	  CRigidBody &body = *(*rIter);
		
		ss<<"------------------------------------"<<std::endl;
		ss<<"Density         : "<<body.m_dDensity<<std::endl;
		ss<<"Mass            : "<<1.0/body.m_dInvMass<<std::endl;
		ss<<"Volume          : "<<body.m_dVolume<<std::endl;
		ss<<"Position        : "<<body.m_vCOM<<std::endl;
		ss<<"Velocity        : "<<body.m_vVelocity<<std::endl;
    ss<<"Shape           : "<<body.m_iShape<<std::endl;
		ss<<"Angle           : "<<body.m_vAngle<<std::endl;
		ss<<"Angular Velocity: "<<body.GetAngVel()<<std::endl;
		ss<<"Force           : "<<body.m_vForce<<std::endl;
		ss<<"Torque          : "<<body.m_vTorque<<std::endl;
	}
	ss<<"------------------------------------"<<std::endl;
	out = ss.str();
	return out;
	
}//


std::ostream &operator << (std::ostream &out, CWorld &world)
{
	//iterators for models and submeshes
	std::vector<CRigidBody*>::iterator rIter;
	for(rIter=world.m_vRigidBodies.begin();rIter!=world.m_vRigidBodies.end();rIter++)
	{
	  CRigidBody &body = *(*rIter);
    CAABB3r box = body.m_pShape->GetAABB();
    COBB3r *obb;
    if(body.m_iShape==CRigidBody::BOX)
    {
      obb = dynamic_cast<COBB3r *>(body.m_pShape);
    }
		std::cout<<"------------------------------------"<<std::endl;
    std::cout<<body.m_iShape<<" # type of body"<<std::endl;
    std::cout<<body.m_vCOM.x<<" "<<body.m_vCOM.y<<" "<<body.m_vCOM.z<<" # position of the body"<<std::endl;
    std::cout<<body.m_vVelocity.x<<" "<<body.m_vVelocity.y<<" "<<body.m_vVelocity.z<<" # velocity"<<std::endl;
    std::cout<<body.GetAngVel().x<<" "<<body.GetAngVel().y<<" "<<body.GetAngVel().z<<" # angular velocity"<<std::endl;
    std::cout<<body.m_vAngle.x<<" "<<body.m_vAngle.y<<" "<<body.m_vAngle.z<<" # angle"<<std::endl;
    std::cout<<body.m_vForce.x<<" "<<body.m_vForce.y<<" "<<body.m_vForce.z<<" # force"<<std::endl;
    std::cout<<body.m_vTorque.x<<" "<<body.m_vTorque.y<<" "<<body.m_vTorque.z<<" # torque"<<std::endl;
    std::cout<<box.m_Extends[0]<<" "<<box.m_Extends[1]<<" "<<box.m_Extends[2]<<" # bounding box extends"<<std::endl;
    //TODO: change for general shapes
    if(body.m_iShape==CRigidBody::BOX)
    {
      std::cout<<obb->m_vUVW[0].x<<" "<<obb->m_vUVW[0].y<<" "<<obb->m_vUVW[0].z<<" # U-Orientation vector"<<std::endl;     
      std::cout<<obb->m_vUVW[1].x<<" "<<obb->m_vUVW[1].y<<" "<<obb->m_vUVW[1].z<<" # U-Orientation vector"<<std::endl;     
      std::cout<<obb->m_vUVW[2].x<<" "<<obb->m_vUVW[2].y<<" "<<obb->m_vUVW[2].z<<" # U-Orientation vector"<<std::endl;     
    }
    std::cout<<body.m_dDensity<<" # density"<<std::endl;
    std::cout<<body.m_Restitution<<" # restitution"<<std::endl;
    std::cout<<body.m_bAffectedByGravity<<" # affected by gravitiy"<<std::endl;
	}
	std::cout<<"------------------------------------"<<std::endl;
	return out;
}//end operator <<

}
