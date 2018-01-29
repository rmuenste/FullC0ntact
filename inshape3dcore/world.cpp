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

World::World()
{
  densityMedium_ = 0.0;
  extGraph_ = false;
  airFriction_ = 0.99;

#ifdef FC_CUDA_SUPPORT
  psystem = 0;
#endif

}

World::World(const World &copy)
{
  rigidBodies_ = copy.rigidBodies_;
  bodies_ = copy.bodies_;
  extGraph_ = copy.extGraph_;
  airFriction_ = copy.airFriction_;
}

World::~World()
{
	//output a message

}


void World::init()
{

}

Real World::getTotalEnergy()
{
	std::vector<RigidBody*>::iterator rIter;
	Real totalEnergy=0.0;
	for(rIter=rigidBodies_.begin();rIter!=rigidBodies_.end();rIter++)
	{
		RigidBody &body = *(*rIter);
		if(body.shapeId_ == RigidBody::BOUNDARYBOX)
			continue;
		totalEnergy += totalEnergy + body.getEnergy();
	}
	return totalEnergy;
}

std::string World::toString()
{
	
	std::string out;
	std::stringstream ss;
	//iterators for models and submeshes
	std::vector<RigidBody*>::iterator rIter;
	for(rIter=rigidBodies_.begin();rIter!=rigidBodies_.end();rIter++)
	{
	  RigidBody &body = *(*rIter);
		
		ss<<"------------------------------------"<<std::endl;
		ss<<"Density         : "<<body.density_<<std::endl;
		ss<<"Mass            : "<<1.0/body.invMass_<<std::endl;
		ss<<"Volume          : "<<body.volume_<<std::endl;
		ss<<"Position        : "<<body.com_<<std::endl;
		ss<<"Velocity        : "<<body.velocity_<<std::endl;
    ss<<"Shape           : "<<body.shape_<<std::endl;
		ss<<"Angle           : "<<body.angle_<<std::endl;
		ss<<"Angular Velocity: "<<body.getAngVel()<<std::endl;
		ss<<"Force           : "<<body.force_<<std::endl;
		ss<<"Torque          : "<<body.torque_<<std::endl;
	}
	ss<<"------------------------------------"<<std::endl;
	out = ss.str();
	return out;
	
}//


std::ostream &operator << (std::ostream &out, World &world)
{
	//iterators for models and submeshes
	std::vector<RigidBody*>::iterator rIter;
	for(rIter=world.rigidBodies_.begin();rIter!=world.rigidBodies_.end();rIter++)
	{
	  RigidBody &body = *(*rIter);
    AABB3r box = body.shape_->getAABB();
    OBB3r *obb = nullptr;
    if(body.shapeId_==RigidBody::BOX)
    {
      obb = dynamic_cast<OBB3r *>(body.shape_);
    }
		std::cout<<"------------------------------------"<<std::endl;
    std::cout<<body.shape_<<" # type of body"<<std::endl;
    std::cout<<body.com_.x<<" "<<body.com_.y<<" "<<body.com_.z<<" # position of the body"<<std::endl;
    std::cout<<body.velocity_.x<<" "<<body.velocity_.y<<" "<<body.velocity_.z<<" # velocity"<<std::endl;
    std::cout<<body.getAngVel().x<<" "<<body.getAngVel().y<<" "<<body.getAngVel().z<<" # angular velocity"<<std::endl;
    std::cout<<body.angle_.x<<" "<<body.angle_.y<<" "<<body.angle_.z<<" # angle"<<std::endl;
    std::cout<<body.force_.x<<" "<<body.force_.y<<" "<<body.force_.z<<" # force"<<std::endl;
    std::cout<<body.torque_.x<<" "<<body.torque_.y<<" "<<body.torque_.z<<" # torque"<<std::endl;
    std::cout<<box.extents_[0]<<" "<<box.extents_[1]<<" "<<box.extents_[2]<<" # bounding box extends"<<std::endl;
    //TODO: change for general shapes
    if(body.shapeId_==RigidBody::BOX)
    {
      std::cout<<obb->uvw_[0].x<<" "<<obb->uvw_[0].y<<" "<<obb->uvw_[0].z<<" # U-Orientation vector"<<std::endl;     
      std::cout<<obb->uvw_[1].x<<" "<<obb->uvw_[1].y<<" "<<obb->uvw_[1].z<<" # U-Orientation vector"<<std::endl;     
      std::cout<<obb->uvw_[2].x<<" "<<obb->uvw_[2].y<<" "<<obb->uvw_[2].z<<" # U-Orientation vector"<<std::endl;     
    }
    std::cout<<body.density_<<" # density"<<std::endl;
    std::cout<<body.restitution_<<" # restitution"<<std::endl;
    std::cout<<body.affectedByGravity_<<" # affected by gravitiy"<<std::endl;
	}
	std::cout<<"------------------------------------"<<std::endl;
	return out;
}//end operator <<

}
