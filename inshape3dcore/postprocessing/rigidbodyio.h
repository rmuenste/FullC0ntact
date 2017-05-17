/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) <year>  <name of author>

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

#ifndef RIGIDBODYWRITER_H
#define RIGIDBODYWRITER_H
#include <vector>
#include <vector3.h>
#include <matrix3x3.h>
#include <string.h>
#include <quaternion.h>

namespace i3d {

class RigidBody;
class World;

/**
*  @brief A class that stores the file header for a rigid body output file
*
*  A class that stores the file header for a rigid body output file.
*  The member variables represent the content of the header.
*/
class RigidBodyHeader
{
public:
/**
  * Number of rigid bodies stored in the file
  */
  int    nParticles_;
  
/**
 * The variable stores the integer rank of the time step
 * in a ordered sequence of time steps
 */  
  int    timeStep_;
  
  int    nOutput_;
  
/**
 * The simulation time of the output file
 */  
  float  simTime_;
  
/**
 * The current time step used in the simulation at the time
 * of the outpu
 */  
  float  deltaT_;
};

/**
*  @brief A class that stores contruction info for a rigid body
*
*  A class that stores contruction info for a rigid body
*/
class BodyStorage
{
public:

  BodyStorage() = default;

  ~BodyStorage() {};  
  
  BodyStorage(const BodyStorage &copy)
  {
    com_         =    copy.com_;       
    velocity_    =    copy.velocity_;
    density_     =    copy.density_;
    volume_      =    copy.volume_;
    invMass_     =    copy.invMass_;
    restitution_  =    copy.restitution_;
    angVel_      =    copy.angVel_;
    angle_       =    copy.angle_;
    shapeId_       =    copy.shapeId_;
    id_          =    copy.id_;
    spheres      = copy.spheres;
    force_       =    copy.force_;
    torque_      =    copy.torque_;
    quat_           =    copy.quat_;
    extents_[0]   =    copy.extents_[0];
    extents_[1]   =    copy.extents_[1];
    extents_[2]   =    copy.extents_[2];   
    uvw_[0]      =    copy.uvw_[0];
    uvw_[1]      =    copy.uvw_[1];
    uvw_[2]      =    copy.uvw_[2];
    affectedByGravity_ = copy.affectedByGravity_;
    matrixAvailable_ = copy.matrixAvailable_;
    memcpy(tensor_,copy.tensor_,9*sizeof(Real));
    memcpy(fileName_,copy.fileName_,255);
  };
  
  VECTOR3      com_;
  VECTOR3      velocity_;
  VECTOR3      uvw_[3];
  Real         density_;
  Real         volume_;
  Real         invMass_;
  Real         restitution_;
  VECTOR3      angVel_;
  VECTOR3      angle_;
  int          shapeId_;
  VECTOR3      force_;
  VECTOR3      torque_;
  Quaternionr quat_;
  Real         extents_[3];
  Real         tensor_[9];
  char         fileName_[256];
  int          affectedByGravity_;
  int          id_;
  int          spheres;
//  bool         matrixAvailable_ = false;
  bool         matrixAvailable_;
};

/**
*  @brief A reader/writer for rigid bodies
*
*  A reader/writer for rigid bodies
*/
class RigidBodyIO
{
	public:

		RigidBodyIO();
		~RigidBodyIO();
		
    /**
    * Writes the bodies in the world to a file
    *
    * @param world The world containing the bodies
    * @param strFileName The name of the output file
    * @param outputBoundary On/Off switch for outputting the boundary
    */
	void write(World &world, const char *strFileName, bool outputBoundary=true);

    /**
    * Writes a selection of bodies from the world to a file
    *
    * @param world The world containing the bodies
    * @param world vIndices An index vector containing the indices of the bodies to write
    * @param strFileName The name of the output file
    */
		void write(World &world, std::vector<int> &vIndices, const char *strFileName);

    /**
    * Reads bodies from a file into the world
    *
    * @param world The world that will contain the bodies
    * @param strFileName The name of the input file
    */
		void read(World &world, const char *strFileName);
		
};

}

#endif // RIGIDBODYWRITER_H
