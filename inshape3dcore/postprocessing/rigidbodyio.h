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
#include <cstring>



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

  VECTOR3      com_;
  VECTOR3      velocity_;
  VECTOR3      uvw_[3];

  VECTOR3      angVel_;
  VECTOR3      angle_;

  VECTOR3      force_;
  VECTOR3      torque_;

  Quaternionr quat_;

  int          shapeId_;
  int          affectedByGravity_;
  int          id_;
  int          spheres;

  Real         density_;
  Real         volume_;
  Real         invMass_;
  Real         restitution_;

  Real         extents_[3];

  std::string  dynamicsType_; 

  Real         tensor_[9];
  char         fileName_[256];

  bool         matrixAvailable_;
  bool         useMeshFiles_;

  std::vector<std::string> meshFiles_;

  BodyStorage() : uvw_{Vec3(1,0,0), Vec3(0,1,0), Vec3(0,0,1)},
                  shapeId_(0), affectedByGravity_(0), id_(-1), spheres(0),
                  density_(1.0), volume_(1.0), invMass_(1.0), restitution_(1.0),
                  extents_{1.0,1.0,1.0}, dynamicsType_{"FULLY_DYNAMIC"}, 
                  tensor_{0,0,0, 0,0,0, 0,0,0},
                  matrixAvailable_(false), useMeshFiles_(false) 
  
  {

  };

  BodyStorage(const Vec3 &p, const Vec3 &r, const Vec3 &d,
              int shapeId, Real rho, Real mass) : com_(p),
                                        uvw_{Vec3(1,0,0), Vec3(0,1,0), Vec3(0,0,1)},
                                        angle_(r),
                                        shapeId_(shapeId), affectedByGravity_(1), id_(-1), 
                                        spheres(0),
                                        density_(rho), volume_(1.0), invMass_(1.0/mass), 
                                        restitution_(1.0),
                                        extents_{d.x, d.y, d.z},
                                        dynamicsType_{"FULLY_DYNAMIC"},
                                        tensor_{0,0,0, 0,0,0, 0,0,0},
                                        matrixAvailable_(false), useMeshFiles_(false)
  {

     velocity_ = Vec3(0,0,0);
     angVel_   = Vec3(0,0,0);
     angle_    = Vec3(0,0,0);
     force_    = Vec3(0,0,0);
     torque_   = Vec3(0,0,0);

  }

  BodyStorage(const Vec3 &p, const Vec3 &r, const Vec3 &d,
              int shapeId, Real rho) : com_(p),
                                        uvw_{Vec3(1,0,0), Vec3(0,1,0), Vec3(0,0,1)},
                                        angle_(r),
                                        shapeId_(shapeId), affectedByGravity_(1), id_(-1), 
                                        spheres(0),
                                        density_(rho), volume_(1.0), invMass_(1.0), 
                                        restitution_(1.0),
                                        extents_{d.x, d.y, d.z},
                                        dynamicsType_{"FULLY_DYNAMIC"},
                                        tensor_{0,0,0, 0,0,0, 0,0,0},
                                        matrixAvailable_(false), useMeshFiles_(false)
  {

     velocity_ = Vec3(0,0,0);
     angVel_   = Vec3(0,0,0);
     angle_    = Vec3(0,0,0);
     force_    = Vec3(0,0,0);
     torque_   = Vec3(0,0,0);

  }

  void toString()
  {
    std::cout << "com:" << com_;

    std::cout << "vel:" << velocity_;

    std::cout << "uvw0:" << uvw_[0];
    std::cout << "uvw1:" << uvw_[1];
    std::cout << "uvw2:" << uvw_[2];

    std::cout << "angVel_:" << angVel_;

    std::cout << "angle:" << angle_;

    std::cout << "force:" << force_;

    std::cout << "torque:" << torque_;

    std::cout << "quat:" << quat_;

    std::cout << "shapeId:" << shapeId_ << std::endl;

    std::cout << "density_:" << density_ << std::endl;

    std::cout << "mass:" << 1./invMass_  << std::endl;

    std::cout << "affectedByGravity_:" << affectedByGravity_  << std::endl;

    Vec3 ext(extents_[0],extents_[1],extents_[2]);
    std::cout << "extents_:" << ext << std::endl;

    std::cout << "matrixAvailable_:" << matrixAvailable_  << std::endl;

  }

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
    std::memcpy(tensor_, copy.tensor_, sizeof tensor_);
    std::memcpy(fileName_, copy.fileName_, sizeof fileName_);
    meshFiles_    = copy.meshFiles_;
    useMeshFiles_ = copy.useMeshFiles_;
  };
  
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
