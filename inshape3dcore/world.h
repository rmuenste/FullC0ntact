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

#ifndef PARTICLEDOMAIN_H
#define PARTICLEDOMAIN_H
#include<vector>
#include<3dmodel.h>
#include <boundingvolumetree3.h>
#include <subdivisioncreator.h>
#include <traits.h>
#include <rigidbody.h>
#include <boundarybox.h>
#include <timecontrol.h>
#include <parinfo.h>
#include <subdomainboundary.h>

#ifdef FC_CUDA_SUPPORT
  #include <particleSystem.h>
#endif

namespace i3d {

/**
* @brief The physics world
*
* A class that holds all the parameters defining the physics world that we intend
* simulate.
*
*/
class World
{
  
private:  

  /**
    * The gravity vector (assumed unit is m/s**2)
    **/  
  VECTOR3 gravity_;  
    
public:

  /**
   * The vector of rigid bodies used in the simulation
   **/
  std::vector<RigidBody*> rigidBodies_;
  
  std::vector<VECTOR3> externalForces_;

  std::list<std::pair<int,int> > sendList_;
  
  /**
   * A describtion of the boundary of the simulation domain
   **/
  CBoundaryBoxr *boundary_;
  
  /**
   * An object that controls the timestep settings
   **/
  TimeControl *timeControl_;

  int particles_;

  int rigidBodiesDynamic_;

  int rigidBodiesStatic_;

  int output_;

  /**
   * On/Off switch for extended contact graph algorithms
   **/
  bool extGraph_;

  /**
   * In case the rigid bodies are immersed into a surrounding medium
   * we have to define the density of the surrounding medium
   **/
  Real densityMedium_;
  
  /**
   * When the simulation is run in parallel, we store
   * the parallel info in this structure
   **/
  CParInfo parInfo_;
  
  /**
   * True in case of a Liquid-Solid simulation and false
   * for a pure solid simulation.
   **/
  bool liquidSolid_;
  
  /**
   * Type of the contact force solver
   */
  int solverType_;
  
  /**
   * Description of the subdomainboundary
   * */
  SubdomainBoundary *subBoundary_;
  
  std::vector<CDistanceMap<Real>* > maps_;  

#ifdef FC_CUDA_SUPPORT
  ParticleSystem *psystem;
#endif
  
  
  World();
  ~World();
  World(const World &copy);

  void init();

  inline void setGravity(const VECTOR3 &gravity)
  {
    gravity_ = gravity;
  }

  inline VECTOR3 getGravity() 
  {
    return gravity_;
  }

  inline VECTOR3 getGravityEffect(RigidBody *body) 
  {
    if(liquidSolid_)
      return body->volume_*(body->density_ - densityMedium_) * body->invMass_ * gravity_;      
    else
      return gravity_;
  }

  inline void setBoundary(CBoundaryBoxr *boundary)
  {
    boundary_ = boundary;
  }

  inline void setTimeControl(TimeControl *timeControl)
  {
    timeControl_ = timeControl;
  }

  Real getTotalEnergy();

  friend std::ostream& operator<<(std::ostream& out, World &world); 

  /**
  * @brief Prints the world configuration to a string
  *
  * @return Returns a string output of the world configuration
  */
  std::string toString();
  
};

}

#endif // PARTICLEDOMAIN_H
