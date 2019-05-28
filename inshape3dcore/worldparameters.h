/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) <2011>  <Raphael Muenster>

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
#ifndef WORLDPARAMETERS_H
#define WORLDPARAMETERS_H



//===================================================
//                     INCLUDES
//===================================================
#include <string>
#include <vector3.h>
#include <vector>
#include <rigidbodyio.h>
#include <json.hpp>

namespace i3d {

struct bndryShape {

  int type; 
  char name[1024];

};

/**
* @brief A class that contains parameters used to create a CWorld.
*
* A class that contains parameters used to create a CWorld.
* @author Raphael Muenster
*
*/
class WorldParameters {
  
public: 

  /**
  *
  * Create new CWorldParameters
  *
  */
  WorldParameters(); 

  ~WorldParameters(); 

  /**
  * This variable controls the simulation start, if it is
  * set to 0, a new simulation is started.
  * 
  * If the variable is set to 1, a simulation is continued from a
  * so called solution file. The name of the solution file is specified in
  * m_sSolution.
  **/
  int startType_;

  /**
  * The variable sets the number of rigid bodies in the simulation. However, this
  * variable is only used, when rigid bodies are specified in the rigid body section
  * of the input data file 'data.TXT'. When placement function are used, this variable
  * has no effect.
  **/
  int bodies_;

  /**
  * Here we can enter user-defined placement functions that produce an initial placement
  * of the rigid bodies used in the simulation.
  **/
  int bodyInit_;

  /**
  * The total number of timesteps used in a pure solid simulation. If we have a liquid-solid
  * simulation this variable has no effect.
  **/
  int nTimesteps_;

  /**
  * The timestep used in a rigid body simulation
  **/
  Real timeStep_;

  /**
  * The refinement level in case we have a mesh
  **/
  int refinementLevel_;

  /**
  * Maximum iterations of the lcp solver
  **/
  int maxIterations_;

  int pipelineIterations_;

  /**
  * Type of the collision force solver
  */
  int boundaryComponents_;

  /**
   * Type of the collision force solver 
   */
  int solverType_;

  /**
  * Set the variable to 1 for a Liquid-Solid simulation and 0 for a pure rigid body simulation
  **/
  int liquidSolid_;

  /**
   * The default density for every body, if no particular density is specified 
   */
  Real defaultDensity_;

  /**
   * The default bounding sphere radius for every body, if no particular density is specified 
   */
  Real defaultRadius_;

  /**
  * Set the densitiy of the surrounding medium for a Liquid-Solid simulation
  **/
  Real densityMedium_;

  /**
   * The precision of the LCP collision response solver 
   */
  Real epsLCPSolver_;

  /**
   * Name of the stored solution file
   */
  std::string solutionFile_;

  /**
   * Name of the rigid body configuration file
   */
  std::string bodyConfigurationFile_;

  /**
   * Name of a ODE configuration file 
   */
  std::string odeConfigurationFile_;

  /**
   * Name of a CGAL configuration file 
   */
  std::string cgalConfigurationFile_;

  /**
   * Gravity used in the simulation
   */
  VECTOR3 gravity_;

  /**
   * Vector of input rigid bodies
   */
  std::vector<BodyStorage> rigidBodies_;

  /**
  * Vector of boundaries
  */
  std::vector<bndryShape> boundaries_;

  /**
  * Extents of the domain
  */
  Real extents_[6];

  /**
  * Boolean flag that is set to true in case the domain extents were given
  * in the data file
  */
  bool hasExtents_;
  
  /**
   * Getter Method for member hasExtents_
   */
  bool hasExtents() {return hasExtents_;};
  
  /**
   * Setter Method for member hasExtents_
   */
  void setHasExtents(bool flag) {hasExtents_=flag;};

  /**
   * Air friction constant to impose static air friction that
   */
  Real airFriction_;
  /**
   * State variable for FBM calculation 
   */
  bool doFBM_;

  /**
   * State variable for dynamics calculation 
   */
  bool doDynamics_;

  /**
   * State variable for rigid body output 
   */
  bool outputRigidBodies_;

  /**
   * This state variable sets whether a default boundary should
   * be added to the rigid body setup.
   * @default false
   */
  bool excludeDefaultBoundary_;

  int meshingStrategy_;

  nlohmann::json jsonData;

  
};

}
#endif
