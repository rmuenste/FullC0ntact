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

#ifndef COLLISIONPIPELINE_H
#define COLLISIONPIPELINE_H
#include <list>
#include <queue>
#include <collisioninfo.h>
#include <collresponse.h>
#include <broadphase.h>
#include <compare.h>
#include <rigidbodymotionmesh.h>
#include <rigidbodymotion.h>
#include <set>
#include <contactgraph.h>


namespace i3d {

class BroadPhaseStrategy;
class CBroadPhaseStrategy2Sphere;
class World;
class CCollisionWall;
class CPhysicalParameters;
class TimeControl;



/**
* @brief A Collision pipeline represents the sequence of actions in the collision module
*
*
*/
class CollisionPipeline
{
  
  protected:
    
  bool update_;
  
  Real collEps_;
  
  
  public:
    
  ContactGraph  *graph_;
  
  std::vector<ContactGroup> groups_;
  
  std::list<CollisionInfo> collInfo_;

  std::set<BroadPhasePair,Comp> broadPhasePairs_;

  std::vector<Contact> contacts_;

  World *world_;

  BroadPhase *broadPhase_;

  BroadPhaseStrategy *strategy_;

  CollResponse *response_;

  TimeControl *timeControl_;

  int solverType_;

  enum
  {
    SPHERE_SPHERE,
    SPHERES,
    BVH_BVH,
    TOI,
    COMPLEX,
    NAIVE,
    SPATIALHASH,
    TOI_HEAP,
    DISCRETE,
    DISCRETECONSTRAINED,
    BOXSHAPED,
    COMPLEXSHAPED,
    RIGIDBODY
  };

  /** 
  * integrator class that can integrate the system to the next time step
  */
  RigidBodyMotion *integrator_;

  /** 
  * number of iterations of the collision pipeline
  */
  int pipelineIterations_;
        
  CollisionPipeline();

  CollisionPipeline(const CollisionPipeline &copy);

  virtual ~CollisionPipeline();
  
  /**
  * Sets up the collision pipeline with user-defined parameters
  *
  * @param  world pointer to the world class
  * @param  lcpIterations number of iterations of the lcp solver
  * @param  pipelineIterations number of iterations of the collisionpipeline
  *
  */
  void init(World *world, int lcpIterations, int pipelineIterations);

  /**
  * Sets up the collision pipeline with user-defined parameters
  *
  * @param  world pointer to the world class
  * @param  solverType type of collision response solver
  * @param  lcpIterations number of iterations of the lcp solver
  * @param  pipelineIterations number of iterations of the collisionpipeline
  *
  */
  void init(World *world, int solverType, int lcpIterations, int pipelineIterations);

  inline void setEPS(Real CollEps) {collEps_=CollEps;};
  
  inline Real getEPS() const {return collEps_;};

  /** 
  * Starts the CD/CR pipeline.
  */
  virtual void startPipeline();
  
  /** 
  * Starts the broad phase algorithm
  */  
  virtual void startBroadPhase();
  
  /** 
  * Starts the middle phase algorithm
  */    
  virtual void startMiddlePhase();
  
  /** 
  * Starts the narrow phase algorithm
  */      
  virtual void startNarrowPhase();
  
  /** 
  * This wrapper contains a call to the contact solver
  */        
  virtual void solveContactProblem();
        
  /** 
  * Integrates the dynamics system and steps it to the next time step
  */              
  void integrateDynamics();

/**
* The error correction step of the pipeline
*/
  void penetrationCorrection();

/**
* Set the broadphase to simple spatialhashing
*/  
  void setBroadPhaseSpatialHash();

/**
* Set the broadphase to hierarchical spatialhashing
*/  
  void setBroadPhaseHSpatialHash();

/**
* Set the broadphase to naive all pairs test
*/  
  void setBroadPhaseNaive();

/**
* Update the acceleration structures if neccessary
*/
  void updateDataStructures();

/**
* Update the contact graph
*/
  void updateContacts(CollisionInfo &collinfo);

/**
* Analyzes the contact configuration
*/  
  void postContactAnalysis();

  void startCollisionWall(void);

  void startCollisionResponseWall(void);

  void processRemoteBodies();

};

}

#endif // COLLISIONPIPELINE_H

