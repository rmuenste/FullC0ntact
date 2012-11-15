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

class CBroadPhaseStrategy;
class CBroadPhaseStrategy2Sphere;
class CWorld;
class CCollisionWall;
class CPhysicalParameters;
class CTimeControl;



/**
* @brief A Collision pipeline represents the sequence of actions in the collision module
*
*
*/
class CCollisionPipeline
{
  public:
    
	CCollisionPipeline();
  
	CCollisionPipeline(const CCollisionPipeline &copy);
  
	virtual ~CCollisionPipeline();
  
/**
* Sets up the collision pipeline with user-defined parameters
*
* @param  pWorld pointer to the world class
* @param  lcpIterations number of iterations of the lcp solver
* @param  pipelineIterations number of iterations of the collisionpipeline
*
*/
  void Init(CWorld *pWorld, int lcpIterations, int pipelineIterations);

/**
* Sets up the collision pipeline with user-defined parameters
*
* @param  pWorld pointer to the world class
* @param  solverType type of collision response solver
* @param  lcpIterations number of iterations of the lcp solver
* @param  pipelineIterations number of iterations of the collisionpipeline
*
*/
  void Init(CWorld *pWorld, int solverType, int lcpIterations, int pipelineIterations);

  inline void SetEPS(Real CollEps) {m_dCollEps=CollEps;};
  
  inline Real GetEPS() const {return m_dCollEps;};

  /** 
  * Starts the CD/CR pipeline.
  */
  virtual void StartPipeline();
  
  /** 
  * Starts the broad phase algorithm
  */  
  virtual void StartBroadPhase();
  
  /** 
  * Starts the middle phase algorithm
  */    
  virtual void StartMiddlePhase();
  
  /** 
  * Starts the narrow phase algorithm
  */      
  virtual void StartNarrowPhase();
  
  /** 
  * This wrapper contains a call to the contact solver
  */        
  virtual void SolveContactProblem();
        
  /** 
  * Integrates the dynamics system and steps it to the next time step
  */              
  void IntegrateDynamics();

/**
* The error correction step of the pipeline
*/
	void PenetrationCorrection();

/**
* Set the broadphase to simple spatialhashing
*/  
  void SetBroadPhaseSpatialHash();

/**
* Set the broadphase to hierarchical spatialhashing
*/  
  void SetBroadPhaseHSpatialHash();

/**
* Set the broadphase to naive all pairs test
*/  
  void SetBroadPhaseNaive();

/**
* Update the acceleration structures if neccessary
*/
  void UpdateDataStructures();

/**
* Update the contact graph
*/
  void UpdateContacts(CCollisionInfo &collinfo);

/**
* Analyzes the contact configuration
*/  
  void PostContactAnalysis();

	void StartCollisionWall(void);

	void StartCollisionResponseWall(void);

  void ProcessRemoteBodies();


  CContactGraph  *m_pGraph;
  std::vector<CContactGroup> m_pGroups;
  
	std::list<CCollisionInfo> m_CollInfo;

  std::set<CBroadPhasePair,Comp> m_BroadPhasePairs;

	std::vector<CContact> vContacts;

	CWorld *m_pWorld;

	CBroadPhase *m_BroadPhase;

	CBroadPhaseStrategy *m_Strategy;

	CCollResponse *m_Response;

	CTimeControl *m_pTimeControl;

  int m_iSolverType;

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
  CRigidBodyMotion *m_pIntegrator;

  /** 
  * number of iterations of the collision pipeline
  */
	int m_iPipelineIterations;

protected:
	bool update;
	Real m_dCollEps;

};

}

#endif // COLLISIONPIPELINE_H
