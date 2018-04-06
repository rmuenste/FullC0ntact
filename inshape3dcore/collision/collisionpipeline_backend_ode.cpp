#include <ode_config.hpp>

dWorldID *myWorldODE;
dJointGroupID *myContactgroupODE;

void odeCallbackWrapper(dWorldID &w, dJointGroupID &cg)
{
  myWorldODE = &w;
  myContactgroupODE = &cg;
}

void nearCallback(void *data, dGeomID o1, dGeomID o2)
{
  assert(o1);
  assert(o2);

  if (dGeomIsSpace(o1) || dGeomIsSpace(o2))
  {
    fprintf(stderr, "testing space %p %p\n", (void*)o1, (void*)o2);
    // colliding a space with something
    dSpaceCollide2(o1, o2, data, &nearCallback);
    // Note we do not want to test intersections within a space,
    // only between spaces.
    return;
  }

  const int N = 32;
  dContact contact[N];
  int n = dCollide(o1, o2, N, &(contact[0].geom), sizeof(dContact));
  if (n > 0)
  {
    for (int i = 0; i<n; i++)
    {
      contact[i].surface.mode = 0;
      contact[i].surface.mu = 50.0; // was: dInfinity
      dJointID c = dJointCreateContact(*myWorldODE, *myContactgroupODE, &contact[i]);
      dJointAttach(c, dGeomGetBody(contact[i].geom.g1), dGeomGetBody(contact[i].geom.g2));
    }
  }
}

template <>
class CollisionPipeline<executionDefault, BackEnd::backendODE> : public BasicPipeline
{

protected:

  bool update_;
  Real collEps_;

public:

  ContactGraph  *graph_;

  std::vector<ContactGroup> groups_;

  std::list<CollisionInfo> collInfo_;

  std::set<BroadPhasePair, Comp> broadPhasePairs_;

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

  CollisionPipeline()
  {

    world_ = nullptr;
    strategy_ = nullptr;
    response_ = nullptr;
    broadPhase_ = nullptr;
    pipelineIterations_ = 5;
    graph_ = new ContactGraph();
    graph_->edges_ = new CollisionHash(1001);

  };

  CollisionPipeline(const CollisionPipeline &copy)
  {

    broadPhase_ = copy.broadPhase_;
    strategy_ = copy.strategy_;
    world_ = copy.world_;
    collInfo_ = copy.collInfo_;

  };

  virtual ~CollisionPipeline()
  {
    if (strategy_ != nullptr)
    {
      delete strategy_;
      strategy_ = nullptr;
    }
    if (response_ != nullptr)
    {
      delete response_;
      response_ = nullptr;
    }
    if (broadPhase_ != nullptr)
    {
      delete broadPhase_;
      broadPhase_ = nullptr;
    }
    delete graph_;
  };

  /**
  * Sets up the collision pipeline with user-defined parameters
  *
  * @param  world pointer to the world class
  * @param  solverType type of collision response solver
  * @param  lcpIterations number of iterations of the lcp solver
  * @param  pipelineIterations number of iterations of the collisionpipeline
  *
  */
  void init(World *world, int solverType, int lcpIterations, int pipelineIterations)
  {
    //set the world pointer
    world_ = world;
    timeControl_ = world->timeControl_;

    switch (solverType)
    {
    case 0:
      response_ = new CollResponseLcp(&collInfo_, world_);
      response_->SetEPS(collEps_);
      {
        CollResponseLcp *response = dynamic_cast<CollResponseLcp *>(response_);
        response->InitSolverPGS(lcpIterations, 1.0);
        solverType_ = 1;
      }
      break;
    case 1:
      response_ = new CollResponseLcp(&collInfo_, world_);
      response_->SetEPS(collEps_);
      {
        CollResponseLcp *response = dynamic_cast<CollResponseLcp *>(response_);
        response->InitSolverPGS(lcpIterations, 1.0);
        solverType_ = 1;
      }
      break;
    case 2:
      response_ = new CollResponseSI(&collInfo_, world_);
      response_->SetEPS(collEps_);
      solverType_ = 2;
      response_->setMaxIterations(lcpIterations);
      response_->setDefEps(5.0e-9);
      break;
    case 3:
      response_ = new CollResponseLcp(&collInfo_, world_);
      response_->SetEPS(collEps_);
      {
        CollResponseLcp *pResponse = dynamic_cast<CollResponseLcp *>(response_);
        pResponse->InitSolverPGS(lcpIterations, 1.0);
      }
      break;
    default:
      std::cerr << "wrong solver type in: collisionpipeline.h" << std::endl;
      std::exit(EXIT_FAILURE);
      break;
    }
    world_->graph_ = graph_;
    pipelineIterations_ = pipelineIterations;
  };

  inline void setEPS(Real CollEps) { collEps_ = CollEps; };

  inline Real getEPS() const { return collEps_; };

  /**
  * Starts the CD/CR pipeline.
  */
  virtual void startPipeline()
  {

    odeCallbackWrapper(world_->world, world_->contactgroup);

    dSpaceCollide(world_->space, 0, &nearCallback);

    dWorldQuickStep(world_->world, world_->timeControl_->GetDeltaT()); // 100 Hz

    dJointGroupEmpty(world_->contactgroup);

  };

  /**
  * Starts the broad phase algorithm
  */
  virtual void startBroadPhase()
  {

  };

  /**
  * Starts the middle phase algorithm
  */
  virtual void startMiddlePhase() {};

  /**
  * Starts the narrow phase algorithm
  */
  virtual void startNarrowPhase() {};

  /**
  * This wrapper contains a call to the contact solver
  */
  virtual void solveContactProblem() {};

  /**
  * Integrates the dynamics system and steps it to the next time step
  */
  void integrateDynamics() {};

  /**
  * The error correction step of the pipeline
  */
  void penetrationCorrection() {};

  /**
  * Set the broadphase to simple spatialhashing
  */
  void setBroadPhaseSpatialHash() {};

  /**
  * Set the broadphase to hierarchical spatialhashing
  */
  void setBroadPhaseHSpatialHash() {};

  /**
  * Set the broadphase to naive all pairs test
  */
  void setBroadPhaseNaive() {};

  /**
  * Update the acceleration structures if neccessary
  */
  void updateDataStructures() {};

  /**
  * Update the contact graph
  */
  void updateContacts(CollisionInfo &collinfo) {};

  /**
  * Analyzes the contact configuration
  */
  void postContactAnalysis() {};

  void startCollisionWall(void) {};

  void startCollisionResponseWall(void) {};

  void processRemoteBodies() {};

private:
  /* data */
};

