#ifndef BASICPIPELINE_HPP_RBQ09NAH
#define BASICPIPELINE_HPP_RBQ09NAH

namespace i3d {

  enum {
    executionDefault,
    dem_gpu,
    dem_gpu_unified
  };

  class BroadPhaseStrategy;
  class CBroadPhaseStrategy2Sphere;
  class World;
  class CCollisionWall;
  class CPhysicalParameters;
  class TimeControl;

  class BasicPipeline
  {
  public:
    BasicPipeline() {};
    virtual ~BasicPipeline() {};

    virtual void init(World *world, int solverType, int lcpIterations, int pipelineIterations) = 0;

    /**
    * Starts the CD/CR pipeline.
    */
    virtual void startPipeline() = 0;

    /**
    * Starts the broad phase algorithm
    */
    virtual void startBroadPhase() = 0;

    /**
    * Starts the middle phase algorithm
    */
    virtual void startMiddlePhase() = 0;

    /**
    * Starts the narrow phase algorithm
    */
    virtual void startNarrowPhase() = 0;

    /**
    * This wrapper contains a call to the contact solver
    */
    virtual void solveContactProblem() = 0;

    /**
    * Integrates the dynamics system and steps it to the next time step
    */
    virtual void integrateDynamics() = 0;

  private:
    /* data */
  };

}

#endif /* end of include guard: BASICPIPELINE_HPP_RBQ09NAH */
