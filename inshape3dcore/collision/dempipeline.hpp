#ifndef DEMPIPELINE_HPP_LTV47MXA
#define DEMPIPELINE_HPP_LTV47MXA

#include <collisionpipeline.h>
#include <difi.cuh>
#include <particledem.cuh>
#include <cuda.h>
#include <cuda_runtime.h>

namespace i3d
{
  template<>
  class CollisionPipeline<dem_gpu> : public BasicPipeline
  {
    public:

      HashGrid<float, cpu> *hg;
      ParticleWorld<float, cpu> *pw;

      CollisionPipeline(){};

      virtual ~CollisionPipeline() {};

      virtual void init(World *world, int solverType, int lcpIterations, int pipelineIterations) override
      {

      }

      virtual void startBroadPhase() override
      {
        calcHash(*hg, *pw);
        sortParticles(*hg);
      }

      virtual void startMiddlePhase() override
      {
        reorderDataAndFindCellStart(*hg, *pw);
      }

      virtual void startNarrowPhase() override
      {

      }

      virtual void solveContactProblem() override
      {
        evalForces(*hg, *pw);
      }

      virtual void integrateDynamics() override
      {
        integrateSystem(pw->pos_, pw->vel_, pw->params_->timeStep_, pw->size_);
        cudaDeviceSynchronize();
      }

      virtual void startPipeline() override
      {
        startBroadPhase();
        startMiddlePhase();

        solveContactProblem();
        integrateDynamics();
      }

    private:
      /* data */
  };

} /* i3d */ 

#endif /* end of include guard: DEMPIPELINE_HPP_LTV47MXA */
