#ifndef DEMPIPELINE_HPP_LTV47MXA
#define DEMPIPELINE_HPP_LTV47MXA

#include <collisionpipeline.h>

namespace i3d
{
  template<>
  class CollisionPipeline<dem_gpu> : public BasicPipeline
  {
    public:

      CollisionPipeline();

      virtual ~CollisionPipeline();

      virtual void init(World *world, int solverType, int lcpIterations, int pipelineIterations) override
      {
        //cuda_init

      }

      virtual void startBroadPhase() override
      {
        //void calcHash(i3d::HashGrid<float, i3d::cpu> &hg, i3d::ParticleWorld<float, i3d::cpu> &pw);
        //void reorderDataAndFindCellStart(i3d::HashGrid<float, i3d::cpu> &hg,
        //  i3d::ParticleWorld<float, i3d::cpu> &pw);
      }

      virtual void startMiddlePhase() override
      {

      }

      virtual void startNarrowPhase() override
      {

      }

      virtual void solveContactProblem() override
      {
//        void collide(i3d::HashGrid<float, i3d::cpu> &hg, i3d::ParticleWorld<float, i3d::cpu> &pw);
      }

      virtual void integrateDynamics() override
      {
//        void integrateSystem(float *pos, float *vel, float deltaTime, unsigned int numParticles);
      }

      virtual void startPipeline() override
      {
        //startBroadPhase();
        //startMiddlePhase();
        //startNarrowPhase();
        //solveContactPhase();
        //integrateDynamics();
      }

    private:
      /* data */
  };

} /* i3d */ 

#endif /* end of include guard: DEMPIPELINE_HPP_LTV47MXA */
