#ifndef COLLISIONPIPELINEODE_HPP_SX6TGWJ7
#define COLLISIONPIPELINEODE_HPP_SX6TGWJ7

#include <collisionpipeline.h>

#ifdef WITH_ODE
#include <ode_config.hpp>
#endif

#include <assert.h>

namespace i3d {

#ifdef WITH_ODE
  dWorldID *myWorldODE;
  dJointGroupID *myContactgroupODE;

  void odeCallbackWrapper(dWorldID &w, dJointGroupID &cg)
  {
    myWorldODE = &w;
    myContactgroupODE = &cg;
  }

  void nearCallback (void *data, dGeomID o1, dGeomID o2)
  {
    assert(o1);
    assert(o2);

    if (dGeomIsSpace(o1) || dGeomIsSpace(o2))
    {
        fprintf(stderr,"testing space %p %p\n", (void*)o1, (void*)o2);
      // colliding a space with something
      dSpaceCollide2(o1,o2,data,&nearCallback);
      // Note we do not want to test intersections within a space,
      // only between spaces.
      return;
    }

    const int N = 32;
    dContact contact[N];
    int n = dCollide (o1,o2,N,&(contact[0].geom),sizeof(dContact));
    if (n > 0) 
    {
      for (int i=0; i<n; i++) 
      {
        contact[i].surface.mode = 0;
        contact[i].surface.mu = 50.0; // was: dInfinity
        dJointID c = dJointCreateContact (*myWorldODE, *myContactgroupODE,&contact[i]);
        dJointAttach (c, dGeomGetBody(contact[i].geom.g1), dGeomGetBody(contact[i].geom.g2));
      }
    }
  }

#endif

/**
* @brief A Collision pipeline represents the sequence of actions in the collision module
*
*/
class CollisionPipelineODE : public CollisionPipeline<executionDefault>
{
  public:
    
    CollisionPipelineODE() : CollisionPipeline<executionDefault>() {

    };
    
    ~CollisionPipelineODE() {};

  /** 
  * Starts the CD/CR pipeline.
  */
  void startPipeline();

  
};

void CollisionPipelineODE::startPipeline()
{

  odeCallbackWrapper(world_->world, world_->contactgroup);

  dSpaceCollide (world_->space,0,&nearCallback);
  
  dWorldQuickStep (world_->world, world_->timeControl_->GetDeltaT()); // 100 Hz

  dJointGroupEmpty (world_->contactgroup);

}

}

#endif /* end of include guard: COLLISIONPIPELINEODE_HPP_SX6TGWJ7 */
