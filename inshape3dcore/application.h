#pragma once
#include <worldparameters.h>
#include <unstructuredgrid.h>
#include <collisionpipeline.h>
#include <timecontrol.h>
#include <world.h>

namespace i3d {

  class Application
  {
  public:

    World myWorld;

    UnstructuredGrid<Real, DTraits> grid_;

    WorldParameters dataFileParams;

    int hasMeshFile_;

    CollisionPipeline myPipeline;

    TimeControl myTimeControl;

    Application();

    ~Application();

  private:

    void configureTimeDiscretization();

    void configureRigidBodies();
  };


}


