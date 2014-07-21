#pragma once
#include <worldparameters.h>
#include <unstructuredgrid.h>
#include <collisionpipeline.h>
#include <timecontrol.h>
#include <world.h>
#include <boundarybox.h>
#include <string>

namespace i3d {

  class Application
  {
  public:

    World myWorld_;

    UnstructuredGrid<Real, DTraits> grid_;

    WorldParameters dataFileParams_;

    int hasMeshFile_;

    CollisionPipeline myPipeline_;

    TimeControl myTimeControl_;

    BoundaryBoxr myBoundary_;

    RigidBodyMotion *myMotion_;

    double xmin_;
    double ymin_;
    double zmin_;
    double xmax_;
    double ymax_;
    double zmax_;

    Application();

    virtual ~Application();

    virtual void init(std::string fileName = std::string("start/data.TXT"));

    virtual void run()=0;

  protected:

    void configureTimeDiscretization();

    void configureRigidBodies();

    void configureBoundary();

    void configureCylinderBoundary();

    void configureHollowCylinderBoundary();

    void writeOutput(int out);

  };


}


