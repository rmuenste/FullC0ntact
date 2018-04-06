#ifndef APPLICATION_H_IACES1LU
#define APPLICATION_H_IACES1LU

#include <worldparameters.h>
#include <unstructuredgrid.h>
#include <collisionpipeline.h>
#include <timecontrol.h>
#include <world.h>
#include <boundarybox.h>
#include <string>

#include <reader.h>
#include <particlefactory.h>
#include <motionintegratorsi.h>
#include <motionintegratordem.h>
#include <iostream>
#include <vtkwriter.h>
#include <iomanip>
#include <sstream>
#include <boundarycyl.h>
#include <algorithm>

namespace i3d {

  template <BackEnd collisionBackend=BackEnd::backendDefault>
  class Application
  {
  public:

    World myWorld_;

    UnstructuredGrid<Real, DTraits> grid_;

    WorldParameters dataFileParams_;

    int hasMeshFile_;

    CollisionPipeline<executionDefault, collisionBackend> myPipeline_;

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

    void writeOutput(int out, bool writeRBCom = false, bool writeRBSpheres = false);

  };

#include <application.cpp>

}

#endif /* end of include guard: APPLICATION_H_IACES1LU */
