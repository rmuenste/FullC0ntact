#include <iostream>
#include <application_ode.hpp>
#include <reader.h>
#include <paramline.h>

#include <ode/odeconfig.h>
#include <assert.h>
#ifdef HAVE_UNISTD_H
#include <unistd.h>
#endif
#include <ode/ode.h>
#include <triangulator.h>
#include <collisionpipeline.h>

// dynamics and collision objects (chassis, 3 wheels, environment)
static dWorldID world;
static dSpaceID space;

static dBodyID cylbody;
static dGeomID cylgeom;

static dBodyID sphbody;
static dGeomID sphgeom;

static dJointGroupID contactgroup;

static bool show_contacts = true;

double simTime = 0.0;
double dt = 0.01;

const double CYLRADIUS = 0.6;
const double CYLLENGTH = 2.0;
const double SPHERERADIUS = 0.5;

#ifdef dDOUBLE
  #define dsDrawBox dsDrawBoxD
  #define dsDrawLine dsDrawLineD
#endif

using ODE_App = i3d::Application<i3d::BackEnd::backendODE>;

namespace i3d {


  class ODE_Test : public ODE_App {

  public:

    ODE_Test() : Application() {

    }

    ~ODE_Test()
    {
      dJointGroupEmpty (myWorld_.contactgroup);
      dJointGroupDestroy (myWorld_.contactgroup);

      dSpaceDestroy (myWorld_.space);
      dWorldDestroy (myWorld_.world);
      dCloseODE();
    }

    void run()
    {
      for (int i(0); i <= 1000; ++i)
      {

        myPipeline_.startPipeline();

        writeOutput(i);
        printf("Time: %f |Step: %d |\n",simTime, i);
        simTime += dt;

      }
    }

  };
}

using namespace i3d;

int main()
{

  //----------------------------------------------
  ODE_Test myApp;

  // init application
  myApp.init("start/sampleRigidBody.xml");

  // run simulation
  myApp.run();

  return 0;

}
