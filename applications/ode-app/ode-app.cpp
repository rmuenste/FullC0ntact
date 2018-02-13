#include <iostream>
#include <application.h>
#include <reader.h>
#include <paramline.h>
#include <collisionpipelineode.hpp>

#include <ode/odeconfig.h>
#include <assert.h>
#ifdef HAVE_UNISTD_H
#include <unistd.h>
#endif
#include <ode/ode.h>
#include <triangulator.h>
#include <json.hpp>

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


namespace i3d {



  class ODE_Test : public Application {

  public:

    CollisionPipelineODE collPipeline_;

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

    void init(std::string fileName)
    {

      size_t pos = fileName.find(".");

      std::string ending = fileName.substr(pos);

      std::transform(ending.begin(), ending.end(), ending.begin(), ::tolower);
      if (ending == ".txt")
      {

        Reader myReader;
        //Get the name of the mesh file from the
        //configuration data file.
        myReader.readParameters(fileName, this->dataFileParams_);

      }//end if
      else if (ending == ".xml")
      {

        FileParserXML myReader;

        //Get the name of the mesh file from the
        //configuration data file.
        myReader.parseDataXML(this->dataFileParams_, fileName);

      }//end if
      else
      {
        std::cerr << "Invalid data file ending: " << ending << std::endl;
        exit(1);
      }//end else

      if (hasMeshFile_)
      {
        std::string fileName;
        grid_.initMeshFromFile(fileName.c_str());
      }
      else
      {
        if (dataFileParams_.hasExtents_)
        {
          grid_.initCube(dataFileParams_.extents_[0], dataFileParams_.extents_[2],
                         dataFileParams_.extents_[4], dataFileParams_.extents_[1],
                         dataFileParams_.extents_[3], dataFileParams_.extents_[5]);
        }
        else
          grid_.initCube(xmin_, ymin_, zmin_, xmax_, ymax_, zmax_);
      }

      //initialize rigid body parameters and
      //placement in the domain
      configureRigidBodies();

      configureBoundary();

      //assign the rigid body ids
      for (unsigned j = 0; j < myWorld_.rigidBodies_.size(); j++)
      {
        myWorld_.rigidBodies_[j]->iID_ = j;
        if (myWorld_.rigidBodies_[j]->shapeId_ == RigidBody::COMPOUND)
        {
          CompoundBody *c = dynamic_cast<CompoundBody*>(myWorld_.rigidBodies_[j]);
          for (unsigned i = 0; i < c->rigidBodies_.size(); i++)
          {
            c->rigidBodies_[i]->iID_ = j;
          }
        }
      }
      configureTimeDiscretization();

      //link the boundary to the world
      myWorld_.setBoundary(&myBoundary_);

      //set the time control
      myWorld_.setTimeControl(&myTimeControl_);

      //set the gravity
      myWorld_.setGravity(dataFileParams_.gravity_);

      //set air friction
      myWorld_.setAirFriction(dataFileParams_.airFriction_);

      collPipeline_.world_ = &myWorld_;

      //Set the collision epsilon
      collPipeline_.setEPS(0.02);

      //set which type of rigid motion we are dealing with
      myMotion_ = new RigidBodyMotion(&myWorld_);

      myWorld_.densityMedium_ = dataFileParams_.densityMedium_;

      myWorld_.liquidSolid_ = dataFileParams_.liquidSolid_;

      myWorld_.graph_ = myPipeline_.graph_;

    }

    void run()
    {
      for (int i(0); i <= 300; ++i)
      {

        collPipeline_.startPipeline();

        outputVTK(i);
        printf("Time: %f |Step: %d |\n",simTime, i);
        simTime += dt;
      }
    }

    void outputVTK(int istep)
    {

      using namespace std;

      std::ostringstream sName;

      sName << "output/model." << std::setw(5) << std::setfill('0') << istep << ".vtk";
      std::string strFileName(sName.str());

      CVtkWriter writer;

      writer.WriteODE2VTK(myWorld_.bodies_, strFileName.c_str());

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
