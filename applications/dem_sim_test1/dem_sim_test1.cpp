#include <iostream>
#include <application.h>
#include <reader.h>
#include <motionintegratorsi.h>
#include <motionintegratordem.h>

namespace i3d {

  class DemSimTest1 : public Application<> {

  public:

    DemSimTest1() : Application() {

    }

    void init(std::string fileName) {

      xmin_ = -2.5f;
      ymin_ = -2.5f;
      zmin_ = -4.5f;
      xmax_ = 2.5f;
      ymax_ = 2.5f;
      zmax_ = 1.5f;

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

     //configureBoundary();
     configureHollowCylinderBoundary();

      //assign the rigid body ids
      for (int j = 0; j < myWorld_.rigidBodies_.size(); j++)
      {
        myWorld_.rigidBodies_[j]->iID_ = j;
        if (myWorld_.rigidBodies_[j]->shapeId_ == RigidBody::COMPOUND)
        {
          CompoundBody *c = dynamic_cast<CompoundBody*>(myWorld_.rigidBodies_[j]);
          for (int i = 0; i < c->rigidBodies_.size(); i++)
          {
            c->rigidBodies_[i]->iID_ = j;
          }
        }
        if(myWorld_.rigidBodies_[j]->shapeId_ == RigidBody::HOLLOWCYLINDER)
        {
          //myWorld_.rigidBodies_[j]->setAngVel(VECTOR3(0,0,3.14));
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

      //Set the collision epsilon
      myPipeline_.setEPS(0.02);

      //initialize the collision pipeline
      myPipeline_.init(&myWorld_, dataFileParams_.solverType_, dataFileParams_.maxIterations_, dataFileParams_.pipelineIterations_);

      //set the broad phase to simple spatialhashing
      myPipeline_.setBroadPhaseHSpatialHash();

      if (dataFileParams_.solverType_ == 2)
      {
        //set which type of rigid motion we are dealing with
        myMotion_ = new MotionIntegratorSI(&myWorld_);
      }
      else if (dataFileParams_.solverType_ == 4)
      {
      //myMotion_ = new MotionIntegratorDEM(&myWorld_);
      myMotion_ = new MotionIntegratorDEM(&myWorld_);
      }
      else
      {
        //set which type of rigid motion we are dealing with
        myMotion_ = new RigidBodyMotion(&myWorld_);
      }

      //set the integrator in the pipeline
      myPipeline_.integrator_ = myMotion_;

      myWorld_.densityMedium_ = dataFileParams_.densityMedium_;

      myWorld_.liquidSolid_ = dataFileParams_.liquidSolid_;

      myPipeline_.response_->m_pGraph = myPipeline_.graph_;

      myWorld_.graph_ = myPipeline_.graph_;


    }

    void run() {

      unsigned nOut = 0;
      myWorld_.extGraph_ = true;
      writeOutput(5000,false,true);
      //start the main simulation loop
      for (; myWorld_.timeControl_->m_iTimeStep <= dataFileParams_.nTimesteps_; myWorld_.timeControl_->m_iTimeStep++)
      {
        Real simTime = myTimeControl_.GetTime();
        Real energy0 = myWorld_.getTotalEnergy();
        std::cout << "------------------------------------------------------------------------" << std::endl;
        std::cout << "## Timestep Nr.: " << myWorld_.timeControl_->m_iTimeStep << " | Simulation time: " << myTimeControl_.GetTime()
          << " | time step: " << myTimeControl_.GetDeltaT() << std::endl;
        std::cout << "Energy: " << energy0 << std::endl;
        std::cout << "------------------------------------------------------------------------" << std::endl;
        std::cout << std::endl;
        int itime=myWorld_.timeControl_->m_iTimeStep;
        myPipeline_.startPipeline();
        Real energy1 = myWorld_.getTotalEnergy();
        std::cout << "Energy after collision: " << energy1 << std::endl;
        std::cout << "Energy difference: " << energy0 - energy1 << std::endl;
        std::cout << "Timestep finished... writing vtk." << std::endl;
        std::cout << "Size World: "<<myWorld_.rigidBodies_.size()<< std::endl;

        //if(nOut%100==0)
          Application::writeOutput(nOut,false,true);
        
        std::cout << "Finished writing vtk." << std::endl;
        nOut++;
        myTimeControl_.SetTime(simTime + myTimeControl_.GetDeltaT());
      }//end for

    }

  };

}

using namespace i3d;

int main()
{

  DemSimTest1 myApp;

  myApp.init("start/rigidbody.xml");

  myApp.run();

  return 0;

}
