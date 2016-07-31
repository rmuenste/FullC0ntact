#include <iostream>
#include <application.h>
#include <reader.h>
#include <motionintegratorsi.h>
#include <meshobject.h>
#include <distancemeshpoint.h>
#include <laplace.h>
#include <common.h>
#include <3dmodel.h>
#include <intersectorray3tri3.h>
#include <perftimer.h>
#include <vtkwriter.h>
#include <termcolor.hpp>
#include <dempipeline.hpp>
#include <cuda.h>
#include <cuda_runtime.h>
#include <ext_unit_tests.cuh>
#include <particledem_um.cuh>


namespace i3d {

  class GPUTest : public Application {

  public:

    CollisionPipeline<dem_gpu_unified> demPipeline;

    GPUTest() : Application() {

    }

    ~GPUTest()
    {
    };

    void deviceInit()
    {
      cuda_init();
    };

    void init(std::string fileName)
    {

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

      std::string meshFile("meshes/sb.tri");
      hasMeshFile_ = 0;

      if (hasMeshFile_)
      {
        std::string fileName;
        grid_.initMeshFromFile(meshFile.c_str());
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
      for (unsigned j(0); j<myWorld_.rigidBodies_.size(); ++j)
        myWorld_.rigidBodies_[j]->iID_ = j;

      configureTimeDiscretization();

      //link the boundary to the world
      myWorld_.setBoundary(&myBoundary_);

      //set the time control
      myWorld_.setTimeControl(&myTimeControl_);

      //set the gravity
      myWorld_.setGravity(dataFileParams_.gravity_);

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

      deviceInit();

    }

    void writeOutput(int out, bool writeRBCom, bool writeRBSpheres)
    {
      std::ostringstream sName, sNameParticles, sphereFile;
      std::string sModel("output/model.vtk");
      std::string sParticleFile("output/particle.vtk");
      std::string sParticle("solution/particles.i3d");
      CVtkWriter writer;
      int iTimestep = out;
      sName << "." << std::setfill('0') << std::setw(5) << iTimestep;
      sNameParticles << "." << std::setfill('0') << std::setw(5) << iTimestep;
      sParticleFile.append(sNameParticles.str());

      std::vector<float> pos_data;
      transfer_data(pos_data);

      writer.WriteGPUParticleFile(pos_data ,sParticleFile.c_str());

      if (out == 0 || out ==1)
      {
        std::ostringstream sNameGrid;
        std::string sGrid("output/grid.vtk");
        sNameGrid << "." << std::setfill('0') << std::setw(5) << iTimestep;
        sGrid.append(sNameGrid.str());
        writer.WriteUnstr(grid_, sGrid.c_str());
      }
    }

    void clean_gpu()
    {
      
    }

    void run()
    {
      for (int i(0); myWorld_.timeControl_->m_iTimeStep <= dataFileParams_.nTimesteps_; ++myWorld_.timeControl_->m_iTimeStep, ++i)
      {
        Real simTime = myTimeControl_.GetTime();
        std::cout << "------------------------------------------------------------------------" << std::endl;

        std::cout << termcolor::green << "## Timestep Nr.: " << myWorld_.timeControl_->m_iTimeStep << " | Simulation time: " << myTimeControl_.GetTime()
          << " | time step: " << myTimeControl_.GetDeltaT() << termcolor::reset << std::endl;

        std::cout << "------------------------------------------------------------------------" << std::endl;
        std::cout << std::endl;
        CPerfTimer timer;
        timer.Start();
        demPipeline.startPipeline();
        std::cout << "Elapsed time gpu[ms]:" << timer.GetTime() << std::endl;
        std::cout << "Timestep finished... writing vtk." << std::endl;
        if (i % 10 == 0)
          writeOutput(i, true, true);
        std::cout << "Finished writing vtk." << std::endl;
        myTimeControl_.SetTime(simTime + myTimeControl_.GetDeltaT());
      }
    }
  };
}

using namespace i3d;

int main()
{
  GPUTest myApp;

  myApp.init("start/sampleRigidBody.xml");

  myApp.run();
  myApp.clean_gpu();
  cudaDeviceReset();
  
  return 0;
}
