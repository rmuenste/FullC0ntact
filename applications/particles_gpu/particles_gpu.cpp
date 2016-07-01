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
#include <difi.cuh>
#include <vtkwriter.h>
#include <termcolor.hpp>
#include <dempipeline.hpp>
#include <cuda.h>
#include <cuda_runtime.h>

namespace i3d {

  class GPUTest : public Application {

  public:
    HashGrid<float, cpu> hg;
    ParticleWorld<float, cpu> pw;
    SimulationParameters<float> params_;
    SimulationParameters<float> *dev_params_;
    CollisionPipeline<dem_gpu> demPipeline;

    GPUTest() : Application() {

    }

    ~GPUTest()
    {
    };

    void deviceInit()
    {
      params_.spring_ = 0.5f;
      params_.damping_ = 0.02f;
      params_.shear_ = 0.1f;
      params_.attraction_ = 0.0f;
      params_.gravity_ = Vector3<float>(0, 0, -0.0003f);
      params_.globalDamping_ = 1.0f;
      params_.boundaryDamping_ = -0.5f;
      params_.particleRadius_ = 1.0f / 64.0f;
      params_.origin_ = Vector3<float>(-1.0f, -1.0f, -1.0f);
      params_.gridx_ = 64;
      params_.gridy_ = 64;
      params_.gridz_ = 64;
      params_.timeStep_ = myTimeControl_.GetDeltaT();
      pw.params_ = &params_;
      hg.size_ = 16384;
      pw.size_ = 16384;

      cuda_init(hg, pw, dev_params_);

      demPipeline.hg = &hg;
      demPipeline.pw = &pw;
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

      std::vector<float> pos_data(4 * pw.size_);
      copy_data(hg,pw,pos_data);
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
      hg.clean();
      pw.clean();
      cuda_clean();
      cudaFree(dev_params_);
    }

    void run() {
      

      for (int i(0); myWorld_.timeControl_->m_iTimeStep <= dataFileParams_.nTimesteps_; ++myWorld_.timeControl_->m_iTimeStep, ++i)
      {
        Real simTime = myTimeControl_.GetTime();
        std::cout << "------------------------------------------------------------------------" << std::endl;

        std::cout << termcolor::green << "## Timestep Nr.: " << myWorld_.timeControl_->m_iTimeStep << " | Simulation time: " << myTimeControl_.GetTime()
          << " | time step: " << myTimeControl_.GetDeltaT() << termcolor::reset << std::endl;

        std::cout << "------------------------------------------------------------------------" << std::endl;
        std::cout << std::endl;
        demPipeline.startPipeline();
        std::cout << "Timestep finished... writing vtk." << std::endl;
        if(i%10==0)
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