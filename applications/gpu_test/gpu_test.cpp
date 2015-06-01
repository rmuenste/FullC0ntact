#include <iostream>
#include <application.h>
#include <reader.h>
#include <motionintegratorsi.h>
#include <meshobject.h>
#include <distancemeshpoint.h>
#include <laplace.h>
#include <common.h>
#include <3dmodel.h>

namespace i3d {

  class GPUTest : public Application {

  public:

    GPUTest() : Application() {

    }

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
      for (int j = 0; j<myWorld_.rigidBodies_.size(); j++)
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

    }

    void run() {
    
      RigidBody *body;
      if (!myWorld_.rigidBodies_.empty())
      {
        body = myWorld_.rigidBodies_.front();
      }
      else
      {
        exit(1);
      }

      CMeshObject<Real> *meshObject = dynamic_cast< CMeshObject<Real> *>(body->shape_);
      C3DModel *model = &meshObject->m_Model;

      my_cuda_func(model);
      writeOutput(0);
      writeOutput(1);
      cleanGPU();

    }

  };

}

using namespace i3d;



int main()
{

  GPUTest myApp;

  myApp.init("start/sampleRigidBody.xml");

  myApp.run();

  return 0;
}
