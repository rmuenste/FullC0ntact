#include <iostream>
#include <application.h>
#include <reader.h>
#include <motionintegratorsi.h>
#include <meshobject.h>
#include <distancemeshpoint.h>
#include <laplace.h>
#include <intersectorray3tri3.h>
#include <perftimer.h>
#include <vtkwriter.h>
#include <termcolor.hpp>
#ifdef FC_CUDA_SUPPORT
#include <difi.cuh>
#include <dempipeline.hpp>
#include <cuda.h>
#include <cuda_runtime.h>
#endif

namespace i3d {
 
  class GridGeneration : public Application {
  public:  
    
  UnstructuredGrid<Real, DTraits> fish_;

  GridGeneration() : Application()
  {
        
  }
  
  virtual ~GridGeneration() {};
  
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

    std::string meshFile("meshes/mesh.tri3d");
    hasMeshFile_ = 1;

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
  
  void run()
  {
    VertexIter<Real> ive;
    std::cout<<"Computing FBM information and distance..."<<std::endl;
    
    grid_.initStdMesh();
    for(int i=0;i<3;i++)
    {
      grid_.refine();
      std::cout<<"Generating Grid level"<<i+1<<std::endl;
      std::cout<<"---------------------"<<std::endl;
      std::cout<<"NVT="<<grid_.nvt_<<" NEL="<<grid_.nel_<<std::endl;
      grid_.initStdMesh();
    }       
    grid_.calcVol();

    std::cout<<"> Grid volume: " << grid_.vol_ <<std::endl;

    fish_.initMeshFromFile("meshes/tryp5.tri3d");
    fish_.initStdMesh();
    fish_.calcVol();

    std::cout<<"> fish volume: " << fish_.vol_ <<std::endl;

    AABB3<Real> box = fish_.getAABB();
    VertexIter<Real> v_it;
    VertexIter<Real> v_end = grid_.vertices_end();
    std::cout<<"> Computing FBM information..."<<std::endl;
    //Vec3 query(10.8,0,0.2);
//    Vec3 query(11.0,0,0.0);

//    if(box.isPointInside(query))
//      std::cout<<"> Inside box..."<<std::endl;
//    else
//      std::cout<<"> Outside box..."<<std::endl;
//
//    if(fish_.pointInsideHexa(250,query))
//      std::cout<<"> Inside..."<<std::endl;
//    else
//      std::cout<<"> Outside..."<<std::endl;

    for(v_it = grid_.vertices_begin(); v_it != v_end; v_it++)
    { 
      
        Vec3 v(*v_it);  
        int id = v_it.idx();

        if(!box.isPointInside(v))
        {
          grid_.m_myTraits[id].iTag=0;
          continue;
        } 
        
        if(fish_.pointInside(v))
        {
          grid_.m_myTraits[id].iTag=1;
        }
        else
        {
          grid_.m_myTraits[id].iTag=0;
        }        

    }


//    Real sliceX = 0.0;
//    for (int j(0); j < 23; ++j)
//    {
//      int vertsInSlice(0);
//      for (ive = grid_.vertices_begin(); ive != grid_.vertices_end(); ive++)
//      {
//        int id = ive.idx();
//        VECTOR3 vQuery((*ive).x, (*ive).y, (*ive).z);
//        if (vQuery.x - sliceX < 1.0e-4)
//        {
//          //std::cout << "> VertexId: " << id << std::endl;
//          vertsInSlice++;
//        }
//      }
//      std::cout << "> Vertices in circular slice " << j + 1 << " : " << vertsInSlice << std::endl;
//    }


    CVtkWriter writer;
    writer.WriteSpringMesh(fish_,"output/fish.vtk");

    writeOutput(0);    

    writeOutput(1);
  }
    
};
  
}

using namespace i3d;


int main()
{

  GridGeneration myApp;
  
  myApp.init("start/sampleRigidBody.xml");
  
  myApp.run();
  
  return 0;
}
