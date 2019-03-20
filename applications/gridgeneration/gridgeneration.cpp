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
#include <geom_config.hpp>
#include <distancegridcgal.hpp>

namespace i3d {
 
  class GridGeneration : public Application<> {
  public:  
    
  GridGeneration() : Application()
  {
        
  }
  
  virtual ~GridGeneration() {};

  void writeOutput(int out)
  {
    std::ostringstream sName, sNameParticles, sphereFile;
    std::string sModel("output/model.vtk");

    CVtkWriter writer;
    int iTimestep = out;
    sName << "." << std::setfill('0') << std::setw(5) << iTimestep;
    sModel.append(sName.str());

    std::cout << "Writing VTK surface mesh to: " << sModel.c_str() << std::endl;
    //Write the grid to a file and measure the time
    writer.WriteRigidBodies(myWorld_.rigidBodies_, sModel.c_str());

    if (out == 0 || out ==1)
    {
      std::ostringstream sNameGrid;
      std::string sGrid("output/grid.vtk");
      sNameGrid << "." << std::setfill('0') << std::setw(5) << iTimestep;
      sGrid.append(sNameGrid.str());
      std::cout << "Writing VTK mesh to: " << sGrid.c_str() << std::endl;
      writer.WriteUnstr(grid_, sGrid.c_str());
    }

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
      std::exit(EXIT_FAILURE);
    }//end else

    //initialize rigid body parameters and
    //placement in the domain
    configureRigidBodies();

    grid_.initCube(-1.0, -1.0, -1.0, 1.0, 1.0, 1.0);

    configureBoundary();

    //assign the rigid body ids
    for (int j = 0; j<myWorld_.rigidBodies_.size(); j++)
      myWorld_.rigidBodies_[j]->iID_ = j;

    //Distance map initialization
    std::set<std::string> fileNames;

    for (auto &body : myWorld_.rigidBodies_)
    {
      if (!(body->shapeId_ == RigidBody::MESH || body->shapeId_ == RigidBody::CGALMESH))
        continue;

      MeshObject<Real, geom_kernel> *meshObject = dynamic_cast<MeshObject<Real, geom_kernel> *>(body->shape_);
      std::string objName = meshObject->getFileName();
      fileNames.insert(objName);
    }

    int iHandle=0;
    for (auto const &myName : fileNames)
    {
      bool created = false;
      for (auto &body : myWorld_.rigidBodies_)
      {

        if (!(body->shapeId_ == RigidBody::MESH || body->shapeId_ == RigidBody::CGALMESH))
          continue;

        MeshObject<Real, geom_kernel> *pMeshObject = dynamic_cast<MeshObject<Real, geom_kernel> *>(body->shape_);

        //pMeshObject->m_BVH.GenTreeStatistics();

        std::string objName = pMeshObject->getFileName();
        if (objName == myName)
        {
          if (created)
          {
            //if map created -> add reference
            body->map_ = myWorld_.maps_.back();
          }
          else
          {
            //if map not created -> create and add reference

            std::cout << "Creating distance map" <<std::endl;
            body->buildDistanceMap();
            myWorld_.maps_.push_back(body->map_);
            created = true;
            CVtkWriter writer;
            std::string n = myName;
            const size_t last = n.find_last_of("\\/");
            if(std::string::npos != last)
            {
              n.erase(0,last);
            }
            const size_t period = n.rfind(".");
            if(std::string::npos != period)
            {
              n.erase(period);
            }
            n.append(".ps");
            std::string dir("output/");
            dir.append(n);
//                writer.writePostScriptTree(pMeshObject->m_BVH,dir.c_str());
          }
        }
      }
    }

    std::cout << "Number of distance maps: " << myWorld_.maps_.size() << std::endl;
    std::cout << "Geometry kernel: " << geom_kernel << std::endl;
    std::cout << "Distance map created..." << std::endl;

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

    CUnstrGridr ugrid;
    for (auto &body : myWorld_.rigidBodies_)
    {

      if (!(body->shapeId_ == RigidBody::MESH || body->shapeId_ == RigidBody::CGALMESH))
        continue;
    
      body->map_->convertToUnstructuredGrid(ugrid);

    }

    ugrid.calcVol();

    RigidBody *body = myWorld_.rigidBodies_[0];

    MeshObject<Real, cgalKernel> *object = dynamic_cast< MeshObject<Real, cgalKernel> *>(body->shape_);

    DistanceGridMesh<Real> distance(&ugrid, object);

    distance.ComputeDistance();
    distance.ComputeElementDistance();

    ugrid.decimate();

    ugrid.initStdMesh();

    distance.ComputeDistance();

    ElementIter e_it = ugrid.elem_begin();

    for(; e_it != ugrid.elem_end(); e_it++) {

      int index = e_it.idx();
      //std::cout << index << ")Hexa volume: " << ugrid.elemVol_[index] << std::endl;

      Hexa &hexa = *e_it;

    }
    
    CVtkWriter writer;

    writer.WriteUnstr(ugrid, "output/DistanceMap.vtk");
    writer.WriteGrid2Tri(ugrid, "meshes/dmap.tri");

//    VertexIter<Real> ive;
//    
//    std::cout<<"Generating std mesh"<<std::endl;
//
//    grid_.initStdMesh();
//
//    int level = 4;
//
//    if (dataFileParams_.refinementLevel_ > 0)
//      level = dataFileParams_.refinementLevel_;
//
//
//    for ( int i=0; i < level; i++)
//    {
//      grid_.refine();
//      std::cout<<"Generating Grid level"<<i+1<<std::endl;
//      std::cout<<"---------------------"<<std::endl;
//      std::cout<<"NVT="<<grid_.nvt_<<" NEL="<<grid_.nel_<<std::endl;
//      grid_.initStdMesh();
//    }       
//
//    std::cout<<"> Computing FBM information..."<<std::endl;
//
//    RigidBody *body = myWorld_.rigidBodies_[0];
//
//    VertexIter<Real> v_it, v_end;
//    v_end = grid_.vertices_end();
//    for(v_it = grid_.vertices_begin(); v_it != v_end; v_it++)
//    {
//      Vec3 v(*v_it);
//      int id = v_it.idx();
//      grid_.m_myTraits[id].iTag=2;
//
//      if(body->isInBody(v))
//      {
//        grid_.m_myTraits[id].iTag=1;
//      }
//      else
//      {
//        grid_.m_myTraits[id].iTag=0;
//      }
//    }

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
