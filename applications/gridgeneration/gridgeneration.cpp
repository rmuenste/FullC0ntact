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
    //std::cout << "Configure rigid bodies." << std::endl;

    grid_.initCube(-12.0, -12.0, -6.0, 12.0, 12.0, 7.0);

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
    
    std::cout<<"Generating std mesh"<<std::endl;

    grid_.initStdMesh();

    int level = 7;

    for ( int i=0; i < level; i++)
    {
      grid_.refine();
      std::cout<<"Generating Grid level"<<i+1<<std::endl;
      std::cout<<"---------------------"<<std::endl;
      std::cout<<"NVT="<<grid_.nvt_<<" NEL="<<grid_.nel_<<std::endl;
      grid_.initStdMesh();
    }       

    std::cout<<"> Computing FBM information..."<<std::endl;

    RigidBody *body = myWorld_.rigidBodies_[0];

    VertexIter<Real> v_it, v_end;
    v_end = grid_.vertices_end();
    for(v_it = grid_.vertices_begin(); v_it != v_end; v_it++)
    {
      Vec3 v(*v_it);
      int id = v_it.idx();
      grid_.m_myTraits[id].iTag=2;

      if(body->isInBody(v))
      {
        grid_.m_myTraits[id].iTag=1;
      }
      else
      {
        grid_.m_myTraits[id].iTag=0;
      }
    }

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
