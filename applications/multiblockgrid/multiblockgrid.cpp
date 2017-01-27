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
#include <string>
#include <fstream>
#include <algorithm>
#ifdef FC_CUDA_SUPPORT
#endif

namespace i3d {
 
  class MultiBlockGrid : public Application {
  public:  
    
  UnstructuredGrid<Real, DTraits> fish_;

  MultiBlockGrid() : Application()
  {
        
  }
  
  virtual ~MultiBlockGrid() {};

  void writeOutput(int out, bool writeRBCom, bool writeRBSpheres)
  {
    CVtkWriter writer;
    int iTimestep = out;


    if (out == 0 || out ==1)
    {
      std::ostringstream sNameGrid;
      std::string sGrid("output/grid.vtk");
      sNameGrid << "." << std::setfill('0') << std::setw(5) << iTimestep;
      sGrid.append(sNameGrid.str());
      writer.WriteUnstr(grid_, sGrid.c_str());
      writer.WriteUnstrXML(grid_, "output/grid.vtu");
      //writer.WriteUnstrFaces(grid_, "output/faces.vtk");
      writer.WriteUnstrFacesXML(grid_, "output/");
      writer.writeVtkMultiBlockFile(grid_, "output/multiblockdatafile.vtm");
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

    std::ifstream file("meshes/FineCoredBOX/rbox.prj");
    std::string str;

    std::vector<std::string> parFiles;

    while(std::getline(file, str))
    {

      if(str.empty())
        continue;

      std::string sFile(str);

      size_t pos = sFile.find_last_of(".");

      std::string sType = sFile.substr(pos);

      if(sType == ".tri")
      {
        std::cout << sFile << std::endl;
        std::string meshFile("meshes/FineCoredBOX/");
        meshFile.append(sFile);
        std::string fileName;
        grid_.initMeshFromFile(meshFile.c_str());
        grid_.initStdMesh();
      }//end if
      else if(sType == ".par")
      {
        std::cout << sFile << std::endl;
        parFiles.push_back(sFile);
      }//end if
      else
      {
      }//end else
    }

    file.close();

    std::vector<ParFileInfo> parFileList;
    for(auto parFileString : parFiles)
    {
      std::string parFileName("meshes/FineCoredBOX/");
      parFileName.append(parFileString.c_str());
      std::ifstream parfile(parFileName.c_str());

      ParFileInfo myInfo;
      parfile >> myInfo.size_; 
      std::cout << "number of nodes: " << myInfo.size_ << std::endl;

      parfile >> myInfo.boundaryType_; 

      std::cout << "Boundary type: " << myInfo.boundaryType_ << std::endl;

      std::getline(parfile, str);

      if(std::getline(parfile, str))
      {
        myInfo.expression_ = str;
        std::cout << "Expression: " << myInfo.expression_ << std::endl;
      }

      while(std::getline(parfile, str))
      {
        if(!str.empty())
        {
          myInfo.nodes_.push_back(std::stoi(str));
        }
      }

      parfile.close();

      std::sort(myInfo.nodes_.begin(), myInfo.nodes_.end());

      std::cout << "Nodes list size: " << myInfo.nodes_.size() << std::endl;
      parFileList.push_back(myInfo);
    }
    std::cout << "Number of param files: " << parFileList.size() << std::endl;

    grid_.parInfo_ = &parFileList;
    for(auto &parList : parFileList)
    {
      ParFileInfo &xx = parList;

      for(unsigned i(0); i < grid_.facesAtBoundary_.size(); ++i)
      {
        int faceindex = grid_.facesAtBoundary_[i];
        int i0 = grid_.verticesAtFace_[faceindex].faceVertexIndices_[0]+1;
        int i1 = grid_.verticesAtFace_[faceindex].faceVertexIndices_[1]+1;
        int i2 = grid_.verticesAtFace_[faceindex].faceVertexIndices_[2]+1;
        int i3 = grid_.verticesAtFace_[faceindex].faceVertexIndices_[3]+1;
        std::vector<int>::iterator low;
        int low0;
        int low1;
        int low2;
        int low3;
        bool found = true;
        low = std::lower_bound(xx.nodes_.begin(), xx.nodes_.end(),i0);

        if(*low != i0)
        {
          found = found & false;
          continue;
        }
        else
          low0 = *low;

        low = std::lower_bound(xx.nodes_.begin(), xx.nodes_.end(),i1);

        if(*low != i1)
        {
          found = found & false;
          continue;
        }
        else
          low1 = *low;

        low = std::lower_bound(xx.nodes_.begin(), xx.nodes_.end(),i2);

        if(*low != i2)
        {
          found = found & false;
          continue;
        }
        else
          low2 = *low;

        low = std::lower_bound(xx.nodes_.begin(), xx.nodes_.end(),i3);

        if(*low != i3)
        {
          found = found & false;
          continue;
        }
        else
          low3 = *low;

        if(found)
        {
//          std::cout << "Found x boundary face: " << faceindex << std::endl;
//          std::cout << low0 << " target: " << i0 << std::endl;
//          std::cout << low1 << " target: " << i1 << std::endl;
//          std::cout << low2 << " target: " << i2 << std::endl;
//          std::cout << low3 << " target: " << i3 << std::endl;
          HexaFace f;
          f.faceVertexIndices_[0] = i0-1;
          f.faceVertexIndices_[1] = i1-1;
          f.faceVertexIndices_[2] = i2-1;
          f.faceVertexIndices_[3] = i3-1;
          xx.verticesAtFace_.push_back(f);
          xx.faces_.push_back(faceindex);
        }
         
      }

    }

    writeOutput(0, false, false);    

    writeOutput(1, false, false);

  }
    
};
  
}

using namespace i3d;


int main()
{

  MultiBlockGrid myApp;
  
  //myApp.init("start/sampleRigidBody.xml");
  
  myApp.run();
  
  return 0;
}


//    grid_.initStdMesh();
//    for(int i=0;i<3;i++)
//    {
//      grid_.refine();
//      std::cout<<"Generating Grid level"<<i+1<<std::endl;
//      std::cout<<"---------------------"<<std::endl;
//      std::cout<<"NVT="<<grid_.nvt_<<" NEL="<<grid_.nel_<<std::endl;
//      grid_.initStdMesh();
//    }       

