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
#include <string>
#include <fstream>
#include <algorithm>
#include <regex>
#ifdef FC_CUDA_SUPPORT
#endif

namespace i3d {

 
  class MultiBlockGrid : public Application<> {
  public:  
    
  UnstructuredGrid<Real, DTraits> fish_;

  std::string projectFile;
  std::string projectPath;

  MultiBlockGrid() : Application()
  {
        
  }
  
  virtual ~MultiBlockGrid() {};

  void writeOutput(int out, bool writeRBCom, bool writeRBSpheres)
  {
    CVtkWriter writer;

    if (out == 0 || out ==1)
    {
      std::ostringstream sNameGrid;
      writer.WriteUnstrXML(grid_, std::string(projectPath + "grid.vtu").c_str());
      writer.WriteUnstrFacesXML(grid_, projectPath.c_str());
      writer.writeVtkMultiBlockFile(grid_,
          std::string(projectPath + "multiblockdatafile.vtm").c_str());
    }
  }
  
  void init(std::string fileName)
  {

  }
  
  void run()
  {

    std::ifstream file(projectFile.c_str());

    if(!file.good())
    {
      std::cout << "Cannot open file " << projectFile << ".\n";
      std::exit(EXIT_FAILURE);
    }

    size_t pos1 = projectFile.find_last_of("/");

    if(pos1 == std::string::npos)
    {
      std::cout << projectFile << " does not seem to be a valid path." << std::endl;
      std::exit(EXIT_FAILURE);
    }

    std::string path(projectFile.substr(0,pos1+1));

    projectPath = path;

    std::vector<std::string> parFiles;
    std::string str;

    int line = 0;
    while(std::getline(file, str))
    {

      std::cout << "line " << line++ << " " << str << std::endl;
      if(str.empty())
        continue;

      std::string sFile(str);
      sFile = std::regex_replace(sFile, std::regex("^ +| +$|( ) +"), "$1");

      size_t pos = sFile.find_last_of(".");

      std::string sType = sFile.substr(pos,4);

      if(sType == ".tri")
      {
        std::cout << sFile << std::endl;
        std::string meshFile(path);
        meshFile.append(sFile);
        std::string fileName;
        grid_.initMeshFromFile(meshFile.c_str());
        grid_.initStdMesh();
      }//end if
      else if(sType.compare(std::string(".par"))==0)
      {
        std::cout << sFile << std::endl;
        parFiles.push_back(sFile);
      }//end if
      else
      {
        std::cout << "skipping: " << sType << " " << str << std::endl;
      }//end else
    }

    file.close();

    std::vector<ParFileInfo> parFileList;
    for(auto parFileString : parFiles)
    {
      std::string parFileName(path);
      parFileName.append(parFileString.c_str());
      std::ifstream parfile(parFileName.c_str());

      if(!parfile.good())
      {
        std::cout << "Cannot open file " << parFileName << ".\n";
        std::exit(EXIT_FAILURE);
      }

      ParFileInfo myInfo;
      myInfo.glob2Loc_ = std::vector<int>(grid_.nvt_, -1);

      size_t pos = parFileString.find_last_of(".");

      myInfo.name_ = parFileString.substr(0,pos);
      std::cout << "File name wo ending: " << myInfo.name_ << std::endl;

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
      std::cout << "size: " << myInfo.glob2Loc_.size() << std::endl;
      for(int j(0); j < grid_.nvt_; ++j)
      {
        std::vector<int>::iterator low;
        int ffval = j+1;
        low = std::lower_bound(myInfo.nodes_.begin(), myInfo.nodes_.end(),ffval);
        if(low != myInfo.nodes_.end() && *low == ffval)
        {
          myInfo.glob2Loc_[j] = std::distance(myInfo.nodes_.begin(),low);
        }
      }

      int _size = 0;
      for(auto val : myInfo.glob2Loc_)
      {
        if(val != -1)
          _size++;
      }

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

        if(low == xx.nodes_.end() || *low != i0)
        {
          found = found & false;
          continue;
        }
        else
          low0 = *low;

        low = std::lower_bound(xx.nodes_.begin(), xx.nodes_.end(),i1);

        if(low == xx.nodes_.end() || *low != i1)
        {
          found = found & false;
          continue;
        }
        else
          low1 = *low;

        low = std::lower_bound(xx.nodes_.begin(), xx.nodes_.end(),i2);

        if(low == xx.nodes_.end() || *low != i2)
        {
          found = found & false;
          continue;
        }
        else
          low2 = *low;

        low = std::lower_bound(xx.nodes_.begin(), xx.nodes_.end(),i3);

        if(low == xx.nodes_.end() || *low != i3)
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

  }
    
};
  
}

using namespace i3d;

void print_help()
{
  std::string message
  (
   "usage: prj2vtm [--help] [--prj-file] pathtoprjfile\n"
   "\n"
   "--help      : print this message\n"
   "--prj-file: specify the path to the project file\n"
   );
  std::cout << message << std::endl;
}

int main(int argc, char *argv[])
{

  if(argc == 1)
  {
      print_help();
      std::exit(EXIT_FAILURE);
  }

  if(argc < 2)
  {
    std::string s(argv[1]);
    if(s.compare(std::string("--help"))==0)
    {
      print_help();
      std::exit(EXIT_FAILURE);
    }
    else
    {
      print_help();
      std::exit(EXIT_FAILURE);
    }
  }

  if(argc < 3)
  {
    print_help();
    std::exit(EXIT_FAILURE);
  }

  MultiBlockGrid myApp;

  bool option_found = false;

  int pos = 1;
  while(pos < argc)
  {
    std::string s(argv[pos]);
    std::cout << s << std::endl;

    if(s.compare(std::string("--prj-file"))==0)
    {
      ++pos;
      myApp.projectFile = std::string(argv[pos]);
      option_found = true;
    }

    ++pos;
  };

  if(!option_found)
  {
    std::cout << "No project file given." << std::endl;
    print_help();
    std::exit(EXIT_FAILURE);
  }
  
  //myApp.init("start/sampleRigidBody.xml");
  
  myApp.run();
  
  return 0;
}


