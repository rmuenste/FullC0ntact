#include <iostream>
#include <application.h>
#include <reader.h>

namespace i3d {

  class DuckPond : public Application {

  public:

    DuckPond() : Application() {

    }

    void init(std::string fileName) {
      using namespace std;

      xmin_ = -2.5f;
      ymin_ = -2.5f;
      zmin_ = -4.5f;
      xmax_ = 2.5f;
      ymax_ = 2.5f;
      zmax_ = 1.5f;

      FileParserXML myReader;

      //Get the name of the mesh file from the
      //configuration data file.
      myReader.parseDataXML(this->dataFileParams_, fileName);

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

         cout<<"startType = "<<dataFileParams_.startType_<<endl; 
         cout<<"solution = "<<dataFileParams_.solutionFile_<<endl; 
         cout<<"nBodies = "<<dataFileParams_.bodies_<<endl;  
         cout<<"bodyInit = "<<dataFileParams_.bodyInit_<<endl; 
         cout<<"bodyFile = "<<dataFileParams_.bodyConfigurationFile_<<endl; 
         cout<<"defaultDensity = "<<dataFileParams_.defaultDensity_<<endl; 
         cout<<"defaultRadius = "<<dataFileParams_.defaultRadius_<<endl; 
         cout<<"gravity = "<<dataFileParams_.gravity_;  
         cout<<"totalTimesteps = "<<dataFileParams_.nTimesteps_<<endl;
         cout<<"lcpSolverIterations = "<<dataFileParams_.maxIterations_<<endl;
         cout<<"collPipelineIterations = "<<dataFileParams_.pipelineIterations_<<endl;
         cout << "domain extents = " << dataFileParams_.extents_[0] << " " << dataFileParams_.extents_[1] << " " << dataFileParams_.extents_[2] << " "
                                     << dataFileParams_.extents_[3] << " " << dataFileParams_.extents_[4] << " " << dataFileParams_.extents_[5] << endl;



    }

    void run() {


    }

  };

}

using namespace i3d;

int main()
{
  
  DuckPond myApp;

  myApp.init("start/sampleRigidBody.xml");

  myApp.run();

  return 0;
}
