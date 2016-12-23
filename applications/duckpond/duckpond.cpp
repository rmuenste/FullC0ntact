#include <iostream>
#include <application.h>
#include <reader.h>
#include <paramline.h>
#include <softbody.hpp>

namespace i3d {

  class DuckPond : public Application {

  public:

    SoftBody<Real, ParamLine<Real>[2]> bull;

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
      
      if(dataFileParams_.hasExtents_)
      {
        cout << "domain extents = " << dataFileParams_.extents_[0] << " " << dataFileParams_.extents_[1] << " " << dataFileParams_.extents_[2] << " "
                                    << dataFileParams_.extents_[3] << " " << dataFileParams_.extents_[4] << " " << dataFileParams_.extents_[5] << endl;
      }
      
      if(dataFileParams_.bodies_ > 0)
      {
        cout<<"type = "<< dataFileParams_.rigidBodies_[0].shapeId_ <<endl; 
        
        cout<<"position = "<< dataFileParams_.rigidBodies_[0].com_ <<endl; 
        
        cout<<"velocity = "<< dataFileParams_.rigidBodies_[0].velocity_ <<endl; 
        
        cout<<"density = "<< dataFileParams_.rigidBodies_[0].density_ <<endl;
        
        cout<<"meshfile = "<< dataFileParams_.rigidBodies_[0].fileName_ <<endl;       
      }

      bull.init();
      std::ostringstream name;
      std::ostringstream name2;
      int step = 0;
      name << "output/line." << std::setfill('0') << std::setw(5) << step << ".vtk";
      name2 << "output/head." << std::setfill('0') << std::setw(5) << step << ".vtk";
      CVtkWriter writer;
      writer.WriteParamLine(bull.geom_[0], name.str().c_str());
      writer.WriteParamLine(bull.geom_[1], name2.str().c_str());

    }

    void run()
    {
      const double pi = 3.1415926535897;

      int istep = 0;

      Real t  = 0.0;
      Real dt = 0.001;

      CVtkWriter writer;

      for(istep=1; t < 2500.0; istep++)
      {
        t+=dt;
        bull.internalForce(t); 
        bull.integrate();
        if(istep%1000==0)
        {
          std::ostringstream name2;
          std::ostringstream name3;
          name2 << "output/line." << std::setfill('0') << std::setw(5) << istep << ".vtk";
          name3 << "output/head." << std::setfill('0') << std::setw(5) << istep << ".vtk";
          writer.WriteParamLine(bull.geom_[0], name2.str().c_str());
          writer.WriteParamLine(bull.geom_[1], name3.str().c_str());
        }
      }
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
