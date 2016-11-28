#include <iostream>
#include <application.h>
#include <reader.h>
#include <paramline.h>

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

    }

    void run() {
      const double pi = 3.1415926535897;
      ParamLine<Real> pl;
      int N_tail = 100;
      pl.vertices_.push_back(Vec3(0,0,0));
      Real l0 = 0.5;
      Real A  = 3.2;
      Real dt = 0.025;
      Real t  = 0.0;
      Real fs = 1.0/120.0;

      Real q  = (4.0 * pi)/(0.5 * Real(N_tail));

      int istep = 0;

      for(int i(1); i < N_tail; ++i)
      {
        Real x = Real(i) * l0; 
        // -2.0 * pi * fs * t + q * x < 9/4 * pi
        // -2.0 * pi * fs * t + q * x >= 9/4 * pi
        Real xl = -2.0 * pi * fs * t + q * x;
        A = 3.2;
  //      if(xl < (9.0/4.0) * pi)
  //      {
  //        A = (1.0 - xl/(9.0/4.0)) * 1.5 + (xl/(9.0/4.0)) * 3.2;
  //      }
        Real y = A * std::sin(-2.0 * pi * fs * t + q * x);
        pl.vertices_.push_back(Vec3(x,y,0)); 
        pl.segments_.push_back(Segment3<Real>(pl.vertices_[i-1], pl.vertices_[i]));
        pl.faces_.push_back(std::pair<int,int>(i-1,i));
      }

      std::ostringstream name;
      name << "output/line." << std::setfill('0') << std::setw(5) << istep << ".vtk";
      CVtkWriter writer;
      writer.WriteParamLine(pl, name.str().c_str());
      
      for(istep=1; istep < 10000; istep++)
      {
        t+=dt;
        for(int i(0); i < N_tail; ++i)
        {
          Real x = Real(i) * l0; 
          Real xl = -2.0 * pi * fs * t + q * x;
          A = 3.2;
//          if(xl < (9.0/4.0) * pi)
//          {
//            A = (1.0 - xl/(9.0/4.0)) * 1.5 + (xl/(9.0/4.0)) * 3.2;
//          }
          Real y = A * std::sin(-2.0 * pi * fs * t + q * x);
          pl.vertices_[i]=Vec3(x,y,0); 
        }
        if(istep%100==0)
        {
          std::ostringstream name2;
          name2 << "output/line." << std::setfill('0') << std::setw(5) << istep << ".vtk";
          writer.WriteParamLine(pl, name2.str().c_str());
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
