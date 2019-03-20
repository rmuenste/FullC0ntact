#include <iostream>
#include <application.h>
#include <reader.h>
#include <distancemap.h>
#include <meshobject.h>
#include <distancemeshpoint.h>
#include <laplace.h>
#include <smoother.h>

namespace i3d {

  class DistanceGridTest : public Application<> {

  public:

    DistanceGridTest() : Application() {

    }

    void init(std::string fileName) {

      grid_.initMeshFromFile("meshes/dmap.tri");
      
      grid_.initStdMesh();

      grid_.calcVol();

      Smoother<Real> smoother(&grid_, 10);
      smoother.smooth();

//      for(int i=0;i<1;i++)
//      {
//        grid_.refine();
//        std::cout<<"Generating Grid level"<<i+1<<std::endl;
//        std::cout<<"---------------------"<<std::endl;
//        std::cout<<"NVT="<<grid_.nvt_<<" NEL="<<grid_.nel_<<std::endl;
//        grid_.initStdMesh();
//      }       

      CVtkWriter writer;

      writer.WriteUnstr(grid_, "output/mesh.vtk");

      
      
    }

    void run() {

    }

  };

}

using namespace i3d;



int main()
{
  
  DistanceGridTest myApp;

  myApp.init("start/sampleRigidBody.xml");

  myApp.run();

  return 0;
}


