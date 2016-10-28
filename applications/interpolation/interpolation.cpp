#include <iostream>
#include <application.h>
#include <tetrahedron.hpp>

namespace i3d {




  class OpenMeshTest : public Application {

  public:


    OpenMeshTest() : Application() {

    }

    void init(std::string fileName) {

      Application::init(fileName);

    }

    void run() {

      Vector3<Real> a(1,2,3);
      Vector3<Real> b(2,2,3);
      Vector3<Real> c(1,3,3);
      Vector3<Real> d(1,2,9);

      Tetrahedron<Real> t(a,b,c,d);

      CVtkWriter writer;
      writer.WriteTetra(t, "output/tetra.vtk");

      Vector3<Real> p[5] = {Vector3<Real>(1.25,2.25,4.5),
                                 Vector3<Real>(1.00,2.50,4.0),
                                 Vector3<Real>(1.00,3.00,3.0),
                                 Vector3<Real>(1.00,2.00,7.8),
                                 Vector3<Real>(0.80,2.40,6.6)};



      for(int i(0); i < 5; ++i)
      {
//        std::cout << "> Signed distance to face a: " << t.signedDistance(p[i],0) << std::endl;
//        std::cout << "> Signed distance to face b: " << t.signedDistance(p[i],1) << std::endl;
//        std::cout << "> Signed distance to face c: " << t.signedDistance(p[i],2) << std::endl;
//        std::cout << "> Signed distance to face d: " << t.signedDistance(p[i],3) << std::endl;

        Real bc[4];
        t.barycentricCoords(p[i],bc[0],bc[1],bc[2],bc[3]);
        std::cout << "> Barycentric coords of p" << i << ": " << bc[0] << " " << bc[1] << " "<< bc[2] << " "<< bc[3] << " "<< std::endl;
        std::cout << "> ---------------------------" << std::endl;

      }

    }

  };

}

using namespace i3d;

int main()
{
  
  OpenMeshTest myApp;

  myApp.init("start/sampleRigidBody.xml");

  myApp.run();

  return 0;
}
