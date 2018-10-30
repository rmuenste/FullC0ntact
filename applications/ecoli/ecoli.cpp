#include <iostream>
#include <application.h>
#include <reader.h>
#include <paramline.h>
#include <softbody.hpp>

namespace i3d {

  class SoftBodyApp : public Application<> {

  public:

    SoftBodyApp() : Application() {

    }

    void init(std::string fileName) {

      using namespace std;

      FileParserXML myReader;

      //Get the name of the mesh file from the
      //configuration data file.
      myReader.parseDataXML(this->dataFileParams_, fileName);

      grid_.initCube(dataFileParams_.extents_[0], dataFileParams_.extents_[2],
                     dataFileParams_.extents_[4], dataFileParams_.extents_[1],
                     dataFileParams_.extents_[3], dataFileParams_.extents_[5]);

    }

    void run()
    {

      Real rho = 2.0;

      std::cout<<"Generating std mesh"<<std::endl;

      grid_.initStdMesh();

      ElementIter e_it, e_end;

      e_end = grid_.elem_end();
      
      for (e_it = grid_.elem_begin(); e_it != e_end; e_it++) {

        Hexa &h = *e_it;
        int id = e_it.idx();

        Real elemVol = grid_.elemVol(id);
        std::cout << "Processing element: " << id << std::endl;
        std::cout << "Volume of element[" << id << "] = " << grid_.elemVol(id) << std::endl;

        CUnstrGrid::VertElemIter ve_it, ve_end;

        ve_end = grid_.VertElemEnd(&h);

        for (ve_it = grid_.VertElemBegin(&h); ve_it != ve_end; ve_it++) {

          int vidx = ve_it.GetInt();
          int elementsAtVertex = grid_.elementsAtVertexIdx_[vidx+1] - grid_.elementsAtVertexIdx_[vidx];
          std::cout << "Processing vertex: " << ve_it.GetInt() << std::endl;
          std::cout << "Number of elements at vertex " << ve_it.GetInt() << " : " << elementsAtVertex << std::endl;

          Real mass = 0.0;
          for (int j = grid_.elementsAtVertexIdx_[vidx], ive = 0; ive < elementsAtVertex; ++ive, ++j) {
            mass += (rho * elemVol) / 8.0;
          }

          std::cout << "Mass of vertex " << ve_it.GetInt() << " : " << mass << std::endl;

        }

      }

    }

  };
}

using namespace i3d;

int main()
{
  
  SoftBodyApp myApp;

  myApp.init("start/sampleRigidBody.xml");

  myApp.run();

  return EXIT_SUCCESS;
}
