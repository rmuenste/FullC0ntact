#include <iostream>
#include <cstdlib>
#include <application.h>
#include <reader.h>
#include <paramline.h>
#include <softbody.hpp>
#include <mymath.h>
#include <algorithm>
#include <iterator>

#include "globals.hpp"

#include "general_definitions.hpp"
#include "mesh_creation.hpp"
#include "pbdbody.hpp"

namespace i3d {

#include "constraints.hpp"
  
  class PositionBasedDynamicsApp : public Application<> {
  /*
  In this unit test we test the PBD kernel DistanceConstraint>:
  - A simple mesh consisting of two triangles and corresponding distance constraints are generated
  - Then the distance constraint is evaluated and should return 0
  - We then move a vertex and reevaluate the distance constraint, which should now give a non-zero value
  */
  public:

    MyMesh mesh_;
    PBDBody body_;

    PositionBasedDynamicsApp() : Application() {
    }

    void writeOFFMesh(MyMesh &mesh) {

      try {
        if (!OpenMesh::IO::write_mesh(mesh, "output2.off")) {
          std::cerr << "Cannot write mesh to file 'output.off'" << std::endl;
          std::exit(EXIT_FAILURE);
        }
        std::cout << "Mesh written to 'output.off'" << std::endl;
      }
      catch( std::exception &x) {
        std::cerr << x.what() << std::endl;
        std::exit(EXIT_FAILURE);
      }

    }

    void init(std::string fileName) {

      mesh_ = generateSimpleMesh();
      body_.constraints_ = generateBendingConstraints(mesh_);

      body_.mesh_ = &mesh_;

      writeOFFMesh(mesh_);

    }

    void updateDistanceConstraints() {
      typedef MyMesh::Point Point;
      typedef OpenMesh::VectorT<double, 3> V3;

      auto e_end = mesh_.edges_end();

      std::cout << "Distance Constraint correction: " << std::endl;
      for (auto e_it = mesh_.edges_begin(); e_it != e_end; ++e_it) {

        VHandle vh0 = mesh_.to_vertex_handle(mesh_.halfedge_handle(*e_it, 0));
        VHandle vh1 = mesh_.to_vertex_handle(mesh_.halfedge_handle(*e_it, 1));

        Point p0 = mesh_.point(vh0);
        Point p1 = mesh_.point(vh1);

        V3 v0(p0[0], p0[1], p0[2]);
        V3 v1(p1[0], p1[1], p1[2]);

        OpenMesh::VectorT<double, 3> dP = mesh_.data(*e_it).dc_.computeCorrection(v0, v1);
        std::cout << "<" << *e_it << ">" << dP[0] << " " << dP[1] << " " << dP[2] << std::endl;
      }

    }

    void manipulateVertex() {
      typedef MyMesh::Point Point;
      VHandle vh0 = mesh_.vertex_handle(0);
      Point p0 = mesh_.point(vh0);
      std::cout << "<" << p0 << std::endl;
      p0[0] = -4.0;
      mesh_.set_point(vh0, p0);
      
    }

    void run()
    {
      updateDistanceConstraints();
      manipulateVertex();
      updateDistanceConstraints();
    }
  };

} //end namespace

using namespace i3d;

int main()
{
  PositionBasedDynamicsApp myApp;

  myApp.init("start/sampleRigidBody.xml");

  myApp.run();
  
  return 0;
}
