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

      body_.distanceConstraints_ = generateDistanceConstraints(mesh_);

      body_.bendingConstraints_ = generateBendingConstraints(mesh_);

      body_.mesh_ = &mesh_;

      writeOFFMesh(mesh_);

    }

    void updateDistanceConstraints() {

      typedef MyMesh::Point Point;
      typedef OpenMesh::VectorT<double, 3> V3;

      auto e_end = mesh_.edges_end();

      std::cout << "Distance Constraint correction: " << std::endl;

      for (auto& constraint : body_.distanceConstraints_) {
        Point p0 =mesh_.point(VHandle(constraint.vertexIdx_[0]));
        Point p1 =mesh_.point(VHandle(constraint.vertexIdx_[1]));

        V3 v0(p0[0], p0[1], p0[2]);
        V3 v1(p1[0], p1[1], p1[2]);

        OpenMesh::VectorT<double, 3> dP = constraint.computeCorrection(v0, v1);
        std::cout << "<" << constraint.edgeIndex << ">" << dP[0] << " " << dP[1] << " " << dP[2] << std::endl;
      }

    }

    void updateBendingConstraints() {

      // we need the rest angle phi0

      for (auto& constraint : body_.bendingConstraints_) {
        Real d = 0, phi = 0, i_d = 0;

        MyMesh::FaceHandle fh0 = mesh_.face_handle(constraint.fidx0);
        MyMesh::FaceHandle fh1 = mesh_.face_handle(constraint.fidx1);

        OpenMesh::Vec3f normal1 = mesh_.normal(fh0);
        OpenMesh::Vec3f normal2 = mesh_.normal(fh1);

        d = OpenMesh::dot(normal1, normal2);
        phi = std::acos(d);
        std::cout << "Angle between normals: " << phi << std::endl;

        if (d < -1.0)
          d = -1.0;
        else if (d > 1.0)
          d = 1.0;

        if (d == -1.0) {
          phi = PI;
        }

        // 180 degree case, triangles are in the same plane
        if (d == 1.0) {
          phi = 0.0;
        }

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
