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
#include "pbd_solver.hpp"

namespace i3d {

#include "constraints.hpp"
  
  class PositionBasedDynamicsApp : public Application<> {
  /*
  In this unit test we test the PBD kernel DistanceConstraint>:
  - A simple mesh consisting of two triangles and corresponding distance constraints are generated
  - Then the distance constraint is evaluated and should return 0
  - We then move a vertex and reevaluate the distance constraint, which should now give a non-zero value
  =======================================================================================================
  Init Function:
    In the init function we procedurally generate the simple mesh and generate 
    - distance constraints and
    - bending constraints
  */
  public:

    MyMesh mesh_;
    PBDBody body_;

    PBDSolver solver_;

    PositionBasedDynamicsApp() : Application() {
    }

    void writeOFFMesh(MyMesh &mesh, const std::string &fileName) {

      try {
        if (!OpenMesh::IO::write_mesh(mesh, fileName)) {
          std::cerr << "Cannot write mesh to file '"<< fileName << "'" << std::endl;
          std::exit(EXIT_FAILURE);
        }
        std::cout << "Mesh written to '" << fileName << "'" << std::endl;
      }
      catch( std::exception &x) {
        std::cerr << x.what() << std::endl;
        std::exit(EXIT_FAILURE);
      }

    }

    void init(std::string fileName) {

      mesh_ = generateSimpleMesh();
      solver_.solverIterations_ = 4;
      solver_.dt_ = 1.0 / 60.0;

      std::cout << "==========Generating Distance Constraints==========" << std::endl;
      body_.distanceConstraints_ = generateDistanceConstraints(mesh_, solver_.solverIterations_);

      std::cout << "==========Generating Bending Constraints===========" << std::endl;
      body_.bendingConstraints_ = generateBendingConstraints(mesh_, solver_.solverIterations_);

      body_.mesh_ = &mesh_;

      std::cout << "==========Computing Vertex Weights===========" << std::endl;

      for (int idx(0); idx < body_.mesh_->n_vertices(); ++idx) {
        body_.weights_.push_back(1.0 / body_.mass);
        body_.velocities_.push_back(MyMesh::Point(0,0,0));
      }
      body_.weights_[0] = 0.0;
      body_.weights_[1] = 0.0;

      writeOFFMesh(mesh_, "start.off");

      solver_.body_ = &body_;

    }

    void updateDistanceConstraints() {

      typedef MyMesh::Point Point;
      typedef OpenMesh::VectorT<double, 3> V3;

      auto e_end = mesh_.edges_end();

      std::cout << "Distance Constraint correction: " << std::endl;

      for (auto& constraint : body_.distanceConstraints_) {
        Point p0 =mesh_.point(mesh_.vertex_handle(constraint.vertexIdx_[0]));
        Point p1 =mesh_.point(mesh_.vertex_handle(constraint.vertexIdx_[1]));

        V3 v0(p0[0], p0[1], p0[2]);
        V3 v1(p1[0], p1[1], p1[2]);

        OpenMesh::VectorT<double, 3> dP = constraint.computeCorrection(v0, v1, body_.weights_[constraint.vertexIdx_[0]], body_.weights_[constraint.vertexIdx_[1]]);
        std::cout << "<" << constraint.edgeIndex << ">" << dP[0] << " " << dP[1] << " " << dP[2] << std::endl;
      }

    }


    void updateBendingConstraints() {

      typedef OpenMesh::Vec3f VertexType;
      typedef float ScalarType;

      std::vector<VertexType> tempVertices(mesh_.n_vertices());

      auto v_it = mesh_.vertices_begin(); 

      for (int idx(0); v_it != mesh_.vertices_end(); ++v_it, ++idx) {
        tempVertices[idx] = mesh_.point((*v_it));
      }

      for (auto& constraint : body_.bendingConstraints_) {
        Real d = 0, phi = 0, i_d = 0;

        // The two edge-connected triangles consist of four vertices
        // p1, p2, p3, p4
        // p1-p2 make up the shared edge and p3, p4 are unique to the
        // respective triangle

        MyMesh::FaceHandle fh0 = mesh_.face_handle(constraint.fidx0);
        MyMesh::FaceHandle fh1 = mesh_.face_handle(constraint.fidx1);

        // Let us get the Vectors from the handles and start computing
        VertexType p1 = tempVertices[constraint.p1];
        VertexType p2 = tempVertices[constraint.p2] - p1;
        VertexType p3 = tempVertices[constraint.p3] - p1;
        VertexType p4 = tempVertices[constraint.p4] - p1;

        VertexType p2p3 = OpenMesh::cross(p2, p3);
        VertexType p2p4 = OpenMesh::cross(p2, p4);

        Real lenp2p3 = OpenMesh::norm(p2p3);

        if (lenp2p3 == 0.0) {
          return;
        }

        Real lenp2p4 = OpenMesh::norm(p2p4);

        if (lenp2p4 == 0.0) {
          return;
        }

        VertexType n1 = p2p3.normalized();
        VertexType n2 = p2p4.normalized();
        std::cout << "<normal" << 1 << ">" << n1[0] << " " << n1[1] << " " << n1[2] << std::endl;
        std::cout << "<normal" << 2 << ">" << n2[0] << " " << n2[1] << " " << n2[2] << std::endl;

        d = OpenMesh::dot(n1, n2);
        phi = std::acos(d);
        std::cout << "Bending Angle between normals: " << phi << std::endl;

        if (d < -1.0) {
          d = -1.0;
          std::cout << "< -1 case: " << std::endl;
        }
        else if (d > 1.0) {
          d = 1.0;
          std::cout << "> 1 case: " << std::endl;
        }

        // 0 degree case, triangles in opposite direction, but folded together 
        if (d == -1.0) {
          phi = std::atan(1.0) * 4.0;
          std::cout << "== -1: " << std::endl;
          std::cout << "phi: " << phi << ":" << constraint.restAngle_ << std::endl;
          if (phi == constraint.restAngle_) {
            continue;
          }
        }

        // 180 degree case, triangles are in the same plane
        if (d == 1.0) {
          phi = 0.0;
          std::cout << "180 Degree case, dot product of normals " << "== 1: " << std::endl;
          if (phi == constraint.restAngle_) {
            continue;
          }
        }

        Real dArcCos = std::sqrt(1.0 - (d * d)) * (phi - constraint.restAngle_);

        VertexType p2n1 = OpenMesh::cross(p2, n1);
        VertexType p2n2 = OpenMesh::cross(p2, n2);
        VertexType p3n2 = OpenMesh::cross(p3, n2);
        VertexType p4n1 = OpenMesh::cross(p4, n1);

        VertexType n1p2 = -p2n1;
        VertexType n2p2 = -p2n2;

        VertexType n1p3 = OpenMesh::cross(n1, p3);
        VertexType n2p4 = OpenMesh::cross(n2, p4);

        // compute the qs
        VertexType q3 = (p2n2 + n1p2 * d) / lenp2p3;
        VertexType q4 = (p2n1 + n2p2 * d) / lenp2p4;
        VertexType q2 = (-(p3n2 + n1p3 * d) / lenp2p3) - ((p4n1 + n2p4 * d) / lenp2p4);

        VertexType q1 = -q2 - q3 - q4;

        ScalarType q1_len2 = OpenMesh::dot(q1, q1);
        ScalarType q2_len2 = OpenMesh::dot(q2, q2);
        ScalarType q3_len2 = OpenMesh::dot(q3, q3);
        ScalarType q4_len2 = OpenMesh::dot(q4, q4);

        ScalarType sum =
          body_.weights_[constraint.p1] * (q1_len2) +
          body_.weights_[constraint.p2] * (q2_len2) +
          body_.weights_[constraint.p3] * (q3_len2) +
          body_.weights_[constraint.p4] * (q4_len2);

        VertexType dP1 = -((body_.weights_[constraint.p1] * dArcCos) / sum) * q1;
        VertexType dP2 = -((body_.weights_[constraint.p2] * dArcCos) / sum) * q2;
        VertexType dP3 = -((body_.weights_[constraint.p3] * dArcCos) / sum) * q3;
        VertexType dP4 = -((body_.weights_[constraint.p4] * dArcCos) / sum) * q4;

        if (body_.weights_[constraint.p1] > 0.0) {
          tempVertices[constraint.p1] += dP1 * constraint.k;
        }
        if (body_.weights_[constraint.p2] > 0.0) {
          tempVertices[constraint.p2] += dP2 * constraint.k;
        }
        if (body_.weights_[constraint.p3] > 0.0) {
          tempVertices[constraint.p3] += dP3 * constraint.k;
        }
        if (body_.weights_[constraint.p4] > 0.0) {
          tempVertices[constraint.p4] += dP4 * constraint.k;
        }

        std::cout << "<dP1" << ">" << dP1[0] << " " << dP1[1] << " " << dP1[2] << std::endl;
        std::cout << "<dP2" << ">" << dP2[0] << " " << dP2[1] << " " << dP2[2] << std::endl;
        std::cout << "<dP3" << ">" << dP3[0] << " " << dP3[1] << " " << dP3[2] << std::endl;
        std::cout << "<dP4" << ">" << dP4[0] << " " << dP4[1] << " " << dP4[2] << std::endl;

      }

    }

    void manipulateVertex() {
      typedef MyMesh::Point Point;
      VHandle vh0 = mesh_.vertex_handle(2);
      Point p0 = mesh_.point(vh0);
      std::cout << "<Manip" << p0 << std::endl;
      p0[0] = -1.67982;
      p0[1] =  6.49421;
      p0[2] =  3.68093;
      mesh_.set_point(vh0, p0);
    }

    void run()
    {
      std::cout << "==========1st Update Distance Constraints==========" << std::endl;
      updateDistanceConstraints();
      std::cout << "==========Vertex Position Update==========" << std::endl;
      manipulateVertex();
      writeOFFMesh(mesh_, "manip.off");
//      std::cout << "==========2nd Update Distance Constraints==========" << std::endl;
//      updateDistanceConstraints();

      std::cout << "==========Update Bending Constraints==========" << std::endl;
      updateBendingConstraints();

      std::cout << "==========Testing PBD Solver==========" << std::endl;
      solver_.solve();
      writeOFFMesh(mesh_, "corr.off");
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
