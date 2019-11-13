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
#include "pbd_body.hpp"
#include "pbd_solver.hpp"
#include <iostream>
#include <ostream>


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

      //mesh_ = generateSimpleMesh();

      mesh_ = generatePlaneMesh();
      solver_.solverIterations_ = 2;
      solver_.dt_ = 1.0 / 60.0;
      
      //==========================PBDBody Configuration==========================
      // Setting the components and parameters of the PBDBody
      body_.setMass( 1.0 / mesh_.n_vertices() );

      std::cout << "==========Generating Distance Constraints==========" << std::endl;
      body_.distanceConstraints_ = generatePlaneDistanceConstraints(mesh_, solver_.solverIterations_);

      std::cout << "==========Generating Bending Constraints===========" << std::endl;
      body_.bendingConstraints_ = generatePlaneBendingConstraints(mesh_, solver_.solverIterations_);

      body_.mesh_ = &mesh_;

      std::cout << "==========Computing Vertex Weights===========" << std::endl;

      for (int idx(0); idx < body_.mesh_->n_vertices(); ++idx) {
        body_.weights_.push_back(1.0 / body_.mass);
        body_.velocities_.push_back(MyMesh::Point(0,0,0));
        body_.forces_.push_back(MyMesh::Point(0,0,0));
      }

      body_.weights_[0] = 0.0;
      body_.weights_[numX] = 0.0;
      //============================PBDBody configured============================

      writeOFFMesh(mesh_, "start.stl");

      solver_.body_ = &body_;

    }

    void manipulateVertex() {
      typedef MyMesh::Point Point;
      VHandle vh0 = mesh_.vertex_handle(2);
      Point p0 = mesh_.point(vh0);
      std::cout << "<Manip" << p0 << std::endl;
      p0[0] =  0;
      p0[1] =  5.5;
      p0[2] =  0;
      mesh_.set_point(vh0, p0);
    }

    void run()
    {
      //manipulateVertex();
      std::cout << "==========Testing PBD Solver==========" << std::endl;
      int iters = 0;
      for (; iters < 10000; ++iters) {
        std::cout << "|Iter: " << iters << "|=========================Testing PBD Solver=========================|Time: " << iters * solver_.dt_ << "|" << std::endl;
        std::ostringstream name;
        name << "mesh." << std::setfill('0') << std::setw(4) << iters << ".stl";
        writeOFFMesh(mesh_, name.str());
        solver_.solve();
      }
      std::ostringstream name;
      name << "mesh." << std::setfill('0') << std::setw(4) << iters << ".stl";
      writeOFFMesh(mesh_, name.str());
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
