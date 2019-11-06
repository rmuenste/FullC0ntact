#ifndef PBD_SOLVER_HPP
#define PBD_SOLVER_HPP

#include <mymath.h>
#include "general_definitions.hpp"

namespace i3d {

  class PBDBody;

  class PBDSolver {

    typedef OpenMesh::Vec3d VectorType;
    public:
      PBDBody* body_;
      int solverIterations_;
      Real dt_;

      std::vector<VectorType> tmpPosition_;

      void solve();

    private:
      void integrateWithDampening();

      void calculatePositionPrediction();

      void solveInternalConstraints();

      void solveDistanceConstraints();

      void solveBendingConstraints();

      void verletIntegration();

  };

}

#endif 
