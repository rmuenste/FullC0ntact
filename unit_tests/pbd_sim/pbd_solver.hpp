#ifndef PBD_SOLVER_HPP
#define PBD_SOLVER_HPP

#include <mymath.h>
#include "general_definitions.hpp"

namespace i3d {

  class PBDBody;

  class PBDSolver {

    typedef OpenMesh::Vec3d VectorType;
    typedef Real ScalarType;

    public:
      PBDBody* body_;
      int solverIterations_;
      ScalarType dt_;

      std::vector<VectorType> tmpPosition_;

      void solve();

      void setDT(ScalarType _dt) { dt_ = _dt; };
      ScalarType getDT() { return dt_;};

      void setIterations(int _iters) { solverIterations_ = _iters; };
      int getIterations() { return solverIterations_; };

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
