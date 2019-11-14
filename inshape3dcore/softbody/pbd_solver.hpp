#ifndef PBD_SOLVER_HPP
#define PBD_SOLVER_HPP

#include <mymath.h>
#include "general_definitions.hpp"
#include "pbd_body.hpp"

namespace i3d {

  template <typename T>
  class PBDSolver {

    typedef T ScalarType;
    typedef OpenMesh::VectorT<ScalarType, 3> VectorType;

    public:
      PBDBody<ScalarType>* body_;
      int solverIterations_;
      ScalarType dt_;

      std::vector<VectorType> tmpPosition_;

      void solve();

      void setDT(ScalarType _dt) { dt_ = _dt; };
      ScalarType getDT() { return dt_;};

      void setIterations(int _iters) { solverIterations_ = _iters; };
      int getIterations() { return solverIterations_; };

      void setBody(PBDBody<ScalarType> *_body) { body_ = _body; };
      PBDBody<ScalarType>* getBody() { return body_; };

    private:
      void integrateWithDampening();

      void calculatePositionPrediction();

      void solveInternalConstraints();

      void solveDistanceConstraints();

      void solveBendingConstraints();

      void verletIntegration();

  };

typedef PBDSolver<double> PBDSolverd;
}

#endif 
