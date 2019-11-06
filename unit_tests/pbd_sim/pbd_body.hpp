#pragma once
#include <vector>

#include "general_definitions.hpp"

namespace i3d {

class PBDBody {
public:

  typedef OpenMesh::Vec3d VectorType;
  typedef float ScalarType;
  MyMesh* mesh_;

  std::vector<BendingConstraint> bendingConstraints_;
  std::vector<DistanceConstraint> distanceConstraints_;

  std::vector<VectorType> velocities_;

  std::vector<ScalarType> weights_;
  ScalarType mass = 1.0 / 4.0;

};

}

