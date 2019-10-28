#pragma once
#include <vector>

namespace i3d {

class PBDBody {
public:

  typedef float ScalarType;
  MyMesh* mesh_;

  std::vector<BendingConstraint> bendingConstraints_;
  std::vector<DistanceConstraint> distanceConstraints_;

  std::vector<ScalarType> weights_;
  ScalarType mass = 1.0 / 4.0;

};

}

