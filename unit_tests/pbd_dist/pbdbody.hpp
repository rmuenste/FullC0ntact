#pragma once
#include <vector>

namespace i3d {


class PBDBody {
public:

  MyMesh* mesh_;

  std::vector<BendingConstraint> bendingConstraints_;
  std::vector<DistanceConstraint> distanceConstraints_;

};

}

