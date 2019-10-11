#pragma once
#include <vector>

namespace i3d {


class PBDBody {
public:

  MyMesh* mesh_;

  std::vector<BendingConstraint> constraints_;

};

}

