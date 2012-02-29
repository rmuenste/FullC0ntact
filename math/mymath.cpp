#include "mymath.h"

namespace i3d
{

  //exlicit template instantiation
  template <> const float CMath<float>::TOLERANCEZERO = 1e-6f;

  template <> const double CMath<double>::TOLERANCEZERO = 1e-8f;

}

