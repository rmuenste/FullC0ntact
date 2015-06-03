#include "mymath.h"

namespace i3d
{

  //exlicit template instantiation
  template <> const float CMath<float>::TOLERANCEZERO = 1e-6f;

  template <> const double CMath<double>::TOLERANCEZERO = 1e-8f;
  template<>
  const Real CMath<Real>::SYS_PI(3.14159265358979323846);

  template<>
  const Real CMath<Real>::EPSILON7(1.0e-8);

  template<>
  const Real CMath<Real>::EPSILON5(1.0e-6);

  template<>
  const Real CMath<Real>::EPSILON4(1.0e-5);

  template<>
  const Real CMath<Real>::EPSILON3(1.0e-4);

  template<>
  const float CMath<float>::SYS_PI(3.14159265358979323846f);

  template<>
  const float CMath<float>::EPSILON7(1.0e-8f);

  template<>
  const float CMath<float>::EPSILON5(1.0e-6f);

  template<>
  const float CMath<float>::EPSILON4(1.0e-5f);

  template<>
  const float CMath<float>::EPSILON3(1.0e-4f);

}

