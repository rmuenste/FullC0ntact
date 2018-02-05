#ifndef ODE_CONFIG_HPP_KQIMYCZM
#define ODE_CONFIG_HPP_KQIMYCZM
#include <string>

#ifdef WITH_ODE
  #define  dDOUBLE
  #include <ode/odeconfig.h>
  #include <ode/ode.h>
#endif

namespace i3d {

#ifdef WITH_ODE

  class BodyODE
  {
  public:
    dGeomID _geomId;
    dBodyID _bodyId;

    std::string _type;
    int _index;

  };

#endif

}

#endif /* end of include guard: ODE_CONFIG_HPP_KQIMYCZM */
