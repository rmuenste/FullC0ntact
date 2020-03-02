#ifndef ODE_CONFIG_HPP_KQIMYCZM
#define ODE_CONFIG_HPP_KQIMYCZM
#include <string>
#include <3dmodel.h>
#include <memory>

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

    std::shared_ptr<Model3D> _meshObject;

    BodyODE() {
      _index = -1;
    };

    BodyODE(const BodyODE& copy) {

      _geomId = copy._geomId;
      _bodyId = copy._bodyId;

      _type = copy._type;
      _index = copy._index;
      _meshObject = copy._meshObject;
      
//      if (_type == "TriMesh")
//        std::cout << "TriMesh copy" << std::endl;

    };

    const BodyODE& operator=(const BodyODE& copy)
    {

      _geomId = copy._geomId;
      _bodyId = copy._bodyId;

      _type = copy._type;
      _index = copy._index;
      _meshObject = copy._meshObject;
//      if (_type == "TriMesh")
//        std::cout << "TriMesh assign" << std::endl;

      return *this;
    }//end  operator

  };

#endif

}

#endif /* end of include guard: ODE_CONFIG_HPP_KQIMYCZM */
