#pragma once
#include <world.h>

#ifdef WITH_ODE
namespace i3d {

class ODERigidBodyWriter {

public:

  ODERigidBodyWriter() {

  };

  virtual ~ODERigidBodyWriter() {};

  void write(const World& world, std::string fileName);

};

};
#endif