#pragma once
#include <world.h>

namespace i3d {

class ODERigidBodyWriter {

public:

  ODERigidBodyWriter() {

  };

  virtual ~ODERigidBodyWriter() {};

  void write(const World& world, std::string fileName);

};

};
