#include <iostream>
#include <offloader.h>
#include <3dmodel.h>

using namespace i3d;

int main()
{

  OffLoader loader;
  Model3D myModel;
  loader.readModelFromFile(&myModel, "test.off");

  return 0;
}
