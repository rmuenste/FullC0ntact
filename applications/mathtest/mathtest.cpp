#include <iostream>
#include <cppinterface.h>

int main()
{

  int rank = 1;

  setoutputidx("1");

  init_fc_rigid_body(&rank);

  return EXIT_SUCCESS;

}
