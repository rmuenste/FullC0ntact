#include <cstdlib>
#include <vector>
#include <algorithm>
#include <iostream>
#include <Eigen/Dense>

using Eigen::MatrixXd;
using namespace Eigen;

int main()
{

  Matrix3d A;
  Vector3d b;

  A << 1, 0, 3, 2, 1, 1, 0, 0, 1;
  b << 3, 3, 4;

  Vector3d x = A.fullPivLu().solve(b);

  std::cout << "Matrix A:" << std::endl << A << std::endl;
  std::cout << "Vector b:" << std::endl << b << std::endl;
  std::cout << "Solution Ax=b :" << std::endl << x << std::endl;

  return EXIT_SUCCESS;
}
