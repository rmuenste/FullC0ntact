#include <cstdlib>
#include <vector>
#include <algorithm>
#include <iostream>
#include <Eigen/Dense>

using Eigen::MatrixXd;
using namespace Eigen;

int main()
{
  MatrixXd m(2, 2);
  m(0, 0) =  3;
  m(1, 0) =  0;
  m(0, 1) = -1;
  m(1, 1) =  4;
  std::cout << m << std::endl;

  Matrix3d mat = Matrix3d::Random();
  mat = (mat + Matrix3d::Constant(1.2)) * 50;

  Vector3d v(1, 2, 3);

  std::cout << "m * v =" << std::endl << mat * v << std::endl;

  Matrix3f A;

  A << 1, 2, 3,
    4, 5, 6,
    7, 8, 9;

  std::cout << A << std::endl;

  return EXIT_SUCCESS;
}
