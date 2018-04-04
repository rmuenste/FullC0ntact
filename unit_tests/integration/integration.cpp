#include <cstdlib>
#include <vector>
#include <algorithm>
#include <iostream>
#include <Eigen/Dense>


template <typename T>
class Quadrature {

public:
  int n;

  virtual T next() = 0;

};


template <typename T>
class Trapzd : public Quadrature<T> {

  // Routine implementing the extended trapezoidal rule.
  public:

    // Limits of integration and current value of integral
    T a, b, d;

    T &func;

    // The constructor takes as inputs func, the function of functor to be integrated between limits a and b, also input
    Trapz(T &funcc, const T aa, const T bb) : func(funcc), a(aa), b(bb)
    {
      n = 0;
    }

    T next()
    {

    }

};


int main()
{

  return EXIT_SUCCESS;
}
