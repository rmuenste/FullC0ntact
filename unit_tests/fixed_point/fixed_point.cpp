#include <cmath>
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


template <typename T, class myfunc>
class Trapz : public Quadrature<T> {

  // Routine implementing the extended trapezoidal rule.
  public:

    // Limits of integration and current value of integral
    T a, b;

    myfunc &func;

    int n;

    // The constructor takes as inputs func, the function of functor to be integrated between limits a and b, also input
    Trapz(myfunc &funcc, const T aa, const T bb) : a(aa), b(bb), func(funcc), n(0)
    {
    }

    T eval(int nn)
    {
      T x = a;
      T h = (b - a)/T(nn);
      T sum = func(x);
      for (int i(1); i <= nn - 1; ++i)
      {
        x += h;
        sum += 2 * func(x);
      }
      sum += func(b);
      T integral = (b - a) * sum / (2. * nn);
      return integral;
    }

    T next()
    {
      return 0.0;
    }

};

template <typename T>
struct ex_func {

  T operator()(T x) const {
    return std::exp(-x);
  };

};


int main()
{
  ex_func<double> f;
  double x = 0.0;
  double x_new = 0.0;
  for (int i(0); i < 10; ++i) {
    x_new = f(x);
    double e_rel = std::abs((x_new - x) / x_new) * 100.0;
    x = x_new;
    std::cout << "x_i = " << x <<  " | e_a = " << e_rel << std::endl;
  }

//  std::cout << "f(0): " << f(0) << std::endl;
//  std::cout << "f(1.5): " << f(1.5) << std::endl;
//  std::cout << "f(3): " << f(3) << std::endl;
//
//  Trapz<double, grav_func<double>> trap_int(f, 0, 3);
//  std::cout << "Int f(0,3): " << trap_int.eval(1) << std::endl;
//  std::cout << "Int f(0,3): " << trap_int.eval(5) << std::endl;
//  std::cout << "Int f(0,3): " << trap_int.eval(1000) << std::endl;

  return EXIT_SUCCESS;
}
