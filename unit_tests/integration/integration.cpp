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

//    T next()
//    {
//      T x, tnm, sum, del;
//      int it, j;
//      n++;
//      if (n == 1)
//      {
//        return (s = 0.5*(b - a)*(func(a) + func(b)));
//      }
//      else
//      {
//        for (it = 1, j = 1;j < n - 1;j++)
//        {
//          it <<= 1;
//        }
//        tnm = it;
//        del = (b - a) / tnm; //This is the spacing of the points to be added.
//        x = a + 0.5*del;
//        for (sum = 0.0, j = 0;j < it;j++, x += del)
//        {
//          sum += func(x);
//        }

//        //This replaces s by its refined value.
//        s = 0.5*(s + (b - a)*sum / tnm);
//        return s;
//      }
//    }

};

template <typename T>
struct grav_func {

  T operator()(T x) const
  {
    return std::sqrt(9.81 * 68.1 / 0.25) * std::tanh( std::sqrt( 9.81 * 0.25 / 68.1) * x);
  };

};


int main()
{
  grav_func<double> f;

  std::cout << "f(0): " << f(0) << std::endl;
  std::cout << "f(1.5): " << f(1.5) << std::endl;
  std::cout << "f(3): " << f(3) << std::endl;

  Trapz<double, grav_func<double>> trap_int(f, 0, 3);
  std::cout << "Int f(0,3): " << trap_int.eval(1) << std::endl;
  std::cout << "Int f(0,3): " << trap_int.eval(5) << std::endl;
  std::cout << "Int f(0,3): " << trap_int.eval(1000) << std::endl;

  return EXIT_SUCCESS;
}
