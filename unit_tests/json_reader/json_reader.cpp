#include <cstdlib>
#include <vector>
#include <algorithm>
#include <iostream>
#include <Eigen/Dense>


template <typename T>
class Base_interp {

public:
  int n, mm, jsav, cor, dj;

  const std::vector<T> &xx, &yy;

  Base_interp(const std::vector<T> &x, const std::vector<T> &y, int m) : n(int(x.size())), mm(m), jsav(0), cor(0), xx(x), yy(y)
  {

  }

  T interp(T x)
  {

    int jlo = locate(x);
    return rawinterp(jlo, x);

  }

  T virtual rawinterp(int jlo, T x) = 0;

/**  Given a value x, return a value j such that x is(insofar as possible) centered in the subrange
  *  xx[j..j + mm - 1], where xx is the stored pointer.The values in xx must be monotonic, either
  *  increasing or decreasing.The returned value is not less than 0, nor greater than n - 1.
  */
  int locate(T x)
  {
    int ju, jm, jl;
    if (n < 2 || mm < 2 || mm > n)throw("locate size error");
    bool ascnd = xx[n - 1] >= xx[0];
    jl = 0;
    ju = n - 1;
    while ((ju - jl) > 1)
    {
      jm = (ju + jl) >> 1;
      if (x >= xx[jm] == ascnd)
        jl = jm;
      else
        ju = jm;
    }
    cor = abs(jl - jsav) > dj ? 0 : 1;
    jsav = jl;
    return std::max<int>(0, std::min<int>(n - mm, jl - ((mm - 2) >> 1)));
  }

  int hunt(T x)
  {
    return 1;
  }

};

template <typename T>
class Linear_interp : public Base_interp<T> {

public:

  using Base_interp<T>::xx;
  using Base_interp<T>::yy;

  Linear_interp(const std::vector<T> &x, const std::vector<T> &y, int m) : Base_interp<T>(x,y,m)
  {

  }

  T rawinterp(int j, T x)
  {

    if (xx[j] == xx[j + 1])
      return yy[j];
    else
    {
      T val = yy[j] + ((x - xx[j]) / (xx[j + 1] - xx[j])) * (yy[j + 1] - yy[j]);
      return val;
    }
    
  }

};

template <typename T>
class Poly_interp : public Base_interp<T> {

public:

  using Base_interp<T>::xx;
  using Base_interp<T>::yy;
  using Base_interp<T>::mm;

  Poly_interp(const std::vector<T> &x, const std::vector<T> &y, int m) : Base_interp<T>(x, y, m)
  {

  }

  T evalLagrange(T x)
  {

    T poly = 0.0;

    for (int k(0); k < mm; ++k)
    {
      T num = 1.0;
      T denom = 1.0;
      for (int j(0); j < mm; ++j)
      {
        if (j == k)
          continue;

        num *= (x - xx[j]);
        denom *= (xx[k] - xx[j]);
      }

      poly += yy[k] * (num / denom);
    }

    return poly;
  }

  T rawinterp(int j, T x)
  {
 
    return evalLagrange(x);

  }

};

template <typename T>
class Newt_interp : public Base_interp<T> {

public:

  using Base_interp<T>::xx;
  using Base_interp<T>::yy;
  using Base_interp<T>::mm;

  Newt_interp(const std::vector<T> &x, const std::vector<T> &y, int m) : Base_interp<T>(x, y, m)
  {

  }

  T evalNewton(T x)
  {

    std::vector<T> b(yy);

    T poly = b[0];
    T pre = 1.0;
    for (int k(1); k < mm; ++k)
    {
      for (int j(mm-1); j >= k; --j)
      {
        b[j] = (b[j] - b[j - 1])/(xx[j] - xx[j - k]);
      }
      pre *= (x - xx[k - 1]);
      poly += b[k] * pre;
    }

    return poly;
  }

  T rawinterp(int j, T x)
  {

    return evalNewton(x);

  }

};

template <typename T>
class QuadSpline_interp : public Base_interp<T> {

public:

  using Base_interp<T>::xx;
  using Base_interp<T>::yy;
  using Base_interp<T>::mm;

  QuadSpline_interp(const std::vector<T> &x, const std::vector<T> &y, int m) : Base_interp<T>(x, y, m)
  {

  }

  int locate(T x)
  {
    for (unsigned i(1); i < xx.size(); ++i)
    {
      if ((x >= xx[i - 1]) && (x <= xx[i]))
      {
        return i - 1;
      }
    }
    return 1;
  }

/**
  * 
  * For m points there are m-1 intervals, so 3 * (m-1) unknowns must be determined.
  * Because of the condition a_1 = 0 this reduces to 3 * (m-1) - 1
  * 
  */
  T rawinterp(int j, T x)
  {

    using namespace Eigen;

    // Matrix dimension
    int n = 3 * (mm - 1) - 1;
    int icol = 0;
    int irow = 0;

    MatrixXd A(n, n);
    VectorXd solx(n);
    VectorXd b(n);
    A.setZero();

    /*
     * Condition 4: 
     * The 2nd derivative f''(x_0) = 2 * a_1 at the first knot is fixed to 0.0  
     * => a_1 = 0
     *   
     */

    /*
     * Condition 1: 
     *  Adjacent polynomials must be equal at the inner knots:
     *   
     */

    A(0, 0) = xx[1];
    A(0, 1) = 1.0;
    b(0) = yy[1];

    A(1, 2) = xx[1] * xx[1];
    A(1, 3) = xx[1];
    A(1, 4) = 1.0;
    b(1) = yy[1];

    irow = 2;
    for (int i(2), ii(0); i < mm-1; ++i, ++ii)
    {
      icol = 2 + ii * 3;
      // a_i
      A(irow, icol) = xx[i] * xx[i];
      // b_i
      A(irow, icol+1) = xx[i];
      // c_i
      A(irow, icol+2) = 1.0;
      // f_i
      b(irow) = yy[i];

      int kcol = icol + 3;
      A(irow + 1, kcol)     = xx[i] * xx[i];
      A(irow + 1, kcol + 1) = xx[i];
      A(irow + 1, kcol + 2) = 1.0;
      b(irow + 1)           = yy[i];

      // advance the row index
      irow += 2;
    }

    /*
     * Condition 2: 
     * The first and last functions must pass through the 
     * first and respectively through the last point 
     */

    // first polynomial through the first point
    A(irow, 0) = xx[0];
    A(irow, 1) = 1.0;
    b(irow) = yy[0];

    // last polynomial through the last point
    irow++;
    icol = n - 3;
    A(irow, icol)   = xx[mm-1] * xx[mm-1];
    A(irow, icol+1) = xx[mm-1];
    A(irow, icol+2) = 1.0;
    b(irow) = yy[mm-1];
    irow++;

//    std::cout << "A: " << std::endl << A << std::endl;
//    std::cout << "b: " << std::endl << b << std::endl;
//    std::cout << "-----------------------------------------------" << std::endl;

    /*
     * Condition 3: 
     * The first derivative at the inner knots must be equal 
     */
    A(irow,0) = 1.0;
    A(irow,2) = -2.0 * xx[1];
    A(irow,3) = -1.0;
    b(irow)   = 0.0;
    irow++;
    for (int j(0); irow < n; ++irow, ++j)
    {
      int ix = 2 + j;

      icol = 2 + j * 3;

      A(irow, icol)     = 2.0 * xx[ix];
      A(irow, icol + 1) = 1.0;

      A(irow, icol + 3) = -2.0 * xx[ix];
      A(irow, icol + 4) = -1.0;

      b(irow)    = 0.0;
    }

//    std::cout << "A: " << std::endl << A << std::endl;
//    std::cout << "b: " << std::endl << b << std::endl;

    solx = A.fullPivLu().solve(b);

//    std::cout << "x: " << std::endl << solx << std::endl;

    std::vector<T> coeff;
    coeff.push_back(0.0);
    for (int i(0); i < n; ++i)
      coeff.push_back(solx[i]);

    return evalSpline(coeff,x);

  }

  T evalSpline(std::vector<T> &coeff, T x)
  {

    int i = locate(x);
    //std::cout << "interval: " << std::endl << i << std::endl;
    int iseg = i * 3;
    return coeff[iseg] * x * x + coeff[iseg + 1] * x + coeff[iseg + 2];
  }

};

template <typename T>
class CubicSpline_interp : public Base_interp<T> {

public:

  using Base_interp<T>::xx;
  using Base_interp<T>::yy;
  using Base_interp<T>::mm;

  CubicSpline_interp(const std::vector<T> &x, const std::vector<T> &y, int m) : Base_interp<T>(x, y, m)
  {

  }

  int locate(T x)
  {
    for (unsigned i(1); i < xx.size(); ++i)
    {
      if ((x >= xx[i - 1]) && (x <= xx[i]))
      {
        return i - 1;
      }
    }
    throw("Value outside of knot vector error");
  }

  inline T hi(const int &i)
  {
    return xx[i + 1] - xx[i];
  }

  inline T fi(const int &i)
  {
    return (yy[i+1]-yy[i])/hi(i);
  }

  /**
  *
  * For m points there are m-1 intervals, so 3 * (m-1) unknowns must be determined.
  * Because of the condition a_1 = 0 this reduces to 3 * (m-1) - 1
  *
  */
  T rawinterp(int j, T x)
  {

    using namespace Eigen;

    // Matrix dimension
    int n = mm;
    int icol = 0;
    int irow = 0;

    MatrixXd A(n, n);
    VectorXd c(n);
    VectorXd b(n-1);
    VectorXd d(n-1);
    VectorXd rhs(n);
    A.setZero();

    // Boundary conditions matrix
    A(0, 0)     = 1.0;
    A(n-1, n-1) = 1.0;

    // Boundary conditions rhs
    rhs(0)        = 0;
    rhs(n-1)      = 0;

    for (int i(1); i <= n - 2; ++i)
    {
      // Assemble the matrix
      A(i, icol) = hi(irow);
      A(i, icol + 1) = 2.0 * (hi(irow) + hi(irow+1));
      A(i, icol + 2) = hi(irow+1);

      // Build the RHS
      rhs(i) = 3.0 * (fi(irow + 1) - fi(irow));

      irow++;
      icol++;
    }

//    std::cout << "A: " << std::endl << A << std::endl;
//    std::cout << "rhs: " << std::endl << rhs << std::endl;

    c = A.fullPivLu().solve(rhs);

    // compute the b_i and d_i coefficients of the cubic splines
    for (int i(0); i < n - 1; ++i)
    {
      b(i) = fi(i) - (hi(i) / 3.0) * (2. * c(i) + c(i + 1));

      d(i) = (c(i + 1) - c(i)) / (3.0 * hi(i));
    }

//    std::cout << "coefficient d_i: " << std::endl << d << std::endl;
//    std::cout << "coefficient c_i: " << std::endl << c << std::endl;
//    std::cout << "coefficient b_i: " << std::endl << b << std::endl;

    return evalSpline(b,c,d,x);

  }

  T evalSpline(const Eigen::VectorXd &b, const Eigen::VectorXd &c,const Eigen::VectorXd &d, T x)
  {

    int i = locate(x);
    //std::cout << "interval: " << std::endl << i << std::endl;
    T x0 = x - xx[i];
    return yy[i] + b[i] * x0 + c[i] * x0 * x0 + d[i] * x0 * x0 * x0;
  }

};


int main()
{
  std::vector<double> x, y;
  x.push_back(0.0);
  x.push_back(1.0);
  x.push_back(2.0);
  x.push_back(3.0);
  x.push_back(4.0);
  x.push_back(5.0);
  x.push_back(6.0);

  y.push_back(0.0);
  y.push_back(0.9);
  y.push_back(1.0);
  y.push_back(0.1);
  y.push_back(-0.8);
  y.push_back(-0.9);
  y.push_back(-0.1);

//  int m = 2;
  Linear_interp<double> li(x,y,2);
  double xx = 5.1;
  std::cout << "Interpolated value for x =(" << xx << ")  : f(x): " << li.interp(xx) << std::endl;

  Poly_interp<double> polyI(x, y, x.size());
  std::cout << "Interpolated value for x =(" << xx << ") by lagrange interpolation : f(x): " << polyI.interp(xx) << std::endl;

  Newt_interp<double> newtI(x, y, x.size());
  std::cout << "Interpolated value for x =(" << xx << ") by newton interpolation : f(x): " << newtI.interp(xx) << std::endl;

//  std::vector<double> qx, qy;
//  x.push_back(3.0);
//  x.push_back(4.5);
//  x.push_back(7.0);
//  x.push_back(9.0);

//  y.push_back(2.5);
//  y.push_back(1.0);
//  y.push_back(2.5);
//  y.push_back(0.5);
//  double xx = 5.0;
  
  QuadSpline_interp<double> qSplineI(x, y, x.size());
  std::cout << "Interpolated value for x =(" << xx << ") by quadratic spline interpolation : f(x): " << qSplineI.interp(xx) << std::endl;

  CubicSpline_interp<double> cSplineI(x, y, x.size());
  std::cout << "Interpolated value for x =(" << xx << ") by cubic spline interpolation : f(x): " << cSplineI.interp(xx) << std::endl;

  return EXIT_SUCCESS;
}
