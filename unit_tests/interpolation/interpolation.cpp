#include <cstdlib>
#include <vector>
#include <algorithm>
#include <iostream>


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

  }

};

template <typename T>
class Linear_interp : public Base_interp<T> {

public:

  Linear_interp(const std::vector<T> &x, const std::vector<T> &y, int m) : Base_interp(x,y,m)
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
  int m = 2;
  Linear_interp<double> li(x,y,2);
  std::cout << "Interpolated value for x =(4.99999) : f(x): " << li.interp(4.99999) << std::endl;
  return EXIT_SUCCESS;
}
