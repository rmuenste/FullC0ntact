#ifndef VECTOR3_INT_HPP_SQIFOJU7
#define VECTOR3_INT_HPP_SQIFOJU7

#include <vector3.h>

namespace i3d {

  template<>
    class Vector3<int> {

      public:

        /* constructor */
        host_dev
          Vector3<int>(int a, int b, int c): x(a), y(b), z(c) {}

        /* copy constructor */
        host_dev
          Vector3<int>(const Vector3<int> &v)
          {
            x = v.x;
            y = v.y;
            z = v.z;
          }

        /* default constructor */
        host_dev
          Vector3():x(0), y(0), z(0){}

        host_dev
          ~Vector3(){};

        host_dev
          inline const Vector3<int>& operator=(const Vector3<int> &v)
          {

            x = v.x;
            y = v.y;
            z = v.z;
            return *this;
          }//end  operator

        host_dev
          inline Vector3<int> operator-() const
          {
            return Vector3<int>(-x,-y,-z);
          }//end operator

        union
        {
          int coords_[3];
          struct
          {
            int x;
            int y;
            int z;
          };
        };
    };

  template<typename T> host_dev 
    inline Vector3<T> operator+(const Vector3<T> &a, const Vector3<T> &b); 

  template<typename T> host_dev
    inline Vector3<T> operator-(const Vector3<T> &a, const Vector3<T> &b); 

  template<typename T> host_dev
    inline Vector3<T> operator*(const Vector3<T> &v, T a);

  template<> host_dev
    inline Vector3<int> operator*(int a,const Vector3<int> &v)
    {
      // Return scaled vector
      return Vector3<int>(v.x * a, v.y * a,v.z * a);
    }//end  operator

  template<> host_dev
    inline Vector3<int> operator*(const Vector3<int> &v, int a)
    {
      // Return scaled vector
      return Vector3<int>(v.x * a, v.y * a,v.z * a);
    }//end  operator

  template<> host_dev
    inline Vector3<int> operator+(const Vector3<int> &a, const Vector3<int> &b) 
    {
      return Vector3<int>(a.x + b.x, a.y + b.y, a.z + b.z);
    }//end  operator

  template<> host_dev
    inline Vector3<int> operator-(const Vector3<int> &a, const Vector3<int> &b) 
    {
      return Vector3<int>(a.x - b.x, a.y - b.y, a.z - b.z);
    }//end  operator

}

#endif /* end of include guard: VECTOR3_INT_HPP_SQIFOJU7 */

