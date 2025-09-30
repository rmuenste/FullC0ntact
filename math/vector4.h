/***************************************************************************
 *   <one line to give the program's name and a brief idea of what it does.>
 *   Copyright (C) <year>  <name of author>
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License along
 *   with this program; if not, write to the Free Software Foundation, Inc.,
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 ***************************************************************************/

#ifndef VECTOR4_H_FC_GUARD
#define VECTOR4_H_FC_GUARD

#include <cmath>
#include <limits>
#include <ostream>

#include <mymath.h>

namespace i3d {

/**
 * @brief Four-component homogeneous vector with basic algebraic helpers.
 *
 * Provides the operations used in transformation code paths while remaining
 * header-only so the class can be compiled for host and device targets.
 */
template<class T>
class Vector4
{
public:
  /** @brief Construct a zero vector with homogeneous component 1. */
  host_dev constexpr Vector4() noexcept : x(T(0)), y(T(0)), z(T(0)), w(T(1)) {}

  /**
   * @brief Construct from explicit coordinates.
   * @param a X component.
   * @param b Y component.
   * @param c Z component.
   * @param d Homogeneous component.
   */
  host_dev constexpr Vector4(T a, T b, T c, T d) noexcept : x(a), y(b), z(c), w(d) {}

  /**
   * @brief Construct from 3D location and assume w = 1.
   * @param a X component.
   * @param b Y component.
   * @param c Z component.
   */
  host_dev constexpr Vector4(T a, T b, T c) noexcept : x(a), y(b), z(c), w(T(1)) {}

  host_dev constexpr Vector4(const Vector4&) noexcept = default;
  host_dev constexpr Vector4(Vector4&&) noexcept = default;
  host_dev ~Vector4() = default;

  host_dev Vector4& operator=(const Vector4&) noexcept = default;
  host_dev Vector4& operator=(Vector4&&) noexcept = default;

  /** @brief Unary minus keeping the homogeneous component intact. */
  host_dev Vector4 operator-() const noexcept
  {
    return Vector4(-x, -y, -z);
  }

  /** @brief Component-wise addition (homogeneous part reset to 1). */
  host_dev Vector4 operator+(const Vector4& v) const noexcept
  {
    return Vector4(x + v.x, y + v.y, z + v.z, T(1));
  }

  /** @brief Component-wise subtraction (homogeneous part reset to 1). */
  host_dev Vector4 operator-(const Vector4& v) const noexcept
  {
    return Vector4(x - v.x, y - v.y, z - v.z, T(1));
  }

  /** @brief Scale by a scalar while preserving the homogeneous component. */
  host_dev Vector4 operator*(T scalar) const noexcept
  {
    return Vector4(x * scalar, y * scalar, z * scalar);
  }

  /** @brief Dot product ignoring the homogeneous component. */
  host_dev T operator*(const Vector4& rhs) const noexcept
  {
    return x * rhs.x + y * rhs.y + z * rhs.z;
  }

  /** @brief Squared Euclidean length. */
  host_dev T squaredNorm() const noexcept
  {
    return x * x + y * y + z * z;
  }

  /** @brief Alias for squaredNorm maintained for legacy callers. */
  host_dev T norm2() const noexcept
  {
    return squaredNorm();
  }

  /** @brief Euclidean length of the spatial components. */
  host_dev T magnitude() const
  {
    return std::sqrt(squaredNorm());
  }

  /** @brief Alias for magnitude to preserve historic API. */
  host_dev T mag() const
  {
    return magnitude();
  }

  /**
   * @brief Normalise the spatial component if length exceeds tolerance.
   * @return true on success, false when the vector is too small.
   */
  host_dev bool tryNormalize(T tolerance = std::numeric_limits<T>::epsilon())
  {
    const T length = magnitude();
    if (length <= tolerance)
    {
      return false;
    }

    const T invLength = T(1) / length;
    x *= invLength;
    y *= invLength;
    z *= invLength;
    w = T(1);
    return true;
  }

  /** @brief Legacy normalisation helper that ignores the status flag. */
  host_dev void Normalize()
  {
    (void)tryNormalize();
  }

  /** @brief Legacy normalisation helper that ignores the status flag. */
  host_dev void normalize()
  {
    (void)tryNormalize();
  }

  /**
   * @brief Vector from point a to point b (homogeneous component reset to 1).
   */
  host_dev static Vector4 createVector(const Vector4& a, const Vector4& b) noexcept
  {
    return Vector4(b.x - a.x, b.y - a.y, b.z - a.z, T(1));
  }

  /** @brief Divide each component by a scalar. */
  host_dev Vector4& operator/=(const T& scalar)
  {
    x /= scalar;
    y /= scalar;
    z /= scalar;
    return *this;
  }

  /** @brief Add another vector in place (homogeneous component reset to 1). */
  host_dev Vector4& operator+=(const Vector4& rhs) noexcept
  {
    x += rhs.x;
    y += rhs.y;
    z += rhs.z;
    return *this;
  }

  /** @brief Subtract another vector in place (homogeneous component reset to 1). */
  host_dev Vector4& operator-=(const Vector4& rhs) noexcept
  {
    x -= rhs.x;
    y -= rhs.y;
    z -= rhs.z;
    return *this;
  }

  /** @brief Scale the spatial components in place. */
  host_dev Vector4& operator*=(const T& scalar) noexcept
  {
    x *= scalar;
    y *= scalar;
    z *= scalar;
    return *this;
  }

  /** @brief Symmetric dot product helper. */
  host_dev static T dot(const Vector4& a, const Vector4& b) noexcept
  {
    return a.x * b.x + a.y * b.y + a.z * b.z;
  }

  /** @brief Cross product of the spatial component. */
  host_dev static Vector4 Cross(const Vector4& lhs, const Vector4& rhs) noexcept
  {
    return Vector4((lhs.y * rhs.z) - (lhs.z * rhs.y),
                   (lhs.z * rhs.x) - (lhs.x * rhs.z),
                   (lhs.x * rhs.y) - (lhs.y * rhs.x),
                   T(1));
  }

  /**
   * @brief Index of the spatial component with maximum absolute magnitude.
   * @return 0, 1, or 2; -1 when all components are zero.
   */
  host_dev int FindMaxAbsComponent() const noexcept
  {
    T maxVal = -std::numeric_limits<T>::max();
    int result = -1;
    for (int i = 0; i < 3; ++i)
    {
      const T absVal = std::fabs(m_dCoords[i]);
      if (absVal > maxVal)
      {
        maxVal = absVal;
        result = i;
      }
    }
    return result;
  }

  /**
   * @brief Backing storage exposed as both array and named components.
   */
  union
  {
    T m_dCoords[4];
    struct
    {
      T x;
      T y;
      T z;
      T w;
    };
  };
};

/**
 * @brief Symmetric scalar multiplication helper.
 */
template<typename T>
host_dev inline Vector4<T> operator*(T scalar, const Vector4<T>& rhs) noexcept
{
  return rhs * scalar;
}

/**
 * @brief Stream output helper for logging and debugging.
 */
template<typename T>
inline std::ostream& operator<<(std::ostream& out, const Vector4<T>& vec)
{
  out << '[' << vec.x << ", " << vec.y << ", " << vec.z << ", " << vec.w << ']';
  return out;
}

using Vector4d = Vector4<double>;
using Vector4f = Vector4<float>;
using Vec4 = Vector4<Real>;

} // namespace i3d

#endif // VECTOR4_H_FC_GUARD
