/***************************************************************************
 * Copyright (C) 2006 by Raphael Münster
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 ***************************************************************************/
#ifndef VECTOR3_H_FLUCNCYE
#define VECTOR3_H_FLUCNCYE

#include <algorithm>
#include <cmath>
#include <limits>
#include <ostream>

#include "mymath.h"
#include "vector4.h"

namespace i3d {

/**
 * @brief Simple three-component vector with basic algebraic operations.
 *
 * Provides common arithmetic helpers, normalisation utilities, and geometric
 * queries that are used throughout the rigid body solvers. The implementation
 * remains header-only so that it can be compiled for host and device targets.
 *
 * @tparam T Floating-point value type (float, double, etc.).
 */
template<class T>
class Vector3
{
public:
  /** @brief Construct a zero initialised vector. */
  host_dev constexpr Vector3() noexcept : x(T(0)), y(T(0)), z(T(0)) {}

  /**
   * @brief Construct from explicit coordinates.
   * @param a X component
   * @param b Y component
   * @param c Z component
   */
  host_dev constexpr Vector3(T a, T b, T c) noexcept : x(a), y(b), z(c) {}

  host_dev constexpr Vector3(const Vector3&) noexcept = default;
  host_dev constexpr Vector3(Vector3&&) noexcept = default;
  host_dev ~Vector3() = default;

  host_dev Vector3& operator=(const Vector3&) noexcept = default;
  host_dev Vector3& operator=(Vector3&&) noexcept = default;

  /**
   * @brief Assign from a homogeneous 4D vector, discarding the w component.
   * @param v Source 4D vector.
   */
  host_dev Vector3& operator=(const Vector4<T>& v) noexcept
  {
    x = v.x;
    y = v.y;
    z = v.z;
    return *this;
  }

  /** @brief Unary minus. */
  host_dev Vector3 operator-() const noexcept
  {
    return Vector3(-x, -y, -z);
  }

  /**
   * @brief Add two vectors component-wise.
   * @param v Right-hand operand.
   * @return Vector sum.
   */
  host_dev Vector3 operator+(const Vector3& v) const noexcept
  {
    return Vector3(x + v.x, y + v.y, z + v.z);
  }

  /**
   * @brief Subtract two vectors component-wise.
   * @param v Right-hand operand.
   * @return Vector difference.
   */
  host_dev Vector3 operator-(const Vector3& v) const noexcept
  {
    return Vector3(x - v.x, y - v.y, z - v.z);
  }

  /**
   * @brief Scale the vector by a scalar.
   * @param scalar Multiplicative factor.
   */
  host_dev Vector3 operator*(T scalar) const noexcept
  {
    return Vector3(x * scalar, y * scalar, z * scalar);
  }

  /**
   * @brief Divide the vector by a scalar.
   * @param scalar Divisor (must be non-zero).
   */
  host_dev Vector3 operator/(const T& scalar) const
  {
    return Vector3(x / scalar, y / scalar, z / scalar);
  }

  /**
   * @brief Dot product assignment operator.
   * @param rhs Right-hand operand.
   * @return Scalar dot product.
   */
  host_dev T operator*(const Vector3& rhs) const noexcept
  {
    return x * rhs.x + y * rhs.y + z * rhs.z;
  }

  /** @brief Squared magnitude of the vector. */
  host_dev T squaredNorm() const noexcept
  {
    return x * x + y * y + z * z;
  }

  /** @brief Alias for squaredNorm provided for backward compatibility. */
  host_dev T norm2() const noexcept
  {
    return squaredNorm();
  }

  /** @brief Euclidean length of the vector. */
  host_dev T magnitude() const
  {
    return std::sqrt(squaredNorm());
  }

  /** @brief Alias for magnitude for historical callers. */
  host_dev T mag() const
  {
    return magnitude();
  }

  /**
   * @brief Normalise the vector if it is longer than @p tolerance.
   *
   * @param tolerance Minimum admissible length (defaults to machine epsilon).
   * @return @c true when the vector was normalised, @c false when the length
   *         was below the tolerance and the vector remains unchanged.
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
    return true;
  }

  /** @brief Legacy normalise helper that ignores the return value. */
  host_dev void Normalize()
  {
    (void)tryNormalize();
  }

  /** @brief Legacy normalise helper that ignores the return value. */
  host_dev void normalize()
  {
    (void)tryNormalize();
  }

  /**
   * @brief Create the vector pointing from @p a to @p b.
   * @param a Start point.
   * @param b End point.
   */
  host_dev static Vector3 createVector(const Vector3& a, const Vector3& b) noexcept
  {
    return b - a;
  }

  /** @brief Divide this vector by a scalar in-place. */
  host_dev Vector3& operator/=(const T& scalar)
  {
    x /= scalar;
    y /= scalar;
    z /= scalar;
    return *this;
  }

  /** @brief Add another vector in-place. */
  host_dev Vector3& operator+=(const Vector3& rhs) noexcept
  {
    x += rhs.x;
    y += rhs.y;
    z += rhs.z;
    return *this;
  }

  /** @brief Subtract another vector in-place. */
  host_dev Vector3& operator-=(const Vector3& rhs) noexcept
  {
    x -= rhs.x;
    y -= rhs.y;
    z -= rhs.z;
    return *this;
  }

  /** @brief Scale this vector in-place. */
  host_dev Vector3& operator*=(const T& scalar) noexcept
  {
    x *= scalar;
    y *= scalar;
    z *= scalar;
    return *this;
  }

  /**
   * @brief Unit axis aligned with the component of greatest magnitude.
   *
   * The sign of the returned axis matches the sign of the dominant component,
   * making this routine useful for fallback directions during constraint
   * generation.
   */
  host_dev Vector3 largestComponentDir() const noexcept
  {
    const T absX = std::fabs(x);
    const T absY = std::fabs(y);
    const T absZ = std::fabs(z);

    Vector3 axis(T(0), T(0), T(0));
    if (absX >= absY && absX >= absZ)
    {
      axis.x = (x >= T(0)) ? T(1) : T(-1);
    }
    else if (absY >= absX && absY >= absZ)
    {
      axis.y = (y >= T(0)) ? T(1) : T(-1);
    }
    else
    {
      axis.z = (z >= T(0)) ? T(1) : T(-1);
    }

    return axis;
  }

  /** @brief Return the symmetric dot product. */
  host_dev static T dot(const Vector3& a, const Vector3& b) noexcept
  {
    return a.x * b.x + a.y * b.y + a.z * b.z;
  }

  /**
   * @brief Angle between two vectors.
   *
   * @return Angle in radians. Returns 0 when one of the vectors is shorter
   *         than machine epsilon.
   */
  host_dev static T AngleBetween(const Vector3& a, const Vector3& b)
  {
    const T lengthA = a.magnitude();
    const T lengthB = b.magnitude();
    const T tolerance = std::numeric_limits<T>::epsilon();

    if (lengthA <= tolerance || lengthB <= tolerance)
    {
      return T(0);
    }

    const T cosAngle = std::clamp(dot(a, b) / (lengthA * lengthB), T(-1), T(1));
    return std::acos(cosAngle);
  }

  /**
   * @brief Generate two orthonormal vectors that complement @p w.
   *
   * The output vectors @p u and @p v satisfy (u, v, w) being a right-handed
   * orthonormal basis as long as @p w is non-zero. Implementation mirrors the
   * algorithm from Eberly (1999).
   */
  static void GenerateComplementBasis(Vector3& u, Vector3& v, const Vector3& w)
  {
    if (std::fabs(w.x) >= std::fabs(w.y))
    {
      const T invLength = T(1) / std::sqrt(w.x * w.x + w.z * w.z);
      u.x = -w.z * invLength;
      u.y = T(0);
      u.z = w.x * invLength;
      v.x = w.y * u.z;
      v.y = w.z * u.x - w.x * u.z;
      v.z = -w.y * u.x;
    }
    else
    {
      const T invLength = T(1) / std::sqrt(w.y * w.y + w.z * w.z);
      u.x = T(0);
      u.y = w.z * invLength;
      u.z = -w.y * invLength;
      v.x = w.y * u.z - w.z * u.y;
      v.y = -w.x * u.z;
      v.z = w.x * u.y;
    }
  }

  /**
   * @brief Cross product of two vectors.
   * @param lhs Left-hand operand.
   * @param rhs Right-hand operand.
   */
  host_dev static Vector3 Cross(const Vector3& lhs, const Vector3& rhs) noexcept
  {
    return Vector3((lhs.y * rhs.z) - (lhs.z * rhs.y),
                   (lhs.z * rhs.x) - (lhs.x * rhs.z),
                   (lhs.x * rhs.y) - (lhs.y * rhs.x));
  }

  /**
   * @brief Backing storage exposed as both array and named components.
   *
   * The union preserves the legacy ABI relied upon by Fortran bindings that
   * alias the coordinates as a raw array.
   */
  union
  {
    T m_dCoords[3];
    struct
    {
      T x;
      T y;
      T z;
    };
  };
};


/**
 * @brief Symmetric scalar multiplication.
 * @tparam T Value type.
 * @param scalar Scale factor applied from the left.
 * @param rhs Vector to scale.
 * @return Resulting vector.
 */
template<class T>
host_dev inline Vector3<T> operator*(T scalar, const Vector3<T>& rhs) noexcept
{
  return rhs * scalar;
}

/**
 * @brief Stream output helper for logging and debugging.
 * @tparam T Value type.
 * @param out Output stream.
 * @param vec Vector to serialise.
 */
template<class T>
inline std::ostream& operator<<(std::ostream& out, const Vector3<T>& vec)
{
  out << '[' << vec.x << ", " << vec.y << ", " << vec.z << ']';
  return out;
}

/// Double-precision three-component vector.
using Vector3d = Vector3<double>;
/// Single-precision three-component vector.
using Vector3f = Vector3<float>;
/// Legacy alias retained for backwards compatibility with legacy code.
using VECTOR3 = Vector3<Real>;
/// Legacy alias retained for backwards compatibility with legacy code.
using v3d = Vector3<Real>;
/// Legacy alias retained for backwards compatibility with legacy code.
using Vec3 = Vector3<Real>;
/// Legacy lowercase alias used in historical CUDA code paths.
using vector3 = Vector3<float>;

} // namespace i3d

#endif // VECTOR3_H_FLUCNCYE
