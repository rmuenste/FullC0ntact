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

#ifndef MATRIX3X3_H_FC_GUARD
#define MATRIX3X3_H_FC_GUARD

#include <cmath>
#include <limits>
#include <ostream>

#include <mymath.h>
#include "vector3.h"

namespace i3d {

/**
 * @brief Simple 3x3 matrix used for rigid-body transforms and solvers.
 *
 * Offers basic arithmetic, norm utilities, and conversion helpers while
 * preserving the legacy memory layout used by the Fortran interface.
 */
template<class T>
class Matrix3x3
{
public:
  /** @brief Construct a zero matrix. */
  host_dev Matrix3x3() noexcept
  {
    SetZero();
  }

  /**
   * @brief Construct from row-major entries.
   */
  host_dev Matrix3x3(T m00, T m01, T m02,
                     T m10, T m11, T m12,
                     T m20, T m21, T m22) noexcept
  {
    m_d00 = m00; m_d01 = m01; m_d02 = m02;
    m_d10 = m10; m_d11 = m11; m_d12 = m12;
    m_d20 = m20; m_d21 = m21; m_d22 = m22;
  }

  /**
   * @brief Construct from an array of nine entries (row-major order).
   */
  host_dev explicit Matrix3x3(const T (&entries)[9]) noexcept
  {
    for (int i = 0; i < 9; ++i)
    {
      m_dEntries[i] = entries[i];
    }
  }

  /**
   * @brief Construct from column vectors.
   */
  host_dev Matrix3x3(const Vector3<T>& c1,
                     const Vector3<T>& c2,
                     const Vector3<T>& c3) noexcept
  {
    m_dEntries[0] = c1.x; m_dEntries[3] = c1.y; m_dEntries[6] = c1.z;
    m_dEntries[1] = c2.x; m_dEntries[4] = c2.y; m_dEntries[7] = c2.z;
    m_dEntries[2] = c3.x; m_dEntries[5] = c3.y; m_dEntries[8] = c3.z;
  }

  host_dev constexpr Matrix3x3(const Matrix3x3&) noexcept = default;
  host_dev constexpr Matrix3x3(Matrix3x3&&) noexcept = default;
  host_dev ~Matrix3x3() = default;

  host_dev Matrix3x3& operator=(const Matrix3x3&) noexcept = default;
  host_dev Matrix3x3& operator=(Matrix3x3&&) noexcept = default;

  /** @brief Factory for the identity matrix. */
  host_dev static Matrix3x3 Identity() noexcept
  {
    return Matrix3x3(T(1), T(0), T(0),
                     T(0), T(1), T(0),
                     T(0), T(0), T(1));
  }

  /** @brief Legacy alias returning the identity matrix. */
  host_dev static Matrix3x3 GenIdentity() noexcept
  {
    return Identity();
  }

  /** @brief Skew-symmetric matrix corresponding to a cross product. */
  host_dev static Matrix3x3 GetSkewMatrix(const Vector3<T>& vector) noexcept
  {
    return Matrix3x3(T(0), -vector.z, vector.y,
                     vector.z, T(0), -vector.x,
                    -vector.y, vector.x, T(0));
  }

  /** @brief Reset to identity. */
  host_dev void SetIdentity() noexcept
  {
    *this = Identity();
  }

  /** @brief Reset to zero. */
  host_dev void SetZero() noexcept
  {
    for (int i = 0; i < 9; ++i)
    {
      m_dEntries[i] = T(0);
    }
  }

  /** @brief Column-normalise the matrix. */
  host_dev void Normalize()
  {
    Vector3<T> c0(m_dEntries[0], m_dEntries[3], m_dEntries[6]);
    Vector3<T> c1(m_dEntries[1], m_dEntries[4], m_dEntries[7]);
    Vector3<T> c2(m_dEntries[2], m_dEntries[5], m_dEntries[8]);
    c0.tryNormalize();
    c1.tryNormalize();
    c2.tryNormalize();
    m_dEntries[0] = c0.x; m_dEntries[3] = c0.y; m_dEntries[6] = c0.z;
    m_dEntries[1] = c1.x; m_dEntries[4] = c1.y; m_dEntries[7] = c1.z;
    m_dEntries[2] = c2.x; m_dEntries[5] = c2.y; m_dEntries[8] = c2.z;
  }

  /** @brief Return the transposed matrix without modifying this instance. */
  host_dev Matrix3x3 GetTransposedMatrix() const noexcept
  {
    return Matrix3x3(m_d00, m_d10, m_d20,
                     m_d01, m_d11, m_d21,
                     m_d02, m_d12, m_d22);
  }

  /** @brief Transpose this matrix in place. */
  host_dev void TransposeMatrix() noexcept
  {
    *this = GetTransposedMatrix();
  }

  /** @brief Populate from yaw (x), pitch (y), roll (z) Euler angles. */
  host_dev void MatrixFromAngles(const Vector3<T>& vRotXYZ) noexcept
  {
    const T a = std::cos(vRotXYZ.x);
    const T b = std::sin(vRotXYZ.x);
    const T c = std::cos(vRotXYZ.y);
    const T d = std::sin(vRotXYZ.y);
    const T e = std::cos(vRotXYZ.z);
    const T f = std::sin(vRotXYZ.z);
    const T ad = a * d;
    const T bd = b * d;

    m_d00 = c * e;
    m_d01 = -c * f;
    m_d02 = d;

    m_d10 = bd * e + a * f;
    m_d11 = -bd * f + a * e;
    m_d12 = -b * c;

    m_d20 = -ad * e + b * f;
    m_d21 = ad * f + b * e;
    m_d22 = a * c;
  }

  /** @brief Determinant of the matrix. */
  host_dev T Determinant() const noexcept
  {
    return m_d00 * (m_d11 * m_d22 - m_d21 * m_d12)
         - m_d01 * (m_d10 * m_d22 - m_d20 * m_d12)
         + m_d02 * (m_d10 * m_d21 - m_d20 * m_d11);
  }

  /**
   * @brief Determinant alias for compatibility with existing code.
   */
  host_dev T Determinate() const noexcept
  {
    return Determinant();
  }

  /**
   * @brief Non-const alias preserved for historical call sites.
   */
  host_dev T Determinate()
  {
    return Determinant();
  }

  /**
   * @brief Compute the inverse matrix.
   * @param tolerance Determinant threshold below which the identity is returned.
   */
  host_dev Matrix3x3 Inverse(T tolerance = std::numeric_limits<T>::epsilon()) const
  {
    const T det = Determinant();
    if (std::fabs(det) <= tolerance)
    {
      return Identity();
    }

    const T invDet = T(1) / det;
    Matrix3x3 inv;
    inv(0,0) = invDet * (m_d11 * m_d22 - m_d21 * m_d12);
    inv(0,1) = invDet * (m_d02 * m_d21 - m_d22 * m_d01);
    inv(0,2) = invDet * (m_d01 * m_d12 - m_d11 * m_d02);

    inv(1,0) = invDet * (m_d12 * m_d20 - m_d22 * m_d10);
    inv(1,1) = invDet * (m_d00 * m_d22 - m_d20 * m_d02);
    inv(1,2) = invDet * (m_d02 * m_d10 - m_d12 * m_d00);

    inv(2,0) = invDet * (m_d10 * m_d21 - m_d20 * m_d11);
    inv(2,1) = invDet * (m_d01 * m_d20 - m_d21 * m_d00);
    inv(2,2) = invDet * (m_d00 * m_d11 - m_d10 * m_d01);
    return inv;
  }

  /** @brief Matrix-vector multiplication. */
  host_dev Vector3<T> operator*(const Vector3<T>& rhs) const noexcept
  {
    return Vector3<T>(m_d00 * rhs.x + m_d01 * rhs.y + m_d02 * rhs.z,
                      m_d10 * rhs.x + m_d11 * rhs.y + m_d12 * rhs.z,
                      m_d20 * rhs.x + m_d21 * rhs.y + m_d22 * rhs.z);
  }

  /** @brief Matrix-matrix multiplication. */
  host_dev Matrix3x3 operator*(const Matrix3x3& rhs) const noexcept
  {
    Matrix3x3 result;
    for (int i = 0; i < 3; ++i)
    {
      for (int j = 0; j < 3; ++j)
      {
        T acc = T(0);
        for (int k = 0; k < 3; ++k)
        {
          acc += (*this)(i, k) * rhs(k, j);
        }
        result(i, j) = acc;
      }
    }
    return result;
  }

  /** @brief Scale by a scalar. */
  host_dev Matrix3x3 operator*(T scalar) const noexcept
  {
    Matrix3x3 result(*this);
    result *= scalar;
    return result;
  }

  /** @brief Add another matrix and return the result. */
  host_dev Matrix3x3 operator+(const Matrix3x3& rhs) const noexcept
  {
    Matrix3x3 result(*this);
    result += rhs;
    return result;
  }

  /** @brief Subtract another matrix and return the result. */
  host_dev Matrix3x3 operator-(const Matrix3x3& rhs) const noexcept
  {
    Matrix3x3 result(*this);
    result -= rhs;
    return result;
  }

  /** @brief In-place addition. */
  host_dev Matrix3x3& operator+=(const Matrix3x3& rhs) noexcept
  {
    for (int i = 0; i < 9; ++i)
    {
      m_dEntries[i] += rhs.m_dEntries[i];
    }
    return *this;
  }

  /** @brief In-place subtraction. */
  host_dev Matrix3x3& operator-=(const Matrix3x3& rhs) noexcept
  {
    for (int i = 0; i < 9; ++i)
    {
      m_dEntries[i] -= rhs.m_dEntries[i];
    }
    return *this;
  }

  /** @brief In-place scalar multiplication. */
  host_dev Matrix3x3& operator*=(T scalar) noexcept
  {
    for (int i = 0; i < 9; ++i)
    {
      m_dEntries[i] *= scalar;
    }
    return *this;
  }

  /** @brief Access matrix entry (row-major). */
  host_dev T& operator()(unsigned int row, unsigned int col) noexcept
  {
    return m_dEntries[3 * row + col];
  }

  /** @brief Access matrix entry (row-major) read-only. */
  host_dev T operator()(unsigned int row, unsigned int col) const noexcept
  {
    return m_dEntries[3 * row + col];
  }

  /** @brief Getter compatible with legacy naming. */
  host_dev T GetEntry(unsigned int row, unsigned int col) const noexcept
  {
    return m_dEntries[3 * row + col];
  }

  /**
   * @brief Backing storage exposed both as array and named entries.
   */
  union
  {
    T m_dEntries[9];
    struct
    {
      T m_d00; T m_d01; T m_d02;
      T m_d10; T m_d11; T m_d12;
      T m_d20; T m_d21; T m_d22;
    };
  };
};

/**
 * @brief Stream output helper for debugging.
 */
template<class T>
inline std::ostream& operator<<(std::ostream& out, const Matrix3x3<T>& rhs)
{
  for (int i = 0; i < 3; ++i)
  {
    out << rhs(i,0) << ' ' << rhs(i,1) << ' ' << rhs(i,2);
    if (i < 2)
    {
      out << '\n';
    }
  }
  return out;
}

using Matrix3x3d = Matrix3x3<double>;
using Matrix3x3f = Matrix3x3<float>;
using Mat3f = Matrix3x3<float>;
using mat3 = Matrix3x3<float>;
using MATRIX3X3 = Matrix3x3<Real>;
using Mat3 = Matrix3x3<Real>;

} // namespace i3d

#endif // MATRIX3X3_H_FC_GUARD
