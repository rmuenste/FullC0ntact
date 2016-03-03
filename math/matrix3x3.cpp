/*
   <one line to give the program's name and a brief idea of what it does.>
   Copyright (C) <year>  <name of author>

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License along
   with this program; if not, write to the Free Software Foundation, Inc.,
   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

*/

#include "matrix3x3.h"

namespace i3d {

  template<class T>
    Matrix3x3<T>::Matrix3x3()
    {

      SetZero();

    }//end constructor

  template<class T>
    Matrix3x3<T>::Matrix3x3(const Vector3<T> &c1,const Vector3<T> &c2,const Vector3<T> &c3)
    {

      this->m_dEntries[0]=c1.x;
      this->m_dEntries[1]=c2.x;
      this->m_dEntries[2]=c3.x;
      this->m_dEntries[3]=c1.y;
      this->m_dEntries[4]=c2.y;
      this->m_dEntries[5]=c3.y;
      this->m_dEntries[6]=c1.z;
      this->m_dEntries[7]=c2.z;
      this->m_dEntries[8]=c3.z;

    }//end constructor

  template<class T>
    Matrix3x3<T>::Matrix3x3(T entries[])
    {

      for (int i = 0; i < 9; ++i)
      {
        m_dEntries[i] = entries[i]; 
      }

    }//end constructor


  template<class T>
    Matrix3x3<T> Matrix3x3<T>::operator *(const Matrix3x3<T> &rhs) const
    {
      Matrix3x3<T> tmp;
      for(int i = 0; i < 3; i++)
      {
        for(int j = 0; j <3; j++)
        {
          for(int k = 0; k < 3; k++)
          {
            tmp.m_dEntries[i*3+j] += m_dEntries[i*3+k] * rhs.m_dEntries[k*3+j];
          }//end for k
        }//end for j
      }//end for i
      return tmp;
    }//end operator*

  template<class T>
    Matrix3x3<T>::Matrix3x3(const Matrix3x3 &copy)
    {

      for (int i = 0; i < 9; ++i)
      {
        m_dEntries[i] = copy.m_dEntries[i]; 
      }

    }//end constructor

  template<class T>
    void Matrix3x3<T>::MatrixFromAngles(const Vector3<T> &vRotXYZ)
    {

      //heading, attitude, bank
      T a,b,c,d,e,f,ad,bd;
      a       = cos(vRotXYZ.x);
      b       = sin(vRotXYZ.x);
      c       = cos(vRotXYZ.y);
      d       = sin(vRotXYZ.y);
      e       = cos(vRotXYZ.z);
      f       = sin(vRotXYZ.z);
      ad      =   a * d;
      bd      =   b * d;
      m_dEntries[0]  =   c * e;
      m_dEntries[1]  =  -c * f;
      m_dEntries[2]  =   d;
      m_dEntries[3]  =  bd * e + a * f;
      m_dEntries[4]  = -bd * f + a * e;
      m_dEntries[5]  =  -b * c;
      m_dEntries[6]  = -ad * e + b * f;
      m_dEntries[7]  =  ad * f + b * e;
      m_dEntries[8]  =   a * c;

      //std::cout<<*this<<std::endl;

      //m_dEntries[0]  =  1.0;
      //m_dEntries[1]  =  0.0;
      //m_dEntries[2]  =  0.0;
      //m_dEntries[3]  =  0.0;
      //m_dEntries[4]  =  0.73168;
      //m_dEntries[5]  =  -0.68163;
      //m_dEntries[6]  =  0.0;
      //m_dEntries[7]  =  0.68163;
      //m_dEntries[8]  =  0.73168;

    }

  template<class T>
    Matrix3x3<T> Matrix3x3<T>::Inverse() const
    {
      Matrix3x3<T> inv;

      //inverse of determinate of this matrix
      T invdet = 1.0/Determinate();

      //entry(0,0)
      inv(0,0) = invdet * (GetEntry(1,1)*GetEntry(2,2) - GetEntry(2,1)*GetEntry(1,2));

      //entry(0,1)
      inv(0,1) = invdet * (GetEntry(0,2)*GetEntry(2,1) - GetEntry(2,2)*GetEntry(0,1));

      //entry(0,2)
      inv(0,2) = invdet * (GetEntry(0,1)*GetEntry(1,2) - GetEntry(1,1)*GetEntry(0,2));

      //entry(1,0)
      inv(1,0) = invdet * (GetEntry(1,2)*GetEntry(2,0) - GetEntry(2,2)*GetEntry(1,0));

      //entry(1,1)
      inv(1,1) = invdet * (GetEntry(0,0)*GetEntry(2,2) - GetEntry(2,0)*GetEntry(0,2));

      //entry(1,2)	
      inv(1,2) = invdet * (GetEntry(0,2)*GetEntry(1,0) - GetEntry(1,2)*GetEntry(0,0));

      //entry(2,0)
      inv(2,0) = invdet * (GetEntry(1,0)*GetEntry(2,1) - GetEntry(2,0)*GetEntry(1,1));

      //entry(2,1)
      inv(2,1) = invdet * (GetEntry(0,1)*GetEntry(2,0) - GetEntry(2,1)*GetEntry(0,0));

      //entry(2,2)
      inv(2,2) = invdet * (GetEntry(0,0)*GetEntry(1,1) - GetEntry(1,0)*GetEntry(0,1));

      return inv;
    }

  //----------------------------------------------------------------------------
  // Explicit instantiation.
  //----------------------------------------------------------------------------
  template class Matrix3x3<float>;

  template class Matrix3x3<double>;
  //----------------------------------------------------------------------------

}
