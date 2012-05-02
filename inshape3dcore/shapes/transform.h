/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) <2011>  <Raphael Muenster>

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

#ifndef TRANSFORM_H
#define TRANSFORM_H

#include<shape.h>
#include<obb3.h>
#include<matrix3x3.h>

namespace i3d {

/** @brief A class to store a transformation
 *
 * A class to perform to store a transformation, for example a ModelWorld transformation of a rigid body
 */  
template<class T>
class CTransform
{

public:
    CTransform(void){};

    CTransform(const CMatrix3x3<T> &basis, const CVector3<T> &origin)
    {
      m_matTransformation=basis;
      m_vOrigin = origin;
    }

    ~CTransform(void){};

    CMatrix3x3<T>& GetMatrix() {return m_matTransformation;};

    CMatrix3x3<T> GetMatrix() const {return m_matTransformation;};

    CVector3<T>   GetOrigin() const {return m_vOrigin;};

    CVector3<T>&  GetOrigin() {return m_vOrigin;};

private:
    CMatrix3x3<T> m_matTransformation;
    CVector3<T>   m_vOrigin;

};


/** @brief A class to perform various motion transformations
 *
 * A class to perform various transformations
 */  
template<class T, class shape>
class CPredictionTransform
{

public:
    CPredictionTransform(void){};
    ~CPredictionTransform(void){};

    shape PredictLinearMotion(const shape &s, const CVector3<T> &vel);
    shape PredictMotion(const shape &s, const CVector3<T> &vel, const CTransform<T> &transform, const CVector3<T> &angvel,T deltaT);


};

typedef CTransform<Real> CTransformr;

}
#endif // TRANSFORM_H
