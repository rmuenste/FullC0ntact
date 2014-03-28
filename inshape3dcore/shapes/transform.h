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
class Transformation
{

public:
    Transformation(void){};

    Transformation(const CMatrix3x3<T> &basis, const CVector3<T> &origin)
    {
      matrix_ = basis;
      origin_ = origin;
    }

    ~Transformation(void){};

    CMatrix3x3<T>& getMatrix() {return matrix_;};

    CMatrix3x3<T> getMatrix() const {return matrix_;};

    CVector3<T>   getOrigin() const {return origin_;};

    CVector3<T>&  getOrigin() {return origin_;};
    
    void Transpose(){matrix_.TransposeMatrix();};

private:
    CMatrix3x3<T> matrix_;
    CVector3<T>   origin_;

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
    shape PredictMotion(const shape &s, const CVector3<T> &vel, const Transformation<T> &transform, const CVector3<T> &angvel,T deltaT);


};

typedef Transformation<Real> Transformationr;

}
#endif // TRANSFORM_H
