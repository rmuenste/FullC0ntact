#pragma once

#include <OpenVolumeMesh/Geometry/VectorT.hh>

template <typename T>
class SpringModel {

public:

  typedef OpenVolumeMesh::Geometry::VectorT<T, 3> VertexType;

  virtual VertexType evaluateForce(const VertexType &q0, const VertexType &q1) {
    return VertexType(0, 0, 0);
  }

};

template <typename T>
class LinearSpring : public SpringModel<T> {

public:

  T springConstant, restLength;

  LinearSpring(T _springConstant, T _restLength) : springConstant(_springConstant), restLength(_restLength) {
    
  };

  VertexType evaluateForce(const VertexType &q0, const VertexType &q1) {
    VertexType dir = (q1 - q0);
    T length = dir.length();  //VertexType::length
    VertexType normalizedDir = dir / length;

    VertexType force = -springConstant * (length - restLength) * normalizedDir;
    return force;
  }

};

template <typename T>
class CubicSpring : public SpringModel<T> {

public:

  T k0, k1, k2, restLength;

  CubicSpring(T _k0, _k1, _k2, T _restLength) : k0(_k0), k1(_k1), k2(_k2), restLength(_restLength) {
    
  };

  VertexType evaluateForce(const VertexType &q0, const VertexType &q1) {
    VertexType dir = (q1 - q0);
    T length = dir.length();  //VertexType::length
    VertexType normalizedDir = dir / length;

    T sum = 0.0;

    VertexType diff = length - restLength;

    sum = k0 * diff + k1 * diff * diff + k3 * diff * diff * diff;

    VertexType force = -sum * normalizedDir;
    return force;
  }

};
