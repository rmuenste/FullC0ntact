#pragma once

#include <OpenVolumeMesh/Geometry/VectorT.hh>
#include <utility>

template <typename T>
class SpringModel {

public:

  typedef OpenVolumeMesh::Geometry::VectorT<T, 3> VertexType;

  virtual VertexType evaluateForce(const VertexType &q0, const VertexType &q1) {
    return VertexType(0, 0, 0);
  }

  virtual std::pair<VertexType, VertexType> evaluateTorsion(const VertexType &q0, const VertexType &q1) {
    return std::make_pair(VertexType(0, 0, 0), VertexType(0, 0, 0));
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

  CubicSpring(T _k0, T _k1, T _k2, T _restLength) : k0(_k0), k1(_k1), k2(_k2), restLength(_restLength) {
    
  };

  VertexType evaluateForce(const VertexType &q0, const VertexType &q1) {

    VertexType dir = (q1 - q0);

    T length = dir.length();  //VertexType::length
    VertexType normalizedDir = dir / length;

    T sum = 0.0;

    T diff = length - restLength;

    // conversion percent
    T strain = (diff / restLength) * 100.0;

    sum = k0 * strain + k1 * strain * strain + k2 * strain * strain * strain;

    VertexType force = -sum * normalizedDir;
    return force;
  }

};

template <typename T>
class LinearTorsionSpring : public SpringModel<T> {

public:

  T springConstant, restAngle;

  LinearTorsionSpring(T _springConstant, T _restAngle) : springConstant(_springConstant), restAngle(_restAngle) {
    
  };

  
  std::pair<VertexType, VertexType> evaluateTorsion(const VertexType &eta0, const VertexType &eta1) {

    VertexType normalizedZeta0 = eta0 / eta0.length();
    VertexType normalizedZeta1 = eta1 / eta1.length();

    ScalarType alpha = OpenVolumeMesh::dot(normalizedZeta0, normalizedZeta1);

    VertexType force0 = -springConstant * (alpha - restAngle) * normalizedZeta0;
    VertexType force1 = -springConstant * (alpha - restAngle) * normalizedZeta1;

    return std::make_pair(force0, force1);
  }

};

template <typename T>
class CubicTorsionSpring : public SpringModel<T> {

public:

  T k0, k1, k2, restAngle;

  CubicTorsionSpring(T _k0, T _k1, T _k2, T _restAngle) : k0(_k0), k1(_k1), k2(_k2), restAngle(_restAngle) {
    
  };
  
  std::pair<VertexType, VertexType> evaluateTorsion(const VertexType &eta0, const VertexType &eta1) {

    VertexType normalizedZeta0 = eta0 / eta0.length();
    VertexType normalizedZeta1 = eta1 / eta1.length();

    ScalarType alpha = OpenVolumeMesh::dot(normalizedZeta0, normalizedZeta1);

    ScalarType alphaDiff = (alpha - restAngle);

    T sum = 0.0;

    sum = k0 * alphaDiff + k1 * alphaDiff * alphaDiff + k2 * alphaDiff * alphaDiff * alphaDiff;

    VertexType force0 = -sum * normalizedZeta0;
    VertexType force1 = -sum * normalizedZeta1;

    return std::make_pair(force0, force1);
  }

};
