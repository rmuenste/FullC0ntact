#pragma once

#include <OpenVolumeMesh/Geometry/VectorT.hh>

template <typename T>
class DeformationAxis {

public:

  typedef OpenVolumeMesh::Geometry::VectorT<T, 3> VertexType;

  // In this vector we store the coordinates of the intersection points of this axis
  // with the faces of the element that the axis belongs to.
  // An axis has 2 intersection points with the element faces. We store the first intersection
  // point as q[0] and the second one as q[1]. The intersection parameters of the first 
  // intersection point q[0] are stored in parameters[0] and the intersected face of the 
  // element is stored in faceIndices[0]. 
  std::vector<VertexType> q;

  VertexType dir;

  // A single deformation axis has 2 intersection points with its element
  // therefore we have 2 sets of parameter values xi, eta for an intersection point. 
  // these 2 sets of parameter values are stored in this vector of pairs. A pair in the 
  // vector represents a pair of parameters xi, eta
  std::vector<std::pair<T, T>> parameters;

  // A deformation axis intersects with 2 faces of its element. The face indices of
  // the intersecting elements are stored in this array
  std::vector<int> faceIndices;

  DeformationAxis() {};

  VertexType getDir() {
    return q[1] - q[0];
  }
  

};