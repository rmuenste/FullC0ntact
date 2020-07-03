#pragma once
// C++ includes
#include <iostream>
#include <vector>

// Include vector classes
#include <OpenVolumeMesh/Geometry/VectorT.hh>

// Include polyhedral mesh kernel
#include <OpenVolumeMesh/Mesh/HexahedralMesh.hh>

class OpenVolMeshVtkWriter {

using ScalarType = double;
using VertexType = OpenVolumeMesh::Geometry::VectorT<ScalarType, 3>;
using HexMesh = OpenVolumeMesh::GeometryKernel<VertexType>;

public:
  
  void writeHexMesh(HexMesh &mesh, const std::string &fileName, int iTimestep);

  void writeUnstructuredVTK(HexMesh &mesh, const std::string &fileName, int iTimestep);

  void writeUnstructuredCellData(HexMesh & mesh, const std::string & fileName);
};