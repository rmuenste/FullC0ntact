#include <cstdlib>
#include <unstructuredgrid.h>
#include <vtkwriter.h>

// Include vector classes
#include <OpenVolumeMesh/Geometry/VectorT.hh>
#include <OpenVolumeMesh/Attribs/NormalAttrib.hh>

// File Manager
#include <OpenVolumeMesh/FileManager/FileManager.hh>

// Include polyhedral mesh kernel
#include <OpenVolumeMesh/Mesh/HexahedralMesh.hh>

// Make some typedefs to facilitate your life
using ScalarType = double;
using VertexType = OpenVolumeMesh::Geometry::VectorT<ScalarType, 3>;
using HexMesh = OpenVolumeMesh::GeometryKernel<VertexType>;

i3d::UnstructuredGrid<double> grid;

int main(int argc, char *argv[]) {

  HexMesh mesh;

  //grid.initMeshFromFile("mesh2x2x4.tri");
  grid.initCube(-1, -1, -1, 1, 1, 1);
  grid.initStdMesh();
  grid.refine();
  grid.initStdMesh();

  i3d::CVtkWriter writer;
  //writer.WriteUnstr(grid, "mesh2x2x4.vtk");
  writer.WriteUnstr(grid, "meshref.vtk");

//  for (int vidx(0); vidx < grid.nvt_; ++vidx) {
//    VertexType vert(grid.vertexCoords_[vidx].x, grid.vertexCoords_[vidx].y, grid.vertexCoords_[vidx].z);
//    mesh.add_vertex(vert);
//  }
//
//  std::vector<OpenVolumeMesh::FaceHandle> faceHandles;
//
//  for (int fidx(0); fidx < grid.nat_; ++fidx) {
//    std::vector<OpenVolumeMesh::VertexHandle> vertices;
//    for (int j(0); j < 4; ++j)
//      vertices.push_back(OpenVolumeMesh::VertexHandle(grid.verticesAtFace_[fidx].faceVertexIndices_[j]));
//
//    faceHandles.push_back(mesh.add_face(vertices));
//  }
//
//  for (int eidx(0); eidx < grid.nel_; ++eidx) {
//
//    std::vector<OpenVolumeMesh::HalfFaceHandle> halffaces;
//    OpenVolumeMesh::FaceHandle f0(grid.hexas_[eidx].hexaFaceIndices_[0]);
//    halffaces.push_back(mesh.halfface_handle(f0, 1));
//
//    OpenVolumeMesh::FaceHandle f1(grid.hexas_[eidx].hexaFaceIndices_[1]);
//    halffaces.push_back(mesh.halfface_handle(f1, 1));
//
//    OpenVolumeMesh::FaceHandle f2(grid.hexas_[eidx].hexaFaceIndices_[2]);
//    halffaces.push_back(mesh.halfface_handle(f2, 0));
//
//    OpenVolumeMesh::FaceHandle f3(grid.hexas_[eidx].hexaFaceIndices_[3]);
//    halffaces.push_back(mesh.halfface_handle(f3, 1));
//
//    OpenVolumeMesh::FaceHandle f4(grid.hexas_[eidx].hexaFaceIndices_[4]);
//    halffaces.push_back(mesh.halfface_handle(f4, 0));
//
//    OpenVolumeMesh::FaceHandle f5(grid.hexas_[eidx].hexaFaceIndices_[5]);
//    halffaces.push_back(mesh.halfface_handle(f5, 1));
//
//    mesh.add_cell(halffaces);
//  }
//
//  OpenVolumeMesh::IO::FileManager fileManager;
//  fileManager.writeFile("mesh2x2x4.ovm", mesh);

  return EXIT_SUCCESS;
}