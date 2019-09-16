#include <cstdlib>
#include <vector>
#include <algorithm>
#include <iostream>

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>

typedef OpenMesh::PolyMesh_ArrayKernelT<> MyMesh;

void writeOFFMesh(MyMesh& mesh) {

  try {
    if (!OpenMesh::IO::write_mesh(mesh, "output.off")) {
      std::cerr << "Cannot write mesh to file 'output.off'" << std::endl;
      std::exit(EXIT_FAILURE);
    }
  }
  catch( std::exception &x) {
    std::cerr << x.what() << std::endl;
    std::exit(EXIT_FAILURE);
  }

}

void buildSimpleCube() {

  MyMesh mesh;

  MyMesh::VertexHandle vhandle[8];

  MyMesh::Point vertices[] = {
    MyMesh::Point(-1, -1, 1), 
    MyMesh::Point( 1, -1, 1),
    MyMesh::Point( 1,  1, 1),
    MyMesh::Point(-1,  1, 1),
    MyMesh::Point(-1, -1,-1),
    MyMesh::Point( 1, -1,-1),
    MyMesh::Point( 1,  1,-1),
    MyMesh::Point(-1,  1,-1)
  };

  int connectivity[][4] = { {0, 1, 2, 3}, {7, 6, 5, 4}, {1, 0, 4, 5}, {2, 1, 5, 6}, {3, 2, 6, 7}, {0, 3, 7, 4} };

  for (unsigned i(0); i < 8; ++i) {
    vhandle[i] = mesh.add_vertex(vertices[i]);
  }

  std::vector<MyMesh::VertexHandle> face_vhandles;

  for (unsigned i(0); i < 6; ++i) {
    face_vhandles.clear();
    face_vhandles.push_back(vhandle[connectivity[i][0]]);
    face_vhandles.push_back(vhandle[connectivity[i][1]]);
    face_vhandles.push_back(vhandle[connectivity[i][2]]);
    face_vhandles.push_back(vhandle[connectivity[i][3]]);
    mesh.add_face(face_vhandles);
  }

  writeOFFMesh(mesh);

}

int main()
{
  buildSimpleCube();
  return EXIT_SUCCESS;
}
