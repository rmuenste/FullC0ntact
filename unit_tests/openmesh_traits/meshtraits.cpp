#include <cstdlib>
#include <vector>
#include <algorithm>
#include <iostream>

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Geometry/VectorT.hh>

struct MyTraits : OpenMesh::DefaultTraits {
  // Point and Normal
  typedef OpenMesh::Vec3d Point;
  typedef OpenMesh::Vec3d Normal;
};

typedef OpenMesh::TriMesh_ArrayKernelT<MyTraits> MyMesh;

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

MyMesh readMesh() {
  MyMesh mesh;

  if (typeid(OpenMesh::vector_traits<MyMesh::Point>::value_type) != typeid(double)) {
    std::cerr << "Data type error" << std::endl;
    std::exit(EXIT_FAILURE);
  }

  if (typeid(OpenMesh::vector_traits<MyMesh::Normal>::value_type) != typeid(double)) {
    std::cerr << "Data type error" << std::endl;
    std::exit(EXIT_FAILURE);
  }

  mesh.request_vertex_normals();
  mesh.request_face_normals();

  OpenMesh::IO::Options opt;

  if (!OpenMesh::IO::read_mesh(mesh, "input.off", opt)) {
    std::cerr << "Error: cannot read from file ico_sphere.off" << std::endl;
    std::exit(EXIT_FAILURE);
  }

  if (!opt.check(OpenMesh::IO::Options::VertexNormal) && mesh.has_face_normals() && mesh.has_vertex_normals()) {
    mesh.update_normals();
  }

  return mesh;
}

void scaleAlongNormal(MyMesh& mesh) {
  typedef MyMesh::VertexIter Viter;
  for (Viter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it) {
    std::cout << "Vertex #" << *v_it << ": " << mesh.point(*v_it);
    mesh.set_point(*v_it, mesh.point(*v_it) + mesh.normal(*v_it));
    std::cout << "moved to " << mesh.point( *v_it ) << std::endl;
  }
}

int main()
{
  MyMesh mesh = readMesh();

  scaleAlongNormal(mesh);

  writeOFFMesh(mesh);

  return EXIT_SUCCESS;
}
