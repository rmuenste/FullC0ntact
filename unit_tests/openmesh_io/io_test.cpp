#include <cstdlib>
#include <vector>
#include <algorithm>
#include <iostream>

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>

typedef OpenMesh::TriMesh_ArrayKernelT<> MyMesh;

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
  if (!OpenMesh::IO::read_mesh(mesh, "ico_sphere.off")) {
    std::cerr << "Error: cannot read from file ico_sphere.off" << std::endl;
    std::exit(EXIT_FAILURE);
  }
  return mesh;
}

void simpleSmoother(MyMesh& mesh, int iterations) {

  typedef MyMesh::Point Point;
  typedef MyMesh::Scalar Scalar;
  typedef MyMesh::VertexIter VIter;
  typedef MyMesh::VertexVertexIter VVIter;

  VIter v_it, v_end(mesh.vertices_end());

  VVIter vv_it;

  Point cog;
  Scalar valence;

  std::vector<Point> cogs;
  std::vector<Point>::iterator cog_it;
  cogs.reserve(mesh.n_vertices());

  for (int i(0); i < iterations; ++i) {

    std::cout << "Smoothing started" << std::endl;
    cogs.clear();

    for (v_it = mesh.vertices_begin(); v_it != v_end; ++v_it) {

      cog[0] = cog[1] = cog[2] = valence = 0.0;

      for (vv_it = mesh.vv_iter(*v_it); vv_it.is_valid(); ++vv_it) {
        cog += mesh.point(*vv_it);
        ++valence;
      }

      cogs.push_back(cog / valence);
    }

    for (v_it = mesh.vertices_begin(), cog_it = cogs.begin(); v_it != v_end; ++v_it, ++cog_it) {
      if (!mesh.is_boundary(*v_it))
        mesh.set_point(*v_it, *cog_it);
    }

  }
  std::cout << "Smoothing finished" << std::endl;
}

int main()
{
  MyMesh mesh = readMesh();
  simpleSmoother(mesh, 10);
  writeOFFMesh(mesh);

  return EXIT_SUCCESS;
}
