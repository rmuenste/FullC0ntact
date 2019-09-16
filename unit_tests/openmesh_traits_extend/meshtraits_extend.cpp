#include <cstdlib>
#include <vector>
#include <algorithm>
#include <iostream>

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Mesh/Traits.hh>
#include <OpenMesh/Core/Geometry/VectorT.hh>

struct MyTraits : OpenMesh::DefaultTraits {
  // Define VertexTraits
  VertexTraits{

    private:
      Point cog_;
    public:
      VertexT() : cog_(Point(0.0f, 0.0f, 0.0f)) {}

      const Point& cog() const { return cog_; }

      void set_cog(const Point& p) { cog_ = p; }

  };
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

  OpenMesh::IO::Options opt;

  if (!OpenMesh::IO::read_mesh(mesh, "input.off", opt)) {
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

  for (int i(0); i < iterations; ++i) {

    std::cout << "Smoothing started" << std::endl;

    for (v_it = mesh.vertices_begin(); v_it != v_end; ++v_it) {

      cog[0] = cog[1] = cog[2] = valence = 0.0;

      for (vv_it = mesh.vv_iter(*v_it); vv_it.is_valid(); ++vv_it) {
        cog += mesh.point(*vv_it);
        ++valence;
      }

      mesh.data(*v_it).set_cog(cog / valence);
    }

    for (v_it = mesh.vertices_begin(); v_it != v_end; ++v_it) {
      if (!mesh.is_boundary(*v_it))
        mesh.set_point(*v_it, mesh.data(*v_it).cog());
    }

  }
  std::cout << "Smoothing finished" << std::endl;
}

void outputVertex(const MyMesh::VertexHandle vh, MyMesh &mesh) {
    std::cout << mesh.point(vh) << std::endl;
}

int main()
{
  MyMesh mesh = readMesh();
  simpleSmoother(mesh, 10);
  writeOFFMesh(mesh);

  MyMesh::VertexHandle myCoolVertexHandle = mesh.vertex_handle(0);

  std::for_each(mesh.vertices_begin(), mesh.vertices_end(), [&](auto vh) {
    std::cout << mesh.point(vh) << std::endl;
  });

  return EXIT_SUCCESS;
}
