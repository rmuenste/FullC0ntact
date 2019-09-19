#include <cstdlib>
#include <vector>
#include <algorithm>
#include <iostream>

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Mesh/Traits.hh>
#include <OpenMesh/Core/Geometry/VectorT.hh>

#include "mesh_creation.hpp"

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

template <typename T>
void writeOFFMesh(T& mesh) {

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

void dihedralAngle() {

  MyMesh mesh_;
  MyMesh::VertexHandle vhandle[4];

  MyMesh::Point vertices[] = {
    MyMesh::Point(-2, 5, 0),
    MyMesh::Point(-1.8, 5, 0),
    MyMesh::Point(-2, 5, 0.2),
    MyMesh::Point(-1.8, 5, 0.2)
  };

  int connectivity[][3] = { {0, 2, 3}, {0, 3, 1} };

  for (unsigned i(0); i < 4; ++i) {
    vhandle[i] = mesh_.add_vertex(vertices[i]);
  }

  std::vector<MyMesh::VertexHandle> face_vhandles;

  for (unsigned i(0); i < 2; ++i) {
    face_vhandles.clear();
    face_vhandles.push_back(vhandle[connectivity[i][0]]);
    face_vhandles.push_back(vhandle[connectivity[i][1]]);
    face_vhandles.push_back(vhandle[connectivity[i][2]]);
    mesh_.add_face(face_vhandles);
  }

  mesh_.request_face_normals();
  mesh_.request_vertex_normals();

  mesh_.update_normals();

  if (!mesh_.has_face_normals())
  {
    std::cerr << "ERROR: Standard face property 'Normals' not available!\n";
    std::exit(EXIT_FAILURE);
  }

  if (!mesh_.has_vertex_normals())
  {
    std::cerr << "ERROR: Standard vertex property 'Normals' not available!\n";
    std::exit(EXIT_FAILURE);
  }


  OpenMesh::Vec3f normal1 = mesh_.normal(mesh_.face_handle(0));
  OpenMesh::Vec3f normal2 = mesh_.normal(mesh_.face_handle(1));

  std::cout << std::acos(OpenMesh::dot(normal1, normal2)) << std::endl;

  writeOFFMesh(mesh_);
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


  typedef OpenMesh::TriMesh_ArrayKernelT<> DefaultMesh;

  DefaultMesh theMesh = generatePlaneMesh();

  writeOFFMesh(theMesh);

  return EXIT_SUCCESS;
}
