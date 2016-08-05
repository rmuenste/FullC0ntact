#include <iostream>
#include <application.h>

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/IO/writer/VTKWriter.hh>
#include <OpenMesh/Core/IO/exporter/BaseExporter.hh>
#include <OpenMesh/Core/IO/exporter/ExporterT.hh>

namespace i3d {

  template <typename T>
  class SpringConstraint
  {
  public:
    T ks;
    T kd;
    T l0;
    T l;
    Vector3<T> L;
    T dt;
    SpringConstraint() : ks(T(0)), kd(T(0)), l0(T(0)), l(T(0)), dt(T(0))
    {

    }

    Vector3<T> evalForce(const Vector3<T> &x0, const Vector3<T> &x1, const Vector3<T> &v0, const Vector3<T> &v1, const Vector3<T> &extForce)
    {
      L = (x1 - x0);
      l = L.mag();
      L = (1.0 / l) * L;
      Vector3<T> f = -ks * (l - l0) * L - kd * (v0 - v1) + extForce;
      return f;
    }

  };

  struct MyTraits : public OpenMesh::DefaultTraits
  {
    // store barycenter of neighbors in this member
    VertexTraits
    {
    private:
      Point  cog_;
    public:
      VertexT() : cog_(Point(0.0f, 0.0f, 0.0f)), force_(0,0,0), vel_(0,0,0), pos_old_(0,0,0), mass_(0) { }
      const Point& cog() const { return cog_; }
      void set_cog(const Point& _p) { cog_ = _p; }
      Vec3 force_;
      Vec3 vel_;
      Vec3 pos_old_;
      Real mass_;
  };
    EdgeTraits
    {
    public:
      SpringConstraint<Real> spring_;

    };
  };


  typedef OpenMesh::TriMesh_ArrayKernelT<MyTraits> Mesh;
  typedef OpenMesh::PolyMesh_ArrayKernelT<MyTraits> PolyMesh;

  class OpenMeshTest : public Application {

  public:

    OpenMeshTest() : Application() {

    }

    void init(std::string fileName) {

      Application::init(fileName);

    }

    void run() {

      unsigned nOut = 0;
      //start the main simulation loop

      Mesh mesh_;
      mesh_.clear();

      PolyMesh polyMesh;
      polyMesh.clear();

      OpenMesh::IO::read_mesh(mesh_, "meshes/engrave.obj");
      std::cout << "> Mesh vertices: " << mesh_.n_vertices() << std::endl;

      OpenMesh::IO::read_mesh(polyMesh, "meshes/cloth.obj");
      std::cout << "> PolyMesh vertices: " << polyMesh.n_vertices() << std::endl;
      std::cout << "> PolyMesh edges: " << polyMesh.n_edges() << std::endl;
      std::cout << "> PolyMesh faces: " << polyMesh.n_faces() << std::endl;
      //OpenMesh::IO::write_mesh(polyMesh, "output/cloth.stl");
      OpenMesh::IO::_VTKWriter_ writer;

      OpenMesh::IO::Options            _opt = OpenMesh::IO::Options::Default;
      std::streamsize    _precision = 6;
      OpenMesh::IO::ExporterT<PolyMesh> exporter(polyMesh);
      writer.write("output/cloth.vtk", exporter, _opt, _precision);
    }

  };

}

using namespace i3d;



int main()
{
  
  OpenMeshTest myApp;

  myApp.init("start/sampleRigidBody.xml");

  myApp.run();

  return 0;
}
