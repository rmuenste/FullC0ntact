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
    SpringConstraint() : ks(T(20.0)), kd(T(0)), l0(T(1)), l(T(0)), dt(T(0))
    {
      l0 = 0.125;
      kd = 0.5;
    }

    Vector3<T> evalForce(const Vector3<T> &x0, const Vector3<T> &x1, const Vector3<T> &v0, const Vector3<T> &v1, const Vector3<T> &extForce)
    {
      L = (x1 - x0);
      l = L.mag();

      // what if vertices coincide?
      L = (1.0 / l) * L;
      Vector3<T> f = ks * (l - l0) * L + kd * (v0 - v1) + extForce;
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
      VertexT() : cog_(Point(0.0f, 0.0f, 0.0f)), force_(0,0,0), vel_(0,0,0), pos_old_(0,0,0), mass_(1) { }
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

    PolyMesh polyMesh;
    Mesh mesh_;

    OpenMeshTest() : Application() {

    }

    void init(std::string fileName) {

      Application::init(fileName);

    }

    void integrate()
    {
      Real dt = 0.001;
      PolyMesh::VertexIter v_it, v_end(polyMesh.vertices_end()); 
      for(v_it = polyMesh.vertices_begin(); v_it!=v_end; ++v_it)
      {
        Vec3 &vel = polyMesh.data(*v_it).vel_; 
        Vec3 &force = polyMesh.data(*v_it).force_; 
        Real &m = polyMesh.data(*v_it).mass_; 
        Vec3 pos = Vec3(polyMesh.point(*v_it)[0], polyMesh.point(*v_it)[1], polyMesh.point(*v_it)[2]);
        
        vel = vel + dt * force * 1.0/m;

        polyMesh.data(*v_it).pos_old_ = Vec3(polyMesh.point(*v_it)[0], polyMesh.point(*v_it)[1], polyMesh.point(*v_it)[2]);

        pos = pos + dt * vel;

        //std::cout << "> old pos: " << polyMesh.data(*v_it).pos_old_ << std::endl;
        //std::cout << "> pos: " << pos << std::endl;
        PolyMesh::Point p(pos.x, pos.y, pos.z);

        polyMesh.set_point(v_it, p);

      }
    }

    void simulate()
    {
      PolyMesh::VertexIter v_it, v_end(polyMesh.vertices_end()); 
      for(v_it = polyMesh.vertices_begin(); v_it!=v_end; ++v_it)
      {
        int id0 = (*v_it).idx(); 
        //std::cout << "> Vertex: [" << (*v_it).idx() << "] " << polyMesh.point(*v_it) << std::endl;
        //std::cout << "> Vertex traits: " << polyMesh.data(*v_it).cog() << std::endl; //.spring_.l << std::endl;
        PolyMesh::VertexEdgeIter ve = polyMesh.ve_iter(*v_it);
        PolyMesh::VertexHandle heh0 = (*v_it);
        Vec3 x0(polyMesh.point(*v_it)[0], polyMesh.point(*v_it)[1], polyMesh.point(*v_it)[2]);
        Vec3 v0(polyMesh.data(heh0).vel_);
        for(; ve.is_valid(); ++ve)
        {
          //std::cout << "> Edge: " << *ve << std::endl;
          //std::cout << "> Edge traits: " << polyMesh.data(*ve).spring_.l << std::endl;
          //std::cout << "> Edge point (0): " << polyMesh.to_vertex_handle(polyMesh.halfedge_handle(*ve,0)).idx() << std::endl;
          //std::cout << "> Edge point (1): " << polyMesh.to_vertex_handle(polyMesh.halfedge_handle(*ve,1)).idx() << std::endl;

          int he0 = polyMesh.to_vertex_handle(polyMesh.halfedge_handle(*ve,0)).idx();
          int he1 = polyMesh.to_vertex_handle(polyMesh.halfedge_handle(*ve,1)).idx();

          PolyMesh::VertexHandle vh0 = polyMesh.to_vertex_handle(polyMesh.halfedge_handle(*ve,0));
          PolyMesh::VertexHandle vh1 = polyMesh.to_vertex_handle(polyMesh.halfedge_handle(*ve,1));

          int id1 = (id0 != he0) ? (he0) : (he1); 
          PolyMesh::VertexHandle heh1 = (id0 != he0) ? (vh0) : (vh1); 

          //std::cout << "> Edge point (0): " << id0 << std::endl;
          //std::cout << "> Edge point (1): " << id1 << std::endl;
          //std::cout << "> Vertex traits: " << polyMesh.data(heh1).cog() << std::endl; //.spring_.l << std::endl;
          //Vector3<T> evalForce(const Vector3<T> &x0, const Vector3<T> &x1, const Vector3<T> &v0, const Vector3<T> &v1, const Vector3<T> &extForce)
          Vec3 x1(polyMesh.point(heh1)[0], polyMesh.point(heh1)[1], polyMesh.point(heh1)[2]);
          Vec3 v1(polyMesh.data(heh1).vel_);
          Vec3 ext(0, 9.81, 0.0);

          if (x0.mag() > 0.01)
            ext.y = 0.0;

          Vec3 f = polyMesh.data(*ve).spring_.evalForce(x0, x1, v0, v1, ext); 
          polyMesh.data(*v_it).force_ += f;
        }
        //std::cout << "> Total force: " << polyMesh.data(*v_it).force_;
        
      }
      integrate();
    }

    void run() {

      unsigned nOut = 0;
      //start the main simulation loop

      mesh_.clear();
      polyMesh.clear();

      OpenMesh::IO::read_mesh(mesh_, "meshes/engrave.obj");
      std::cout << "> Mesh vertices: " << mesh_.n_vertices() << std::endl;

      OpenMesh::IO::read_mesh(polyMesh, "meshes/cloth_high.obj");
      std::cout << "> PolyMesh vertices: " << polyMesh.n_vertices() << std::endl;
      std::cout << "> PolyMesh edges: " << polyMesh.n_edges() << std::endl;
      std::cout << "> PolyMesh faces: " << polyMesh.n_faces() << std::endl;
      //OpenMesh::IO::write_mesh(polyMesh, "output/cloth.stl");
      OpenMesh::IO::_VTKWriter_ writer;

      for(int istep(0); istep < 100; ++istep)
      {

        simulate();
        std::ostringstream name;
        name << "output/cloth." << std::setfill('0') << std::setw(5) << istep << ".vtk";
        OpenMesh::IO::Options            _opt = OpenMesh::IO::Options::Default;
        std::streamsize            _precision = 6;
        OpenMesh::IO::ExporterT<PolyMesh> exporter(polyMesh);
        writer.write(name.str().c_str(), exporter, _opt, _precision);

      }

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
