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
    SpringConstraint() : ks(T(0.6)), kd(T(-0.25)), l0(T(1)), l(T(0)), dt(T(0))
    {
    }

    Vector3<T> evalForce(const Vector3<T> &x0, const Vector3<T> &x1, const Vector3<T> &v0, const Vector3<T> &v1)
    {
      L = (x0 - x1);
      l = L.mag();
      if(l == 0.0f)
        return Vector3<T>(0,0,0);

      // what if vertices coincide?
      Vector3<T> LN = (1.0 / l) * L;
      T tl = -ks * (l - l0);
      T tr = kd * ((v0 - v1) * L)/l;
//      std::cout << "> l: " << l << std::endl;
//      std::cout << "> L: " << L << std::endl;
//      std::cout << "> lt: " << tl << std::endl;
//      std::cout << "> rt: " << tr << std::endl;
      Vector3<T> f = (tl + tr) * LN; 
//      std::cout << "> force: " << f << std::endl;

      //Vector3<T> f = (ks * (l - l0) + kd * ((v0 - v1) * L)) * LN; 

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
      VertexT() : cog_(Point(0.0f, 0.0f, 0.0f)), force_(0,0,0), vel_(0,0,0), pos_old_(0,0,0), mass_(0.5), fixed_(false) { }
      const Point& cog() const { return cog_; }
      void set_cog(const Point& _p) { cog_ = _p; }
      Vec3 force_;
      Vec3 vel_;
      Vec3 pos_old_;
      Real mass_;
      bool fixed_;
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
      Real dt = 1.0/60.0;
      PolyMesh::VertexIter v_it, v_end(polyMesh.vertices_end()); 
      for(v_it = polyMesh.vertices_begin(); v_it!=v_end; ++v_it)
      {

        if(polyMesh.data(*v_it).fixed_)
          continue;

        Vec3 &vel = polyMesh.data(*v_it).vel_; 

        Vec3 &force = polyMesh.data(*v_it).force_; 
        Vec3 g(0,-0.00981,0);
        force += g;

        force += -0.0125 * vel;

        Real &m = polyMesh.data(*v_it).mass_; 
        Vec3 pos = Vec3(polyMesh.point(*v_it)[0], polyMesh.point(*v_it)[1], polyMesh.point(*v_it)[2]);

        
        vel = vel + dt * force * (1.0/m);

        polyMesh.data(*v_it).pos_old_ = Vec3(polyMesh.point(*v_it)[0], polyMesh.point(*v_it)[1], polyMesh.point(*v_it)[2]);

        pos = pos + dt * vel;

        //std::cout << "> old pos: " << polyMesh.data(*v_it).pos_old_ << std::endl;
        //std::cout << "> pos: " << pos << std::endl;
        PolyMesh::Point p(pos.x, pos.y, pos.z);

        polyMesh.set_point(*v_it, p);
        force = Vec3(0,0,0);

      }
    }

    void simulate()
    {
      PolyMesh::EdgeIter e_it =polyMesh.edges_begin();
      for(; e_it != polyMesh.edges_end(); ++e_it)
      {

//        std::cout << "> Edge: " << *e_it << std::endl;
//        std::cout << "> Edge traits: " << polyMesh.data(*e_it).spring_.l0 << std::endl;
//        std::cout << "> Edge point (0):st " << polyMesh.to_vertex_handle(polyMesh.halfedge_handle(*e_it,0)).idx() << std::endl;
//        std::cout << "> Edge point (1): " << polyMesh.to_vertex_handle(polyMesh.halfedge_handle(*e_it,1)).idx() << std::endl;
        
        PolyMesh::VertexHandle vh0 = polyMesh.to_vertex_handle(polyMesh.halfedge_handle(*e_it,0));
        PolyMesh::VertexHandle vh1 = polyMesh.to_vertex_handle(polyMesh.halfedge_handle(*e_it,1));

        Vec3 x0(polyMesh.point(vh0)[0], polyMesh.point(vh0)[1], polyMesh.point(vh0)[2]);
        Vec3 v0(polyMesh.data(vh0).vel_);
        Vec3 x1(polyMesh.point(vh1)[0], polyMesh.point(vh1)[1], polyMesh.point(vh1)[2]);
        Vec3 v1(polyMesh.data(vh1).vel_);


        Vec3 f = polyMesh.data(*e_it).spring_.evalForce(x0, x1, v0, v1); 
        polyMesh.data(vh0).force_ += f;
        polyMesh.data(vh1).force_ -= f;
      }
      integrate();
    }

    void init()
    {
      PolyMesh::EdgeIter e_it =polyMesh.edges_begin();
      for(; e_it != polyMesh.edges_end(); ++e_it)
      {
        PolyMesh::VertexHandle vh0 = polyMesh.to_vertex_handle(polyMesh.halfedge_handle(*e_it,0));
        PolyMesh::VertexHandle vh1 = polyMesh.to_vertex_handle(polyMesh.halfedge_handle(*e_it,1));

        Vec3 x0(polyMesh.point(vh0)[0], polyMesh.point(vh0)[1], polyMesh.point(vh0)[2]);
        Vec3 v0(polyMesh.data(vh0).vel_);
        Vec3 x1(polyMesh.point(vh1)[0], polyMesh.point(vh1)[1], polyMesh.point(vh1)[2]);
        Vec3 v1(polyMesh.data(vh1).vel_);

        polyMesh.data(*e_it).spring_.l0 = (x0-x1).mag();
      }

      std::cout << "> Number of structural springs: " << polyMesh.n_edges() << std::endl;
      e_it =polyMesh.edges_begin();
      std::cout << "> Rest length: " << polyMesh.data(*e_it).spring_.l0 << std::endl;

      PolyMesh::VertexIter v_it, v_end(polyMesh.vertices_end()); 
      for(v_it = polyMesh.vertices_begin(); v_it!=v_end; ++v_it)
      {
        Vec3 p(polyMesh.point(*v_it)[0], polyMesh.point(*v_it)[1], polyMesh.point(*v_it)[2]);

//        std::cout << "> idx: " << (*v_it).idx() << std::endl;
//        std::cout << "> pos: " << p << std::endl;
        if((*v_it).idx() == 2 || (*v_it).idx() == 285)
        {
          polyMesh.data(*v_it).fixed_ = true;
        }
      }
    }

    void run() {

      int steps = 10000;
      //start the main simulation loop

      mesh_.clear();
      polyMesh.clear();

      OpenMesh::IO::read_mesh(mesh_, "meshes/engrave.obj");
      std::cout << "> Mesh vertices: " << mesh_.n_vertices() << std::endl;

      OpenMesh::IO::read_mesh(polyMesh, "meshes/cloth2.obj");
      std::cout << "> PolyMesh vertices: " << polyMesh.n_vertices() << std::endl;
      std::cout << "> PolyMesh edges: " << polyMesh.n_edges() << std::endl;
      std::cout << "> PolyMesh faces: " << polyMesh.n_faces() << std::endl;
      //OpenMesh::IO::write_mesh(polyMesh, "output/cloth.stl");
      OpenMesh::IO::_VTKWriter_ writer;

      init();

      for(int istep(0); istep < steps; ++istep)
      {

        simulate();
        std::ostringstream name;
        name << "output/cloth." << std::setfill('0') << std::setw(5) << istep << ".vtk";
        OpenMesh::IO::Options            _opt = OpenMesh::IO::Options::Default;
        std::streamsize            _precision = 6;
        OpenMesh::IO::ExporterT<PolyMesh> exporter(polyMesh);
        writer.write(name.str().c_str(), exporter, _opt, _precision);
        std::cout << "> Time step: " << istep << std::endl;

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
