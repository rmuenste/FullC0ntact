#include <iostream>
#include <application.h>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/IO/writer/VTKWriter.hh>
#include <OpenMesh/Core/IO/exporter/BaseExporter.hh>
#include <OpenMesh/Core/IO/exporter/ExporterT.hh>

namespace i3d {

  struct MyTraits : public OpenMesh::DefaultTraits
  {
    // store barycenter of neighbors in this member
    VertexTraits
    {
    private:
      Point  cog_;
    public:
      VertexT() : cog_(Point(0.0f, 0.0f, 0.0f)), force_(0, 0, 0), vel_(0, 0, 0), pos_old_(0, 0, 0), mass_(0.5), fixed_(false), flagella_(false) { }
      const Point& cog() const { return cog_; }
      void set_cog(const Point& _p) { cog_ = _p; }
      Vec3 force_;
      Vec3 vel_;
      Vec3 pos_old_;
      Real mass_;
      bool fixed_;
      bool flagella_;
      float t;
  };
    //EdgeTraits
    //{
    //public:
    //  SpringConstraint<Real> spring_;

    //};
  };

  typedef OpenMesh::TriMesh_ArrayKernelT<MyTraits> Mesh;
  typedef OpenMesh::PolyMesh_ArrayKernelT<MyTraits> PolyMesh;


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

    PolyMesh::VertexHandle vh0;
    PolyMesh::VertexHandle vh1;

    SpringConstraint() : ks(T(0.5)), kd(T(-0.25)), l0(T(1)), l(T(0)), dt(T(0))
    {
    }

    SpringConstraint(T _ks, T _kd) : ks(_ks), kd(_kd), l0(T(1)), l(T(0)), dt(T(0))
    {
    }

    virtual Vector3<T> evalForce(const Vector3<T> &x0, const Vector3<T> &x1, const Vector3<T> &v0, const Vector3<T> &v1)
    {
      L = (x0 - x1);
      l = L.mag();
      if(l == 0.0f)
        return Vector3<T>(0,0,0);

      // what if vertices coincide?
      Vector3<T> LN = (1.0 / l) * L;
      T tl = -ks * (l - l0);
      T tr = kd * ((v0 - v1) * L)/l;
      Vector3<T> f = (tl + tr) * LN; 

      return f;
    }

  };

  template <typename T>
  class ShearSpringConstraint : public SpringConstraint<T>
  {
  public:
    T ks;
    T kd;
    T l0;
    T l;
    Vector3<T> L;
    T dt;
    ShearSpringConstraint() : ks(T(0.5)), kd(T(-0.25)), l0(T(1)), l(T(0)), dt(T(0))
    {
    }

    virtual Vector3<T> evalForce(const Vector3<T> &x0, const Vector3<T> &x1, const Vector3<T> &v0, const Vector3<T> &v1)
    {
      L = (x0 - x1);
      l = L.mag();
      if (l == 0.0f)
        return Vector3<T>(0, 0, 0);

      // what if vertices coincide?
      Vector3<T> LN = (1.0 / l) * L;
      T tl = -ks * (l - l0);
      T tr = kd * ((v0 - v1) * L) / l;
      Vector3<T> f = (tl + tr) * LN;
      return f;
    }

  };

  class OpenMeshTest : public Application {

  public:

    PolyMesh polyMesh;
    Mesh mesh_;
    std::vector <SpringConstraint<Real>> mysprings_;
    unsigned vrow; // = 21;

    OpenMeshTest() : Application(), vrow(21) {

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

        //Vec3 g(0,-0.00981,0);
        Vec3 g(0,0.00,0);

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

      const float myPI = 3.1415927;
      for(v_it = polyMesh.vertices_begin(); v_it!=v_end; ++v_it)
      {

        if(!polyMesh.data(*v_it).flagella_)
          continue;

        Vec3 &vel = polyMesh.data(*v_it).vel_; 

        Vec3 &force = polyMesh.data(*v_it).force_; 
        float &td = polyMesh.data(*v_it).t; 

        Vec3 pos_old = Vec3(polyMesh.point(*v_it)[0], polyMesh.point(*v_it)[1], polyMesh.point(*v_it)[2]);

        Vec3 pos = Vec3(polyMesh.point(*v_it)[0], polyMesh.point(*v_it)[1], polyMesh.point(*v_it)[2]);
        polyMesh.data(*v_it).pos_old_ = Vec3(polyMesh.point(*v_it)[0], polyMesh.point(*v_it)[1], polyMesh.point(*v_it)[2]);

        pos = pos;

//        float myDeltaT = 2.0f * myPI/20.0f ;
        Real dt = 1.0/60.0;

        pos.y = 8.0f*dt*std::sin(td);
        td += dt;
        
        vel = pos - pos_old;

        PolyMesh::Point p(pos.x, pos.y, pos.z);

        polyMesh.set_point(*v_it, p);

        force = Vec3(0,0,0);

      }

    }

    void simulate()
    {
      for (auto &s : mysprings_)
      {
        PolyMesh::VertexHandle vh0 = s.vh0;
        PolyMesh::VertexHandle vh1 = s.vh1;

        Vec3 x0(polyMesh.point(vh0)[0], polyMesh.point(vh0)[1], polyMesh.point(vh0)[2]);
        Vec3 v0(polyMesh.data(vh0).vel_);
        Vec3 x1(polyMesh.point(vh1)[0], polyMesh.point(vh1)[1], polyMesh.point(vh1)[2]);
        Vec3 v1(polyMesh.data(vh1).vel_);


        Vec3 f = s.evalForce(x0, x1, v0, v1);
        polyMesh.data(vh0).force_ += f;
        polyMesh.data(vh1).force_ -= f;
      }
      integrate();
      addProvotDynamicInverse();
    }

    void addProvotDynamicInverse() {

      for (auto &s : mysprings_)
      {
        PolyMesh::VertexHandle vh0 = s.vh0;
        PolyMesh::VertexHandle vh1 = s.vh1;

        Vec3 x0(polyMesh.point(vh0)[0], polyMesh.point(vh0)[1], polyMesh.point(vh0)[2]);
        Vec3 v0(polyMesh.data(vh0).vel_);
        Vec3 x1(polyMesh.point(vh1)[0], polyMesh.point(vh1)[1], polyMesh.point(vh1)[2]);
        Vec3 v1(polyMesh.data(vh1).vel_);

        Vec3 deltaP = x0 - x1;
        float dist = deltaP.mag();
        if (dist > s.l0)
        {
          dist -= (s.l0);
          dist /= 2.0f;
          deltaP.normalize();
          deltaP *= dist;
          if (polyMesh.data(vh0).fixed_) {
            polyMesh.data(vh1).vel_ += deltaP;
          }
          else if (polyMesh.data(vh1).fixed_) {
            polyMesh.data(vh0).vel_ -= deltaP;
          }
          else {
            polyMesh.data(vh0).vel_ -= deltaP;
            polyMesh.data(vh1).vel_ += deltaP;
          }
        }
      }
    }

    void init()
    {
      PolyMesh::EdgeIter e_it =polyMesh.edges_begin();
      // Add structural springs
      for(; e_it != polyMesh.edges_end(); ++e_it)
      {
        PolyMesh::VertexHandle vh0 = polyMesh.to_vertex_handle(polyMesh.halfedge_handle(*e_it,0));
        PolyMesh::VertexHandle vh1 = polyMesh.to_vertex_handle(polyMesh.halfedge_handle(*e_it,1));

        Vec3 x0(polyMesh.point(vh0)[0], polyMesh.point(vh0)[1], polyMesh.point(vh0)[2]);
        Vec3 v0(polyMesh.data(vh0).vel_);
        Vec3 x1(polyMesh.point(vh1)[0], polyMesh.point(vh1)[1], polyMesh.point(vh1)[2]);
        Vec3 v1(polyMesh.data(vh1).vel_);

        SpringConstraint<Real> s;
        s.l0 = (x0 - x1).mag();
        s.vh0 = vh0;
        s.vh1 = vh1;
        mysprings_.push_back(s);
      }

      std::cout << "> Number of structural springs: " << polyMesh.n_edges() << std::endl;
      e_it =polyMesh.edges_begin();
      std::cout << "> Rest length: " << mysprings_.front().l0 << std::endl;
      std::cout << "> Number of shear springs: " << polyMesh.n_faces() * 2 << std::endl;

      // Add shear springs
      PolyMesh::FaceIter f_it, f_end(polyMesh.faces_end());
      for (unsigned y(0); y < (vrow - 1); ++y)
      {
        for (unsigned x(0); x < (vrow - 1); ++x)
        {
          PolyMesh::VertexHandle baseHandle = polyMesh.vertex_handle((y * vrow) + x);

          SpringConstraint<Real> s(0.5,-0.25);
          Vec3 x0(polyMesh.point(baseHandle)[0], polyMesh.point(baseHandle)[1], polyMesh.point(baseHandle)[2]);
          s.vh0 = baseHandle;

          baseHandle = polyMesh.vertex_handle( (y + 1) * vrow + x + 1);
          s.vh1 = baseHandle;
          Vec3 x1(polyMesh.point(baseHandle)[0], polyMesh.point(baseHandle)[1], polyMesh.point(baseHandle)[2]);

          s.l0 = (x0 - x1).mag();
          mysprings_.push_back(s);

          SpringConstraint<Real> s1(0.5, -0.25);
          baseHandle = polyMesh.vertex_handle( (y + 1) * vrow + x );
          s1.vh0 = baseHandle;
          x0 = Vec3(polyMesh.point(baseHandle)[0], polyMesh.point(baseHandle)[1], polyMesh.point(baseHandle)[2]);

          baseHandle = polyMesh.vertex_handle( (y * vrow) + x + 1);
          s1.vh1 = baseHandle;
          x1 = Vec3(polyMesh.point(baseHandle)[0], polyMesh.point(baseHandle)[1], polyMesh.point(baseHandle)[2]);

          s1.l0 = (x0 - x1).mag();
          mysprings_.push_back(s1);

        }
      }

      // Add Bend Springs
      for (unsigned y(0); y < (vrow); ++y)
      {
        for (unsigned x(0); x < (vrow - 2); ++x)
        {
          // Horizontal bend springs
          PolyMesh::VertexHandle baseHandle = polyMesh.vertex_handle((y * vrow) + x);

          SpringConstraint<Real> s(0.85,-0.25);
          Vec3 x0(polyMesh.point(baseHandle)[0], polyMesh.point(baseHandle)[1], polyMesh.point(baseHandle)[2]);
          s.vh0 = baseHandle;

          baseHandle = polyMesh.vertex_handle( (y * vrow) + x + 2);
          s.vh1 = baseHandle;
          Vec3 x1(polyMesh.point(baseHandle)[0], polyMesh.point(baseHandle)[1], polyMesh.point(baseHandle)[2]);

          s.l0 = (x0 - x1).mag();
          mysprings_.push_back(s);

        }
          // Last column Horizontal bend springs
          PolyMesh::VertexHandle baseHandle = polyMesh.vertex_handle((y * vrow) + (vrow - 3));
          SpringConstraint<Real> s(0.85,-0.25);
          Vec3 x0(polyMesh.point(baseHandle)[0], polyMesh.point(baseHandle)[1], polyMesh.point(baseHandle)[2]);
          s.vh0 = baseHandle;

          baseHandle = polyMesh.vertex_handle((y * vrow) + (vrow - 1));
          s.vh1 = baseHandle;
          Vec3 x1(polyMesh.point(baseHandle)[0], polyMesh.point(baseHandle)[1], polyMesh.point(baseHandle)[2]);

          s.l0 = (x0 - x1).mag();
          mysprings_.push_back(s);
      }
      
      // Add Bend Springs
      for (unsigned y(0); y < (vrow); ++y)
      {
        // Vertical bend springs
        for (unsigned x(0); x < (vrow - 2); ++x)
        {
          PolyMesh::VertexHandle baseHandle = polyMesh.vertex_handle((x * vrow) + y);

          SpringConstraint<Real> s(0.85,-0.25);
          Vec3 x0(polyMesh.point(baseHandle)[0], polyMesh.point(baseHandle)[1], polyMesh.point(baseHandle)[2]);
          s.vh0 = baseHandle;

          baseHandle = polyMesh.vertex_handle( ((x + 2) * vrow) + y);
          s.vh1 = baseHandle;
          Vec3 x1(polyMesh.point(baseHandle)[0], polyMesh.point(baseHandle)[1], polyMesh.point(baseHandle)[2]);

          s.l0 = (x0 - x1).mag();
          mysprings_.push_back(s);

        }
          // Last row of vertical bend springs
          PolyMesh::VertexHandle baseHandle = polyMesh.vertex_handle(((vrow -3) * vrow) + y);
          SpringConstraint<Real> s(0.85,-0.25);
          Vec3 x0(polyMesh.point(baseHandle)[0], polyMesh.point(baseHandle)[1], polyMesh.point(baseHandle)[2]);
          s.vh0 = baseHandle;

          baseHandle = polyMesh.vertex_handle(((vrow - 1) * vrow) + y) ;
          s.vh1 = baseHandle;
          Vec3 x1(polyMesh.point(baseHandle)[0], polyMesh.point(baseHandle)[1], polyMesh.point(baseHandle)[2]);

          s.l0 = (x0 - x1).mag();
          mysprings_.push_back(s);
      }

      PolyMesh::VertexIter v_it, v_end(polyMesh.vertices_end()); 
      for(v_it = polyMesh.vertices_begin(); v_it!=v_end; ++v_it)
      {
        Vec3 p(polyMesh.point(*v_it)[0], polyMesh.point(*v_it)[1], polyMesh.point(*v_it)[2]);

        if((*v_it).idx() == 0 || (*v_it).idx() == 20 || (*v_it).idx() == 420 || (*v_it).idx() == 440)
        {
          polyMesh.data(*v_it).fixed_ = true;
        }
      }

      int j(0);
      const float myPI = 3.1415927;
      for(int i(10); i <= 430; i+=21)
      {
        PolyMesh::VertexHandle baseHandle = polyMesh.vertex_handle(i);
        polyMesh.data(baseHandle).flagella_ = true;
        polyMesh.data(baseHandle).t = j * 2.0f * myPI/20.0f ;
        ++j;
      }

    }

    void run() {

      int steps = 10000;
      //start the main simulation loop

      mesh_.clear();
      polyMesh.clear();

      OpenMesh::IO::read_mesh(mesh_, "meshes/engrave.obj");
      std::cout << "> Mesh vertices: " << mesh_.n_vertices() << std::endl;

      OpenMesh::IO::read_mesh(polyMesh, "meshes/mycloth20.obj");
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
