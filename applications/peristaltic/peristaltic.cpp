#include <iostream>
#include <application.h>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/IO/writer/VTKWriter.hh>
#include <OpenMesh/Core/IO/exporter/BaseExporter.hh>
#include <OpenMesh/Core/IO/exporter/ExporterT.hh>
#include <softbody.hpp>

namespace i3d {

  struct MyTraits : public OpenMesh::DefaultTraits
  {
    // store barycenter of neighbors in this member
    VertexTraits
    {
    private:
      Point  cog_;
    public:
      VertexT() : cog_(Point(0.0f, 0.0f, 0.0f)) { }
      const Point& cog() const { return cog_; }
      void set_cog(const Point& _p)
      {
         cog_ = _p;
      }
    };
  };

  typedef OpenMesh::TriMesh_ArrayKernelT<MyTraits> Mesh;
  typedef OpenMesh::PolyMesh_ArrayKernelT<MyTraits> PolyMesh;


  template<>
  class PeristalticSwimmer<Real, Mesh > : public BasicSoftBody<Real, Mesh>
  {
  public:

    int istep_;

    Real A_;

    Real q_;

    Real N_;

    Real f_;

    Real m_;

    Real k_tail_;

    Real kd_;

    Real ks_;

    Real a0_;

    Real l0_;

    Real dt_;

    Real rho_;

    Real massAll_;

    Real volume_;

    int n_mid_;

    int n_head_;

    int offset_;

    Vec3 com_;

    Vec3 velocity_;

    Transformationr transform_;

    std::vector< SimpleSpringConstraint<Real> > springs_;

    std::vector< Vector3<Real> > u_;

    std::vector< Vector3<Real> > force_;

    int up_;

    int strokeCount_;

    int nForce_;

    PeristalticSwimmer() : A_(3.2), N_(60), f_(1.0 / 60.0), a0_(1.0), l0_(1.0*a0_)
    {
      transform_.setOrigin(Vec3(0, 0, 0));

      u_.reserve(N_);
      force_.reserve(N_);

      velocity_ = Vec3(0, 0, 0);

      up_ = true;

      strokeCount_ = 0;

      nForce_ = 4;
    };

    virtual ~PeristalticSwimmer() {};

    void calcCOM()
    {
    }

    Vec3 getCOM()
    {
    }

    void setOrigin(const Vec3 &origin)
    {
    }

    void calcVolume()
    {
    }

    bool isInBody(const Vec3 &vQuery, int &id) const
    {
      return false;
    }

    Vec3 getVelocity(const Vec3 &vQuery, int ind)
    {
      Vec3 velocity(0, 0, 0);
      return velocity;
    }

    void configureStroke(Real t, int istep)
    {
    }

    void step(Real t, Real dt, int it)
    {
      dt_ = dt;

      configureStroke(t, it);

      internalForce(t, it);

      integrate();
    }

    void internalForce(Real t, int istep)
    {


      for (unsigned i(0); i < springs_.size(); ++i)
      {
        SimpleSpringConstraint<Real> &spring = springs_[i];

        Vector3<Real> f = spring.evalForce();

        force_[spring.i0_] += f;
        force_[spring.i1_] -= f;
      }

      //std::cout << "> Force end: " << force_[99].z << " (pg*micrometer)/s^2 " <<std::endl; 

    };

    void applyForce(Real dt)
    {

    };


    void init()
    {

      Real xx = 0 * l0_;
      Real yy = 0;

      ks_ = 320.0;
      kd_ = -0.2;

      u_.push_back(Vec3(0, 0, 0));

      force_.push_back(Vec3(0, 0, 0));

      for (int k = 1; k < N_; ++k)
      {

        Real x = Real(k) * l0_;

        Real y = 0;

        u_.push_back(Vec3(0, 0, 0));
        force_.push_back(Vec3(0, 0, 0));

//        springs_.push_back(
//          SimpleSpringConstraint<Real>(ks_, kd_, l0_, k - 1, k,
//            &geom_.vertices_[k - 1],
//            &geom_.vertices_[k],
//            &u_[k - 1],
//            &u_[k]
//            ));

      }

    };

    void integrate()
    {

      std::vector<Vector3<Real>> &u0 = u_;
      std::vector<Vector3<Real>> &f0 = force_;

      for (int i(0); i < N_; ++i)
      {
        Vec3 &vel = u0[i];

        Vec3 &force = f0[i];

        Real m = 1.0;

        if (i < 5)
          m = 10000.0;

        vel = vel + dt_ * force * (1.0 / m);

        force = Vec3(0, 0, 0);
      }
    };
  };


  class Peristaltic : public Application {

  public:

    PolyMesh polyMesh;
    Mesh mesh_;

    Peristaltic() : Application() {

    }

    void init(std::string fileName) {

      Application::init(fileName);

    }

    void integrate()
    {
      //      Real dt = 1.0/60.0;
      //      PolyMesh::VertexIter v_it, v_end(polyMesh.vertices_end()); 
      //      for(v_it = polyMesh.vertices_begin(); v_it!=v_end; ++v_it)
      //      {

      //        if(polyMesh.data(*v_it).fixed_)
      //          continue;

      //        Vec3 &vel = polyMesh.data(*v_it).vel_; 

      //        Vec3 &force = polyMesh.data(*v_it).force_; 

      //        Vec3 g(0,-0.00981,0);
      //        //Vec3 g(0,0.00,0);

      //        force += g;

      //        force += -0.0125 * vel;

      //        Real &m = polyMesh.data(*v_it).mass_; 
      //        Vec3 pos = Vec3(polyMesh.point(*v_it)[0], polyMesh.point(*v_it)[1], polyMesh.point(*v_it)[2]);

      //        
      //        vel = vel + dt * force * (1.0/m);

      //        polyMesh.data(*v_it).pos_old_ = Vec3(polyMesh.point(*v_it)[0], polyMesh.point(*v_it)[1], polyMesh.point(*v_it)[2]);

      //        pos = pos + dt * vel;

      //        //std::cout << "> old pos: " << polyMesh.data(*v_it).pos_old_ << std::endl;
      //        //std::cout << "> pos: " << pos << std::endl;
      //        PolyMesh::Point p(pos.x, pos.y, pos.z);

      //        polyMesh.set_point(*v_it, p);
      //        force = Vec3(0,0,0);

      //      }

      //      const float myPI = 3.1415927;
      //      for(v_it = polyMesh.vertices_begin(); v_it!=v_end; ++v_it)
      //      {
      //
      //        if(!polyMesh.data(*v_it).flagella_)
      //          continue;
      //
      //        Vec3 &vel = polyMesh.data(*v_it).vel_; 
      //
      //        Vec3 &force = polyMesh.data(*v_it).force_; 
      //        float &td = polyMesh.data(*v_it).t; 
      //
      //        Vec3 pos_old = Vec3(polyMesh.point(*v_it)[0], polyMesh.point(*v_it)[1], polyMesh.point(*v_it)[2]);
      //
      //        Vec3 pos = Vec3(polyMesh.point(*v_it)[0], polyMesh.point(*v_it)[1], polyMesh.point(*v_it)[2]);
      //        polyMesh.data(*v_it).pos_old_ = Vec3(polyMesh.point(*v_it)[0], polyMesh.point(*v_it)[1], polyMesh.point(*v_it)[2]);
      //
      //        pos = pos;
      //
      ////        float myDeltaT = 2.0f * myPI/20.0f ;
      //        Real dt = 1.0/60.0;
      //
      //        pos.y = 8.0f*dt*std::sin(td);
      //        td += dt;
      //        
      //        vel = pos - pos_old;
      //
      //        PolyMesh::Point p(pos.x, pos.y, pos.z);
      //
      //        polyMesh.set_point(*v_it, p);
      //
      //        force = Vec3(0,0,0);
      //
      //      }

    }

    void init()
    {
      //      PolyMesh::EdgeIter e_it =polyMesh.edges_begin();
      //      // Add structural springs
      //      for(; e_it != polyMesh.edges_end(); ++e_it)
      //      {
      //        PolyMesh::VertexHandle vh0 = polyMesh.to_vertex_handle(polyMesh.halfedge_handle(*e_it,0));
      //        PolyMesh::VertexHandle vh1 = polyMesh.to_vertex_handle(polyMesh.halfedge_handle(*e_it,1));

      //        Vec3 x0(polyMesh.point(vh0)[0], polyMesh.point(vh0)[1], polyMesh.point(vh0)[2]);
      //        Vec3 v0(polyMesh.data(vh0).vel_);
      //        Vec3 x1(polyMesh.point(vh1)[0], polyMesh.point(vh1)[1], polyMesh.point(vh1)[2]);
      //        Vec3 v1(polyMesh.data(vh1).vel_);

      //        SpringConstraint<Real> s;
      //        s.l0 = (x0 - x1).mag();
      //        s.vh0 = vh0;
      //        s.vh1 = vh1;
      //        mysprings_.push_back(s);
      //      }

      //      std::cout << "> Number of structural springs: " << polyMesh.n_edges() << std::endl;
      //      e_it =polyMesh.edges_begin();
      //      std::cout << "> Rest length: " << mysprings_.front().l0 << std::endl;
      //      std::cout << "> Number of shear springs: " << polyMesh.n_faces() * 2 << std::endl;

      //      // Add shear springs
      //      PolyMesh::FaceIter f_it, f_end(polyMesh.faces_end());
      //      for (unsigned y(0); y < (vrow - 1); ++y)
      //      {
      //        for (unsigned x(0); x < (vrow - 1); ++x)
      //        {
      //          PolyMesh::VertexHandle baseHandle = polyMesh.vertex_handle((y * vrow) + x);

      //          SpringConstraint<Real> s(0.5,-0.25);
      //          Vec3 x0(polyMesh.point(baseHandle)[0], polyMesh.point(baseHandle)[1], polyMesh.point(baseHandle)[2]);
      //          s.vh0 = baseHandle;

      //          baseHandle = polyMesh.vertex_handle( (y + 1) * vrow + x + 1);
      //          s.vh1 = baseHandle;
      //          Vec3 x1(polyMesh.point(baseHandle)[0], polyMesh.point(baseHandle)[1], polyMesh.point(baseHandle)[2]);

      //          s.l0 = (x0 - x1).mag();
      //          mysprings_.push_back(s);

      //          SpringConstraint<Real> s1(0.5, -0.25);
      //          baseHandle = polyMesh.vertex_handle( (y + 1) * vrow + x );
      //          s1.vh0 = baseHandle;
      //          x0 = Vec3(polyMesh.point(baseHandle)[0], polyMesh.point(baseHandle)[1], polyMesh.point(baseHandle)[2]);

      //          baseHandle = polyMesh.vertex_handle( (y * vrow) + x + 1);
      //          s1.vh1 = baseHandle;
      //          x1 = Vec3(polyMesh.point(baseHandle)[0], polyMesh.point(baseHandle)[1], polyMesh.point(baseHandle)[2]);

      //          s1.l0 = (x0 - x1).mag();
      //          mysprings_.push_back(s1);

      //        }
      //      }

      //      // Add Bend Springs
      //      for (unsigned y(0); y < (vrow); ++y)
      //      {
      //        for (unsigned x(0); x < (vrow - 2); ++x)
      //        {
      //          // Horizontal bend springs
      //          PolyMesh::VertexHandle baseHandle = polyMesh.vertex_handle((y * vrow) + x);

      //          SpringConstraint<Real> s(0.85,-0.25);
      //          Vec3 x0(polyMesh.point(baseHandle)[0], polyMesh.point(baseHandle)[1], polyMesh.point(baseHandle)[2]);
      //          s.vh0 = baseHandle;

      //          baseHandle = polyMesh.vertex_handle( (y * vrow) + x + 2);
      //          s.vh1 = baseHandle;
      //          Vec3 x1(polyMesh.point(baseHandle)[0], polyMesh.point(baseHandle)[1], polyMesh.point(baseHandle)[2]);

      //          s.l0 = (x0 - x1).mag();
      //          mysprings_.push_back(s);

      //        }
      //          // Last column Horizontal bend springs
      //          PolyMesh::VertexHandle baseHandle = polyMesh.vertex_handle((y * vrow) + (vrow - 3));
      //          SpringConstraint<Real> s(0.85,-0.25);
      //          Vec3 x0(polyMesh.point(baseHandle)[0], polyMesh.point(baseHandle)[1], polyMesh.point(baseHandle)[2]);
      //          s.vh0 = baseHandle;

      //          baseHandle = polyMesh.vertex_handle((y * vrow) + (vrow - 1));
      //          s.vh1 = baseHandle;
      //          Vec3 x1(polyMesh.point(baseHandle)[0], polyMesh.point(baseHandle)[1], polyMesh.point(baseHandle)[2]);

      //          s.l0 = (x0 - x1).mag();
      //          mysprings_.push_back(s);
      //      }
      //      
      //      // Add Bend Springs
      //      for (unsigned y(0); y < (vrow); ++y)
      //      {
      //        // Vertical bend springs
      //        for (unsigned x(0); x < (vrow - 2); ++x)
      //        {
      //          PolyMesh::VertexHandle baseHandle = polyMesh.vertex_handle((x * vrow) + y);

      //          SpringConstraint<Real> s(0.85,-0.25);
      //          Vec3 x0(polyMesh.point(baseHandle)[0], polyMesh.point(baseHandle)[1], polyMesh.point(baseHandle)[2]);
      //          s.vh0 = baseHandle;

      //          baseHandle = polyMesh.vertex_handle( ((x + 2) * vrow) + y);
      //          s.vh1 = baseHandle;
      //          Vec3 x1(polyMesh.point(baseHandle)[0], polyMesh.point(baseHandle)[1], polyMesh.point(baseHandle)[2]);

      //          s.l0 = (x0 - x1).mag();
      //          mysprings_.push_back(s);

      //        }
      //          // Last row of vertical bend springs
      //          PolyMesh::VertexHandle baseHandle = polyMesh.vertex_handle(((vrow -3) * vrow) + y);
      //          SpringConstraint<Real> s(0.85,-0.25);
      //          Vec3 x0(polyMesh.point(baseHandle)[0], polyMesh.point(baseHandle)[1], polyMesh.point(baseHandle)[2]);
      //          s.vh0 = baseHandle;

      //          baseHandle = polyMesh.vertex_handle(((vrow - 1) * vrow) + y) ;
      //          s.vh1 = baseHandle;
      //          Vec3 x1(polyMesh.point(baseHandle)[0], polyMesh.point(baseHandle)[1], polyMesh.point(baseHandle)[2]);

      //          s.l0 = (x0 - x1).mag();
      //          mysprings_.push_back(s);
      //      }

      //      PolyMesh::VertexIter v_it, v_end(polyMesh.vertices_end()); 
      //      for(v_it = polyMesh.vertices_begin(); v_it!=v_end; ++v_it)
      //      {
      //        Vec3 p(polyMesh.point(*v_it)[0], polyMesh.point(*v_it)[1], polyMesh.point(*v_it)[2]);

      //        //if((*v_it).idx() == 0 || (*v_it).idx() == 20 || (*v_it).idx() == 420 || (*v_it).idx() == 440)
      //        if((*v_it).idx() == 0 || (*v_it).idx() == 40)
      //        {
      //          polyMesh.data(*v_it).fixed_ = true;
      //        }
      //      }

      //      int j(0);
      //      const float myPI = 3.1415927;
      //      for(int i(10); i <= 430; i+=21)
      //      {
      //        PolyMesh::VertexHandle baseHandle = polyMesh.vertex_handle(i);
      //        polyMesh.data(baseHandle).flagella_ = true;
      //        polyMesh.data(baseHandle).t = j * 2.0f * myPI/20.0f ;
      //        ++j;
      //      }

    }

    void run() {

      int steps = 0;
      //start the main simulation loop

      mesh_.clear();
      polyMesh.clear();

      OpenMesh::IO::read_mesh(mesh_, "meshes/cyl.obj");
      std::cout << "> Mesh vertices: " << mesh_.n_vertices() << std::endl;
      OpenMesh::IO::write_mesh(mesh_, "output/cyl.stl");

//      OpenMesh::IO::read_mesh(polyMesh, "meshes/mycloth20.obj");
//      std::cout << "> PolyMesh vertices: " << polyMesh.n_vertices() << std::endl;
//      std::cout << "> PolyMesh edges: " << polyMesh.n_edges() << std::endl;
//      std::cout << "> PolyMesh faces: " << polyMesh.n_faces() << std::endl;
//      //OpenMesh::IO::write_mesh(polyMesh, "output/cloth.stl");

//      init();

    }

  };

};


int main()
{

  using namespace i3d;

  i3d::Peristaltic myApp;

  myApp.init("start/sampleRigidBody.xml");

  myApp.run();

  return 0;
}
