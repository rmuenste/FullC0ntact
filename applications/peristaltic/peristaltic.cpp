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
      VertexT() : cog_(Point(0.0f, 0.0f, 0.0f)), vel_(0, 0, 0), pos_(0,0,0), mass_(0.0), idx_(-1) { }
      const Point& cog() const { return cog_; }
      void set_cog(const Point& _p) { cog_ = _p; }
      Vec3 vel_;
      Vec3 pos_;
      Real mass_;
      int idx_;
      float t;
    };

  };

  typedef OpenMesh::TriMesh_ArrayKernelT<MyTraits> Mesh;
  typedef OpenMesh::PolyMesh_ArrayKernelT<MyTraits> PolyMesh;

  template<>
  class PeristalticSwimmer<Real, Mesh > : public BasicSoftBody<Real, Mesh>
  {
  public:

    int istep_;

    Real A_, q_, N_, f_, m_, k_tail_, kd_, ks_;
    Real a0_, l0_, dt_, rho_, massAll_, volume_;

    int n_mid_, n_head_, offset_;

    Vec3 com_;

    Vec3 velocity_;

    Transformationr transform_;

    std::vector< SimpleSpringConstraint<Real> > springs_;

    std::vector< Vector3<Real> > u_;

    std::vector< Vector3<Real> > force_;

    std::vector< std::vector<Mesh::VertexHandle> > rings_;

    Mesh mesh_;

    PeristalticSwimmer() : A_(3.2), N_(60), f_(1.0 / 60.0), a0_(1.0), l0_(1.0*a0_)
    {
      transform_.setOrigin(Vec3(0, 0, 0));

      u_.reserve(N_);
      force_.reserve(N_);

      rings_.reserve(14);

      velocity_ = Vec3(0, 0, 0);
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


    void init()
    {

      Real xx = 0 * l0_;
      Real yy = 0;

      ks_ = 1000.0;

      kd_ = -0.2;

      Mesh::VertexIter v_it = mesh_.vertices_begin();
      v_it = mesh_.vertices_begin();
      for (; v_it != mesh_.vertices_end(); ++v_it)
      {
        u_.push_back(Vec3(0, 0, 0));
        force_.push_back(Vec3(0, 0, 0));
      }

      Mesh::EdgeIter e_it =mesh_.edges_begin();
      // Add structural springs
      for(; e_it != mesh_.edges_end(); ++e_it)
      {
        Mesh::VertexHandle vh0 = mesh_.to_vertex_handle(mesh_.halfedge_handle(*e_it,0));
        Mesh::VertexHandle vh1 = mesh_.to_vertex_handle(mesh_.halfedge_handle(*e_it,1));

        Vec3 x0(mesh_.point(vh0)[0],
                mesh_.point(vh0)[1],
                mesh_.point(vh0)[2]
        );

        Vec3 x1(mesh_.point(vh1)[0],
                mesh_.point(vh1)[1],
                mesh_.point(vh1)[2]
        );

        mesh_.data(vh0).pos_ = x0;
        mesh_.data(vh0).idx_ = vh0.idx();

        mesh_.data(vh1).pos_ = x1;
        mesh_.data(vh1).idx_ = vh1.idx();

        l0_ = (x1 - x0).mag();

        springs_.push_back(SimpleSpringConstraint<Real>(ks_, kd_, l0_, vh0.idx(), vh1.idx(),
                                                        &mesh_.data(vh0).pos_,
                                                        &mesh_.data(vh1).pos_,
                                                        &u_[vh0.idx()],
                                                        &u_[vh1.idx()]
                                                        ));
      }

      for (int i(0); i < 14; ++i)
        rings_.push_back(std::vector<Mesh::VertexHandle>());

      Real dy = 0.8 / 13.0;

      int iring(0);
      Real yring = -0.4;

      for (unsigned j = 0; j < rings_.size(); ++j)
      {

        v_it = mesh_.vertices_begin();
        for (; v_it != mesh_.vertices_end(); ++v_it)
        {
          Vec3 x(mesh_.point(*v_it)[0],
                 mesh_.point(*v_it)[1],
                 mesh_.point(*v_it)[2]);

          if (std::abs(x.y - yring) < 0.01)
          {
            ++iring;
            Mesh::VertexHandle vh = *v_it;
            rings_[j].push_back(vh);
          }
        }
        yring += dy;
        std::cout << "Vertices in ring: " << rings_[j].size() << std::endl;
      }


    };

    void internalForce(Real t, int istep)
    {

      Mesh::VertexHandle vh;

      int iring(0);
      Real yring = -0.4;

      for (int i(0); i < rings_[0].size(); ++i)
      {
//        std::cout << "Vertex handle: " << rings_[0][i].idx() << std::endl;
        Mesh::VertexHandle vh = rings_[0][i];
        int iv = rings_[0][i].idx();

        force_[iv].y = -12;

      }

      for (unsigned i(0); i < springs_.size(); ++i)
      {
        SimpleSpringConstraint<Real> &spring = springs_[i];

        Vector3<Real> f = spring.evalForce();

        force_[spring.i0_] += f;
        force_[spring.i1_] -= f;
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

        vel = vel + dt_ * force * (1.0 / m);

        Mesh::VertexHandle vh(i);

        Mesh::Point pos = mesh_.point(vh);
        Mesh::Point vP(vel.x, vel.y, vel.z);

        pos = pos + dt_ * vP;

        mesh_.set_point(vh, pos);

        pos = mesh_.point(vh);

        mesh_.data(vh).pos_ = Vec3(pos[0],pos[1],pos[2]);

        force = Vec3(0, 0, 0);
      }

    };

    void step(Real t, Real dt, int it)
    {
      dt_ = dt;

      configureStroke(t, it);

      internalForce(t, it);

      integrate();
    }

  };

  class Peristaltic : public Application<> {

  public:

    PolyMesh polyMesh;

    int  steps_ = 2000;
    int  dt_ = 0.01;
    int  time_ = 0.0;
    int  step_ = 0;

    PeristalticSwimmer<Real, Mesh> peristalticSwimmer_;

    Peristaltic() : Application() {

    }

    void init(std::string fileName) {

      Application::init(fileName);
      peristalticSwimmer_.mesh_.clear();
      OpenMesh::IO::read_mesh(peristalticSwimmer_.mesh_, "meshes/cyl.obj");
      std::cout << "> Mesh vertices: " << peristalticSwimmer_.mesh_.n_vertices() << std::endl;
      std::cout << "> Mesh edges: " <<    peristalticSwimmer_.mesh_.n_edges() << std::endl;
      std::cout << "> Mesh faces: " <<    peristalticSwimmer_.mesh_.n_faces() << std::endl;
      peristalticSwimmer_.init();

    }

    void run() {

      steps_ = 1000;

      dt_ = 0.01;
      Real mydt = 0.01;
      time_ = 0.0;
      step_ = 0;

      for(int istep(0); istep <= steps_; ++istep)
      {
        peristalticSwimmer_.step(time_,mydt, istep);
        std::cout << "> Step: " << istep << std::endl;
        CVtkWriter writer;

        OpenMesh::IO::ExporterT<Mesh> exporter(peristalticSwimmer_.mesh_);
        OpenMesh::IO::_VTKWriter_ om_writer;

        std::ostringstream name;
        name << "output/cyl." << std::setfill('0') << std::setw(5) << istep << ".vtk";
        OpenMesh::IO::write_mesh(peristalticSwimmer_.mesh_, name.str().c_str());
        time_ += mydt;
        step_++;
      }

//      OpenMesh::IO::read_mesh(polyMesh, "meshes/mycloth20.obj");
//      std::cout << "> PolyMesh vertices: " << polyMesh.n_vertices() << std::endl;
//      std::cout << "> PolyMesh edges: " << polyMesh.n_edges() << std::endl;
//      std::cout << "> PolyMesh faces: " << polyMesh.n_faces() << std::endl;
//      //OpenMesh::IO::write_mesh(polyMesh, "output/cloth.stl");

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
