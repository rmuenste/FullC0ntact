#ifndef MYOPENGLWIDGET_H
#define MYOPENGLWIDGET_H

#include <QGLWidget>
#include <application.h>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/IO/writer/VTKWriter.hh>
#include <OpenMesh/Core/IO/exporter/BaseExporter.hh>
#include <OpenMesh/Core/IO/exporter/ExporterT.hh>
#include <QTimer>
#include <QTime>
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

    Mesh mesh;

    PeristalticSwimmer() : A_(3.2), N_(60), f_(1.0 / 60.0), a0_(1.0), l0_(1.0*a0_)
    {
      transform_.setOrigin(Vec3(0, 0, 0));

      u_.reserve(N_);
      force_.reserve(N_);

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

      }

      Mesh::EdgeIter e_it =mesh.edges_begin();
      // Add structural springs
      for(; e_it != mesh.edges_end(); ++e_it)
      {
        Mesh::VertexHandle vh0 = mesh.to_vertex_handle(mesh.halfedge_handle(*e_it,0));
        Mesh::VertexHandle vh1 = mesh.to_vertex_handle(mesh.halfedge_handle(*e_it,1));
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

  template <typename T>
  class SpringConstraint
  {
  public:

    enum class Spring_type
    {
      StructuralSpring,
      ShearSpring,
      BendSpring
    };

    T ks;
    T kd;
    T l0;
    T l;
    Vector3<T> L;
    T dt;

    Spring_type st_;


    PolyMesh::VertexHandle vh0;
    PolyMesh::VertexHandle vh1;

    SpringConstraint() : ks(T(0.5)), kd(T(-0.25)), l0(T(1)), l(T(0)), dt(T(0))
    {
      st_ = Spring_type::StructuralSpring;
    }

    SpringConstraint(T _ks, T _kd, Spring_type _st) : ks(_ks), kd(_kd), l0(T(1)), l(T(0)), dt(T(0))
    {
      st_ = _st;
    }

    SpringConstraint(T _ks, T _kd) : ks(_ks), kd(_kd), l0(T(1)), l(T(0)), dt(T(0))
    {
      st_ = Spring_type::StructuralSpring;
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

    virtual Vector3<T> evalForce2(const Vector3<T> &x0, const Vector3<T> &x1, const Vector3<T> &v0, const Vector3<T> &v1, T _ks, T _ksh, T _kb)
    {
      L = (x0 - x1);
      l = L.mag();
      if(l == 0.0f)
        return Vector3<T>(0,0,0);

      Vector3<T> f;

      if(st_ == Spring_type::StructuralSpring)
      {
        // what if vertices coincide?
        Vector3<T> LN = (1.0 / l) * L;
        T tl = -_ks * (l - l0);
        T tr = kd * ((v0 - v1) * L)/l;
        f = (tl + tr) * LN;
      }
      else if(st_ == Spring_type::ShearSpring)
      {
        // what if vertices coincide?
        Vector3<T> LN = (1.0 / l) * L;
        T tl = -_ksh * (l - l0);
        T tr = kd * ((v0 - v1) * L)/l;
        f = (tl + tr) * LN;
      }
      else if(st_ == Spring_type::BendSpring)
      {
        // what if vertices coincide?
        Vector3<T> LN = (1.0 / l) * L;
        T tl = -_kb * (l - l0);
        T tr = kd * ((v0 - v1) * L)/l;
        f = (tl + tr) * LN;
      }
      else
      {
        // what if vertices coincide?
        Vector3<T> LN = (1.0 / l) * L;
        T tl = -ks * (l - l0);
        T tr = kd * ((v0 - v1) * L)/l;
        f = (tl + tr) * LN;
      }

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

    Mesh mesh_;
    PolyMesh polyMesh;
    std::vector <SpringConstraint<Real>> mysprings_;
    unsigned vrow; // = 21;

    int start_time_;
    int last_time_;
    int simTime_;
    int deltaT_;
    Real dt_;

    OpenMeshTest() : Application(), vrow(21), simTime_(0), deltaT_(16) {

    }

    void setStartTime(int _start)
    {
      start_time_ = _start;
      last_time_ = _start;
      std::cout << "DeltaT: " << deltaT_ << " [msecs]" << std::endl;
      std::cout << "Simulation Time: " << simTime_ << " [msecs]" << std::endl;
    }

    void calcTimeStep(int _toStart)
    {
      int diff = std::abs(last_time_ - _toStart);
      last_time_ = _toStart;
      //std::cout << "DeltaT: " << diff << " [msecs]" << std::endl;
      //simTime_ += diff;
      //std::cout << "Simulation Time: " << simTime_ << " [msecs]" << std::endl;
    }

    void init(std::string fileName) {

      Application::init(fileName);

    }

    void integrate()
    {

      //Real dt = 1.0/60.0;
      Real dt = dt_;

    }

    void simulate()
    {

    }

    void simulateGUI(int _ks, int _ksh, int _kb)
    {

      dt_ = 3.0 * Real(deltaT_);
      dt_ *= 1e-3;
    }

    void addProvotDynamicInverse() {

    }

    void run()
    {

    }

    void initSprings()
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

//          SpringConstraint<Real> s(0.5,-0.25, SpringConstraint<Real>::Spring_type::ShearSpring);
//          Vec3 x0(polyMesh.point(baseHandle)[0], polyMesh.point(baseHandle)[1], polyMesh.point(baseHandle)[2]);
//          s.vh0 = baseHandle;

//          baseHandle = polyMesh.vertex_handle( (y + 1) * vrow + x + 1);
//          s.vh1 = baseHandle;
//          Vec3 x1(polyMesh.point(baseHandle)[0], polyMesh.point(baseHandle)[1], polyMesh.point(baseHandle)[2]);

//          s.l0 = (x0 - x1).mag();
//          mysprings_.push_back(s);

//          SpringConstraint<Real> s1(0.5, -0.25, SpringConstraint<Real>::Spring_type::ShearSpring);
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

//          SpringConstraint<Real> s(0.85,-0.25, SpringConstraint<Real>::Spring_type::BendSpring);
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
//          SpringConstraint<Real> s(0.85,-0.25, SpringConstraint<Real>::Spring_type::BendSpring);
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

//          SpringConstraint<Real> s(0.85,-0.25, SpringConstraint<Real>::Spring_type::BendSpring);
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
//          SpringConstraint<Real> s(0.85,-0.25, SpringConstraint<Real>::Spring_type::BendSpring);
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
//        if((*v_it).idx() == 0 || (*v_it).idx() == 20)
//        {
//          polyMesh.data(*v_it).fixed_ = true;
//        }
//      }

    }

    void loadClothMesh()
    {
      mesh_.clear();
      polyMesh.clear();
      polyMesh.request_vertex_normals();
      polyMesh.request_face_normals();


      // enable most options for now
      OpenMesh::IO::Options opt;
      OpenMesh::IO::Options opt2;
      opt += OpenMesh::IO::Options::VertexNormal;
      opt += OpenMesh::IO::Options::FaceNormal;

      mesh_.request_face_normals();
      mesh_.request_vertex_normals();

      OpenMesh::IO::read_mesh(polyMesh, "meshes/cyl.obj", opt);
      std::cout << "> PolyMesh vertices: " << polyMesh.n_vertices() << std::endl;
      std::cout << "> PolyMesh edges: " << polyMesh.n_edges() << std::endl;
      std::cout << "> PolyMesh faces: " << polyMesh.n_faces() << std::endl;
      //OpenMesh::IO::write_mesh(polyMesh, "output/cloth.stl");
      opt2 = opt;

      // update face and vertex normals
      if ( !opt2.check( OpenMesh::IO::Options::FaceNormal ) )
      {
        polyMesh.update_face_normals();
        std::cout << "Generating face normals\n";
      }
      else
        std::cout << "File provides face normals\n";

      if ( !opt2.check( OpenMesh::IO::Options::VertexNormal ) )
        polyMesh.update_vertex_normals();
      else
          std::cout << "File provides vertex normals\n";

    }

    void clearMesh()
    {
      polyMesh.clear();
    }

  };

}

class MyOpenGLWidget : public QGLWidget
{
    Q_OBJECT
public:
    MyOpenGLWidget(QWidget *parent = 0);
    ~MyOpenGLWidget();

    QTimer *timer;
    QTime *time_;

    QTime startTime;
    i3d::OpenMeshTest myApp;
    bool firstTime;
    unsigned drawMode_;
    std::vector< i3d::Vector3<float> > gridVertices_;

    i3d::PolyMesh polyMesh_;

    void drawMesh(i3d::PolyMesh &pm);

    i3d::PeristalticSwimmer<i3d::Real, i3d::Mesh > peristalticSwimmer_;

    void loadOpenMesh()
    {
      polyMesh_.clear();
      polyMesh_.request_vertex_normals();
      polyMesh_.request_face_normals();


      // enable most options for now
      OpenMesh::IO::Options opt;
      OpenMesh::IO::Options opt2;
      opt += OpenMesh::IO::Options::VertexNormal;
      opt += OpenMesh::IO::Options::FaceNormal;

      polyMesh_.request_face_normals();
      polyMesh_.request_vertex_normals();


      OpenMesh::IO::read_mesh(peristalticSwimmer_.mesh, "meshes/cyl.obj", opt);

      std::cout << "> Mesh vertices: " << peristalticSwimmer_.mesh.n_vertices() << std::endl;
      std::cout << "> Mesh edges: "    << peristalticSwimmer_.mesh.n_edges()    << std::endl;
      std::cout << "> Mesh faces: "    << peristalticSwimmer_.mesh.n_faces()    << std::endl;

      opt2 = opt;

      // update face and vertex normals
      if ( !opt2.check( OpenMesh::IO::Options::FaceNormal ) )
      {
        polyMesh_.update_face_normals();
        std::cout << "Generating face normals\n";
      }
      else
        std::cout << "File provides face normals\n";

      if ( !opt2.check( OpenMesh::IO::Options::VertexNormal ) )
        polyMesh_.update_vertex_normals();
      else
          std::cout << "File provides vertex normals\n";

    }

public slots:

protected:
    void initializeGL();
    void paintGL();
    void resizeGL(int width, int height);
    void repaint();

    QSize minimumSizeHint() const;
    QSize sizeHint() const;
    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);

public slots:
    // slots for xyz-rotation slider
    void setXRotation(int angle);
    void setYRotation(int angle);
    void setZRotation(int angle);
    
    // slots for draw style    
    void drawStyleChanged(int _style);

    void structSlider_valueChanged(int value);
    void shearSlider_valueChanged(int value);
    void bendSlider_valueChanged(int value);

    void reset();

signals:
    // signaling rotation from mouse movement
    void xRotationChanged(int angle);
    void yRotationChanged(int angle);
    void zRotationChanged(int angle);

private:
    void draw();
    void drawAxes();

    int xRot;
    int yRot;
    int zRot;

    int kStruct;
    int kShear;
    int kBend;

    QPoint lastPos;

};

#endif // MYOPENGLWIDGET_H
