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
  };

  typedef OpenMesh::TriMesh_ArrayKernelT<MyTraits> Mesh;
  typedef OpenMesh::PolyMesh_ArrayKernelT<MyTraits> PolyMesh;

  template <typename T, typename handleType>
  class SpringConstraint
  {
  public:
    T ks;
    T kd;
    T l0;
    T l;
    Vector3<T> L;
    T dt;

    handleType vh0;
    handleType vh1;

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

      T bias = 1.0f;
      if(l < l0/10.0f)
        bias = 6.0;

      return bias * f;
    }
  };

  class CompareFunctor
  {
  public:
    bool operator()(SpringConstraint<Real, int> s1, SpringConstraint<Real, int> s2) const
    {
      if (s1.vh0 < s2.vh0)
        return true;

      if (s1.vh0 == s2.vh0)
      {
        if (s1.vh1 < s2.vh1)
          return true;
      }

      return false;
    }
  };

  //class SpringMesh
  //{
  //public:

  //  // Add structural springs
  //  for (; e_it != polyMesh.edges_end(); ++e_it)
  //  {
  //    PolyMesh::VertexHandle vh0 = polyMesh.to_vertex_handle(polyMesh.halfedge_handle(*e_it, 0));
  //    PolyMesh::VertexHandle vh1 = polyMesh.to_vertex_handle(polyMesh.halfedge_handle(*e_it, 1));

  //    Vec3 x0(polyMesh.point(vh0)[0], polyMesh.point(vh0)[1], polyMesh.point(vh0)[2]);
  //    Vec3 v0(polyMesh.data(vh0).vel_);
  //    Vec3 x1(polyMesh.point(vh1)[0], polyMesh.point(vh1)[1], polyMesh.point(vh1)[2]);
  //    Vec3 v1(polyMesh.data(vh1).vel_);

  //    SpringConstraint<Real, PolyMesh::VertexHandle> s;
  //    s.l0 = (x0 - x1).mag();
  //    s.vh0 = vh0;
  //    s.vh1 = vh1;
  //    mysprings_.push_back(s);
  //  }

  //  std::cout << "> Number of structural springs: " << polyMesh.n_edges() << std::endl;

  //};

  class OpenMeshTest : public Application {

  public:
    
    PolyMesh polyMesh;
    Mesh mesh_;
    std::vector < SpringConstraint<Real, PolyMesh::VertexHandle> > mysprings_;
    std::vector < SpringConstraint<Real, int> > springs_;

    std::set<SpringConstraint<Real, int>, CompareFunctor> springSet_;

    unsigned vrow; // = 21;
    Real dt_;
    Real time_;
    int steps_;
    int step_;

    OpenMeshTest() : Application(), vrow(21) {

    }

    void init(std::string fileName) {

      Application::init(fileName);
      std::string meshFile("meshes/tryp5.tri3d");
      grid_.initMeshFromFile(meshFile.c_str());
      std::cout << "> Building standard mesh info." << std::endl;
      grid_.initStdMesh();

   }

    void writeOut(int out)
    {
      std::ostringstream sName, sNameParticles, sphereFile;
      std::string sModel("output/model.vtk");
      std::string sParticleFile("output/particle.vtk");
      std::string sParticle("solution/particles.i3d");
      CVtkWriter writer;
      int iTimestep = out;
      sName << "." << std::setfill('0') << std::setw(5) << iTimestep;
      sNameParticles << "." << std::setfill('0') << std::setw(5) << iTimestep;
      sModel.append(sName.str());
      sParticleFile.append(sName.str());
      sParticle.append(sNameParticles.str());
      sphereFile << "output/spheres.vtk." << std::setfill('0') << std::setw(5) << iTimestep;
      //Write the grid to a file and measure the time

      std::ostringstream sNameGrid;
      std::string sGrid("output/grid.vtk");
      sNameGrid << "." << std::setfill('0') << std::setw(5) << iTimestep;
      sGrid.append(sNameGrid.str());
      writer.WriteSpringMesh(grid_, sGrid.c_str());
    }

    void flagellaFunction()
    {

      VertexIter<Real> v_it;
      VertexIter<Real> v_end = grid_.vertices_end();

      for(v_it = grid_.vertices_begin(); v_it!=v_end; v_it++)
      {
        int Idx = v_it.idx();
        if(!grid_.m_myTraits[Idx].flagella_)
          continue;

        Vec3 &vel = grid_.m_myTraits[Idx].vel_;

        Vec3 &force = grid_.m_myTraits[Idx].force_;

        Vec3 pos = Vec3(grid_.vertexCoords_[Idx]);

        Real dt = 0.0025;

        Real &td = grid_.m_myTraits[Idx].t_;
        
        //force.z = pos.x * 2.0 * dt * std::sin(td);
        vel.z = pos.x * 1.5 * dt * std::sin(0.25 * td);
        //vel.z = pos.x * pos.x * 0.01; 
        //std::cout << "> force1: " << force << " idx: " << Idx << std::endl;
        td+=dt;

        Vec3 old_pos = grid_.m_myTraits[Idx].pos_old_;

      }

    }

    void flagellaFunction2()
    {

      VertexIter<Real> v_it;
      VertexIter<Real> v_end = grid_.vertices_end();
      for(v_it = grid_.vertices_begin(); v_it!=v_end; v_it++)
      {
        int Idx = v_it.idx();
        if(!grid_.m_myTraits[Idx].flagella_)
          continue;

        Vec3 &vel = grid_.m_myTraits[Idx].vel_;

        Vec3 &force = grid_.m_myTraits[Idx].force_;

        Vec3 pos = Vec3(grid_.vertexCoords_[Idx]);

        Real damp = 0.005;
        Real &td = grid_.m_myTraits[Idx].t_;
        
        Real r = Real(step_)/Real(steps_);

        pos.z = r * 50.0f * damp * std::sin(td);

        Vec3 old_pos = grid_.m_myTraits[Idx].pos_old_;

        vel = pos - old_pos;

        grid_.vertexCoords_[Idx] = pos;

        force = Vec3(0,0,0);
      }
    }

    void integrate()
    {
      //Real dt = 1.0/60.0;
      //Real dt = 0.0025;
      Real dt = dt_;

      VertexIter<Real> v_it;
      VertexIter<Real> v_end = grid_.vertices_end();
      for(v_it = grid_.vertices_begin(); v_it!=v_end; v_it++)
      {
        int Idx = v_it.idx();

        if(grid_.m_myTraits[Idx].fixed_)
          continue;

        Vec3 &vel = grid_.m_myTraits[Idx].vel_;

        Vec3 &force = grid_.m_myTraits[Idx].force_;

        //Vec3 g(0,-0.00981,0);
        Vec3 g(0,0.00,0);
//!        if(grid_.m_myTraits[Idx].flagella_)
//!          g = Vec3(0, -0.00981, 0);

        //force += g;

        //force += -0.0125 * vel;

        Real &m = grid_.m_myTraits[Idx].mass_;

        Vec3 pos = Vec3(grid_.vertexCoords_[Idx]);
        
        vel = vel + dt * force * (1.0/m);
//        if(grid_.m_myTraits[Idx].flagella_)
//        {
//          std::cout << "> force: " << force << " idx: " << Idx << std::endl;
//          std::cout << "> vel: " << vel << " idx: " << Idx << std::endl;
//        }
        //std::cout << "> vel: " << vel << std::endl;
        //std::cout << "> m: " << m << std::endl;

        grid_.m_myTraits[Idx].pos_old_ = pos;

        pos.z = pos.z + dt * vel.z;

        //std::cout << "> old pos: " << polyMesh.data(*v_it).pos_old_ << std::endl;
        //std::cout << "> pos: " << pos << std::endl;
        grid_.vertexCoords_[Idx] = pos;

        //force = Vec3(0,0,0);
      }

      flagellaFunction2();

//      for(v_it = grid_.vertices_begin(); v_it!=v_end; v_it++)
//      {
//        int Idx = v_it.idx();
//        if(!grid_.m_myTraits[Idx].flagella_)
//          continue;
//
//        Vec3 &vel = grid_.m_myTraits[Idx].vel_;
//
//        Vec3 &force = grid_.m_myTraits[Idx].force_;
//
//        Vec3 pos = Vec3(grid_.vertexCoords_[Idx]);
//
//        Real dt = 1.0/60.0;
//
//        Real &td = grid_.m_myTraits[Idx].t_;
//        
//        pos.z = 3.0f*dt*std::sin(td);
//
//        Vec3 old_pos = grid_.m_myTraits[Idx].pos_old_;
//
//        td += dt;
//
//        vel = pos - old_pos;
//
//        grid_.vertexCoords_[Idx] = pos;
//
//        force = Vec3(0,0,0);
//      }

    }

    void simulate()
    {
      VertexIter<Real> v_it;
      VertexIter<Real> v_end = grid_.vertices_end();
      for(v_it = grid_.vertices_begin(); v_it!=v_end; v_it++)
      {

        int Idx = v_it.idx();

        if(grid_.m_myTraits[Idx].fixed_)
          continue;

        grid_.m_myTraits[Idx].force_ = Vec3(0,0,0);

      }

      for (auto &s : springs_)
      {
        int vh0 = s.vh0;
        int vh1 = s.vh1;

        Vec3 x0(grid_.vertexCoords_[vh0]);
        Vec3 v0(grid_.m_myTraits[vh0].vel_);
        Vec3 x1(grid_.vertexCoords_[vh1]);
        Vec3 v1(grid_.m_myTraits[vh1].vel_);


        Vec3 f = s.evalForce(x0, x1, v0, v1);
        if(grid_.m_myTraits[vh0].flagella_)
        {
          grid_.m_myTraits[vh1].force_ -= 2.0 * f;
        }
        else if(grid_.m_myTraits[vh1].flagella_)
        {
          grid_.m_myTraits[vh0].force_ += 2.0 * f;
        }
        else
        {
          grid_.m_myTraits[vh0].force_ += f;
          grid_.m_myTraits[vh1].force_ -= f;
        }
      }
      integrate();
      addProvotDynamicInverse();
    }

    void addProvotDynamicInverse() {

      for (auto &s : springs_)
      {
        int vh0 = s.vh0;
        int vh1 = s.vh1;

        Vec3 x0(grid_.vertexCoords_[vh0]);
        Vec3 v0(grid_.m_myTraits[vh0].vel_);
        Vec3 x1(grid_.vertexCoords_[vh1]);
        Vec3 v1(grid_.m_myTraits[vh1].vel_);

        Vec3 deltaP = x0 - x1;
        float dist = deltaP.mag();
        if (dist > s.l0)
        {
          dist -= (s.l0);
          dist /= 2.0f;
          deltaP.normalize();
          deltaP *= dist;
          if (grid_.m_myTraits[vh0].fixed_) {
            grid_.m_myTraits[vh1].vel_ += deltaP;
          }
          else if (grid_.m_myTraits[vh1].fixed_) {
            grid_.m_myTraits[vh0].vel_ -= deltaP;
          }
          else {
            if(grid_.m_myTraits[vh0].flagella_)
            {
                grid_.m_myTraits[vh1].vel_ += 2.0 * deltaP;
            }
            else if(grid_.m_myTraits[vh1].flagella_)
            {
                grid_.m_myTraits[vh0].vel_ -= 2.0 * deltaP;
            }
            else
            {
                grid_.m_myTraits[vh0].vel_ -= deltaP;
                grid_.m_myTraits[vh1].vel_ += deltaP;
            }
          }
        }
      }
    }

    void init()
    {

      EdgeIter e_it = grid_.edge_begin();
      for (; e_it != grid_.edge_end(); e_it++)
      {
//        SpringConstraint<Real, int> s;
        SpringConstraint<Real, int> s(0.1, -0.05);

        s.vh0 = (e_it.Get()->edgeVertexIndices_[0] < e_it.Get()->edgeVertexIndices_[1]) ? e_it.Get()->edgeVertexIndices_[0] : e_it.Get()->edgeVertexIndices_[1];
        s.vh1 = (e_it.Get()->edgeVertexIndices_[1] > e_it.Get()->edgeVertexIndices_[0]) ? e_it.Get()->edgeVertexIndices_[1] : e_it.Get()->edgeVertexIndices_[0];

        Vec3 x0(grid_.vertexCoords_[s.vh0]);
        Vec3 x1(grid_.vertexCoords_[s.vh1]);
        s.l0 = (x0 - x1).mag();
        springSet_.insert(s);

      }

      std::cout << "> Structural springs: " << springSet_.size() << std::endl;

      int shearSprings = 0;
      ElementIter el_it = grid_.elem_begin();
      for (; el_it != grid_.elem_end(); el_it++)
      {

        //SpringConstraint<Real, int> s;
        SpringConstraint<Real, int> s(0.1, -0.05);

        s.vh0 = (el_it.Get()->hexaVertexIndices_[0] < el_it.Get()->hexaVertexIndices_[6]) ? el_it.Get()->hexaVertexIndices_[0] : el_it.Get()->hexaVertexIndices_[6];
        s.vh1 = (el_it.Get()->hexaVertexIndices_[6] > el_it.Get()->hexaVertexIndices_[0]) ? el_it.Get()->hexaVertexIndices_[6] : el_it.Get()->hexaVertexIndices_[0];

        Vec3 x0(grid_.vertexCoords_[s.vh0]);
        Vec3 x1(grid_.vertexCoords_[s.vh1]);
        s.l0 = (x0 - x1).mag();
        springSet_.insert(s);

        shearSprings++;

        // 2nd spring
        //SpringConstraint<Real, int> s1;
        SpringConstraint<Real, int> s1(0.1, -0.05);

        s1.vh0 = (el_it.Get()->hexaVertexIndices_[1] < el_it.Get()->hexaVertexIndices_[7]) ? el_it.Get()->hexaVertexIndices_[1] : el_it.Get()->hexaVertexIndices_[7];
        s1.vh1 = (el_it.Get()->hexaVertexIndices_[7] > el_it.Get()->hexaVertexIndices_[1]) ? el_it.Get()->hexaVertexIndices_[7] : el_it.Get()->hexaVertexIndices_[1];

        x0 = Vec3(grid_.vertexCoords_[s1.vh0]);
        x1 = Vec3(grid_.vertexCoords_[s1.vh1]);
        s1.l0 = (x0 - x1).mag();
        springSet_.insert(s1);

        shearSprings++;

        // 3rd spring
        //SpringConstraint<Real, int> s2;
        SpringConstraint<Real, int> s2(0.1, -0.05);

        s2.vh0 = (el_it.Get()->hexaVertexIndices_[2] < el_it.Get()->hexaVertexIndices_[4]) ? el_it.Get()->hexaVertexIndices_[2] : el_it.Get()->hexaVertexIndices_[4];
        s2.vh1 = (el_it.Get()->hexaVertexIndices_[4] > el_it.Get()->hexaVertexIndices_[2]) ? el_it.Get()->hexaVertexIndices_[4] : el_it.Get()->hexaVertexIndices_[2];

        x0 = Vec3(grid_.vertexCoords_[s2.vh0]);
        x1 = Vec3(grid_.vertexCoords_[s2.vh1]);
        s2.l0 = (x0 - x1).mag();
        springSet_.insert(s2);

        shearSprings++;

        // 4th spring
        //SpringConstraint<Real, int> s3;
        SpringConstraint<Real, int> s3(0.1, -0.05);

        s3.vh0 = (el_it.Get()->hexaVertexIndices_[3] < el_it.Get()->hexaVertexIndices_[5]) ? el_it.Get()->hexaVertexIndices_[3] : el_it.Get()->hexaVertexIndices_[5];
        s3.vh1 = (el_it.Get()->hexaVertexIndices_[5] > el_it.Get()->hexaVertexIndices_[3]) ? el_it.Get()->hexaVertexIndices_[5] : el_it.Get()->hexaVertexIndices_[3];

        x0 = Vec3(grid_.vertexCoords_[s3.vh0]);
        x1 = Vec3(grid_.vertexCoords_[s3.vh1]);
        s3.l0 = (x0 - x1).mag();
        springSet_.insert(s3);

        shearSprings++;

      }


      HFaceIter f_it = grid_.faces_begin();
      for (; f_it != grid_.faces_end(); f_it++)
      {

        SpringConstraint<Real, int> s;

        s.vh0 = (f_it.Get()->faceVertexIndices_[0] < f_it.Get()->faceVertexIndices_[2]) ? f_it.Get()->faceVertexIndices_[0] : f_it.Get()->faceVertexIndices_[2];
        s.vh1 = (f_it.Get()->faceVertexIndices_[2] > f_it.Get()->faceVertexIndices_[0]) ? f_it.Get()->faceVertexIndices_[2] : f_it.Get()->faceVertexIndices_[0];

        Vec3 x0(grid_.vertexCoords_[s.vh0]);
        Vec3 x1(grid_.vertexCoords_[s.vh1]);
        s.l0 = (x0 - x1).mag();
        springSet_.insert(s);

        shearSprings++;

        // 2nd spring
        SpringConstraint<Real, int> s1;

        s1.vh0 = (f_it.Get()->faceVertexIndices_[1] < f_it.Get()->faceVertexIndices_[3]) ? f_it.Get()->faceVertexIndices_[1] : f_it.Get()->faceVertexIndices_[3];
        s1.vh1 = (f_it.Get()->faceVertexIndices_[3] > f_it.Get()->faceVertexIndices_[1]) ? f_it.Get()->faceVertexIndices_[3] : f_it.Get()->faceVertexIndices_[1];

        x0 = Vec3(grid_.vertexCoords_[s1.vh0]);
        x1 = Vec3(grid_.vertexCoords_[s1.vh1]);
        s1.l0 = (x0 - x1).mag();
        springSet_.insert(s1);

        shearSprings++;

      }

      VertexIter<Real> v_it = grid_.vertices_begin();
      UnstructuredGrid<Real, DTraits>::ElemVertIter ev_it  = grid_.begin(v_it);
      UnstructuredGrid<Real, DTraits>::ElemVertIter ev_end = grid_.end(v_it);
      std::vector<int> eav;
      for(;ev_it != ev_end; ev_it++)
      {

        std::cout << "> Element at vertex 0: " <<  ev_it.Get() << std::endl;
        eav.push_back(ev_it.Get());
      }

      //------------------------------------
      for(int vidx(0); vidx < grid_.nvt_; ++vidx)
      {
        VertexVertexIter vv_it = grid_.VertexVertexBegin(vidx);
        VertexVertexIter vv_end = grid_.VertexVertexEnd(vidx);
        for(;vv_it!=vv_end; vv_it++)
        {
          int myidx = vv_it.idx();
//          std::cout << "> VertexVertx: " <<  vv_it.idx() << std::endl;

          VertexVertexIter vvv_it  = grid_.VertexVertexBegin(myidx);
          VertexVertexIter vvv_end = grid_.VertexVertexEnd(myidx);

          for(;vvv_it!=vvv_end; vvv_it++)
          {
            int myidx2 = vvv_it.idx();

            if(myidx2==vidx)
              continue;

            UnstructuredGrid<Real, DTraits>::ElemVertIter evv_it  = grid_.begin(myidx2);
            UnstructuredGrid<Real, DTraits>::ElemVertIter evv_end = grid_.end(myidx2);
            bool common = false;
            for(;evv_it != evv_end; evv_it++)
            {
//              std::cout << "> Element at vertex " << myidx2 << " : " <<  evv_it.Get() << std::endl;
              int eidx = evv_it.Get();
              for(int j(0); j < eav.size(); ++j)
              {
                if(eav[j]==eidx) 
                {
                  common=true;
//                  std::cout << "> Common element " << myidx2 << " : " <<  evv_it.Get() << std::endl;
                  break;
                }
              }
              if(common)break;
            }

            if(!common)
            {
              //std::cout << "> Bend spring: " << vidx << " " << myidx2 << std::endl;
              SpringConstraint<Real, int> s(0.9, -0.3);

              s.vh0 = (vidx < myidx2) ? vidx : myidx2;
              s.vh1 = (myidx2 > vidx) ? myidx2 : vidx;

              Vec3 x0(grid_.vertexCoords_[s.vh0]);
              Vec3 x1(grid_.vertexCoords_[s.vh1]);
              s.l0 = (x0 - x1).mag();
              springSet_.insert(s);

            }

          }

        }
      }
      //------------------------------------

      for (auto &s : springSet_)
      {
        springs_.push_back(s);
      }
      std::cout << "> ShearSprings: " << shearSprings << std::endl;
      std::cout << "> Total springs: " << springSet_.size() << std::endl;

      VertexIter<Real> v_end = grid_.vertices_end();
      int j(0);
      const float myPI = 3.1415927;
      for (v_it = grid_.vertices_begin(); v_it != v_end; v_it++)
      {
        int Idx = v_it.idx();
        grid_.m_myTraits[Idx].fixed_ = false;
        grid_.m_myTraits[Idx].flagella_ = false;
      }
      for (v_it = grid_.vertices_begin(); v_it != v_end; v_it++)
      {
        int Idx = v_it.idx();
        Vec3 &c = grid_.vertexCoords_[Idx];
        if(std::abs(c.y) < 1e-4 && std::abs(c.z) < 1e-4)
        {
          grid_.m_myTraits[Idx].flagella_ = true;
//          std::cout << "> Flagella: " << Idx << std::endl;
//          std::cout << "> x: " << int(c.x) << std::endl;
          j=int(c.x);
          grid_.m_myTraits[Idx].t_ = j * 1.0 * myPI/22.0;
//          grid_.m_myTraits[Idx].t_ = 0.0;

//          VertexVertexIter vvv_it  = grid_.VertexVertexBegin(Idx);
//          VertexVertexIter vvv_end = grid_.VertexVertexEnd(Idx);
//
//          for(;vvv_it!=vvv_end; vvv_it++)
//          {
//            int myidx2 = vvv_it.idx();
//            grid_.m_myTraits[myidx2].flagella_ = true;
//            grid_.m_myTraits[myidx2].t_ = 0.0;
//
//          }

        }

        grid_.m_myTraits[Idx].mass_ = 0.5;
      }

    }

    void run() {

      steps_ = 1000;

      init();
      dt_ = 1.0/Real(steps_);
      time_ = 0.0;
      step_ = 0;

      for(int istep(0); istep <= steps_; ++istep)
      {

        simulate();
        //std::ostringstream name;
        //name << "output/cloth." << std::setfill('0') << std::setw(5) << istep << ".vtk";
        //OpenMesh::IO::Options            _opt = OpenMesh::IO::Options::Default;
        //std::streamsize            _precision = 6;
        //OpenMesh::IO::ExporterT<PolyMesh> exporter(polyMesh);
        //writer.write(name.str().c_str(), exporter, _opt, _precision);
        std::cout << "> Time step: " << istep << " Time: " << time_ << std::endl;
        writeOut(istep);
        time_ += dt_;
        step_++;

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
