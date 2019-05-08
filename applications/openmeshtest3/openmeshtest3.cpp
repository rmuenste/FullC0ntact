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

    virtual Vector3<T> evalDistConstraint(const Vector3<T> &x0, const Vector3<T> &x1, const Vector3<T> &v0, const Vector3<T> &v1)
    {
      L = (x0 - x1);
      l = L.mag();
      if(l == 0.0f)
        return Vector3<T>(0,0,0);

      // what if vertices coincide?
      Vector3<T> LN = (1.0 / l) * L;
      T kd = 0.5;
      T tl = (l - l0);

      Vector3<T> f = kd * (tl) * LN; 

      return f;
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

  class OpenMeshTest : public Application<> {

  public:

    UnstructuredGrid<Real, DTraits> fish_;
    
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
    Real speed_;
    Real dampening_;

    OpenMeshTest() : Application(), vrow(21) {

    }

    void init(std::string fileName) {

      Application::init(fileName);
      std::string meshFile("meshes/tryp5.tri3d");
      fish_.initMeshFromFile(meshFile.c_str());
      std::cout << "> Building standard mesh info." << std::endl;
      fish_.initStdMesh();
      fish_.calcVol();
      std::cout<<"> fish volume: " << fish_.vol_ <<std::endl;

      meshFile = std::string("meshes/mesh.tri3d");
      grid_.initMeshFromFile(meshFile.c_str());

      grid_.initStdMesh();
      for(int i=0;i<3;i++)
      {
        grid_.refine();
        std::cout<<"Generating Grid level"<<i+1<<std::endl;
        std::cout<<"---------------------"<<std::endl;
        std::cout<<"NVT="<<grid_.nvt_<<" NEL="<<grid_.nel_<<std::endl;
        grid_.initStdMesh();
      }       
      grid_.calcVol();

      std::cout<<"> Grid volume: " << grid_.vol_ <<std::endl;

    }

    void writeOut(int out)
    {
      std::ostringstream sName, sNameParticles, gridFile;
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
      gridFile << "output/grid.vtk." << std::setfill('0') << std::setw(5) << iTimestep;
      //Write the grid to a file and measure the time

      std::ostringstream sNameGrid;
      std::string sGrid("output/fish.vtk");
      sNameGrid << "." << std::setfill('0') << std::setw(5) << iTimestep;
      sGrid.append(sNameGrid.str());
      writer.WriteSpringMesh(fish_, sGrid.c_str());
      writer.WriteUnstr(grid_, gridFile.str().c_str());

    }

    void flagellaFunction2()
    {
      VertexIter<Real> v_it;
      VertexIter<Real> v_end = fish_.vertices_end();
      for(v_it = fish_.vertices_begin(); v_it!=v_end; v_it++)
      {
        int Idx = v_it.idx();
        if(!fish_.m_myTraits[Idx].flagella_)
          continue;

        Vec3 &vel = fish_.m_myTraits[Idx].vel_;

        Vec3 &force = fish_.m_myTraits[Idx].force_;

        Vec3 pos = Vec3(fish_.vertexCoords_[Idx]);

        Real &td = fish_.m_myTraits[Idx].t_;
        
        pos.z = 100.0f*dampening_*std::sin(td);

        Vec3 old_pos = fish_.m_myTraits[Idx].pos_old_;

        td += speed_;

        vel = pos - old_pos;

        fish_.vertexCoords_[Idx] = pos;

        force = Vec3(0,0,0);
      }
    }

    void flagellaFunction3()
    {

      VertexIter<Real> v_it;
      VertexIter<Real> v_end = fish_.vertices_end();
      for(v_it = fish_.vertices_begin(); v_it!=v_end; v_it++)
      {
        int Idx = v_it.idx();
        if(!fish_.m_myTraits[Idx].flagella_)
          continue;

        Vec3 &vel = fish_.m_myTraits[Idx].vel_;

        Vec3 &force = fish_.m_myTraits[Idx].force_;

        Vec3 pos = Vec3(fish_.vertexCoords_[Idx]);

        Real &td = fish_.m_myTraits[Idx].t_;
        
        Real r = Real(step_)/Real(steps_);

        pos.z = r * 100.0f * dampening_ * std::sin(td);

        Vec3 old_pos = fish_.m_myTraits[Idx].pos_old_;

        vel = pos - old_pos;

        fish_.vertexCoords_[Idx] = pos;

        force = Vec3(0,0,0);
      }
    }

    void integrate2()
    {

      Real dt = dt_;

      VertexIter<Real> v_it;
      VertexIter<Real> v_end = fish_.vertices_end();
      for(v_it = fish_.vertices_begin(); v_it!=v_end; v_it++)
      {
        int Idx = v_it.idx();

        if(fish_.m_myTraits[Idx].fixed_)
          continue;

        Vec3 &vel = fish_.m_myTraits[Idx].vel_;

        Vec3 &force = fish_.m_myTraits[Idx].force_;

        Vec3 g(0,0.00,0);

        Real &m = fish_.m_myTraits[Idx].mass_;

        Vec3 pos = Vec3(fish_.vertexCoords_[Idx]);
        
        vel = vel + dt * force * (1.0/m);

        fish_.m_myTraits[Idx].pos_old_ = pos;

        pos.z = pos.z + dt * vel.z;

        fish_.vertexCoords_[Idx] = pos;

        force = Vec3(0,0,0);
      }
    }

    void integrate()
    {

      Real dt = dt_;

      VertexIter<Real> v_it;
      VertexIter<Real> v_end = fish_.vertices_end();
      for(v_it = fish_.vertices_begin(); v_it!=v_end; v_it++)
      {
        int Idx = v_it.idx();

        if(fish_.m_myTraits[Idx].fixed_)
          continue;

        Vec3 &vel = fish_.m_myTraits[Idx].vel_;

        Vec3 &force = fish_.m_myTraits[Idx].force_;

        Vec3 g(0,0.00,0);

        force += g;

        force += -0.0125 * vel;

        Real &m = fish_.m_myTraits[Idx].mass_;

        Vec3 pos = Vec3(fish_.vertexCoords_[Idx]);
        
        vel = vel + dt * force * (1.0/m);

        fish_.m_myTraits[Idx].pos_old_ = pos;

        pos.z = pos.z + dt * vel.z;

        fish_.vertexCoords_[Idx] = pos;

        force = Vec3(0,0,0);
      }

      flagellaFunction2();

    }

    void simulate()
    {

      addProvotDynamicInverse();
      for (auto &s : springs_)
      {
        int vh0 = s.vh0;
        int vh1 = s.vh1;

        Vec3 x0(fish_.vertexCoords_[vh0]);
        Vec3 v0(fish_.m_myTraits[vh0].vel_);
        Vec3 x1(fish_.vertexCoords_[vh1]);
        Vec3 v1(fish_.m_myTraits[vh1].vel_);

        Vec3 f = s.evalForce(x0, x1, v0, v1);
        fish_.m_myTraits[vh0].force_ += f;
        fish_.m_myTraits[vh1].force_ -= f;
      }
      integrate();
      for (auto &s : springs_)
      {
        int vh0 = s.vh0;
        int vh1 = s.vh1;

        Vec3 x0(fish_.vertexCoords_[vh0]);
        Vec3 v0(fish_.m_myTraits[vh0].vel_);
        Vec3 x1(fish_.vertexCoords_[vh1]);
        Vec3 v1(fish_.m_myTraits[vh1].vel_);


        Vec3 f = s.evalDistConstraint(x0, x1, v0, v1);
        if(fish_.m_myTraits[vh0].flagella_)
        {
          fish_.m_myTraits[vh1].force_ += f;
        }
        else if(fish_.m_myTraits[vh1].flagella_)
        {
          fish_.m_myTraits[vh0].force_ -= f;
        }
        else
        {
          fish_.m_myTraits[vh0].force_ -= f;
          fish_.m_myTraits[vh1].force_ += f;
        }
      }
      integrate2();

    }

    void addProvotDynamicInverse() {

      for (auto &s : springs_)
      {
        int vh0 = s.vh0;
        int vh1 = s.vh1;

        Vec3 x0(fish_.vertexCoords_[vh0]);
        Vec3 v0(fish_.m_myTraits[vh0].vel_);
        Vec3 x1(fish_.vertexCoords_[vh1]);
        Vec3 v1(fish_.m_myTraits[vh1].vel_);

        Vec3 deltaP = x0 - x1;
        float dist = deltaP.mag();
        if (dist > s.l0)
        {
          dist -= (s.l0);
          dist /= 2.0f;
          deltaP.normalize();
          deltaP *= dist;
          if (fish_.m_myTraits[vh0].flagella_) {
            fish_.m_myTraits[vh1].vel_ += 2.0 * deltaP;
          }
          else if (fish_.m_myTraits[vh1].flagella_) {
            fish_.m_myTraits[vh0].vel_ -= 2.0 * deltaP;
          }
          else {
            fish_.m_myTraits[vh0].vel_ -= deltaP;
            fish_.m_myTraits[vh1].vel_ += deltaP;
          }
        }
      }
    }

    void init()
    {

      EdgeIter e_it = fish_.edge_begin();
      for (; e_it != fish_.edge_end(); e_it++)
      {
        SpringConstraint<Real, int> s;

        s.vh0 = (e_it.Get()->edgeVertexIndices_[0] < e_it.Get()->edgeVertexIndices_[1]) ? e_it.Get()->edgeVertexIndices_[0] : e_it.Get()->edgeVertexIndices_[1];
        s.vh1 = (e_it.Get()->edgeVertexIndices_[1] > e_it.Get()->edgeVertexIndices_[0]) ? e_it.Get()->edgeVertexIndices_[1] : e_it.Get()->edgeVertexIndices_[0];

        Vec3 x0(fish_.vertexCoords_[s.vh0]);
        Vec3 x1(fish_.vertexCoords_[s.vh1]);
        s.l0 = (x0 - x1).mag();
        springSet_.insert(s);

      }

      std::cout << "> Structural springs: " << springSet_.size() << std::endl;

      int shearSprings = 0;
      ElementIter el_it = fish_.elem_begin();
      for (; el_it != fish_.elem_end(); el_it++)
      {

        SpringConstraint<Real, int> s;

        s.vh0 = (el_it.Get()->hexaVertexIndices_[0] < el_it.Get()->hexaVertexIndices_[6]) ? el_it.Get()->hexaVertexIndices_[0] : el_it.Get()->hexaVertexIndices_[6];
        s.vh1 = (el_it.Get()->hexaVertexIndices_[6] > el_it.Get()->hexaVertexIndices_[0]) ? el_it.Get()->hexaVertexIndices_[6] : el_it.Get()->hexaVertexIndices_[0];

        Vec3 x0(fish_.vertexCoords_[s.vh0]);
        Vec3 x1(fish_.vertexCoords_[s.vh1]);
        s.l0 = (x0 - x1).mag();
        springSet_.insert(s);

        shearSprings++;

        // 2nd spring
        SpringConstraint<Real, int> s1;

        s1.vh0 = (el_it.Get()->hexaVertexIndices_[1] < el_it.Get()->hexaVertexIndices_[7]) ? el_it.Get()->hexaVertexIndices_[1] : el_it.Get()->hexaVertexIndices_[7];
        s1.vh1 = (el_it.Get()->hexaVertexIndices_[7] > el_it.Get()->hexaVertexIndices_[1]) ? el_it.Get()->hexaVertexIndices_[7] : el_it.Get()->hexaVertexIndices_[1];

        x0 = Vec3(fish_.vertexCoords_[s1.vh0]);
        x1 = Vec3(fish_.vertexCoords_[s1.vh1]);
        s1.l0 = (x0 - x1).mag();
        springSet_.insert(s1);

        shearSprings++;

        // 3rd spring
        SpringConstraint<Real, int> s2;

        s2.vh0 = (el_it.Get()->hexaVertexIndices_[2] < el_it.Get()->hexaVertexIndices_[4]) ? el_it.Get()->hexaVertexIndices_[2] : el_it.Get()->hexaVertexIndices_[4];
        s2.vh1 = (el_it.Get()->hexaVertexIndices_[4] > el_it.Get()->hexaVertexIndices_[2]) ? el_it.Get()->hexaVertexIndices_[4] : el_it.Get()->hexaVertexIndices_[2];

        x0 = Vec3(fish_.vertexCoords_[s2.vh0]);
        x1 = Vec3(fish_.vertexCoords_[s2.vh1]);
        s2.l0 = (x0 - x1).mag();
        springSet_.insert(s2);

        shearSprings++;

        // 4th spring
        SpringConstraint<Real, int> s3;

        s3.vh0 = (el_it.Get()->hexaVertexIndices_[3] < el_it.Get()->hexaVertexIndices_[5]) ? el_it.Get()->hexaVertexIndices_[3] : el_it.Get()->hexaVertexIndices_[5];
        s3.vh1 = (el_it.Get()->hexaVertexIndices_[5] > el_it.Get()->hexaVertexIndices_[3]) ? el_it.Get()->hexaVertexIndices_[5] : el_it.Get()->hexaVertexIndices_[3];

        x0 = Vec3(fish_.vertexCoords_[s3.vh0]);
        x1 = Vec3(fish_.vertexCoords_[s3.vh1]);
        s3.l0 = (x0 - x1).mag();
        springSet_.insert(s3);

        shearSprings++;

      }


      HFaceIter f_it = fish_.faces_begin();
      for (; f_it != fish_.faces_end(); f_it++)
      {

        SpringConstraint<Real, int> s;

        s.vh0 = (f_it.Get()->faceVertexIndices_[0] < f_it.Get()->faceVertexIndices_[2]) ? f_it.Get()->faceVertexIndices_[0] : f_it.Get()->faceVertexIndices_[2];
        s.vh1 = (f_it.Get()->faceVertexIndices_[2] > f_it.Get()->faceVertexIndices_[0]) ? f_it.Get()->faceVertexIndices_[2] : f_it.Get()->faceVertexIndices_[0];

        Vec3 x0(fish_.vertexCoords_[s.vh0]);
        Vec3 x1(fish_.vertexCoords_[s.vh1]);
        s.l0 = (x0 - x1).mag();
        springSet_.insert(s);

        shearSprings++;

        // 2nd spring
        SpringConstraint<Real, int> s1;

        s1.vh0 = (f_it.Get()->faceVertexIndices_[1] < f_it.Get()->faceVertexIndices_[3]) ? f_it.Get()->faceVertexIndices_[1] : f_it.Get()->faceVertexIndices_[3];
        s1.vh1 = (f_it.Get()->faceVertexIndices_[3] > f_it.Get()->faceVertexIndices_[1]) ? f_it.Get()->faceVertexIndices_[3] : f_it.Get()->faceVertexIndices_[1];

        x0 = Vec3(fish_.vertexCoords_[s1.vh0]);
        x1 = Vec3(fish_.vertexCoords_[s1.vh1]);
        s1.l0 = (x0 - x1).mag();
        springSet_.insert(s1);

        shearSprings++;

      }

      VertexIter<Real> v_it = fish_.vertices_begin();
      UnstructuredGrid<Real, DTraits>::ElemVertIter ev_it  = fish_.begin(v_it);
      UnstructuredGrid<Real, DTraits>::ElemVertIter ev_end = fish_.end(v_it);
      std::vector<int> eav;
      for(;ev_it != ev_end; ev_it++)
      {
        eav.push_back(ev_it.Get());
      }

      //------------------------------------
      for(int vidx(0); vidx < fish_.nvt_; ++vidx)
      {
        VertexVertexIter vv_it = fish_.VertexVertexBegin(vidx);
        VertexVertexIter vv_end = fish_.VertexVertexEnd(vidx);
        for(;vv_it!=vv_end; vv_it++)
        {
          int myidx = vv_it.idx();

          VertexVertexIter vvv_it  = fish_.VertexVertexBegin(myidx);
          VertexVertexIter vvv_end = fish_.VertexVertexEnd(myidx);

          for(;vvv_it!=vvv_end; vvv_it++)
          {
            int myidx2 = vvv_it.idx();

            if(myidx2==vidx)
              continue;

            UnstructuredGrid<Real, DTraits>::ElemVertIter evv_it  = fish_.begin(myidx2);
            UnstructuredGrid<Real, DTraits>::ElemVertIter evv_end = fish_.end(myidx2);
            bool common = false;
            for(;evv_it != evv_end; evv_it++)
            {
              int eidx = evv_it.Get();
              for(int j(0); j < eav.size(); ++j)
              {
                if(eav[j]==eidx) 
                {
                  common=true;
                  break;
                }
              }
              if(common)break;
            }

            if(!common)
            {
              SpringConstraint<Real, int> s(0.1, -0.25);

              s.vh0 = (vidx < myidx2) ? vidx : myidx2;
              s.vh1 = (myidx2 > vidx) ? myidx2 : vidx;

              Vec3 x0(fish_.vertexCoords_[s.vh0]);
              Vec3 x1(fish_.vertexCoords_[s.vh1]);
              s.l0 = (x0 - x1).mag();
              springSet_.insert(s);

            }

          }

        }
      }

      for (auto &s : springSet_)
      {
        springs_.push_back(s);
      }
      std::cout << "> ShearSprings: " << shearSprings << std::endl;
      std::cout << "> Total springs: " << springSet_.size() << std::endl;

      VertexIter<Real> v_end = fish_.vertices_end();
      int j(0);
      const float myPI = 3.1415927;
      for (v_it = fish_.vertices_begin(); v_it != v_end; v_it++)
      {
        int Idx = v_it.idx();
        fish_.m_myTraits[Idx].fixed_ = false;
        fish_.m_myTraits[Idx].flagella_ = false;
      }
      for (v_it = fish_.vertices_begin(); v_it != v_end; v_it++)
      {
        int Idx = v_it.idx();
        Vec3 &c = fish_.vertexCoords_[Idx];
        if(std::abs(c.y) < 1e-4 && std::abs(c.z) < 1e-4)
        {
          fish_.m_myTraits[Idx].flagella_ = true;
          j=int(c.x);
          fish_.m_myTraits[Idx].t_ = j * 1.0 * myPI/22.0;
        }

        fish_.m_myTraits[Idx].mass_ = 0.5;
      }

    }

    void integrateInitial()
    {
      Real dt = dt_;

      VertexIter<Real> v_it;
      VertexIter<Real> v_end = fish_.vertices_end();
      for(v_it = fish_.vertices_begin(); v_it!=v_end; v_it++)
      {

        int Idx = v_it.idx();

        if(fish_.m_myTraits[Idx].fixed_)
          continue;

        Vec3 &vel = fish_.m_myTraits[Idx].vel_;

        Vec3 &force = fish_.m_myTraits[Idx].force_;

        Vec3 g(0,0.00,0);

        Real &m = fish_.m_myTraits[Idx].mass_;

        Vec3 pos = Vec3(fish_.vertexCoords_[Idx]);
        
        vel = vel + dt * force * (1.0/m);

        fish_.m_myTraits[Idx].pos_old_ = pos;

        pos.z = pos.z + dt * vel.z;

        fish_.vertexCoords_[Idx] = pos;

      }

      flagellaFunction3();

    }

    void simulateInitial()
    {
      VertexIter<Real> v_it;
      VertexIter<Real> v_end = fish_.vertices_end();
      for(v_it = fish_.vertices_begin(); v_it!=v_end; v_it++)
      {

        int Idx = v_it.idx();

        if(fish_.m_myTraits[Idx].fixed_)
          continue;

        fish_.m_myTraits[Idx].force_ = Vec3(0,0,0);

      }

      for (auto &s : springs_)
      {
        int vh0 = s.vh0;
        int vh1 = s.vh1;

        Vec3 x0(fish_.vertexCoords_[vh0]);
        Vec3 v0(fish_.m_myTraits[vh0].vel_);
        Vec3 x1(fish_.vertexCoords_[vh1]);
        Vec3 v1(fish_.m_myTraits[vh1].vel_);


        Vec3 f = s.evalForce(x0, x1, v0, v1);
        if(fish_.m_myTraits[vh0].flagella_)
        {
          fish_.m_myTraits[vh1].force_ -= 2.0 * f;
        }
        else if(fish_.m_myTraits[vh1].flagella_)
        {
          fish_.m_myTraits[vh0].force_ += 2.0 * f;
        }
        else
        {
          fish_.m_myTraits[vh0].force_ += f;
          fish_.m_myTraits[vh1].force_ -= f;
        }
      }
      integrateInitial();
      for (auto &s : springs_)
      {
        int vh0 = s.vh0;
        int vh1 = s.vh1;

        Vec3 x0(fish_.vertexCoords_[vh0]);
        Vec3 v0(fish_.m_myTraits[vh0].vel_);
        Vec3 x1(fish_.vertexCoords_[vh1]);
        Vec3 v1(fish_.m_myTraits[vh1].vel_);


        Vec3 f = s.evalDistConstraint(x0, x1, v0, v1);
        if(fish_.m_myTraits[vh0].flagella_)
        {
          fish_.m_myTraits[vh1].force_ += f;
        }
        else if(fish_.m_myTraits[vh1].flagella_)
        {
          fish_.m_myTraits[vh0].force_ -= f;
        }
        else
        {
          fish_.m_myTraits[vh0].force_ -= f;
          fish_.m_myTraits[vh1].force_ += f;
        }
      }
      integrate2();
      addProvotDynamicInverse();
    }

    void initialCondition()
    {
      steps_ = 1000;

      dt_ = 1.0/Real(steps_);
      time_ = 0.0;
      step_ = 0;
      dampening_ = 0.0025;

      for(int istep(0); istep <= steps_; ++istep)
      {

        simulateInitial();
        std::cout << "> Time step: " << istep << " Time: " << time_ << std::endl;
        time_ += dt_;
        step_++;

      }
    }

    void run() {

      init();

      initialCondition();
      std::cout << "> Inital condition set. "  << std::endl;

      steps_ = 40000;
      speed_ = 0.001;
      dt_ = 0.0025;

      AABB3<Real> box = fish_.getAABB();
      for(int istep(0); istep <= steps_; ++istep)
      {

        simulate();
        //std::ostringstream name;
        //name << "output/cloth." << std::setfill('0') << std::setw(5) << istep << ".vtk";
        //OpenMesh::IO::Options            _opt = OpenMesh::IO::Options::Default;
        //std::streamsize            _precision = 6;
        //OpenMesh::IO::ExporterT<PolyMesh> exporter(polyMesh);
        //writer.write(name.str().c_str(), exporter, _opt, _precision);
        std::cout << "> Time step: " << istep << std::endl;
        if(istep%100==0)
        {
          VertexIter<Real> v_it;
          VertexIter<Real> v_end = grid_.vertices_end();
          for(v_it = grid_.vertices_begin(); v_it != v_end; v_it++)
          { 

            Vec3 v(*v_it);  
            int id = v_it.idx();

            if(!box.isPointInside(v))
            {
              grid_.m_myTraits[id].iTag=0;
              continue;
            } 
            
            if(fish_.pointInside(v))
            {
              grid_.m_myTraits[id].iTag=1;
            }
            else
            {
              grid_.m_myTraits[id].iTag=0;
            }        

          }
          std::cout<<"> Computing FBM information..."<<std::endl;
          writeOut(istep);
        }
//          if(istep==2200)
//          {
//
//            //Vec3 q(8.875,0.0,0.5);
//            Vec3 q(grid_.vertexCoords_[3061]);
//            if(box.isPointInside(q))
//            {
//              std::cout<<"> inside box"<<std::endl;
//            } 
//            //fish_.pointInsideHexaDebug(56,q);
//            if(fish_.pointInsideHexaDebug(58,q))
//              std::cout<<"> inside hexa 58"<<std::endl;
//            if(!fish_.pointInside(q))
//              std::cout<<"> Not inside...wrong"<<std::endl;
//            std::exit(EXIT_FAILURE);
//          }

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
