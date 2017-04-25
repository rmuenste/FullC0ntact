#include <iostream>
#include <application.h>
#include <reader.h>
#include <distancemap.h>
#include <meshobject.h>
#include <distancemeshpoint.h>

namespace i3d {

  class Voxelization : public Application {

  public:

    Voxelization() : Application() {

    }

    void init(std::string fileName) {

      Application::init(fileName);
      
      grid_.initStdMesh();
      for(int i=0;i<5;i++)
      {
        grid_.refine();
        std::cout<<"Generating Grid level"<<i+1<<std::endl;
        std::cout<<"---------------------"<<std::endl;
        std::cout<<"NVT="<<grid_.nvt_<<" NEL="<<grid_.nel_<<std::endl;
        grid_.initStdMesh();
      }       

      RigidBody *body = this->myWorld_.rigidBodies_[0];
      CMeshObjectr *pMeshObject = dynamic_cast<CMeshObjectr *>(body->shape_);      
      VertexIter<Real> vIter;
      std::cout<<"Computing FBM information and distance..."<<std::endl;
      int icount=0;
      if (body->isInBody(grid_.vertexCoords_[497]))
      {
        std::cout << "in\n";
      }
      else
        std::cout << "out" << std::endl;

      //for(vIter=grid_.vertices_begin();vIter!=grid_.vertices_end();vIter++)
      for (int j(0); j < grid_.nvt_; ++j)
      {
        VECTOR3 vec = grid_.vertexCoords_[j];
        
        if(body->isInBody(vec))
        {
          grid_.m_myTraits[j].iTag=1;
        }
        else
        {
          grid_.m_myTraits[j].iTag=0;
        }        
        
        CDistanceMeshPoint<Real> distMeshPoint(&pMeshObject->m_BVH,vec);
        Real dist = distMeshPoint.ComputeDistanceEpsNaive(1);
        
        if(grid_.m_myTraits[j].iTag)
          dist*=-1.0;
        
        grid_.m_myTraits[j].distance=dist;        
        
        if(icount%1000==0)
        {
          std::cout<<"Progress: "<<icount<<"/"<<grid_.nvt_<<std::endl;        
        }        
        icount++;
      }      
    }

    void writeOutput(int out)
    {
      std::ostringstream sName;
      std::string sModel("output/model.vtk");

      CVtkWriter writer;
      int iTimestep = out;
      sName << "." << std::setfill('0') << std::setw(5) << iTimestep;
      sModel.append(sName.str());

      //Write the grid to a file and measure the time
      writer.WriteRigidBodies(myWorld_.rigidBodies_, sModel.c_str());

      if (out == 0 || out ==1)
      {
        std::ostringstream sNameGrid;
        std::string sGrid("output/grid.vtk");
        sNameGrid << "." << std::setfill('0') << std::setw(5) << iTimestep;
        sGrid.append(sNameGrid.str());
        writer.WriteUnstr(grid_, sGrid.c_str());
      }
    }

    void run()
    {
      unsigned nOut = 0;
      writeOutput(nOut);
      writeOutput(nOut+1);
    }

  };

}

using namespace i3d;



int main()
{
  Voxelization myApp;

  myApp.init("start/sampleRigidBody.xml");

  myApp.run();

  return 0;
}


