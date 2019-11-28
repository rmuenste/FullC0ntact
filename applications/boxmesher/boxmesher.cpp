
#include <iostream>
#include <application.h>
#include <reader.h>
#include <motionintegratorsi.h>
#include <meshobject.h>
#include <distancemeshpoint.h>
#include <laplace.h>
#include <intersectorray3tri3.h>
#include <perftimer.h>
#include <vtkwriter.h>
#include <geom_config.hpp>
#include <distancegridcgal.hpp>
#include <laplace_alpha.hpp>
#include <meshdecimater.hpp>
#include <distancemapbuilder.hpp>

namespace i3d {
 
  class BoxMesher : public Application<> {

  public:  
    
  BoxMesher() : Application()
  {
        
  }
  
  virtual ~BoxMesher() {};

  void writeOutput(int out)
  {
    std::ostringstream sName, sNameParticles, sphereFile;
    std::string sModel("output/model.vtk");

    CVtkWriter writer;
    int iTimestep = out;
    sName << "." << std::setfill('0') << std::setw(5) << iTimestep;
    sModel.append(sName.str());

    std::cout << "Writing VTK surface mesh to: " << sModel.c_str() << std::endl;
    //Write the grid to a file and measure the time
    writer.WriteRigidBodies(myWorld_.rigidBodies_, sModel.c_str());

  }
  
  void init(std::string fileName)
  {

    size_t pos = fileName.find(".");

    std::string ending = fileName.substr(pos);

    std::transform(ending.begin(), ending.end(), ending.begin(), ::tolower);
    if (ending == ".txt")
    {

      Reader myReader;
      //Get the name of the mesh file from the
      //configuration data file.
      myReader.readParameters(fileName, this->dataFileParams_);

    }//end if
    else if (ending == ".xml")
    {

      FileParserXML myReader;

      //Get the name of the mesh file from the
      //configuration data file.
      myReader.parseDataXML(this->dataFileParams_, fileName);

    }//end if
    else
    {
      std::cerr << "Invalid data file ending: " << ending << std::endl;
      std::exit(EXIT_FAILURE);
    }//end else

    //initialize rigid body parameters and
    //placement in the domain
    configureRigidBodies();

    grid_.initCube(-1.0, -1.0, -1.0, 1.0, 1.0, 1.0);

    configureBoundary();

    //assign the rigid body ids
    for (int j = 0; j<myWorld_.rigidBodies_.size(); j++)
      myWorld_.rigidBodies_[j]->iID_ = j;

  }
  
  void run()
  {

    CUnstrGridr ugrid;
    CVtkWriter writer;

    for (auto &body : myWorld_.rigidBodies_)
    {

      if (!(body->shapeId_ == RigidBody::MESH || body->shapeId_ == RigidBody::CGALMESH))
        continue;

      std::cout << "Creating distance map" <<std::endl;

      DistanceMapBuilder<Real> dmapBuilder(body, &dataFileParams_);

      if(dataFileParams_.hasUserMeshingParameters_) {
        dmapBuilder.buildDistanceMapUser();
      } else {
        dmapBuilder.buildDistanceMap();
      }

    }

    std::cout << "> Initial distance map built!" << std::endl;
    for (auto &body : myWorld_.rigidBodies_)
    {

      if (!(body->shapeId_ == RigidBody::MESH || body->shapeId_ == RigidBody::CGALMESH))
        continue;
    
      body->map_->convertToUnstructuredGrid(ugrid);

    }

    ugrid.calcVol();
    writer.WriteUnstr(ugrid, "output/InitialGrid.vtk");

    RigidBody *body = myWorld_.rigidBodies_[0];

    MeshObject<Real, cgalKernel> *object = dynamic_cast< MeshObject<Real, cgalKernel> *>(body->shape_);

    DistanceGridMesh<Real> distance(&ugrid, object);

    distance.ComputeDistance();

    ugrid.initStdMesh();

    LaplaceAlpha<Real> smoother(&ugrid, object, dataFileParams_.adaptationSteps_);
    smoother.smooth();

    distance.ComputeDistance();
    distance.ComputeElementDistance();

    MeshDecimater<Real> decimater(&ugrid, object, &dataFileParams_);
    decimater.decimate();

    distance.ComputeDistance();

    writer.WriteUnstr(ugrid, "output/DistanceMap.01.vtk");
    writer.WriteUnstr(ugrid, "output/DistanceMap.02.vtk");
    writer.WriteGrid2Tri(ugrid, "meshes/dmap.tri", Vec3(0,0,-375));

    writeOutput(0);    
    writeOutput(1);
  }
    
};
}

using namespace i3d;

int main()
{

  BoxMesher myApp;
  
  myApp.init("start/sampleRigidBody.xml");
  
  myApp.run();
  
  return EXIT_SUCCESS;
}
