#include <iostream>
#include <application.h>
#include <reader.h>
#include <paramline.h>
#include <collisionpipelineode.hpp>

#include <ode/odeconfig.h>
#include <assert.h>
#ifdef HAVE_UNISTD_H
#include <unistd.h>
#endif
#include <ode/ode.h>
#include <triangulator.h>
#include <json.hpp>

// dynamics and collision objects (chassis, 3 wheels, environment)
static dWorldID world;
static dSpaceID space;

static dBodyID cylbody;
static dGeomID cylgeom;

static dBodyID sphbody;
static dGeomID sphgeom;

static dJointGroupID contactgroup;

static bool show_contacts = true;

double simTime = 0.0;
double dt = 0.01;

const double CYLRADIUS = 0.6;
const double CYLLENGTH = 2.0;
const double SPHERERADIUS = 0.5;

#ifdef dDOUBLE
  #define dsDrawBox dsDrawBoxD
  #define dsDrawLine dsDrawLineD
#endif


namespace i3d {



  class ODE_Test : public Application {

  public:

    CollisionPipelineODE collPipeline_;

    ODE_Test() : Application() {

    }

    ~ODE_Test()
    {
      dJointGroupEmpty (myWorld_.contactgroup);
      dJointGroupDestroy (myWorld_.contactgroup);

      dSpaceDestroy (myWorld_.space);
      dWorldDestroy (myWorld_.world);
      dCloseODE();
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
        exit(1);
      }//end else

      if (hasMeshFile_)
      {
        std::string fileName;
        grid_.initMeshFromFile(fileName.c_str());
      }
      else
      {
        if (dataFileParams_.hasExtents_)
        {
          grid_.initCube(dataFileParams_.extents_[0], dataFileParams_.extents_[2],
                         dataFileParams_.extents_[4], dataFileParams_.extents_[1],
                         dataFileParams_.extents_[3], dataFileParams_.extents_[5]);
        }
        else
          grid_.initCube(xmin_, ymin_, zmin_, xmax_, ymax_, zmax_);
      }

      //initialize rigid body parameters and
      //placement in the domain
      configureRigidBodies();

      configureBoundary();

      //assign the rigid body ids
      for (unsigned j = 0; j < myWorld_.rigidBodies_.size(); j++)
      {
        myWorld_.rigidBodies_[j]->iID_ = j;
        if (myWorld_.rigidBodies_[j]->shapeId_ == RigidBody::COMPOUND)
        {
          CompoundBody *c = dynamic_cast<CompoundBody*>(myWorld_.rigidBodies_[j]);
          for (unsigned i = 0; i < c->rigidBodies_.size(); i++)
          {
            c->rigidBodies_[i]->iID_ = j;
          }
        }
      }
      configureTimeDiscretization();

      //link the boundary to the world
      myWorld_.setBoundary(&myBoundary_);

      //set the time control
      myWorld_.setTimeControl(&myTimeControl_);

      //set the gravity
      myWorld_.setGravity(dataFileParams_.gravity_);

      //set air friction
      myWorld_.setAirFriction(dataFileParams_.airFriction_);

      collPipeline_.world_ = &myWorld_;

      //Set the collision epsilon
      collPipeline_.setEPS(0.02);

      //set which type of rigid motion we are dealing with
      myMotion_ = new RigidBodyMotion(&myWorld_);

      myWorld_.densityMedium_ = dataFileParams_.densityMedium_;

      myWorld_.liquidSolid_ = dataFileParams_.liquidSolid_;

      myWorld_.graph_ = myPipeline_.graph_;

    }

    void run()
    {
      for (int i(0); i <= 300; ++i)
      {

        collPipeline_.startPipeline();

        outputVTK(i);
        printf("Time: %f |Step: %d |\n",simTime, i);
        simTime += dt;
      }

    }

    void outputVTK(int istep)
    {

      using namespace std;

      std::ostringstream sName;

      sName << "output/model." << std::setw(5) << std::setfill('0') << istep << ".vtk";
      std::string strFileName(sName.str());

      CVtkWriter writer;

      writer.WriteODE2VTK(myWorld_.bodies_, strFileName.c_str());

//      FILE * myfile = fopen(strFileName.c_str(),"w");

//      if (myfile == nullptr) {
//        cout<<"Error opening file: "<<strFileName<<endl;
//        std::exit(EXIT_FAILURE);
//      }

//      //total number of vertices
//      int iVerts=0;
//      //total number of polygons
//      int iPolys=0;
//      //iterators for models and submeshes
//      vector<Mesh3D>::iterator meshIter;
//      vector<Model3D>::iterator modelIter;
//      vector<RigidBody*>::iterator rIter;
//      vector<Model3D> pModels;
//      vector<int> vVerts;
//      vector<int>::iterator vertsIter;
//      int ioffset=0;

//      for (auto &body : myWorld_.bodies_)
//      {

//        int gId = dGeomGetClass(body._geomId);

//        if (gId == dSphereClass)
//        {
//          const dReal *SPos = dBodyGetPosition(body._bodyId);
//          const dReal *SRot = dBodyGetRotation(body._bodyId);
//          float spos[3] = {SPos[0], SPos[1], SPos[2]};
//          float srot[12] = { SRot[0], SRot[1], SRot[2], SRot[3], SRot[4], SRot[5], SRot[6], SRot[7], SRot[8], SRot[9], SRot[10], SRot[11] };

//          CTriangulator<Real, Sphere<Real> > triangulator;

//          Real rad = Real(dGeomSphereGetRadius(body._geomId));

//          Spherer Sphere(Vec3(0,0,0),rad);
//          
//          Model3D model_out=triangulator.Triangulate(&Sphere);

//          double entries[9] = { SRot[0], SRot[1], SRot[2], /* */ SRot[4], SRot[5], SRot[6], /* */ SRot[8], SRot[9], SRot[10] };

//          MATRIX3X3 transform(entries);
//          model_out.meshes_[0].transform_ = transform;
//          model_out.meshes_[0].com_ = Vec3(SPos[0], SPos[1], SPos[2]);
//          model_out.meshes_[0].TransformModelWorld();
//          pModels.push_back(model_out);
//        }
//        else if (gId == dPlaneClass)
//        {
//          dVector4 v;
//          dGeomPlaneGetParams(body._geomId, v);
//          Planer pl(v[0], v[1], v[2], v[3]);

//          CTriangulator<Real, Plane<Real> > triangulator;
//          Model3D model_out=triangulator.Triangulate(pl);
//          pModels.push_back(model_out);
//        }
//        else if (gId == dBoxClass)
//        {
//          const dReal *SPos = dBodyGetPosition(body._bodyId);
//          const dReal *SRot = dBodyGetRotation(body._bodyId);
//          float spos[3] = {SPos[0], SPos[1], SPos[2]};
//          float srot[12] = { SRot[0], SRot[1], SRot[2], SRot[3], SRot[4], SRot[5], SRot[6], SRot[7], SRot[8], SRot[9], SRot[10], SRot[11] };

//          CTriangulator<Real, OBB3<Real> > triangulator;

//          dVector3 dim;
//          dGeomBoxGetLengths(body._geomId, dim);
//          
//          OBB3r box(Vec3(0, 0, 0), Vec3(1, 0, 0), Vec3(0, 1, 0), Vec3(0, 0, 1), 0.5 * dim[0], 0.5 * dim[1], 0.5 * dim[2]); 
//          
//          Model3D model_out=triangulator.Triangulate(box);

//          double entries[9] = { SRot[0], SRot[1], SRot[2], /* */ SRot[4], SRot[5], SRot[6], /* */ SRot[8], SRot[9], SRot[10] };

//          MATRIX3X3 transform(entries);
//          model_out.meshes_[0].transform_ = transform;
//          model_out.meshes_[0].com_ = Vec3(SPos[0], SPos[1], SPos[2]);
//          model_out.meshes_[0].TransformModelWorld();

//          pModels.push_back(model_out);
//        }
//        else if (gId == dCylinderClass)
//        {
//          const dReal *SPos = dBodyGetPosition(body._bodyId);
//          const dReal *SRot = dBodyGetRotation(body._bodyId);
//          float spos[3] = {SPos[0], SPos[1], SPos[2]};
//          float srot[12] = { SRot[0], SRot[1], SRot[2], 
//                             SRot[3], SRot[4], SRot[5], 
//                             SRot[6], SRot[7], SRot[8], 
//                             SRot[9], SRot[10], SRot[11] };

//          CTriangulator<Real, Cylinder<Real> > triangulator;

//          dReal radius, l;
//          dGeomCylinderGetParams(body._geomId, &radius, &l);
//          
//          Cylinderr cyl(Vec3(0,0,0),
//                        Vec3(0,0,1),
//                        radius, 0.5 * l);

//          
//          Model3D model_out=triangulator.Triangulate(cyl);

//          double entries[9] = { SRot[0], SRot[1], SRot[2], /* */ 
//                                SRot[4], SRot[5], SRot[6], /* */ 
//                                SRot[8], SRot[9], SRot[10] };

//          MATRIX3X3 transform(entries);
//          model_out.meshes_[0].transform_ = transform;
//          model_out.meshes_[0].com_ = Vec3(SPos[0], SPos[1], SPos[2]);
//          model_out.meshes_[0].TransformModelWorld();

//          pModels.push_back(model_out);
//        }
//      }

//      for(modelIter = pModels.begin();modelIter!=pModels.end();modelIter++)
//      {
//        Model3D &pModel = *modelIter;
//        int ivertsModel=0;
//        for(meshIter=pModel.meshes_.begin();meshIter!=pModel.meshes_.end();meshIter++)
//        {
//          Mesh3D &pMesh=*meshIter;
//          iVerts+=pMesh.numVerts_;
//          iPolys+=pMesh.numFaces_;
//          ivertsModel+=pMesh.numVerts_;
//        }
//        vVerts.push_back(ivertsModel);
//      }

//      fprintf(myfile,"# vtk DataFile Version 2.0\n");
//      fprintf(myfile,"Generated by InShape 2.x\n");
//      fprintf(myfile,"ASCII\n");
//      fprintf(myfile,"DATASET POLYDATA\n");
//      fprintf(myfile,"POINTS %i double\n",iVerts);

//      int count=0;
//      //write the actual vertex data
//      for(modelIter = pModels.begin();modelIter!=pModels.end();modelIter++)
//      {
//        Model3D &pModel = *modelIter;
//        for(meshIter=pModel.meshes_.begin();meshIter!=pModel.meshes_.end();meshIter++)
//        {
//          Mesh3D &pMesh=*meshIter;
//          for(int i=0;i<pMesh.vertices_.size();i++)
//          {
//            fprintf(myfile,"%f %f %f \n",pMesh.vertices_[i].x,pMesh.vertices_[i].y,pMesh.vertices_[i].z);
//          }//end for
//        }//for
//        count++;
//      }//end for

//      int lengthPolyList=4*iPolys;
//      fprintf(myfile,"POLYGONS %i %i \n",iPolys,lengthPolyList);
//      vertsIter = vVerts.begin();
//      for(modelIter = pModels.begin();modelIter!=pModels.end();modelIter++)
//      {
//        Model3D &pModel = *modelIter;
//        int ivertsModel = *vertsIter;
//        for(meshIter=pModel.meshes_.begin();meshIter!=pModel.meshes_.end();meshIter++)
//        {
//          Mesh3D &pMesh=*meshIter;
//          for(int i=0;i<pMesh.numFaces_;i++)
//          {
//            TriFace &tri = pMesh.faces_[i];
//            fprintf(myfile,"3 %i %i %i \n",tri[0]+ioffset, tri[1]+ioffset, tri[2]+ioffset);
//          }//end for faces
//          ioffset+=pMesh.numVerts_;
//        }//for submeshes
//        vertsIter++;
//      }//for models

//      fprintf(myfile,"POINT_DATA %i \n",iVerts);
//      fprintf(myfile,"SCALARS dummy double 1 \n");
//      fprintf(myfile,"LOOKUP_TABLE default\n");
//      int modelid=0;
//      for(modelIter = pModels.begin();modelIter!=pModels.end();modelIter++)
//      {
//        Model3D &pModel = *modelIter;
//        for(meshIter=pModel.meshes_.begin();meshIter!=pModel.meshes_.end();meshIter++)
//        {
//          Mesh3D &pMesh=*meshIter;
//          for(int i=0;i<pMesh.vertices_.size();i++)
//          {
//            fprintf(myfile,"%f\n",(pMesh.vertices_[0]-pMesh.vertices_[i]).mag());
//          }//end for
//        }//for
//        modelid++;
//      }//end for

//      fclose (myfile);
    }

  };
}

using namespace i3d;

int main()
{

  //----------------------------------------------
  ODE_Test myApp;

  // init application
  myApp.init("start/sampleRigidBody.xml");

  // run simulation
  myApp.run();

  return 0;
}
