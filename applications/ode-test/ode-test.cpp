#include <iostream>
#include <application.h>
#include <reader.h>
#include <paramline.h>

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

  static void nearCallback (void *data, dGeomID o1, dGeomID o2)
  {
    assert(o1);
    assert(o2);

    if (dGeomIsSpace(o1) || dGeomIsSpace(o2))
    {
        fprintf(stderr,"testing space %p %p\n", (void*)o1, (void*)o2);
      // colliding a space with something
      dSpaceCollide2(o1,o2,data,&nearCallback);
      // Note we do not want to test intersections within a space,
      // only between spaces.
      return;
    }

    const int N = 32;
    dContact contact[N];
    int n = dCollide (o1,o2,N,&(contact[0].geom),sizeof(dContact));
    if (n > 0) 
    {
      for (int i=0; i<n; i++) 
      {
        contact[i].surface.mode = 0;
        contact[i].surface.mu = 50.0; // was: dInfinity
        dJointID c = dJointCreateContact (world,contactgroup,&contact[i]);
        dJointAttach (c, dGeomGetBody(contact[i].geom.g1), dGeomGetBody(contact[i].geom.g2));
      }
    }
  }


  class ODE_Test : public Application {

  public:

    ODE_Test() : Application() {

    }

    void init(std::string fileName) {

      using namespace std;

      FileParserXML myReader;

      //Get the name of the mesh file from the
      //configuration data file.
      myReader.parseDataXML(this->dataFileParams_, fileName);

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

      cout<<"startType = "<<dataFileParams_.startType_<<endl; 
      cout<<"solution = "<<dataFileParams_.solutionFile_<<endl; 
      cout<<"nBodies = "<<dataFileParams_.bodies_<<endl;  
      cout<<"bodyInit = "<<dataFileParams_.bodyInit_<<endl; 
      cout<<"bodyFile = "<<dataFileParams_.bodyConfigurationFile_<<endl; 
      cout<<"defaultDensity = "<<dataFileParams_.defaultDensity_<<endl; 
      cout<<"defaultRadius = "<<dataFileParams_.defaultRadius_<<endl; 
      cout<<"gravity = "<<dataFileParams_.gravity_;  
      cout<<"totalTimesteps = "<<dataFileParams_.nTimesteps_<<endl;
      cout<<"lcpSolverIterations = "<<dataFileParams_.maxIterations_<<endl;
      cout<<"collPipelineIterations = "<<dataFileParams_.pipelineIterations_<<endl;
      
      if(dataFileParams_.hasExtents_)
      {
        cout << "domain extents = " << dataFileParams_.extents_[0] << " " << dataFileParams_.extents_[1] << " " << dataFileParams_.extents_[2] << " "
                                    << dataFileParams_.extents_[3] << " " << dataFileParams_.extents_[4] << " " << dataFileParams_.extents_[5] << endl;
      }
      
      if(dataFileParams_.bodies_ > 0)
      {
        cout<<"type = "<< dataFileParams_.rigidBodies_[0].shapeId_ <<endl; 
        cout<<"position = "<< dataFileParams_.rigidBodies_[0].com_ <<endl; 
        cout<<"velocity = "<< dataFileParams_.rigidBodies_[0].velocity_ <<endl; 
        cout<<"density = "<< dataFileParams_.rigidBodies_[0].density_ <<endl;
        cout<<"meshfile = "<< dataFileParams_.rigidBodies_[0].fileName_ <<endl;       
      }

    }

    void run()
    {
    }

    void outputVTK(int istep)
    {

      using namespace std;

      std::ostringstream sName;

      sName << "output/model." << std::setw(5) << std::setfill('0') << istep << ".vtk";
      std::string strFileName(sName.str());

      FILE * myfile = fopen(strFileName.c_str(),"w");

      if (myfile == nullptr) {
        cout<<"Error opening file: "<<strFileName<<endl;
        std::exit(EXIT_FAILURE);
      }

      //total number of vertices
      int iVerts=0;
      //total number of polygons
      int iPolys=0;
      //iterators for models and submeshes
      vector<Mesh3D>::iterator meshIter;
      vector<Model3D>::iterator modelIter;
      vector<RigidBody*>::iterator rIter;
      vector<Model3D> pModels;
      vector<int> vVerts;
      vector<int>::iterator vertsIter;
      int ioffset=0;

      for (auto &body : myWorld_.bodies_)
      {

        int gId = dGeomGetClass(body._geomId);

        if (gId == dSphereClass)
        {
          const dReal *SPos = dBodyGetPosition(body._bodyId);
          const dReal *SRot = dBodyGetRotation(body._bodyId);
          float spos[3] = {SPos[0], SPos[1], SPos[2]};
          float srot[12] = { SRot[0], SRot[1], SRot[2], SRot[3], SRot[4], SRot[5], SRot[6], SRot[7], SRot[8], SRot[9], SRot[10], SRot[11] };

          CTriangulator<Real, Sphere<Real> > triangulator;

          Real rad = Real(dGeomSphereGetRadius(body._geomId));

          Spherer Sphere(Vec3(0,0,0),rad);
          
          Model3D model_out=triangulator.Triangulate(&Sphere);

          double entries[9] = { SRot[0], SRot[1], SRot[2], /* */ SRot[4], SRot[5], SRot[6], /* */ SRot[8], SRot[9], SRot[10] };

          MATRIX3X3 transform(entries);
          model_out.meshes_[0].transform_ = transform;
          model_out.meshes_[0].com_ = Vec3(SPos[0], SPos[1], SPos[2]);
          model_out.meshes_[0].TransformModelWorld();
          pModels.push_back(model_out);
        }
        else if (gId == dPlaneClass)
        {
          dVector4 v;
          dGeomPlaneGetParams(body._geomId, v);
          Planer pl(v[0], v[1], v[2], v[3]);

          CTriangulator<Real, Plane<Real> > triangulator;
          Model3D model_out=triangulator.Triangulate(pl);
          pModels.push_back(model_out);
        }
        else if (gId == dBoxClass)
        {
          const dReal *SPos = dBodyGetPosition(body._bodyId);
          const dReal *SRot = dBodyGetRotation(body._bodyId);
          float spos[3] = {SPos[0], SPos[1], SPos[2]};
          float srot[12] = { SRot[0], SRot[1], SRot[2], SRot[3], SRot[4], SRot[5], SRot[6], SRot[7], SRot[8], SRot[9], SRot[10], SRot[11] };

          CTriangulator<Real, OBB3<Real> > triangulator;

          dVector3 dim;
          dGeomBoxGetLengths(body._geomId, dim);
          
          OBB3r box(Vec3(0, 0, 0), Vec3(1, 0, 0), Vec3(0, 1, 0), Vec3(0, 0, 1), 0.5 * dim[0], 0.5 * dim[1], 0.5 * dim[2]); 
          
          Model3D model_out=triangulator.Triangulate(box);

          double entries[9] = { SRot[0], SRot[1], SRot[2], /* */ SRot[4], SRot[5], SRot[6], /* */ SRot[8], SRot[9], SRot[10] };

          MATRIX3X3 transform(entries);
          model_out.meshes_[0].transform_ = transform;
          model_out.meshes_[0].com_ = Vec3(SPos[0], SPos[1], SPos[2]);
          model_out.meshes_[0].TransformModelWorld();

          pModels.push_back(model_out);
        }
        else if (gId == dCylinderClass)
        {
          const dReal *SPos = dBodyGetPosition(body._bodyId);
          const dReal *SRot = dBodyGetRotation(body._bodyId);
          float spos[3] = {SPos[0], SPos[1], SPos[2]};
          float srot[12] = { SRot[0], SRot[1], SRot[2], 
                             SRot[3], SRot[4], SRot[5], 
                             SRot[6], SRot[7], SRot[8], 
                             SRot[9], SRot[10], SRot[11] };

          CTriangulator<Real, Cylinder<Real> > triangulator;

          dReal radius, l;
          dGeomCylinderGetParams(body._geomId, &radius, &l);
          
          Cylinderr cyl(Vec3(0,0,0),
                        Vec3(0,0,1),
                        radius, 0.5 * l);

          
          Model3D model_out=triangulator.Triangulate(cyl);

          double entries[9] = { SRot[0], SRot[1], SRot[2], /* */ 
                                SRot[4], SRot[5], SRot[6], /* */ 
                                SRot[8], SRot[9], SRot[10] };

          MATRIX3X3 transform(entries);
          model_out.meshes_[0].transform_ = transform;
          model_out.meshes_[0].com_ = Vec3(SPos[0], SPos[1], SPos[2]);
          model_out.meshes_[0].TransformModelWorld();

          pModels.push_back(model_out);
        }
      }

      for(modelIter = pModels.begin();modelIter!=pModels.end();modelIter++)
      {
        Model3D &pModel = *modelIter;
        int ivertsModel=0;
        for(meshIter=pModel.meshes_.begin();meshIter!=pModel.meshes_.end();meshIter++)
        {
          Mesh3D &pMesh=*meshIter;
          iVerts+=pMesh.numVerts_;
          iPolys+=pMesh.numFaces_;
          ivertsModel+=pMesh.numVerts_;
        }
        vVerts.push_back(ivertsModel);
      }

      fprintf(myfile,"# vtk DataFile Version 2.0\n");
      fprintf(myfile,"Generated by InShape 2.x\n");
      fprintf(myfile,"ASCII\n");
      fprintf(myfile,"DATASET POLYDATA\n");
      fprintf(myfile,"POINTS %i double\n",iVerts);

      int count=0;
      //write the actual vertex data
      for(modelIter = pModels.begin();modelIter!=pModels.end();modelIter++)
      {
        Model3D &pModel = *modelIter;
        for(meshIter=pModel.meshes_.begin();meshIter!=pModel.meshes_.end();meshIter++)
        {
          Mesh3D &pMesh=*meshIter;
          for(int i=0;i<pMesh.vertices_.size();i++)
          {
            fprintf(myfile,"%f %f %f \n",pMesh.vertices_[i].x,pMesh.vertices_[i].y,pMesh.vertices_[i].z);
          }//end for
        }//for
        count++;
      }//end for

      int lengthPolyList=4*iPolys;
      fprintf(myfile,"POLYGONS %i %i \n",iPolys,lengthPolyList);
      vertsIter = vVerts.begin();
      for(modelIter = pModels.begin();modelIter!=pModels.end();modelIter++)
      {
        Model3D &pModel = *modelIter;
        int ivertsModel = *vertsIter;
        for(meshIter=pModel.meshes_.begin();meshIter!=pModel.meshes_.end();meshIter++)
        {
          Mesh3D &pMesh=*meshIter;
          for(int i=0;i<pMesh.numFaces_;i++)
          {
            TriFace &tri = pMesh.faces_[i];
            fprintf(myfile,"3 %i %i %i \n",tri[0]+ioffset, tri[1]+ioffset, tri[2]+ioffset);
          }//end for faces
          ioffset+=pMesh.numVerts_;
        }//for submeshes
        vertsIter++;
      }//for models

      fprintf(myfile,"POINT_DATA %i \n",iVerts);
      fprintf(myfile,"SCALARS dummy double 1 \n");
      fprintf(myfile,"LOOKUP_TABLE default\n");
      int modelid=0;
      for(modelIter = pModels.begin();modelIter!=pModels.end();modelIter++)
      {
        Model3D &pModel = *modelIter;
        for(meshIter=pModel.meshes_.begin();meshIter!=pModel.meshes_.end();meshIter++)
        {
          Mesh3D &pMesh=*meshIter;
          for(int i=0;i<pMesh.vertices_.size();i++)
          {
            fprintf(myfile,"%f\n",(pMesh.vertices_[0]-pMesh.vertices_[i]).mag());
          }//end for
        }//for
        modelid++;
      }//end for

      fclose (myfile);
    }

    // simulation loop
    void simulationLoop (int istep)
    {
      dSpaceCollide (space,0,&nearCallback);

      dWorldQuickStep (world, 0.01); // 100 Hz

      dJointGroupEmpty (contactgroup);

      outputVTK(istep);

      printf("Time: %f |Step: %d |\n",simTime, istep);
      simTime += dt;
    }

  };
}

using namespace i3d;

// called when a key pressed
static void command (int cmd)
{
  switch (cmd) 
  {
    case ' ':
      break;
  }
}


using json = nlohmann::json;

int main()
{

  //----------------------------------------------
  ODE_Test myApp;

//  myApp.init("start/sampleRigidBody.xml");

  //----------------------------------------------

  dMass m;

  // create world
  dInitODE2(0);
  world = dWorldCreate();
  space = dHashSpaceCreate (0);
  contactgroup = dJointGroupCreate (0);
  dWorldSetGravity (world,0,0,-9.8);
  dWorldSetQuickStepNumIterations (world, 32);

  dQuaternion q;

  //----------------------------------------------

  std::ifstream i("cube.json");
  json j;
  i >> j;
  std::cout << j << std::endl;

  for (int i(0); i < j.size(); ++i)
  {

    Vec3 p(j[i]["Pos"][0], j[i]["Pos"][1], j[i]["Pos"][2]);
    Vec3 d(j[i]["Dim"][0], j[i]["Dim"][1], j[i]["Dim"][2]);
    Vec3 q(j[i]["Rot"][0], j[i]["Rot"][1], j[i]["Rot"][2]);

    BodyODE b;

    if (j[i]["Type"] == "Sphere")
    {
      sphbody = dBodyCreate (world);

      b._bodyId = sphbody;

      dMassSetSphere (&m,1,d.y);
      dBodySetMass (b._bodyId,&m);

      sphgeom = dCreateSphere(0, 0.5 * d.y);
      b._geomId = sphgeom;

      dGeomSetBody (b._geomId,b._bodyId);

      dBodySetPosition (b._bodyId, p.x, p.y, p.z);
      dSpaceAdd (space, b._geomId);

      myApp.myWorld_.bodies_.push_back(b);
    }
    else if (j[i]["Type"] == "Plane")
    {
      dGeomID p = dCreatePlane (space,0,0,1, 0.0);

      b._geomId = p;
      b._bodyId = dBodyID(-10);
      myApp.myWorld_.bodies_.push_back(b);
    }
    else if (j[i]["Type"] == "Cube")
    {
      b._bodyId = dBodyCreate (world);

      dMatrix3 rMat;
      dRFromEulerAngles(rMat, q.x, q.y, q.z); 

      dBodySetRotation(b._bodyId, rMat); 

      dMassSetBox(&m, 1.0, d.x, d.y, d.z);

      dBodySetMass (b._bodyId,&m);

      b._geomId = dCreateBox(0, d.x, d.y, d.z);

      dGeomSetBody (b._geomId,b._bodyId);

      dBodySetPosition (b._bodyId, p.x , p.y, p.z);

      dSpaceAdd (space, b._geomId);

      myApp.myWorld_.bodies_.push_back(b);
    }
    else if (j[i]["Type"] == "Cylinder")
    {
      b._bodyId = dBodyCreate (world);

      dMatrix3 rMat;
      dRFromEulerAngles(rMat, q.x, q.y, q.z); 

      dBodySetRotation(b._bodyId, rMat); 

      Real rad = 0.5 * d.x;
      Real &l  = d.z;

      // compute mass for a cylinder with density 1.0
      dMassSetCylinder(&m, 1.0, 3, rad, l);

      // set the mass for the cylinder body
      dBodySetMass (b._bodyId,&m);

      b._geomId = dCreateCylinder(0, rad, l);

      // assign the geometry to the rigid body
      dGeomSetBody (b._geomId,b._bodyId);

      dBodySetPosition (b._bodyId, p.x , p.y, p.z);
      dSpaceAdd (space, b._geomId);

      myApp.myWorld_.bodies_.push_back(b);
    }

  }

  //----------------------------------------------

  // run simulation
  for (int i(0); i <= 300; ++i)
  {
    myApp.simulationLoop(i);
  }

  dJointGroupEmpty (contactgroup);
  dJointGroupDestroy (contactgroup);

  dGeomDestroy(sphgeom);

  //dGeomDestroy (cylgeom);

  dSpaceDestroy (space);
  dWorldDestroy (world);
  dCloseODE();

//  myApp.run();

  return 0;
}
