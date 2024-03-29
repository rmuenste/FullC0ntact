#include <iomanip>
// simulation loop
void simulationLoop (int istep)
{
  dSpaceCollide (space,0,&nearCallback);

  dWorldQuickStep (world, myTimeControl.GetDeltaT()); // 100 Hz

  dJointGroupEmpty (contactgroup);

  //printf("Time: %f |Step: %d |\n",simTime, istep);
  //simTime += dt;
}

/** 
 *
 * Wrapper for VTK output for the ode backend 
 * @param iout integer pointer to the timestamp of the output file
 **/
template<> void write_rigid_bodies<i3d::BackEnd::backendODE>(int *iout)
{

  using namespace std;

  int istep = *iout;

  std::ostringstream sName;

  sName << "_vtk/model." << std::setw(5) << std::setfill('0') << istep << ".vtk";
  std::string strFileName(sName.str());

  CVtkWriter writer;

  writer.WriteODE2VTK(myWorld.bodies_, strFileName.c_str());

}

/** 
 *
 * Start the collision pipeline for the ode backend 
 * @param p Parallel rigid body data format
 */
template<> void startcollisionpipeline<i3d::BackEnd::backendODE>()
{
  //start the collision pipeline
  myPipeline.startPipeline();
}

/** 
 * Interface function to get the hydrodynamic forces
 * from the fluid solver. 
 * @brief Get the hydrodynamic forces from the fluid solver
 */
template<> void velocityupdate<i3d::BackEnd::backendODE>()
{

  std::vector<double> ForceX(myWorld.rigidBodies_.size());
  std::vector<double> ForceY(myWorld.rigidBodies_.size());
  std::vector<double> ForceZ(myWorld.rigidBodies_.size());

  std::vector<double> TorqueX(myWorld.rigidBodies_.size());
  std::vector<double> TorqueY(myWorld.rigidBodies_.size());
  std::vector<double> TorqueZ(myWorld.rigidBodies_.size());

  //get the forces from the cfd-solver
  communicateforce(ForceX.data(),ForceY.data(),ForceZ.data(),
      TorqueX.data(),TorqueY.data(),TorqueZ.data());

  std::vector<VECTOR3> vForce;
  std::vector<VECTOR3> vTorque;  

  for(int count = 0; count < myWorld.rigidBodies_.size(); ++count)
  {
    vForce.push_back(Vec3(ForceX[count],ForceY[count],ForceZ[count]));
    vTorque.push_back(Vec3(TorqueX[count],TorqueY[count],TorqueZ[count]));
    RigidBody *body = myWorld.rigidBodies_[count];

    if(body->shapeId_ == RigidBody::BOUNDARYBOX)
      continue;

    BodyODE &b = myWorld.bodies_[body->odeIndex_-1];
    Vec3 f;
    f.x = 0.5448 * (body->force_.x + ForceX[count]);
    f.y = 0.5448 * (body->force_.y + ForceY[count]);
    f.z = 0.5448 * (body->force_.z + ForceZ[count]);

    dBodyAddForce(b._bodyId, f.x,
        f.y,
        f.z);

    dBodyAddTorque(b._bodyId, TorqueX[count],
        TorqueY[count],
        TorqueZ[count]);

    if(myWorld.parInfo_.getId()==1)
    {
      //RigidBody *body = myWorld.rigidBodies_[id];
      //std::cout << "> Force update for ode backend: " << f  << std::endl;
    }

  }

  if(myWorld.parInfo_.getId()==1)
  {
    //const dReal *bforce = dBodyGetForce(b._bodyId);
    //int id = 0;
    //RigidBody *body = myWorld.rigidBodies_[id];
    //std::cout << "> Velocity update for default backend " << std::endl;
    //std::cout << "> HydroForceLin: " << maxForce << " (pg*micrometer)/s^2 " <<std::endl; 
    //std::cout << "> HydroForceLin: " << myWorld.rigidBodies_[0]->force_  ; 
    //    std::cout << "> Force max index: " << imax <<std::endl; 
    //std::cout << "> Force end2: " << vForce[99].z << " (pg*micrometer)/s^2 " <<std::endl; 
  }

}

/** 
 * This function handles a request from the fluid solver
 * to update the rigid body state after the RB-solver has
 * done a step.
 * @brief Handles a request for particle state update 
 */
template <>
  void update_particle_state<i3d::BackEnd::backendODE>
(double *px, double *py, double *pz,
 double *vx, double *vy, double *vz,
 double *ax, double *ay, double *az,
 double *avx, double *avy, double *avz,
 int *iid
 )
{

  int id = *iid;

  RigidBody *body = myWorld.rigidBodies_[id];

  BodyODE &b = myWorld.bodies_[body->odeIndex_-1];

  const double *pos = dBodyGetPosition(b._bodyId);

  body->com_.x = pos[0];
  body->com_.y = pos[1];
  body->com_.z = pos[2];

  //  std::cout << std::setprecision(9);
  //  std::cout << body->com_;

  const double *vel = dBodyGetLinearVel(b._bodyId);

  body->velocity_.x = vel[0];
  body->velocity_.y = vel[1];
  body->velocity_.z = vel[2];

  const double *avel = dBodyGetAngularVel(b._bodyId);

  body->setAngVel(Vec3(avel[0],
        avel[1],
        avel[2]));

  const double *quat = dBodyGetQuaternion(b._bodyId);

  Quaternionr q(quat[1],quat[2],quat[3],quat[0]);

  Vec3 euler = q.convertToEuler();

  *ax = euler.x; 
  *ay = euler.y; 
  *az = euler.z; 

  const dReal *SPos = dBodyGetPosition(b._bodyId);
  const dReal *SRot = dBodyGetRotation(b._bodyId);
  Real spos[3] = {SPos[0], SPos[1], SPos[2]};
  Real srot[12] = { SRot[0], SRot[1], SRot[2], 
    SRot[3], SRot[4], SRot[5], 
    SRot[6], SRot[7], SRot[8], 
    SRot[9], SRot[10], SRot[11] };


  Real entries[9] = { SRot[0], SRot[1], SRot[2], /* */ 
    SRot[4], SRot[5], SRot[6], /* */ 
    SRot[8], SRot[9], SRot[10] };


  MATRIX3X3 transform(entries);

  body->setTransformationMatrix(transform);
  body->setQuaternion(q);

  getpos(px,py,pz,iid);
  getvel(vx,vy,vz,iid);
  getangvel(avx,avy,avz,iid);

}

/**
 * This function is a wrapper for the point classification routines
 * which computes whether or not a point is inside the geometry with index iID
 * 
 * @brief Handles a request for a point classification query
 */
  template<>
void isinelementid<i3d::BackEnd::backendODE>(double *dx, double *dy, double *dz, int *iID, int *isin)
{

  int id = *iID;
  Real x = *dx;
  Real y = *dy;
  Real z = *dz;
  Vector3<Real> vec(x, y, z);
  int in = 0;

  RigidBody *body = myWorld.rigidBodies_[id];


  BodyODE &b = myWorld.bodies_[body->odeIndex_ - 1];

  const double *pos = dBodyGetPosition(b._bodyId);

  body->com_.x = pos[0];
  body->com_.y = pos[1];
  body->com_.z = pos[2];

  //  if(myWorld.parInfo_.getId()==1)
  //  {
  //    std::cout << ">" << body->com_; 
  //    std::cout << "> " << body->odeIndex_ << std::endl; 
  //  }

  if (body->shapeId_ == RigidBody::MESH)
  {
    //check if inside, if so then leave the function
    if (body->isInBody(vec))
    {
      in = 1;
    }
  }
#ifdef WITH_CGAL
  else if (body->shapeId_ == RigidBody::CGALMESH)
  {
    // Generate a direction vector for the ray
    Vec dir = random_vector();

    Vec3 vDir(dir.x(), dir.y(), dir.z());

    //check if inside, if so then leave the function
    if (body->isInBody(vec, vDir))
    {
      in = 1;
    }
  }
#endif 
  else
  {
    //check if inside, if so then leave the function
    if (body->isInBody(vec))
    {
      in = 1;
    }
  }

  *isin = in;

}//end isinelement

/**
 * This function is a wrapper for the distance calculation
 * which computes the minimum distance to the geometry with index iID
 *
 * @brief Handles a request for a distance query
 */
  template <>
void getClosestPointid<i3d::BackEnd::backendODE>(double *dx, double *dy, double *dz,
    double *px, double *py, double *pz,
    double *dist, int *iid)
{

}

/**
 * This function is a wrapper for the distance calculation
 * which computes the minimum distance to the geometry with index iID
 *
 * @brief Handles a request for a distance query
 */
  template<>
void getdistanceid<i3d::BackEnd::backendODE>(double *dx,double *dy,double *dz, double *dist, int *iid)
{
  Real x,y,z;
  x=*dx;
  y=*dy;
  z=*dz;
  double ddist;
  int id = *iid;
  Vector3<Real> vec(x,y,z);
  RigidBody *pBody = myWorld.rigidBodies_[id];

  BodyODE &b = myWorld.bodies_[pBody->odeIndex_-1];

  const double *pos = dBodyGetPosition(b._bodyId);

  pBody->com_.x = pos[0];
  pBody->com_.y = pos[1];
  pBody->com_.z = pos[2];

  if(pBody->shapeId_ == RigidBody::MESH)
  {
    ddist = 0;
    *dist = ddist;
    std::cout << "Developer Warning: Check if the default distance module is working correctly." << std::endl;
    std::exit(EXIT_FAILURE);
    return;
    MeshObject<Real> *pMeshObjectOrig = dynamic_cast< MeshObject<Real> *>(pBody->shape_);
    CDistanceMeshPoint<Real> distMeshPoint(&pMeshObjectOrig->getBvhTree(),vec);
    ddist = distMeshPoint.ComputeDistance();
    *dist=ddist;
  }
  else if (pBody->shapeId_ == RigidBody::CGALMESH)
  {
    ddist = pBody->getMinimumDistance(vec);
    *dist=ddist;
  }
  else if(pBody->shapeId_ == RigidBody::CYLINDER)
  {
    VECTOR3 vLocal = vec - pBody->com_;
    MATRIX3X3 trans = pBody->getTransformationMatrix();
    trans.TransposeMatrix();
    vLocal = trans * vLocal ;    

    Cylinder<Real> *cylinder = dynamic_cast< Cylinder<Real> *>(pBody->shape_);
    CDistancePointCylinder<Real> distCylMesh(vLocal,*cylinder);
    ddist = distCylMesh.ComputeDistance();
    *dist=ddist;
  }
  else if(pBody->shapeId_ == RigidBody::SPHERE)
  {
    VECTOR3 vLocal = vec - pBody->com_;
    Real ddd = vLocal.mag();

    Sphere<Real> *sphere = dynamic_cast< Sphere<Real> *>(pBody->shape_);
    ddd -= sphere->getRadius();

    *dist=ddd;
  }
  else
  {

  }

}//end getdistance

template<>
void write_sol_rb<i3d::BackEnd::backendODE>(int iout)
{

  if(myWorld.parInfo_.getId() == 1)
  {

    std::string folder("_sol_rb");

    if(!fs::exists(folder))
    {
      fs::create_directory(folder);
    }

    folder.append("/");
    folder.append(std::to_string(iout));

    if(!fs::exists(folder))
    {
      fs::create_directory(folder);
    }

    nlohmann::json array_explicit = nlohmann::json::array();

    std::ostringstream nameRigidBodies;
    nameRigidBodies << folder << "/rb.dmp";

    std::string n(nameRigidBodies.str());

    std::cout << "Writing ODE dmp file: " << n << std::endl;

    // Function: istream &read(char *buf, streamsize num)
    // Read in <num> chars from the invoking stream
    // into the buffer <buf>
    ofstream out(n, ios::out);
    
    if(!out)
    {
      cout << "Cannot open file: "<< n << endl;
      std::exit(EXIT_FAILURE);
    }

    for (BodyODE &b : myWorld.bodies_) {

      RigidBody &body = *myWorld.rigidBodies_[b._index];

      std::string myType;
      
      if (body.shapeId_ == RigidBody::SPHERE)
      {
        myType = std::string("Sphere");

        Real rad = Real(dGeomSphereGetRadius(b._geomId));
        Real dim = 2. * rad;

        Quaternionr quat = body.getQuaternion();

        Vec3 eulerAngles = quat.convertToEuler();
        
        const double *avel = dBodyGetAngularVel(b._bodyId);

        nlohmann::json j2 = {
          {"Type", myType},
          {"IsDynamic", "1"},
          {"Pos", {body.com_.x, body.com_.y, body.com_.z}},
          {"Rot", {quat.x, quat.y, quat.z}},
          {"Vel", {body.velocity_.x, body.velocity_.y, body.velocity_.z}},
          {"Norm", {1, 0, 0}},
          {"Dim", {dim, dim, dim}},
          {"AngVel", {avel[0], avel[1], avel[2]}}
        };

        array_explicit.push_back(j2);

      }

    }

//    for (auto &b : myWorld.boundaryGeometries_) {
//
//      std::string myType = std::string("Plane");
//
//      dReal normal[4];
//
//      dGeomPlaneGetParams(b._geomId, normal);
//
//      Vec3 pos(normal[0], normal[1], normal[2]);
//
//      pos = pos * normal[3];
//
//      nlohmann::json j2 = {
//        {"Type", myType},
//        {"IsDynamic", "1"},
//        {"Pos", {pos.x, pos.y, pos.z}},
//        {"Rot", {0, 0, 0}},
//        {"Vel", {0, 0, 0}},
//        {"Norm", {normal[0], normal[1], normal[2]}},
//        {"Dim", {0.0, 0.0, 0.0}},
//        {"AngVel", {0, 0, 0}}
//      };
//
//      array_explicit.push_back(j2);
//
//    }

    out << std::setw(4) << array_explicit << std::endl;

    out.close();

  }

}

/**
 * This function is a wrapper for the point projection
 * which computes the closest point to the geometry with index iID
 *
 * @brief Handles a request for a distance query
 */
  template <>
void projectOnBoundaryid<i3d::BackEnd::backendODE>(double *dx, double *dy, double *dz,
    double *px, double *py, double *pz,
    double *dist, int *iid)
{

  Real x, y, z;
  x = *dx;
  y = *dy;
  z = *dz;
  int id = *iid;
  Vector3<Real> vec(x, y, z);

#ifdef WITH_CGAL

  Vec3 p = myWorld.bndry_.boundaryShapes_[id]->projectPoint(vec);

  double d = 1.0;
  *dist = d;
  *px = p.x;
  *py = p.y;
  *pz = p.z;

#else

  *dist = 0.0;
  *px = 0.0;
  *py = 0.0;
  *pz = 0.0;

#endif

}
