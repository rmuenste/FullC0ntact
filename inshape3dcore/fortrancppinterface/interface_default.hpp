/** 
 *
 * Start the collision pipeline for the default backend 
 * @param p Parallel rigid body data format
 */
template<> void startcollisionpipeline<i3d::BackEnd::backendDefault>()
{
  //start the collision pipeline
  myPipeline.startPipeline();
}

/** 
 *
 * Wrapper for VTK output for the defaul backend 
 * @param iout integer pointer to the timestamp of the output file
 **/
template<>
void write_rigid_bodies<i3d::BackEnd::backendDefault>(int *iout)
{

  using namespace std;

  int istep = *iout;

  std::ostringstream sName;

  sName << "_vtk/model." << std::setw(5) << std::setfill('0') << istep << ".vtk";
  std::string strFileName(sName.str());

  CVtkWriter writer;

  writer.WriteRigidBodies(myWorld.rigidBodies_,strFileName.c_str());

}

/** 
 * Interface function to get the hydrodynamic forces
 * from the fluid solver. 
 * @brief Get the hydrodynamic forces from the fluid solver
 */
template<> void velocityupdate<i3d::BackEnd::backendDefault>()
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

  std::vector<Vec3> vForce;
  std::vector<Vec3> vTorque;  

  for(int count = 0; count < myWorld.rigidBodies_.size(); ++count)
  {
    vForce.push_back(Vec3(ForceX[count],ForceY[count],ForceZ[count]));
    vTorque.push_back(Vec3(TorqueX[count],TorqueY[count],TorqueZ[count]));
  }


  if(myWorld.parInfo_.getId()==1)
  {
    //int id = 0;
    //RigidBody *body = myWorld.rigidBodies_[id];
    //std::cout << "> Velocity update for default backend " << std::endl;
    //    std::cout << "> Force max: " << maxForce << " (pg*micrometer)/s^2 " <<std::endl; 
    //    std::cout << "> Force max index: " << imax <<std::endl; 
    //std::cout << "> Force end2: " << vForce[99].z << " (pg*micrometer)/s^2 " <<std::endl; 
  }

  //calculate the forces in the current timestep by a semi-implicit scheme
  myPipeline.integrator_->updateForces(vForce,vTorque);

  if(myWorld.parInfo_.getId()==1)
  {

    std::cout << "====================" << std::endl;
    std::cout << "    Hydro-Force     " << std::endl;
    std::cout << "====================" << std::endl;

    std::cout << termcolor::bold << termcolor::green << myWorld.parInfo_.getId() <<  
                " > Hydro Force[microgram*mm/s^2]: " << vForce[0] << termcolor::reset;

    std::cout << termcolor::bold << termcolor::blue << myWorld.parInfo_.getId() <<  
                " > Hydro torque[microgram*mm^2/s^2]: " << vTorque[0] << termcolor::reset;

  }

}

/** 
 * This function handles a request from the fluid solver
 * to update the rigid body state after the RB-solver has
 * done a step.
 * @brief Handles a request for particle state update 
 */
template <>
void update_particle_state<i3d::BackEnd::backendDefault>
                          (double *px, double *py, double *pz,
                           double *vx, double *vy, double *vz,
                           double *ax, double *ay, double *az,
                           double *avx, double *avy, double *avz,
                           int *iid
                          )
{
  getpos(px,py,pz,iid);
  getvel(vx,vy,vz,iid);
  getangle(ax,ay,az,iid);
  getangvel(avx,avy,avz,iid);
}

/**
* This function is a wrapper for the point classification routines
* which computes wheter or not a point is inside the geometry with index iID
*
* @brief Handles a request for a point classification query
*/
template <>
void isinelementid<i3d::BackEnd::backendDefault>(double *dx, double *dy, double *dz, int *iID, int *isin)
{
  int id = *iID;
  Real x = *dx;
  Real y = *dy;
  Real z = *dz;
  Vector3<Real> vec(x, y, z);
  int in = 0;

  RigidBody *body = myWorld.rigidBodies_[id];

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

}

/**
* This function is a wrapper for the distance calculation
* which computes the minimum distance to the geometry with index iID
*
* @brief Handles a request for a distance query
*/
template <>
void getClosestPointid<i3d::BackEnd::backendDefault>(double *dx, double *dy, double *dz,
                                       double *px, double *py, double *pz,
                                       double *dist, int *iid)
{

  Real x, y, z;
  x = *dx;
  y = *dy;
  z = *dz;
  int id = *iid;
  Vector3<Real> vec(x, y, z);
  RigidBody *pBody = myWorld.rigidBodies_[id];

  if (pBody->shapeId_ == RigidBody::CGALMESH)
  {
     std::pair<Real, Vec3> res = pBody->getClosestPoint(vec);
    *dist = res.first;
    *px = res.second.x;
    *py = res.second.y;
    *pz = res.second.z;
  }
  else
  {
    *dist = 0.0;
    *px = 0.0;
    *py = 0.0;
    *pz = 0.0;
  }

}

/**
* This function is a wrapper for the point projection
* which computes the closest point to the geometry with index iID
*
* @brief Handles a request for a distance query
*/
template <>
void projectOnBoundaryid<i3d::BackEnd::backendDefault>(double *dx, double *dy, double *dz,
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
  *px = x;
  *py = y;
  *pz = z;

#endif

}

/**
* This function is a wrapper for the distance calculation
* which computes the minimum distance to the geometry with index iID
*
* @brief Handles a request for a distance query
*/
template<>
void getdistanceid<i3d::BackEnd::backendDefault>(double *dx,double *dy,double *dz, double *dist, int *iid)
{
  Real x,y,z;
  x=*dx;
  y=*dy;
  z=*dz;
  double ddist;
  int id = *iid;
  Vector3<Real> vec(x,y,z);
  RigidBody *pBody = myWorld.rigidBodies_[id];

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
