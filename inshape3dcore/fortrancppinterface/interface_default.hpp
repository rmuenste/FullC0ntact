/** 
 *
 * Start the collision pipeline for the default backend 
 * @param p Parallel rigid body data format
 */
template<> void startcollisionpipeline<backendDefault>()
{
  //start the collision pipeline
  myPipeline.startPipeline();
}

/** 
 * Interface function to get the hydrodynamic forces
 * from the fluid solver. 
 * @brief Get the hydrodynamic forces from the fluid solver
 */
template<> void velocityupdate<backendDefault>()
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

}

/** 
 * This function handles a request from the fluid solver
 * to update the rigid body state after the RB-solver has
 * done a step.
 * @brief Handles a request for particle state update 
 */
template <>
void update_particle_state<backendDefault>
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
* which computes whether or not a point is inside the geometry with index iID
*
* @brief Handles a request for a point classification query
*/
template <>
void isinelementid<backendDefault>(double *dx, double *dy, double *dz, int *iID, int *isin)
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
    Vector dir = random_vector();

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
template<>
void getdistanceid<backendDefault>(double *dx,double *dy,double *dz, double *dist, int *iid)
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
