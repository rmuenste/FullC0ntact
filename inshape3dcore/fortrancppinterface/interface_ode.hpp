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
 * Start the collision pipeline for the ode backend 
 * @param p Parallel rigid body data format
 */
template<> void startcollisionpipeline<backendODE>()
{
  //start the collision pipeline
  collPipeline_.startPipeline();
}

/** 
 * Interface function to get the hydrodynamic forces
 * from the fluid solver. 
 * @brief Get the hydrodynamic forces from the fluid solver
 */
template<> void velocityupdate<backendODE>()
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
void update_particle_state<backendODE>
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
template<>
void isinelementid<backendODE>(double *dx, double *dy, double *dz, int *iID, int *isin)
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

}//end isinelement
