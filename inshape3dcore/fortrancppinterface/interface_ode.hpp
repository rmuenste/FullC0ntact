
// this is called by dSpaceCollide when two objects in space are
// potentially colliding.
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
  simulationLoop(mystep);
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
    BodyODE &b = myWorld.bodies_[body->odeIndex_];
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

  BodyODE &b = myWorld.bodies_[body->odeIndex_];

  const double *pos = dBodyGetPosition(b._bodyId);

  body->com_.x = pos[0];
  body->com_.y = pos[1];
  body->com_.z = pos[2];

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
