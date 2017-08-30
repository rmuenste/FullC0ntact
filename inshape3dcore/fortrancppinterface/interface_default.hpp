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
