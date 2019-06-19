extern "C" void velocityupdate_soft()
{

  double *ForceX  = new double[myWorld.rigidBodies_.size()];
  double *ForceY  = new double[myWorld.rigidBodies_.size()];
  double *ForceZ  = new double[myWorld.rigidBodies_.size()];
  double *TorqueX = new double[myWorld.rigidBodies_.size()];
  double *TorqueY = new double[myWorld.rigidBodies_.size()];
  double *TorqueZ = new double[myWorld.rigidBodies_.size()];
  
  //get the forces from the cfd-solver
#ifdef WIN32
  //COMMUNICATEFORCE(ForceX,ForceY,ForceZ,TorqueX,TorqueY,TorqueZ);
#else
#ifdef FEATFLOWLIB
  communicateforce_(ForceX,ForceY,ForceZ,TorqueX,TorqueY,TorqueZ);
#endif
#endif
  
  std::vector<Vec3> vForce;
  std::vector<Vec3> vTorque;  
  
  std::vector<RigidBody*>::iterator vIter;  
  int count = 0;

  for(vIter=myWorld.rigidBodies_.begin();vIter!=myWorld.rigidBodies_.end();vIter++,count++)
  {
    RigidBody *body = *vIter;
    vForce.push_back(Vec3(ForceX[count],ForceY[count],ForceZ[count]));
    vTorque.push_back(Vec3(TorqueX[count],TorqueY[count],TorqueZ[count]));

    if(body->shapeId_ == RigidBody::BOUNDARYBOX)
      continue;

    body->force_ = vForce[count];
  }

  Vec3 maxForce(0,0,0);
  int imax = 0;
//  for (int i = 0; i < vForce.size(); ++i)
//  {
//    if(maxForce.mag() < vForce[i].mag())
//    {
//      maxForce = vForce[i];
//      imax = i;
//    } 
//  }

  Real scale = 1.0; //1e-5;
  //Real forcez = scale * myWorld.rigidBodies_[49]->force_.z;

  for(int j(0); j < myWorld.rigidBodies_.size(); ++j)
  {
    myWorld.rigidBodies_[j]->forceResting_ = scale * myWorld.rigidBodies_[j]->force_;
  }

//  if(myWorld.parInfo_.getId()==1)
//  {
//    //std::cout << "> count: " << count << std::endl;
//    //std::cout << "> Force max: " << maxForce << " (pg*micrometer)/s^2 " <<std::endl; 
//    for(int j(0); j < myWorld.rigidBodies_.size()-1; ++j)
//    {
//      std::cout << " > Force: " << myWorld.rigidBodies_[j]->force_;
//    }
//  }

  // clean up
  delete[] ForceX;
  delete[] ForceY;
  delete[] ForceZ;
  delete[] TorqueX;
  delete[] TorqueY;
  delete[] TorqueZ;      
  
}

extern "C" void getsoftvel(double *x,double *y,double *z,
                           double *vx,double *vy,double *vz, int *ip)
{

  int id = *ip;
  id--;

  Vector3<Real> vec(*x,*y,*z);
  
  Vector3<Real> vel = softBody_->getVelocity(vec,id);

  *vx = vel.x;
  *vy = vel.y;
  *vz = vel.z;

}

extern "C" void getsoftcom(double *dx,double *dy,double *dz)
{
}

extern "C" void getsoftmass(double *dmass)
{
  *dmass = softBody_->massAll_;
}

extern "C" void stepsoftbody(double *fx,double *fy,double *fz,double *dt)
{

  Vec3 f(*fx,*fy,*fz);
  Real _dt = *dt;
  int id = myWorld.parInfo_.getId();
//  if(id == 1)
//  {
//  std::cout << termcolor::bold << termcolor::blue << 
//    termcolor::reset  << std::endl;
//
//    std::cout <<  termcolor::bold << 
//      termcolor::blue << "|----------------------------------------------------------------|" << 
//    termcolor::reset  << std::endl;
//
//
//    std::cout << "> Force fluid: " << myWorld.rigidBodies_[49]->force_.z //vForce[99].z 
//      << " (pg*micrometer)/s^2 " <<std::endl; 
//
//    std::cout << "> Time: " << simTime_ << "s | " << _dt << 
//                 "s |it: " << istep_<< std::endl;
//  }

  softBody_->step(simTime_, _dt, istep_);

  if(id == 1)
  {
//    //std::cout << "forcex: " << f.x <<std::endl; 
//    //std::cout << "velocity: " << softBody_.velocity_; 
//    //std::cout << "pos: " << softBody_.transform_.getOrigin(); 
//    std::cout << "> Velocity end: " << softBody_.u_[49].z << " micrometer/s " <<std::endl; 
//    std::cout << "> Step finished " << std::endl;
//
//    std::cout <<  termcolor::bold << 
//      termcolor::blue << "|----------------------------------------------------------------|" << 
//    termcolor::reset  << std::endl;
  }

  istep_++;
  simTime_ += _dt;

  for(int i(0); i < softBody_->geom_.vertices_.size(); ++i)
  {
    myWorld.rigidBodies_[i]->com_      = softBody_->geom_.vertices_[i];
    myWorld.rigidBodies_[i]->velocity_ = softBody_->u_[i];
  }

}

extern "C" void insidesoftbody(double *dx,double *dy,double *dz, int *iID, int *isin)
{

  int id = *iID;
  Real x = *dx;
  Real y = *dy;
  Real z = *dz;  
  Vector3<Real> vec(x,y,z);
  int in=0;

  int bullid=0;
  if(softBody_->isInBody(vec,bullid))
  {
    in = 1;
  }

  *isin=bullid;

}//end isinelement
