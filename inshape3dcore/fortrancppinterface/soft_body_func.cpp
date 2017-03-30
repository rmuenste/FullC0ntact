
extern "C" void getsoftvel(double *x,double *y,double *z,
                           double *vx,double *vy,double *vz, int *ip)
{

  int id = *ip;
  id--;

  Vector3<Real> vec(*x,*y,*z);
  
  Vector3<Real> vel = softBody_.getVelocity(vec,id);

  *vx = vel.x;
  *vy = vel.y;
  *vz = vel.z;

}

extern "C" void getsoftcom(double *dx,double *dy,double *dz)
{
  Vector3<Real> vec = softBody_.getCOM();
  *dx = vec.x;
  *dy = vec.y;
  *dz = vec.z;
}

extern "C" void getsoftmass(double *dmass)
{
  *dmass = softBody_.massAll_;
}

extern "C" void stepsoftbody(double *fx,double *fy,double *fz,double *dt)
{
  Vec3 f(*fx,*fy,*fz);
  Real _dt = *dt;
  softBody_.step(simTime_, _dt, istep_);
  int id = myWorld.parInfo_.getId();
  if(id == 1)
  {
    std::cout << "forcex: " << f.x <<std::endl; 
    std::cout << "velocity: " << softBody_.velocity_; 
    std::cout << "pos: " << softBody_.transform_.getOrigin(); 
  }
  istep_++;
  simTime_ += _dt;
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
  if(softBody_.isInBody(vec,bullid))
  {
    in = 1;
  }

  *isin=bullid;

}//end isinelement
