
//-------------------------------------------------------------------------------------------------------

void getforce(double* dx, double* dy, double* dz, int* iID)
{
  int i = *iID;
  Vector3<double> force(myWorld.rigidBodies_[i]->force_.x,
                         myWorld.rigidBodies_[i]->force_.y,
                         myWorld.rigidBodies_[i]->force_.z);
  *dx=force.x;
  *dy=force.y;
  *dz=force.z;
}

//-------------------------------------------------------------------------------------------------------

void gettorque(double* dx, double* dy, double* dz, int* iID)
{
  int i = *iID; 
  Vector3<double> torque(myWorld.rigidBodies_[i]->torque_.x,
                          myWorld.rigidBodies_[i]->torque_.y,
                          myWorld.rigidBodies_[i]->torque_.z);
  *dx=torque.x;
  *dy=torque.y;
  *dz=torque.z;
}
//-------------------------------------------------------------------------------------------------------

extern "C" void setposition(double *dx,double *dy,double *dz)
{
  Real x = *dx;
  Real y = *dy;
  Real z = *dz;
  
  Vector3<Real> vNewPos(x,y,z);

  Model.meshes_[0].moveToPosition(vNewPos);
}

//-------------------------------------------------------------------------------------------------------

extern "C" void setrotation(double *dx,double *dy,double *dz)
{
  Real x = *dx;
  Real y = *dy;
  Real z = *dz;
  

  Vector3<Real> vNewRot(x,y,z);

}

//-------------------------------------------------------------------------------------------------

void setelement(int* iel, int* iID)
{
  int i = *iID;
  myWorld.rigidBodies_[i]->element_ = *iel;
  myWorld.rigidBodies_[i]->elementsPrev_ = 1;
}

//-------------------------------------------------------------------------------------------------

void getelement(int* iel, int* iID)
{
  int i = *iID;
  *iel = myWorld.rigidBodies_[i]->element_;
}

//-------------------------------------------------------------------------------------------------

extern "C" void setpositionid(double *dx,double *dy,double *dz,int *iID)
{
  Real x = *dx;
  Real y = *dy;
  Real z = *dz;
  
  int id = *iID;
  Vector3<Real> vNewPos(x,y,z);
  myWorld.rigidBodies_[id]->translateTo(vNewPos);
}

//-------------------------------------------------------------------------------------------------

extern "C" void setrotationid(double *dx,double *dy,double *dz,int *iID)
{
  Real x = *dx;
  Real y = *dy;
  Real z = *dz; 
  
  int id = *iID;
  Vector3<Real> vNewRot(x,y,z);
  myWorld.rigidBodies_[id]->angle_ = vNewRot;
}

//-------------------------------------------------------------------------------------------------

extern "C" void setvelocityid(double *dx,double *dy,double *dz,int *iID)
{
  Real x = *dx;
  Real y = *dy;
  Real z = *dz; 
  
  int id = *iID;  
  Vector3<Real> vNewVel(x,y,z);
  myWorld.rigidBodies_[id]->velocity_ = vNewVel;
}

//-------------------------------------------------------------------------------------------------

extern "C" void setangvelid(double *dangvelx,double *dangvely,double *dangvelz,int *iid)
{
  i3d::Real x = *dangvelx;
  i3d::Real y = *dangvely;
  i3d::Real z = *dangvelz;
  
  int id = *iid;  
  Vector3<i3d::Real> vNewAngVel(x,y,z);
  myWorld.rigidBodies_[id]->getAngVel() = vNewAngVel;
}

//-------------------------------------------------------------------------------------------------

extern "C" void setforce(double *dforcex,double *dforcey,double *dforcez,int *iid)
{
  i3d::Real x = *dforcex;
  i3d::Real y = *dforcey;
  i3d::Real z = *dforcez;
  
  int id = *iid;  
  Vector3<i3d::Real> vNewForce(x,y,z);
  myWorld.rigidBodies_[id]->force_ = vNewForce;
}

//-------------------------------------------------------------------------------------------------

extern "C" void settorque(double *dtorquex,double *dtorquey,double *dtorquez,int *iid)
{
  i3d::Real x = *dtorquex;
  i3d::Real y = *dtorquey;
  i3d::Real z = *dtorquez;
  
  int id = *iid;  
  Vector3<i3d::Real> vNewTorque(x,y,z);
  myWorld.rigidBodies_[id]->torque_ = vNewTorque;
}

//-------------------------------------------------------------------------------------------------

extern "C" void getresponse(double *dux,double *duy,double *duz,
                            double *dcorrx,double *dcorry,double *dcorrz,
                            double *domx,double *domy,double *domz,int *iID)
{
  
}

//-------------------------------------------------------------------------------------------------

extern "C" void getpos(double *dx,double *dy,double *dz,int *iID)
{
  int i = *iID;
  Vector3<double> pos(myWorld.rigidBodies_[i]->com_.x,myWorld.rigidBodies_[i]->com_.y,myWorld.rigidBodies_[i]->com_.z);
  *dx=pos.x;
  *dy=pos.y;
  *dz=pos.z;
}

//-------------------------------------------------------------------------------------------------

extern "C" void getvel(double *dx,double *dy,double *dz,int *iID)
{
  int i = *iID; 
  Vector3<double> vel(myWorld.rigidBodies_[i]->velocity_.x,myWorld.rigidBodies_[i]->velocity_.y,myWorld.rigidBodies_[i]->velocity_.z);
  *dx=vel.x;
  *dy=vel.y;
  *dz=vel.z;
}

extern "C" void getsoftbodyvel(double *dx,double *dy,double *dz,
                               double *vx,double *vy,double *vz,
                               double *t)
{

  Real timens = *t; 
  Vec3 p(*dx,*dy,*dz);
  Vec3 v = bull.getVelocity(p,timens);
  *vx = v.x;
  *vy = v.y;
  *vz = v.z;
}

extern "C" void softbodyinternal(double *time)
{
  Real t = *time;
  bull.internalForce(t);
}

//-------------------------------------------------------------------------------------------------

extern "C" void getnumparticles(int *nParts)
{
  
  *nParts=myWorld.rigidBodies_.size();
}

extern "C" void getradius(double *drad, int *iid)
{
  int i = *iid;
  AABB3r box = myWorld.rigidBodies_[i]->shape_->getAABB();
  double rad  = box.extents_[0];
  *drad = rad;
}

//-------------------------------------------------------------------------------------------------

extern "C" void getangle(double *dangx,double *dangy,double *dangz,int *iid)
{
  int i = *iid; 

  Quaternionr q0 = myWorld.rigidBodies_[i]->getQuaternion();

  Vector3<double> angle = q0.convertToEuler();
                       
  *dangx=angle.x;
  *dangy=angle.y;
  *dangz=angle.z;
}

extern "C" void getangvel(double *dangvelx,double *dangvely,double *dangvelz,int *iid)
{
  int i = *iid;
  Vector3<double> angvel(myWorld.rigidBodies_[i]->getAngVel().x,
                         myWorld.rigidBodies_[i]->getAngVel().y,
                         myWorld.rigidBodies_[i]->getAngVel().z);

  *dangvelx=angvel.x;
  *dangvely=angvel.y;
  *dangvelz=angvel.z;
}

//-------------------------------------------------------------------------------------------------

extern "C" void getdensity(double *ddens, int *iid)
{
  int i = *iid;
  *ddens=myWorld.rigidBodies_[i]->density_;
}

//-------------------------------------------------------------------------------------------------

extern "C" void settimestep(double *dTime)
{
  double dT = *dTime;
  
  //set the timestep
  myTimeControl.SetDeltaT(dT);
  myTimeControl.SetPreferredTimeStep(dT);
  //myPipeline.m_pTimeControl->SetDeltaT(dT);
}

//-------------------------------------------------------------------------------------------------

extern "C" void settime(double *dTime)
{
  double dT = *dTime;
  
  //set the timestep
  myTimeControl.SetTime(dT);
}

void setoutputidx(char startFrom[60])
{
  std::cout << startFrom  << std::endl;
  std::string startIdx(startFrom);

  solIdx = std::stoi(startIdx);
}
