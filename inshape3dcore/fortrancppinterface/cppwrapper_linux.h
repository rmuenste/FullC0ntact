#ifndef CPPWRAPPER_LINUX_H_PHBGRGXW
#define CPPWRAPPER_LINUX_H_PHBGRGXW

#ifdef FEATFLOWLIB

extern "C" void communicateforce_(double *fx, double *fy, double *fz, double *tx, double *ty, double *tz);

extern "C" 
void communicateforce(double *fx, double *fy, double *fz, double *tx, double *ty, double *tz)
{
  communicateforce_(fx,fy,fz,tx,ty,tz);
}

#ifdef OPTIC_FORCES
extern "C" void get_optic_forces_()
{
  get_optic_forces();
}
#endif

#ifdef WITH_ODE
extern "C" void init_fc_ode_(int *iid)
{
  init_fc_ode(iid);
}


extern "C" 
void update_particle_state_(double *px, double *py, double *pz,
                            double *vx, double *vy, double *vz,
                            double *ax, double *ay, double *az,
                            double *avx, double *avy, double *avz,
                            int *iid
                           )
{

   update_particle_state<backendODE>(px, py, pz,
                                     vx, vy, vz,
                                     ax, ay, az,
                                     avx,avy,avz,
                                     iid);

}

#else

extern "C" 
void update_particle_state_(double *px, double *py, double *pz,
                            double *vx, double *vy, double *vz,
                            double *ax, double *ay, double *az,
                            double *avx, double *avy, double *avz,
                            int *iid
                           )
{

   update_particle_state<backendDefault>(px, py, pz,
                                         vx, vy, vz,
                                         ax, ay, az,
                                         avx,avy,avz,
                                         iid);

}

#endif

extern "C" void browniandisplacement_()
{
  brownianDisplacement();
}

extern "C" void testpointer_(double *u, double *v, double *w, double **p)
{

  std::cout << "U address: " << u << std::endl;
  std::cout << "V address: " << v << std::endl;
  std::cout << "W address: " << w << std::endl;
   
  std::cout << "U Double address: " << *p << std::endl;
  std::cout << "V Double address: " << *(p + 1) << std::endl;
  std::cout << "W Double address: " << p[8] << std::endl;

}

extern "C" void getrandfloat_(double point[])
{
  getrandfloat(point);
}

extern "C" void gettiming_(double *time)
{
  gettiming(time);
}

extern "C" void starttiming_()
{
  starttiming();
}

extern "C" void getsoftvel_(double *x,double *y,double *z,
    double *vx,double *vy,double *vz,int *ip)
{
  getsoftvel(x,y,z,
      vx,vy,vz,ip);
}

extern "C" void bndryproj_(double *dx,double *dy,double *dz, double *dxx, double *dyy, double *dzz)
{
  bndryproj(dx,dy,dz,dxx,dyy,dzz);
}

extern "C" void bndryprojid_(double *dx,double *dy,double *dz, double *dxx, double *dyy, double *dzz, int *id)
{
  bndryprojid(dx,dy,dz,dxx,dyy,dzz,id);
}

extern "C" void setdomainbox_(double vmin[3], double vmax[3])
{
  setdomainbox(vmin,vmax);
}

extern "C" void inituniformgrid_(double vmin[3], double vmax[3], double element[][3])
{
  inituniformgrid(vmin,vmax,element);
}

extern "C" void initbdryparam_()
{
  initbdryparam();
}

extern "C" void addbdryparam_(int *iBnds, int *itype, char *name, int length)
{
  addbdryparam(iBnds,itype,name,length);
}

extern "C" void addelement2list_(int *iel, int *ibody)
{
  addelement2list(iel,ibody);
}

extern "C" void updateelementsprev_(int *ibody)
{
  updateelementsprev(ibody);
}

extern "C" void uniformgridinsert_(int *iel, double center[3])
{
  uniformgridinsert(iel,center);
}

extern "C" void ug_insertelement_(int *iel, double center[3], double *size)
{
  ug_insertelement(iel,center,size);
}

extern "C" void ug_resetuniformgrid_()
{
  ug_resetuniformgrid();
}

extern "C" void ug_querystatus_()
{
  ug_querystatus();
}

extern "C" void ug_pointquery_(double center[3], int *iiel)
{
  ug_pointquery(center,iiel);
}

extern "C" void ug_getelements_(int ielem[])
{
  ug_getelements(ielem);
}

extern "C" void elementsize_(double element[][3], double *size)
{
  elementsize(element, size);
}

extern "C" void intersecthexbody_(double dMinMax[][3], int *iid, int *intersection)
{
  intersecthexbody(dMinMax,iid,intersection);
}

extern "C" void vertexorderxyz_(int *invt,int iorder[], double dcorvg[][3])
{
  vertexorderxyz(invt,iorder,dcorvg);
}

extern "C" void setstartbb_(double *dx,double *dy,double *dz,double *dist)
{
  setstartbb(dx,dy,dz,dist);
}

extern "C" void addelement2bndlist_(int *iel, int *idofs, int *ibody)
{
  addelement2bndlist(iel,idofs,ibody);
}

extern "C" void updatemax0_(double *dx,double *dy,double *dz,double *dist)
{
  updateMax0(dx,dy,dz,dist);
}

extern "C" void setmaxm1_(double *dx,double *dy,double *dz,double *dist)
{
  setMaxM1(dx,dy,dz,dist);
}

extern "C" void getdistancebbid_(double *dx,double *dy,double *dz, double *dist, int *iid)
{
  getdistancebbid(dx,dy,dz,dist,iid);
}

extern "C" void init_fc_rigid_body_(int *iid)
{
  init_fc_rigid_body(iid);
}

extern "C" void init_fc_soft_body_(int *iid)
{
  init_fc_soft_body(iid);
}

extern "C" void fallingparticles_()
{
  fallingparticles();
}

extern "C" void initdeform_()
{
  initdeform();
}

extern "C" void initpointlocation_()
{
  initpointlocation();
}

extern "C" void initaneurysm_()
{
  initaneurysm();
}

extern "C" void intersecbodyelement_(int *ibody,int *iel,double vertices[][3])
{
  intersecbodyelement(ibody,iel,vertices);
}

extern "C" void velocityupdate_soft_()
{
  velocityupdate_soft();
}

extern "C" void velocityupdate_()
{
  velocityupdate<backend>();
}

extern "C" void isinobstacle_(double *dx,double *dy,double *dz,int *isin)
{
  isinobstacle(dx,dy,dz,isin);
}

extern "C" void isboundarybody_(int *isboundary, int *ibodyc)
{
  isboundarybody(isboundary,ibodyc);
}

extern "C" void intersectdomainbody_(int *ibody,int *domain,int *intersection)
{
  intersectdomainbody(ibody,domain,intersection);
}

extern "C" void logposition_()
{
  logposition();
}

extern "C" void logvelocity_()
{
  logvelocity();
}

extern "C" void logangularvelocity_()
{
  logangularvelocity();
}

extern "C" void logcollision_()
{
  logcollision();
}

extern "C" void setelement_(int *iel, int *iID)
{
  setelement(iel,iID);
}

extern "C" void setmyid_(int *myid)
{
  setmyid(myid);
}

extern "C" void setangvelid_(double *dangvelx,double *dangvely,double *dangvelz,int *iid)
{
  setangvelid(dangvelx,dangvely,dangvelz,iid);
}

extern "C" void settimestep_(double *dTime)
{
  settimestep(dTime);
}

extern "C" void settime_(double *dTime)
{
  settime(dTime);
}

extern "C" void gettotalelements_(int* nel, int* ibody)
{
  gettotalelements(nel,ibody);
}

extern "C" void getelementsprev_(int* nel, int* ibody)
{
  getelementsprev(nel,ibody);
}

extern "C" void getallelements_(int* elements, int* ibody)
{
  getallelements(elements,ibody);
}

extern "C" void getelementarray_(int *elements, int *idofselement, int *ibody)
{
  getelementarray(elements,idofselement,ibody);
}

extern "C" void getelements_(int* elements, int* ibody)
{
  getelements(elements, ibody);
}

extern "C" void getdensity_(double *ddens, int *iid)
{
  getdensity(ddens,iid);
}

extern "C" void getnumparticles_(int *nParts)
{
  getnumparticles(nParts);
}

extern "C" void getradius_(double *drad, int *iid)
{
  getradius(drad,iid);
}

extern "C" void getangle_(double *dangx,double *dangy,double *dangz,int *iid)
{
  getangle(dangx,dangy,dangz,iid);
}

extern "C" void getangvel_(double *dangvelx,double *dangvely,double *dangvelz,int *iid)
{
  getangvel(dangvelx,dangvely,dangvelz,iid);
}

extern "C" void gettorque_(double *dx,double *dy,double *dz,int *iID)
{
  gettorque(dx,dy,dz,iID);
}

extern "C" void getforce_(double *dx,double *dy,double *dz,int *iID)
{
  getforce(dx,dy,dz,iID);
}

extern "C" void getelementsinside_(int *iel, int *ibody)
{
  getelementsinside(iel,ibody);
}

extern "C" void getelementsbndry_(int *iel, int *ibody)
{
  getelementsbndry(iel,ibody);
}

extern "C" void startcollisionpipeline_()
{
  startcollisionpipeline<backend>();
}

extern "C" void clearcollisionpipeline_()
{
  clearcollisionpipeline();
}

extern "C" void clearelementlists_(int *ibody)
{
  clearelementlists(ibody);
}

extern "C" void writeparticles_(int *iout)
{
  writeparticles(iout);
}

extern "C" void writesoftbody_(int *iout)
{
  writesoftbody(iout);
}

extern "C" void writepvtu_(int *iNodes,int *iTime)
{
  writepvtu(iNodes,iTime);
}

extern "C" void setelementarray_(double elementsize[], int *iel)
{
  setelementarray(elementsize,iel);
}

extern "C" void write_sol_time_(int *iout, int *istep, double *simTime)
{

  int o   = *iout;
  int i   = *istep;
  double st  = *simTime;

  write_sol_time(o,i,st);
}

extern "C" void read_sol_time_(char startFrom[60], int *istep, double *simTime)
{

  read_sol_time(startFrom,istep,simTime);

}


extern "C" void write_sol_vel_(int *out, int *lvl, int *comp, int *nel_fine, int *nel_coarse, int *dofsInE, 
                                int elemmap[], int *edofs, double *u, double *v, double *w)
{
  int o   = *out;
  int l   = *lvl;
  int nf  = *nel_fine;
  int nc  = *nel_coarse;
  int die = *dofsInE;
  int cp  = *comp;

  write_sol_vel(o, l, cp, nf, nc, die, elemmap, edofs, u, v, w);
}

extern "C" void read_sol_vel_(char startFrom[60], int *lvl, int *comp, int *nel_fine, int *nel_coarse, int *dofsInE, 
                                int elemmap[], int *edofs, double *u, double *v, double *w)
{
  int l   = *lvl;
  int nf  = *nel_fine;
  int nc  = *nel_coarse;
  int die = *dofsInE;
  int cp  = *comp;

  read_sol_vel(startFrom, l, cp, nf, nc, die, elemmap, edofs, u, v, w);
}


extern "C" void write_sol_pres_(int *out, int *lvl, int *nel_fine, int *nel_coarse, int *dofsInE, 
                                int elemmap[], int *edofs, double pres[])
{
  int o   = *out;
  int l   = *lvl;
  int nf  = *nel_fine;
  int nc  = *nel_coarse;
  int die = *dofsInE;

  write_sol_pres(o, l, nf, nc, die, elemmap, edofs, pres);
}

extern "C" void read_sol_pres_(char startFrom[60], int *lvl, int *nel_fine, int *nel_coarse, int *dofsInE, 
                                int elemmap[], int *edofs, double pres[])
{
  int l   = *lvl;
  int nf  = *nel_fine;
  int nc  = *nel_coarse;
  int die = *dofsInE;

  read_sol_pres(startFrom, l, nf, nc, die, elemmap, edofs, pres);
}

extern "C" void writexml_(int *iNEL,int *iNVT,int iKVERT[][8],double dcorvg[][3],double vu[],double vv[],double vw[],double vp[],double dist[],int *iNode,int *iTime)
{

  writexml(iNEL,iNVT,iKVERT,dcorvg,vu,vv,vw,vp,dist,iNode,iTime);
}

extern "C" void writevtk22_(int *iNEL,int *iNVT,int iKVERT[][8],double dcorvg[][3],double dmon1[],double dmon2[],double df[],double du[],double dgradx[],double dgrady[],double dgradz[],double *dt, double *ddt,int *ivl, int *imst, int *itst,int *ismst)
{
  writevtk22(iNEL,iNVT,iKVERT,dcorvg,dmon1,dmon2,df,du,dgradx,dgrady,dgradz,dt,ddt,ivl,imst,itst,ismst);
}

extern "C" void writevtk23_(int *iNEL,int *iNVT, int iKVERT[][8],double dcorvg[][3],
    double dmon[],double dsize[],double dratio[],double *DT,double *DDT,int *ivl,int *imst,int *itst,int *ismst)
{
  writevtk23(iNEL,iNVT,iKVERT,dcorvg,dmon,dsize,dratio,DT,DDT,ivl,imst,itst,ismst);
}

extern "C" void writetri_(int *iNEL,int iKVERT[][8],double dcorvg[][3],int *iNode)
{
  writetri(iNEL,iKVERT,dcorvg,iNode);
}

extern "C" void writeuniformgrid_()
{
  writeuniformgrid();
}

extern "C" void queryuniformgrid_(int* ibody)
{
  queryuniformgrid(ibody);
}


// here we call the collision detection and give back corrections
// of the velocity and position
//
// call startCollisionPipeline
//
//
//do IP=1,IPARTS
// update the particle positions
// and velocities here
//end do


extern "C" void writepolygon_(int *iout)
{
  writepolygon(iout);
}

extern "C" void writeuniformgridlist_()
{
  writeuniformgridlist();
}

extern "C" void getelement_(int *iel, int *iID)
{
  getelement(iel,iID);
}


extern "C" void getdistance_(double *dx,double *dy,double *dz,double *ddist)
{
  getdistance(dx,dy,dz,ddist);
}

extern "C" void getdistanceid_(double *dx,double *dy,double *dz, double *dist, int *iid)
{
  getdistanceid(dx,dy,dz,dist,iid);
}

extern "C" void isinelementid_(double *dx,double *dy,double *dz, int *iID, int *isin)
{
  isinelementid(dx,dy,dz,iID,isin);
}

extern "C" void insidesoftbody_(double *dx,double *dy,double *dz, int *iID, int *isin)
{
  insidesoftbody(dx,dy,dz,iID,isin);
}

extern "C" void isinelementperf_(double *dx,double *dy,double *dz,int *isin)
{
  isinelementperf(dx,dy,dz,isin);
}

extern "C" void isinelement_(double *dx,double *dy,double *dz,int *isin)
{
  isinelement(dx,dy,dz,isin);
}//end getcenter

extern "C" void setposition_(double *dx,double *dy,double *dz)
{
  setposition(dx,dy,dz);
}

extern "C" void setrotation_(double *dx,double *dy,double *dz)
{
  setrotation(dx,dy,dz);
}

extern "C" void settorque_(double *dx,double *dy,double *dz,int *iID)
{
  settorque(dx,dy,dz,iID);
}

extern "C" void setforce_(double *dx,double *dy,double *dz,int *iID)
{
  setforce(dx,dy,dz,iID);
}

extern "C" void setpositionid_(double *dx,double *dy,double *dz,int *iID)
{
  setpositionid(dx,dy,dz,iID);
}

extern "C" void setrotationid_(double *dx,double *dy,double *dz,int *iID)
{
  setrotationid(dx,dy,dz,iID);
}

extern "C" void setvelocityid_(double *dx,double *dy,double *dz,int *iID)
{
  setvelocityid(dx,dy,dz,iID);
}

extern "C" void getpos_(double *dx,double *dy,double *dz,int *iID)
{
  getpos(dx,dy,dz,iID);
}
extern "C" void getvel_(double *dx,double *dy,double *dz,int *iID)
{
  getvel(dx,dy,dz,iID);
}

extern "C" void getsoftbodyvel_(double *dx,double *dy,double *dz,
    double *vx,double *vy,double *vz,
    double *t)
{
  getsoftbodyvel(dx,dy,dz,vx,vy,vz,t); 
}

extern "C" void softbodyinternal_(double *time)
{
  softbodyinternal(time);
}

extern "C" void getsoftcom_(double *dx,double *dy,double *dz)
{
  getsoftcom(dx,dy,dz);
}

extern "C" void getsoftmass_(double *dmass)
{
  getsoftmass(dmass);
}

extern "C" void stepsoftbody_(double *fx,double *fy,double *fz,double *dt)
{
  stepsoftbody(fx,fy,fz,dt);
}

#endif


#endif /* end of include guard: CPPWRAPPER_LINUX_H_PHBGRGXW */
