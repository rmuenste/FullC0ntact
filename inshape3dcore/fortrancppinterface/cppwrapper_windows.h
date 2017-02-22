#ifndef CPPWRAPPER_WINDOWS_H_FJVBVX35
#define CPPWRAPPER_WINDOWS_H_FJVBVX35

#ifdef FEATFLOWLIB

extern "C" void GETRANDFLOAT(double point[])
{
  getrandfloat(point);
}

extern "C" void GETTIMING(double *time)
{
  gettiming(time);
}

extern "C" void STARTTIMING()
{
  starttiming();
}

extern "C" void BNDRYPROJ(double *dx,double *dy,double *dz, double *dxx, double *dyy, double *dzz)
{
  bndryproj(dx,dy,dz,dxx,dyy,dzz);
}

extern "C" void BNDRYPROJID(double *dx,double *dy,double *dz, double *dxx, double *dyy, double *dzz, int *id)
{
  bndryprojid(dx,dy,dz,dxx,dyy,dzz,id);
}

extern "C" void SETDOMAINBOX(double vmin[3], double vmax[3])
{
  setdomainbox(vmin,vmax);
}

extern "C" void INITUNIFORMGRID(double vmin[3], double vmax[3], double element[][3])
{
  inituniformgrid(vmin,vmax,element);
}

extern "C" void INITBDRYPARAM()
{
  initbdryparam();
}

extern "C" void ADDBDRYPARAM(int *iBnds, int *itype, char *name, int length)
{
  addbdryparam(iBnds,itype,name,length);
}

extern "C" void ADDELEMENT2LIST(int *iel, int *ibody)
{
  addelement2list(iel,ibody);
}

extern "C" void UPDATEELEMENTSPREV(int *ibody)
{
  updateelementsprev(ibody);
}

extern "C" void UNIFORMGRIDINSERT(int *iel, double center[3])
{
  uniformgridinsert(iel,center);
}

extern "C" void UG_INSERTELEMENT(int *iel, double center[3], double *size)
{
  ug_insertelement(iel,center,size);
}

extern "C" void UG_RESETUNIFORMGRID()
{
  ug_resetuniformgrid();
}

extern "C" void UG_QUERYSTATUS()
{
  ug_querystatus();
}

extern "C" void UG_POINTQUERY(double center[3], int *iiel)
{
  ug_pointquery(center,iiel);
}

extern "C" void UG_GETELEMENTS(int ielem[])
{
  ug_getelements(ielem);
}

extern "C" void ELEMENTSIZE(double element[][3], double *size)
{
  elementsize(element, size);
}

extern "C" void INTERSECTHEXBODY(double dMinMax[][3], int *iid, int *intersection)
{
  intersecthexbody(dMinMax,iid,intersection);
}

extern "C" void VERTEXORDERXYZ(int *invt,int iorder[], double dcorvg[][3])
{
  vertexorderxyz(invt,iorder,dcorvg);
}

extern "C" void SETSTARTBB(double *dx,double *dy,double *dz,double *dist)
{
  setstartbb(dx,dy,dz,dist);
}

extern "C" void ADDELEMENT2BNDLIST(int *iel, int *idofs, int *ibody)
{
  addelement2bndlist(iel,idofs,ibody);
}

extern "C" void UPDATEMAX0(double *dx,double *dy,double *dz,double *dist)
{
  updateMax0(dx,dy,dz,dist);
}

extern "C" void SETMAXM1(double *dx,double *dy,double *dz,double *dist)
{
  setMaxM1(dx,dy,dz,dist);
}

extern "C" void GETDISTANCEBBID(double *dx,double *dy,double *dz, double *dist, int *iid)
{
  getdistancebbid(dx,dy,dz,dist,iid);
}

extern "C" void INIT_FC_RIGID_BODY(int *iid)
{
	init_fc_rigid_body(iid);
}

extern "C" void FALLINGPARTICLES()
{
	fallingparticles();
}

extern "C" void INITDEFORM()
{
  initdeform();
}

extern "C" void INITPOINTLOCATION()
{
  initpointlocation();
}

extern "C" void INITANEURYSM()
{
  initaneurysm();
}

extern "C" void INTERSECBODYELEMENT(int *ibody,int *iel,double vertices[][3])
{
  intersecbodyelement(ibody,iel,vertices);
}

extern "C" void VELOCITYUPDATE()
{
  velocityupdate();
}

extern "C" void ISINOBSTACLE(double *dx,double *dy,double *dz,int *isin)
{
	isinobstacle(dx,dy,dz,isin);
}

extern "C" void ISBOUNDARYBODY(int *isboundary, int *ibodyc)
{
  isboundarybody(isboundary,ibodyc);
}

extern "C" void INTERSECTDOMAINBODY(int *ibody,int *domain,int *intersection)
{
  intersectdomainbody(ibody,domain,intersection);
}

extern "C" void LOGPOSITION()
{
	logposition();
}

extern "C" void LOGVELOCITY()
{
	logvelocity();
}

extern "C" void LOGANGULARVELOCITY()
{
	logangularvelocity();
}

extern "C" void LOGCOLLISION()
{
	logcollision();
}

extern "C" void SETELEMENT(int *iel, int *iID)
{
  setelement(iel,iID);
}

extern "C" void SETMYID(int *myid)
{
  setmyid(myid);
}

extern "C" void SETANGVELID(double *dangvelx,double *dangvely,double *dangvelz,int *iid)
{
	setangvelid(dangvelx,dangvely,dangvelz,iid);
}

extern "C" void SETTIMESTEP(double *dTime)
{
	settimestep(dTime);
}

extern "C" void SETTIME(double *dTime)
{
	settime(dTime);
}

extern "C" void GETTOTALELEMENTS(int* nel, int* ibody)
{
  gettotalelements(nel,ibody);
}

extern "C" void GETELEMENTSPREV(int* nel, int* ibody)
{
  getelementsprev(nel,ibody);
}

extern "C" void GETALLELEMENTS(int* elements, int* ibody)
{
  getallelements(elements,ibody);
}

extern "C" void GETELEMENTARRAY(int *elements, int *idofselement, int *ibody)
{
  getelementarray(elements,idofselement,ibody);
}

extern "C" void GETELEMENTS(int* elements, int* ibody)
{
  getelements(elements, ibody);
}

extern "C" void GETDENSITY(double *ddens, int *iid)
{
	getdensity(ddens,iid);
}

extern "C" void GETNUMPARTICLES(int *nParts)
{
	getnumparticles(nParts);
}

extern "C" void GETRADIUS(double *drad, int *iid)
{
	getradius(drad,iid);
}

extern "C" void GETANGLE(double *dangx,double *dangy,double *dangz,int *iid)
{
	getangle(dangx,dangy,dangz,iid);
}

extern "C" void GETANGVEL(double *dangvelx,double *dangvely,double *dangvelz,int *iid)
{
	getangvel(dangvelx,dangvely,dangvelz,iid);
}

extern "C" void GETTORQUE(double *dx,double *dy,double *dz,int *iID)
{
	gettorque(dx,dy,dz,iID);
}

extern "C" void GETFORCE(double *dx,double *dy,double *dz,int *iID)
{
	getforce(dx,dy,dz,iID);
}

extern "C" void GETELEMENTSINSIDE(int *iel, int *ibody)
{
  getelementsinside(iel,ibody);
}

extern "C" void GETELEMENTSBNDRY(int *iel, int *ibody)
{
  getelementsbndry(iel,ibody);
}


extern "C" void _startcollisionpipeline()
{
	startcollisionpipeline();
}

extern "C" void STARTCOLLISIONPIPELINE()
{
	startcollisionpipeline();
}

extern "C" void CLEARCOLLISIONPIPELINE()
{
	clearcollisionpipeline();
}

extern "C" void CLEARELEMENTLISTS(int *ibody)
{
  clearelementlists(ibody);
}

extern "C" void WRITEPARTICLES(int *iout)
{
	writeparticles(iout);
}

extern "C" void WRITEPVTU(int *iNodes,int *iTime)
{
  writepvtu(iNodes,iTime);
}

extern "C" void SETELEMENTARRAY(double elementsize[], int *iel)
{
  setelementarray(elementsize,iel);
}

extern "C" void WRITEXML(int *iNEL,int *iNVT,int iKVERT[][8],double dcorvg[][3],double vu[],double vv[],double vw[],double vp[],double dist[],int *iNode,int *iTime)
{

  writexml(iNEL,iNVT,iKVERT,dcorvg,vu,vv,vw,vp,dist,iNode,iTime);
}

extern "C" void WRITEVTK22(int *iNEL,int *iNVT,int iKVERT[][8],double dcorvg[][3],double dmon1[],double dmon2[],double df[],double du[],double dgradx[],double dgrady[],double dgradz[],double *dt, double *ddt,int *ivl, int *imst, int *itst,int *ismst)
{
  writevtk22(iNEL,iNVT,iKVERT,dcorvg,dmon1,dmon2,df,du,dgradx,dgrady,dgradz,dt,ddt,ivl,imst,itst,ismst);
}

extern "C" void WRITEVTK23(int *iNEL,int *iNVT, int iKVERT[][8],double dcorvg[][3],
double dmon[],double dsize[],double dratio[],double *DT,double *DDT,int *ivl,int *imst,int *itst,int *ismst)
{
  writevtk23(iNEL,iNVT,iKVERT,dcorvg,dmon,dsize,dratio,DT,DDT,ivl,imst,itst,ismst);
}

extern "C" void WRITETRI(int *iNEL,int iKVERT[][8],double dcorvg[][3],int *iNode)
{
  writetri(iNEL,iKVERT,dcorvg,iNode);
}

extern "C" void WRITEUNIFORMGRID()
{
  writeuniformgrid();
}

extern "C" void QUERYUNIFORMGRID(int* ibody)
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


extern "C" void WRITEPOLYGON(int *iout)
{
	writepolygon(iout);
}

extern "C" void WRITEUNIFORMGRIDLIST()
{
  writeuniformgridlist();
}

extern "C" void GETELEMENT(int *iel, int *iID)
{
  getelement(iel,iID);
}


extern "C" void GETDISTANCE(double *dx,double *dy,double *dz,double *ddist)
{
 getdistance(dx,dy,dz,ddist);
}

extern "C" void GETDISTANCEID(double *dx,double *dy,double *dz, double *dist, int *iid)
{
  getdistanceid(dx,dy,dz,dist,iid);
}

extern "C" void ISINELEMENTID(double *dx,double *dy,double *dz, int *iID, int *isin)
{
	isinelementid(dx,dy,dz,iID,isin);
}

extern "C" void ISINELEMENTPERF(double *dx,double *dy,double *dz,int *isin)
{
	isinelementperf(dx,dy,dz,isin);
}

extern "C" void ISINELEMENT(double *dx,double *dy,double *dz,int *isin)
{
	isinelement(dx,dy,dz,isin);
}//end getcenter

extern "C" void SETPOSITION(double *dx,double *dy,double *dz)
{
	setposition(dx,dy,dz);
}

extern "C" void SETROTATION(double *dx,double *dy,double *dz)
{
	setrotation(dx,dy,dz);
}

extern "C" void SETTORQUE(double *dx,double *dy,double *dz,int *iID)
{
	settorque(dx,dy,dz,iID);
}

extern "C" void SETFORCE(double *dx,double *dy,double *dz,int *iID)
{
	setforce(dx,dy,dz,iID);
}

extern "C" void SETPOSITIONID(double *dx,double *dy,double *dz,int *iID)
{
	setpositionid(dx,dy,dz,iID);
}

extern "C" void SETROTATIONID(double *dx,double *dy,double *dz,int *iID)
{
	setrotationid(dx,dy,dz,iID);
}

extern "C" void SETVELOCITYID(double *dx,double *dy,double *dz,int *iID)
{
	setvelocityid(dx,dy,dz,iID);
}

extern "C" void GETPOS(double *dx,double *dy,double *dz,int *iID)
{
  getpos(dx,dy,dz,iID);
}
extern "C" void GETVEL(double *dx,double *dy,double *dz,int *iID)
{
  getvel(dx,dy,dz,iID);
}

extern "C" void GETSOFTBODYVEL(double *dx,double *dy,double *dz,
                                double *vx,double *vy,double *vz,
                                double *t)
{
  getsoftbodyvel(dx,dy,dz,vx,vy,vz,t); 
}

extern "C" void SOFTBODYINTERNAL(double *time)
{
  softbodyinternal(time);
}

extern "C" void GETSOFTCOM(double *dx,double *dy,double *dz)
{
  getsoftcom(dx,dy,dz);
}

extern "C" void GETSOFTMASS(double *dmass)
{
  getsoftmass(dmass);
}

extern "C" void STEPSOFTBODY(double *fx,double *fy,double *fz,double *dt)
{
  stepsoftbody(fx,fy,fz,dt);
}

#endif

#endif /* end of include guard: CPPWRAPPER_WINDOWS_H_FJVBVX35 */
