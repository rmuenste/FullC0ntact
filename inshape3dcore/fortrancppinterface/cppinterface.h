
#ifndef _CPPINTERFACE_H_

#define _CPPINTERFACE_H_

#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <list>
#include <3dmodel.h>
#include <distops3.h>
#include <genericloader.h>
#include <world.h>
#include <collisionpipeline.h>
//===================================================
//					DEFINES
//===================================================

#define MAX_PARTICLES 128

//================================================
//		    		GLOBALS
//================================================

extern "C" void addelement2list(int *iel, int *ibody);
extern "C" void addelement2bndlist(int *iel, int *idofs, int *ibody);
extern "C" void addbdryparam(int *iBnds,char *name, int length);

extern "C" void elementsize(double element[][3], double *size);
extern "C" void setelementarray(double elementsize[], int *iel);

extern "C" void fallingparticles();
extern "C" void initdeform();
extern "C" void initpointlocation();
extern "C" void initaneurysm();
extern "C" void intersecbodyelement(int *ibody,int *iel, double vertices[][3]);
extern "C" void intersectdomainbody(int *ibody,int *domain,int *intersection);

extern "C" void startcollisionpipeline();
extern "C" void clearcollisionpipeline();
extern "C" void clearelementlists(int *ibody);
extern "C" void logdistance();
extern "C" void logposition();
extern "C" void logvelocity();
extern "C" void logcollision();
extern "C" void logangularvelocity();

//extern "C" void insidesolid(double *dx,double *dy,double *dz,int *isin);
extern "C" void isinelement(double *dx,double *dy,double *dz,int *isin);
extern "C" void isinobstacle(double *dx,double *dy,double *dz,int *isin);
extern "C" void isinelementid(double *dx,double *dy,double *dz, int *iID, int *isin);
extern "C" void isinelementperf(double *dx,double *dy,double *dz,int *isin);
extern "C" void isboundarybody(int *isboundary, int *ibodyc);
extern "C" void intersecthexbody(double dMinMax[][3], int *iid, int *intersection);
extern "C" void inituniformgrid(double vmin[3], double vmax[3], double element[][3]);
extern "C" void initbdryparam();

extern "C" void setmyid(int *myid);
extern "C" void setposition(double *dx,double *dy,double *dz);
extern "C" void setrotation(double *dx,double *dy,double *dz);
extern "C" void setpositionid(double *dx,double *dy,double *dz,int *iID);
extern "C" void setrotationid(double *dx,double *dy,double *dz,int *iID);
extern "C" void setvelocityid(double *dx,double *dy,double *dz,int *iID);
extern "C" void setangvelid(double *dangvelx,double *dangvely,double *dangvelz,int *iid);
extern "C" void settimestep(double *dTime);
extern "C" void settime(double *dTime);
extern "C" void setforce(double *dx,double *dy,double *dz,int *iID);
extern "C" void settorque(double *dx,double *dy,double *dz,int *iID);
extern "C" void setelement(int *iel, int *iID);
extern "C" void updateMax0(double *dx,double *dy,double *dz,double *dist);
extern "C" void setMaxM1(double *dx,double *dy,double *dz,double *dist);

extern "C" void writexml(int *iNEL,int *iNVT,int iKVERT[][8],double dcorvg[][3],double vu[],double vv[],double vw[],double vp[],double dist[],int *iNode,int *iTime);
extern "C" void writepvtu(int *iNodes,int *iTime);
extern "C" void writeparticles(int *iout);
extern "C" void writeuniformgrid();
extern "C" void writepolygon(int *iout);
extern "C" void writetri(int *iNEL,int iKVERT[][8],double dcorvg[][3],int *iNode);

extern "C" void writevtk22(int *iNEL,int *iNVT,int iKVERT[][8],double dcorvg[][3],double dmon1[],double dmon2[],double df[],double du[],double dgradx[],double dgrady[],double dgradz[],double *dt, double *ddt,int *ivl, int *imst, int *itst,int *ismst);

extern "C" void writevtk23(int *iNEL,int *iNVT, int iKVERT[][8],double dcorvg[][3],
double dmon[],double dsize[],double dratio[],double *DT,double *DDT,int *ivl,int *imst,int *itst,int *ismst);

extern "C" void writeuniformgridlist();

extern "C" void getdistance(double *dx,double *dy,double *dz,double *ddist);
extern "C" void getdistanceid(double *dx,double *dy,double *dz, double *dist, int *iid);
extern "C" void getdistancebbid(double *dx,double *dy,double *dz, double *dist, int *iid);
extern "C" void getnumparticles(int *nParts);
extern "C" void getradius(double *drad, int *iid);
extern "C" void getangle(double *dangx,double *dangy,double *dangz,int *iid);
extern "C" void getangvel(double *dangvelx,double *dangvely,double *dangvelz,int *iid);
extern "C" void getpos(double *dx,double *dy,double *dz,int *iID);
extern "C" void getvel(double *dx,double *dy,double *dz,int *iID);
extern "C" void getforce(double *dx,double *dy,double *dz,int *iID);
extern "C" void gettorque(double *dx,double *dy,double *dz,int *iID);
extern "C" void getdensity(double *ddens, int *iid);
extern "C" void getelement(int *iel, int *iID);
extern "C" void gettiming(double *time);

extern "C" void getelementsinside(int *iel, int *ibody);
extern "C" void getelementsbndry(int *iel, int *ibody);

extern "C" void getelementarray(int *elements, int *idofselement, int *ibody);
extern "C" void getallelements(int* elements, int* ibody);
extern "C" void gettotalelements(int* nel, int* ibody);
extern "C" void getelementsprev(int* nel, int* ibody);
extern "C" void getelements(int* elements, int* ibody);
extern "C" void getrandfloat(double point[]);
extern "C" void velocityupdate();
extern "C" void dumprigidbodies();
extern "C" void dumpworld();

extern "C" void vertexorderxyz(int *invt,int iorder[], double dcorvg[][3]);
extern "C" void setstartbb(double *dx,double *dy,double *dz,double *dist);
extern "C" void setdomainbox(double vmin[3], double vmax[3]);

extern "C" void updateelementsprev(int *ibody);
extern "C" void uniformgridinsert(int *iel, double center[3]);
extern "C" void queryuniformgrid(int *ibody);
extern "C" void ug_insertelement(int *iel, double center[3], double *size);
extern "C" void ug_querystatus();
extern "C" void ug_pointquery(double center[3], int *iiel);
extern "C" void ug_getelements(int ielem[]);
extern "C" void ug_resetuniformgrid();
extern "C" void starttiming();
extern "C" void bndryproj(double *dx,double *dy,double *dz, double *dxx, double *dyy, double *dzz);
extern "C" void bndryprojid(double *dx,double *dy,double *dz, double *dxx, double *dyy, double *dzz,int *id);


#ifdef FEATFLOWLIB

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

extern "C" void addbdryparam_(int *iBnds,char *name, int length)
{
  addbdryparam(iBnds,name,length);
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

extern "C" void velocityupdate_()
{
  velocityupdate();
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


extern "C" void _startcollisionpipeline()
{
	startcollisionpipeline();
}

extern "C" void startcollisionpipeline_()
{
	startcollisionpipeline();
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

extern "C" void writepvtu_(int *iNodes,int *iTime)
{
  writepvtu(iNodes,iTime);
}

extern "C" void setelementarray_(double elementsize[], int *iel)
{
  setelementarray(elementsize,iel);
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
#endif

#endif
