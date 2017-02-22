
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
extern "C" void addbdryparam(int *iBnds, int *itype, char *name, int length);

extern "C" void elementsize(double element[][3], double *size);
extern "C" void setelementarray(double elementsize[], int *iel);

extern "C" void fallingparticles();
extern "C" void init_fc_rigid_body(int *iid);
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
extern "C" void getsoftbodyvel(double *dx,double *dy,double *dz,
                               double *vx,double *vy,double *vz,double *t);
extern "C" void softbodyinternal(double *time);
extern "C" void stepsoftbody(double *fx,double *fy,double *fz,double *dt);

extern "C" void getelementsinside(int *iel, int *ibody);
extern "C" void getelementsbndry(int *iel, int *ibody);
extern "C" void getsoftcom(double *dx,double *dy,double *dz);
extern "C" void getsoftmass(double *dmass);

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

#ifdef _MSC_VER
  #include <cppwrapper_windows.h> 
#else
  #include <cppwrapper_linux.h> 
#endif


#endif
