
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
#include <basicpipeline.hpp>
#include <collisionpipeline.h>

#include <string>
#include <aabb3.h>
#include <iostream>
#include <genericloader.h>
#include <unstructuredgrid.h>
#include <distops3.h>
#include <triangulator.h>
#include <iomanip>
#include <sstream>
#include <intersectorray3tri3.h>
#include <vtkwriter.h>
#include <world.h>
#include <particlefactory.h>
#include <collisionpipeline.h>
#include <distancetriangle.h>
#include <boundingvolumetree3.h>
#include <subdivisioncreator.h>
#include <traits.h>
#include <boundarybox.h>
#include <timecontrol.h>
#include <log.h>
#include <rigidbodyio.h>
#include <meshobject.h>
#include <reader.h>
#include <deformparameters.h>
#include <objloader.h>
#include <hspatialhash.h>
#include <broadphasestrategy.h>
#include <distancemeshpoint.h>
#include <distancetriangle.h>
#include <intersector2aabb.h>
#include <perftimer.h>
#include <motionintegratorsi.h>
#include <uniformgrid.h>
#include <huniformgrid.h>
#include <boundarycyl.h>
#include <segmentlistreader.h>
#include <distancepointpline.h>
#include <distancepointcylinder.h>
#include <memory>
#include <distanceaabbpoint.h>

#include <iostream>
#include <fstream>
#include <application.h>

#ifdef WITH_OPENMESH
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/IO/writer/VTKWriter.hh>
#include <OpenMesh/Core/IO/exporter/BaseExporter.hh>
#include <OpenMesh/Core/IO/exporter/ExporterT.hh>
#include <pbd_body.hpp>
#include <pbd_solver.hpp>
#endif


#ifdef WITH_ODE
#include <ode/odeconfig.h>
#include <json.hpp>
constexpr i3d::BackEnd backend = i3d::BackEnd::backendODE;
#else
constexpr i3d::BackEnd backend = i3d::BackEnd::backendDefault;
#endif

#include <random>

#include <softbody.hpp>
#include <mymath.h>


//===================================================
//					DEFINES
//===================================================

#define MAX_PARTICLES 128

//================================================
//                     GLOBALS
//================================================

extern "C" void addelement2list(int *iel, int *ibody);
extern "C" void addelement2bndlist(int *iel, int *idofs, int *ibody);
extern "C" void addbdryparam(int *iBnds, int *itype, char *name, int length);


extern "C" void brownianDisplacement();

extern "C" void elementsize(double element[][3], double *size);
extern "C" void setelementarray(double elementsize[], int *iel);

extern "C" void fallingparticles();
extern "C" void init_fc_rigid_body(int *iid);
extern "C" void init_fc_cgal(int *iid);

extern "C" void init_fc_soft_body(int *iid);
extern "C" void initdeform();
extern "C" void initpointlocation();
extern "C" void initaneurysm();
extern "C" void intersecbodyelement(int *ibody,int *iel, double vertices[][3]);
extern "C" void intersectdomainbody(int *ibody,int *domain,int *intersection);

void get_dynamics_type(int *iid, int *dynType);

template <i3d::BackEnd collisionBackend>
void startcollisionpipeline();

template <i3d::BackEnd collisionBackend>
void velocityupdate();

template <i3d::BackEnd collisionBackend>
void update_particle_state(double *px, double *py, double *pz,
                           double *vx, double *vy, double *vz,
                           double *ax, double *ay, double *az,
                           double *avx, double *avy, double *avz,
                           int *iid
                          );

template <i3d::BackEnd collisionBackend>
void projectOnBoundaryid(double *dx, double *dy, double *dz,
  double *px, double *py, double *pz,
  double *dist, int *iid);

extern "C" void clearcollisionpipeline();

extern "C" void communicateforce(double *fx, double *fy, double *fz, double *tx, double *ty, double *tz);

extern "C" void clearelementlists(int *ibody);
extern "C" void logposition();
extern "C" void logvelocity();

extern "C" void isinelement(double *dx,double *dy,double *dz,int *isin);

extern "C" void isinobstacle(double *dx,double *dy,double *dz,int *isin);

template <i3d::BackEnd collisionBackend>
void isinelementid(double *dx,double *dy,double *dz, int *iID, int *isin);

template <i3d::BackEnd collisionBackend>
void write_rigid_bodies(int *iout);

template <i3d::BackEnd collisionBackend>
void write_sol_rb(int iout);

extern "C" void insidesoftbody(double *dx,double *dy,double *dz, int *iID, int *isin);

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

void write_1d_header(int *iout, int *my1DOut_nol, int n);

extern "C" void writeparticles(int *iout);

extern "C" void writesoftbody(int *iout);

extern "C" void writeuniformgrid();
extern "C" void writepolygon(int *iout);
extern "C" void writetri(int *iNEL,int iKVERT[][8],double dcorvg[][3],int *iNode);

extern "C" void writevtk22(int *iNEL,int *iNVT,int iKVERT[][8],double dcorvg[][3],double dmon1[],double dmon2[],double df[],double du[],double dgradx[],double dgrady[],double dgradz[],double *dt, double *ddt,int *ivl, int *imst, int *itst,int *ismst);

extern "C" void writevtk23(int *iNEL,int *iNVT, int iKVERT[][8],double dcorvg[][3],
double dmon[],double dsize[],double dratio[],double *DT,double *DDT,int *ivl,int *imst,int *itst,int *ismst);

extern "C" void writeuniformgridlist();

extern "C" void getdistance(double *dx,double *dy,double *dz,double *ddist);


template <i3d::BackEnd collisionBackend>
void getdistanceid(double *dx,double *dy,double *dz, double *dist, int *iid);


template <i3d::BackEnd collisionBackend>
void getClosestPointid(double *dx, double *dy, double *dz,
                       double *px, double *py, double *pz,
                       double *dist, int *iid);

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

extern "C" void softbodyinternal(double *time);
extern "C" void stepsoftbody(double *fx,double *fy,double *fz,double *dt);

extern "C" void getelementsinside(int *iel, int *ibody);
extern "C" void getelementsbndry(int *iel, int *ibody);

extern "C" void getsoftvel(double *x,double *y,double *z,
                               double *vx,double *vy,double *vz, int *ip);

extern "C" void getsoftcom(double *dx,double *dy,double *dz);
extern "C" void getsoftmass(double *dmass);
extern "C" void getsoftbodyvel(double *dx,double *dy,double *dz,
                               double *vx,double *vy,double *vz,double *t);

extern "C" void getelementarray(int *elements, int *idofselement, int *ibody);
extern "C" void getallelements(int* elements, int* ibody);
extern "C" void gettotalelements(int* nel, int* ibody);
extern "C" void getelementsprev(int* nel, int* ibody);
extern "C" void getelements(int* elements, int* ibody);
extern "C" void getrandfloat(double point[]);


#ifdef OPTIC_FORCES
extern "C" void get_optic_forces();
#endif

//extern "C" void velocityupdate();

extern "C" void velocityupdate_soft();

extern "C" void dumprigidbodies();
extern "C" void dumpworld();

extern "C" void vertexorderxyz(int *invt,int iorder[], double dcorvg[][3]);
extern "C" void setstartbb(double *dx,double *dy,double *dz,double *dist);
extern "C" void setdomainbox(double vmin[3], double vmax[3]);

extern "C" void ode_get_position();

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

void clean_output_array();
void add_output_array(double *array);

void parse_header_line(char headerLine[1024],int *inel);

void write_sol_time(int iout, int istep, double simTime);

void read_sol_time(char startFrom[60], int *istep, double *simTime);

void setoutputidx(char startFrom[60]);

void write_sol_pres(int iout, int lvl, int nel_fine, int nel_coarse, int dofsInE,
                    int elemmap[], int *edofs, double pres[]);

void read_sol_pres(char startFrom[60], int lvl, int nel_fine, int nel_coarse, int dofsInE,
                    int elemmap[], int *edofs, double pres[]);

void write_sol_vel(int iout, int lvl, int comp, int nel_fine, int nel_coarse, int dofsInE,
                    int elemmap[], int *edofs, double *u, double *v, double *w);

void read_sol_vel(char startFrom[60], int lvl, int comp, int nel_fine, int nel_coarse, int dofsInE,
                    int elemmap[], int *edofs, double *u, double *v, double *w);

void write_q2_comp(std::ofstream &out, int iout, int lvl, int comp,
                   int nel_fine, int nel_coarse, int dofsInE,
                   int elemmap[], int *edofs, double *u);

void write_q2_sol(char startFrom[60], int iout, int lvl, int comp,
                   int nel_fine, int nel_coarse, int dofsInE,
                   int elemmap[], int *edofs);

void read_q2_sol(char userField[60], char startFrom[60], int lvl, int comp,
                 int nel_fine, int nel_coarse, int dofsInE,
                 int elemmap[], int *edofs);

void write_sol_rb(int iout);

void read_sol_rb(char startFrom[60]);

using namespace i3d;

//==================================================================================================================

//==================================================================================================================
#endif
