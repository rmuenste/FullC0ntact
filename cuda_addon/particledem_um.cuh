#ifndef PARTICLEDEM_UM_CUH_TADONS9R
#define PARTICLEDEM_UM_CUH_TADONS9R

#include <3dmodel.h>
#include <distancemap.h>
#include <rigidbody.h>
#include <uniformgrid.h>
#include <worldparameters.h>


void cuda_init();

void calcHash();

void reorderDataAndFindCellStart();

void evalForces();

void sortParticles();

void integrateSystem();

void transfer_data(std::vector<float> &positions);

#endif /* end of include guard: PARTICLEDEM_UM_CUH_TADONS9R */
