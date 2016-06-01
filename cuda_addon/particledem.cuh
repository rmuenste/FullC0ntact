#ifndef PARTICLEDEM_CUH_TADONS9R
#define PARTICLEDEM_CUH_TADONS9R

#include <3dmodel.h>
#include <distancemap.h>
#include <rigidbody.h>
#include <uniformgrid.h>
#include <worldparameters.h>


void cuda_init(i3d::HashGrid<float, i3d::cpu> &hg,
  i3d::ParticleWorld<float, i3d::cpu> &pw,
  i3d::SimulationParameters<float>*& params);

void cuda_clean();

void calcHash(i3d::HashGrid<float, i3d::cpu> &hg, i3d::ParticleWorld<float, i3d::cpu> &pw);

void reorderDataAndFindCellStart(i3d::HashGrid<float, i3d::cpu> &hg,
  i3d::ParticleWorld<float, i3d::cpu> &pw);

void evalForces(i3d::HashGrid<float, i3d::cpu> &hg, i3d::ParticleWorld<float, i3d::cpu> &pw);

void sortParticles(i3d::HashGrid<float, i3d::cpu> &hg);

void integrateSystem(float *pos, float *vel, float deltaTime, unsigned int numParticles);

#endif /* end of include guard: PARTICLEDEM_CUH_TADONS9R */
