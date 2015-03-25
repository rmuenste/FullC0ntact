#ifndef DEMFRICTION1_H
#define DEMFRICTION1_H

#include <contact.h>

namespace i3d {

  /**
  * @brief Class encapsulating operations for the friction DEM model
  *
  * Class with various methods for DEM force calculation
  *
  */
  class DemFriction1
  {
  public:

    /**
    * elastic repulsion response
    */
    Real alpha;

    /**
    * viscous dampening response
    */
    Real beta;

    /**
    * elastic repulsion strength
    */
    Real k_alpha;
    Real k_alpha2;

    /**
    * viscous dampening strength
    */
    Real k_beta;

    /**
    * coefficient of static friction
    */
    Real mu_s;

    /**
    * static friction strength
    */
    Real k_t;

    /**
    * coefficient of rolling friction
    */
    Real mu_r;

    /**
    * static friction response
    */
    Real gamma;

    /**
    * Poisson ratio of the material
    */
    Real nu;

    /**
    * Young's modulus
    */
    Real E_j, E_k;

    /**
    * Effective Young's modulus
    */
    Real E_jk;

    /**
    * Effective Radius
    */
    Real R_jk;

    DemFriction1(void)
    {
      alpha = 3.0 / 2.0;
      beta = 0.5;
      mu_s = 0.5;
      mu_r = 0.00025;

      nu = 0.36;

      E_j = E_k = 9.6E9;

      Real E_jk_r = 2.0 / E_j;
      E_jk = 1.0 / E_jk_r;

      R_jk = 1.0 / 0.05;

      k_alpha = (4.0 / 3.0) * sqrt(R_jk) * E_jk / (2.0*(1.0 - pow(nu, 2.0)));

      Real E_eff = (1.0 - pow(nu, 2.0)) / E_j + (1.0 - pow(nu, 2.0)) / E_k;

      Real R_eff = pow(0.05, 2.0) / (0.1);

      k_alpha2 = (4.0 / 3.0) * 1.0/E_eff * sqrt(R_eff);

      k_beta = 50.0;

    };

    ~DemFriction1(void){};

    void evalCompoundBoundary(Real kN, Real gammaN, Real mu, Real gammaT, Contact &contact);

    void evalCompoundCompound(Real kN, Real gammaN, Real mu, Real gammaT, Contact &contact);

    void evalCompoundMesh(Real kN, Real gammaN, Real mu, Real gammaT, Contact &contact);

    void evalCompoundBox(Real kN, Real gammaN, Real mu, Real gammaT, Contact &contact);



  };

}

#endif
