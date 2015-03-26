#ifndef DEMBASIC_H
#define DEMBASIC_H

#include <contact.h>

namespace i3d {

  /**
  * @brief Class encapsulating operations for the basic DEM model
  *
  * Class with various methods for DEM force calculation
  *
  */
  class DemBasic
  {
  public:

    /**
     * Time step of the simulation
     */
    Real dt_;

    Real getDt() {return dt_;};

    void setDt(Real dt) {dt_ = dt;};

    DemBasic(void) : dt_(0.0) {};

    DemBasic(Real dt) : dt_(dt) {};

    virtual ~DemBasic(void){};

    virtual void evalCompoundBoundary(Real kN, Real gammaN, Real mu, Real gammaT, Contact &contact);

    virtual void evalCompoundCompound(Real kN, Real gammaN, Real mu, Real gammaT, Contact &contact);

    virtual void evalCompoundMesh(Real kN, Real gammaN, Real mu, Real gammaT, Contact &contact);

    virtual void evalCompoundBox(Real kN, Real gammaN, Real mu, Real gammaT, Contact &contact);

  };

}

#endif
