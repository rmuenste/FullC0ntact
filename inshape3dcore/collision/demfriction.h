#ifndef DEMFRICTION_H
#define DEMFRICTION_H
#include <dembasic.h>
#include <contact.h>

namespace i3d {

  /**
  * @brief Class encapsulating operations for the friction DEM model
  *
  * Class with various methods for DEM force calculation
  *
  */
  class DemFriction : public DemBasic
  {
  public:
    DemFriction(void){};

    ~DemFriction(void){};

    DemFriction(Real dt) : DemBasic(dt) {};

    void evalCompoundBoundary(Real kN, Real gammaN, Real mu, Real gammaT, Contact &contact);

    void evalCompoundCompound(Real kN, Real gammaN, Real mu, Real gammaT, Contact &contact);

    void evalCompoundMesh(Real kN, Real gammaN, Real mu, Real gammaT, Contact &contact);

    void evalCompoundBox(Real kN, Real gammaN, Real mu, Real gammaT, Contact &contact);

  };

}

#endif
