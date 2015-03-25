#ifndef DEMFRICTION_H
#define DEMFRICTION_H

#include <contact.h>

namespace i3d {

  /**
  * @brief Class encapsulating operations for the friction DEM model
  *
  * Class with various methods for DEM force calculation
  *
  */
  class DemFriction
  {
  public:
    DemFriction(void){};

    ~DemFriction(void){};

    static void evalCompoundBoundary(Real kN, Real gammaN, Real mu, Real gammaT, Contact &contact);

    static void evalCompoundCompound(Real kN, Real gammaN, Real mu, Real gammaT, Contact &contact);

    static void evalCompoundMesh(Real kN, Real gammaN, Real mu, Real gammaT, Contact &contact);

    static void evalCompoundBox(Real kN, Real gammaN, Real mu, Real gammaT, Contact &contact);

  };

}

#endif
