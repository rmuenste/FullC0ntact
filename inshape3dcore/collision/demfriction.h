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
    DemBasic(void){};

    ~DemBasic(void){};

    static void evalCompoundBoundary(Real kN, Real gammaN, Real mu, Real gammaT, Contact &contact);

    static void evalCompoundCompound(Real kN, Real gammaN, Real mu, Real gammaT, Contact &contact);

    static void evalCompoundMesh(Real kN, Real gammaN, Real mu, Real gammaT, Contact &contact);

    static void evalCompoundBox(Real kN, Real gammaN, Real mu, Real gammaT, Contact &contact);

  };

}

#endif
