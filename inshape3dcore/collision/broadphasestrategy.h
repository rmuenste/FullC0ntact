/*
   This library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Library General Public
   License version 2 as published by the Free Software Foundation.

   This library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Library General Public License for more details.

   You should have received a copy of the GNU Library General Public License
   along with this library; see the file COPYING.LIB.  If not, write to
   the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
   Boston, MA 02110-1301, USA.
*/

#ifndef BROADPHASESTRATEGY_H
#define BROADPHASESTRATEGY_H
#include <list>
#include <collisioninfo.h>
#include <timecontrol.h>
#include <implicitgrid.h>
#include <set>

namespace i3d {

class World;

/**
* @brief Base class for broadphase strategies
*
* 
*/

class BroadPhaseStrategy
{
  
  public:
    
  BroadPhaseStrategy(){};
  
  BroadPhaseStrategy(World* pDomain);
  
  virtual ~BroadPhaseStrategy();  
  
  /**
   * This method should handle all initializations needed
   * to run the broadphase strategy. Each derived broadphase 
   * strategy should implement its own initialization method.
   */
  virtual void init();  
  
  /**
   * This method starts execution of the broadphase strategy
   * and as a result should fill the m_BroadPhasePairs set with
   * the pairs of bodies that are in close proximity
   */
  virtual void start();

  inline void setEps_(Real CollEps) {collEps_ = CollEps;};

  inline Real getEps_() const {return collEps_;};

  World     *world_;

  TimeControl *timeControl_;

  Real collEps_;

  ImplicitGrid *implicitGrid_;

  std::set<BroadPhasePair,Comp> *broadPhasePairs_;  
  
};

}

#endif // BROADPHASESTRATEGY_H
