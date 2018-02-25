/***************************************************************************
 *   Copyright (C) 2006-2010 by Raphael Muenster   *
 *   raphael@Cortez   *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

// #include <string>
// #include <aabb3.h>
// #include <iostream>
// #include <genericloader.h>
// #include <unstructuredgrid.h>
// #include <distops3.h>
// #include <triangulator.h>
// #include <iomanip>
// #include <sstream>
// #include <intersectorray3tri3.h>
// #include <vtkwriter.h>
// #include <world.h>
// #include <particlefactory.h>
// #include <collisionpipeline.h>
// #include <rigidbodymotionmesh.h>
// #include <mymath.h>
// #include <distancetriangle.h>
// #include <boundingvolumetree3.h>
// #include <subdivisioncreator.h>
// #include <traits.h>
// #include <boundarybox.h>
// #include <timecontrol.h>
// #include <matrixnxn.h>
// #include <vectorn.h>
// #include <linearsolvergauss.h>
// #include <reader.h>
// #include <worldparameters.h>
// #include <globalheader.h>
// #include <meshobject.h>
// #include <subdivisioncreator.h>
// #include <boundingvolumetree3.h>
// #include <hspatialhash.h>
// #include <broadphasestrategy.h>
// #include <objloader.h>
// #include <motionintegratorsi.h>
// #include <uniformgrid.h>
// #include <huniformgrid.h>
// #include <perftimer.h>
// #include <ugsizeheuristicstd.h>
// #include <distancemeshpoint.h>
// #include <distancemap.h>
// #include <segmentlistreader.h>
// #include <distancepointpline.h>
// 
// using namespace i3d;
// 
// Real a = CMath<Real>::MAXREAL;
// CUnstrGrid myGrid;
// World myWorld;
// CollisionPipeline myPipeline;
// RigidBodyMotion *myMotion;
// CSubdivisionCreator subdivider;
// CBoundaryBoxr myBoundary;
// TimeControl myTimeControl;
// WorldParameters myParameters;
// Real startTime=0.0;
// CHUniformGrid<Real,ElementCell> myUniformGrid;
// 
// int perrowx;
// int perrowy;
// int perrowz;
// int nTotal = 1000;
// int iOut=0;
// 
// double xmin = -2.0f;
// double ymin = -2.0f;
// double zmin = -4.0f;
// double xmax = 2.0f;
// double ymax = 2.0f;
// double zmax = 4.0f;
// Real radius = Real(0.05);
// int iReadGridFromFile = 1;
// int *islots=NULL;

#include <iostream>
#include <application.h>

namespace i3d {

  class ShapesTest : public Application<> {

  public:

    ShapesTest() : Application() {

    }

    void init(std::string fileName) {

      Application::init(fileName);

    }

    void run() {

      unsigned nOut = 0;
      //start the main simulation loop
      for (; myWorld_.timeControl_->m_iTimeStep <= dataFileParams_.nTimesteps_; myWorld_.timeControl_->m_iTimeStep++)
      {
        Real simTime = myTimeControl_.GetTime();
        Real energy0 = myWorld_.getTotalEnergy();
        std::cout << "------------------------------------------------------------------------" << std::endl;
        std::cout << "## Timestep Nr.: " << myWorld_.timeControl_->m_iTimeStep << " | Simulation time: " << myTimeControl_.GetTime()
          << " | time step: " << myTimeControl_.GetDeltaT() << std::endl;
        std::cout << "Energy: " << energy0 << std::endl;
        std::cout << "------------------------------------------------------------------------" << std::endl;
        std::cout << std::endl;
        myPipeline_.startPipeline();
        Real energy1 = myWorld_.getTotalEnergy();
        std::cout << "Energy after collision: " << energy1 << std::endl;
        std::cout << "Energy difference: " << energy0 - energy1 << std::endl;
        std::cout << "Timestep finished... writing vtk." << std::endl;
        Application::writeOutput(nOut);
        std::cout << "Finished writing vtk." << std::endl;
        nOut++;
        myTimeControl_.SetTime(simTime + myTimeControl_.GetDeltaT());
      }//end for

    }

  };

}

using namespace i3d;



int main()
{
  
  ShapesTest myApp;

  myApp.init("start/sampleRigidBody.xml");

  myApp.run();

  return 0;
}
