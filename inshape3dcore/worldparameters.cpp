/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) <2011>  <Raphael Muenster>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with this program; if not, write to the Free Software Foundation, Inc.,
    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
*/ 


//===================================================
//                     INCLUDES
//===================================================


#include "worldparameters.h"

namespace i3d {

  WorldParameters::WorldParameters() : startType_(0), bodies_(0), bodyInit_(0), 
                                       nTimesteps_(0), timeStep_(0.01), maxIterations_(0),
                                       pipelineIterations_(0), solverType_(0), liquidSolid_(0),
                                       defaultDensity_(1.5), defaultRadius_(0.05),densityMedium_(1.0),
                                       epsLCPSolver_(1e-5),solutionFile_("solution/particles.start0"),bodyConfigurationFile_("defaultBodies.i3d"),
                                       odeConfigurationFile_(""), 
                                       cgalConfigurationFile_(""), 
                                       gravity_(0, 0, -10.0), rigidBodies_(), extents_{-1.0, 1.0, -1.0, 1.0, -1.0, 1.0},
                                       hasExtents_(false), airFriction_(1.0),doFBM_(true),doDynamics_(true), outputRigidBodies_(false), excludeDefaultBoundary_(false)
  {

  }

  WorldParameters::~WorldParameters() 
  {

  }

}

