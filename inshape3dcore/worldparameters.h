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
#ifndef WORLDPARAMETERS_H
#define WORLDPARAMETERS_H



//===================================================
//                     INCLUDES
//===================================================
#include <string>
#include <vector3.h>
#include <vector>
#include <rigidbodyio.h>

namespace i3d {

/**
* @brief A class that contains parameters used to create a CWorld.
*
* A class that contains parameters used to create a CWorld.
* @author Raphael Muenster
*
*/
class CWorldParameters {
  
public: 

/**
*
* Create new CWorldParameters
*
*/
CWorldParameters(); 

~CWorldParameters(); 

int m_iStartType;

int m_iBodies;

int m_iBodyInit;

int m_iTotalTimesteps;

int m_iMaxIterations;

int m_iPipelineIterations;

Real m_dDefaultDensity;

Real m_dDefaultRadius;

//m_DefaultBoundingBox;

std::string m_sSolution;

std::string m_sBodyFile;

VECTOR3 m_vGrav;

std::vector<sRigidBody> m_vRigidBodies;

};

}
#endif
