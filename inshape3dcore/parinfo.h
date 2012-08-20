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
#ifndef PARINFO_H
#define PARINFO_H



//===================================================
//                     INCLUDES
//===================================================
#include <string>
#include <vector3.h>
#include <vector>
#include <list>
#include <rigidbodyio.h>
#ifdef FC_MPI_SUPPORT
#include <mpi.h>
#endif

namespace i3d {

///@cond HIDDEN_SYMBOLS
class CGroupInfo {
public:
  int m_iRoot;
  int m_iSize;
};
///@cond

///@cond HIDDEN_SYMBOLS
class CompareGroups
{
public:
  bool operator()(CGroupInfo g1, CGroupInfo g2)
  {
    if(g1.m_iRoot < g2.m_iRoot)
      return true;
    else
      return false;
  }
};
///@cond

/**
* @brief A class that contains the parallel info structures
*
* A class that contains information about the current process in
*   a parallel simulation
* @author Raphael Muenster
*
*/
class CParInfo {
  
public: 

/**
*
* Create new CParInfo
*
*/
CParInfo(); 

~CParInfo(); 

void SetID(int myid) {m_iMyID = myid; };
int  GetID() {return m_iMyID;};

int m_iMyID;
int m_iNumNeighbors;

#ifdef FC_MPI_SUPPORT
MPI_Group m_Neighbors;
int m_iGroupRank;
MPI_Comm m_NeighComm;

//all my group + root of each group
//an ordered list of the groups this partition is in
std::vector<CGroupInfo> m_Groups;
MPI_Group *m_AllGroups;
MPI_Comm  *m_AllComm;
#endif

};

}
#endif
