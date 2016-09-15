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
#include <rigidbodyio.h>
#ifdef FC_MPI_SUPPORT
#include <mpi.h>
#endif

namespace i3d {

/**
* @brief A class that contains the parallel info structures
*
* A class that contains information about the current process in
*   a parallel simulation
* @author Raphael Muenster
*
*/
  class ParInfo {

    public: 

      /**
       *
       * Create new CParInfo
       *
       */
      ParInfo(); 

      ~ParInfo(); 

      void setId(int myid) {id_ = myid; };
      int  getId() {return id_;};

      int id_;
      int nNeighbors_;

#ifdef FC_MPI_SUPPORT
      MPI_Group m_Neighbors;
      int m_iGroupRank;
      MPI_Comm m_NeighComm;
#endif

  };

}
#endif
