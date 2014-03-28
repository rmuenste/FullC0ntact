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

#ifndef SUBDOMAIN_H
#define SUBDOMAIN_H
#include<vector>
#include <traits.h>
#include <rigidbody.h>
#include <boundarybox.h>
#include <parinfo.h>
#include <aabb3.h>

namespace i3d {

/**
* @brief A class that describes a subdomain in a domain decomposition
*
* 
* 
*
*/
class SubDomain
{
public:

  SubDomain();

  SubDomain(int id, const CAABB3r &boundary);

  ~SubDomain();

  SubDomain(const SubDomain &copy);

  /**
   * A describtion of the boundary of the simulation domain
   **/
	CAABB3r m_boxBoundary;

  /**
   * rank of the subdomain
   **/
  int m_iID;

  /**
   * When the simulation is run in parallel, we store
   * the parallel info in this structure
   **/
  CParInfo m_myParInfo;
  
};

}

#endif // SUBDOMAIN_H
