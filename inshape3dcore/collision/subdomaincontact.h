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

#ifndef SUBDOMAINCONTACT_H
#define SUBDOMAINCONTACT_H

#include <mathglobals.h>
#include <vector3.h>
#include <rigidbody.h>
#include <contact.h>

namespace i3d {

/**
* @brief A contact with a subdomain boundary that has special information
*
* A class that stores the detailed contact information with a subdomain boundary
* 
*/
class CSubdomainContact : public Contact
{
public:
  /**
  * Creates a new empty contact
  */
	CSubdomainContact(void);

  /**
  * Copies a contact
  */
	CSubdomainContact(const CSubdomainContact &copy);

	~CSubdomainContact(void);

  int m_iNeighbor;

};

}

#endif
