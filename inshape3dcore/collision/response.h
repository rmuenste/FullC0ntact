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

#ifndef RESPONSE_H
#define RESPONSE_H

#include <vector3.h>

namespace i3d {

class Response
{
public:
	Response(void);

	Response(const VECTOR3 &v1,const VECTOR3 &v2);

	Response(const VECTOR3 &v1,const VECTOR3 &v2,int i1, int i2);

	Response(const VECTOR3 &v1,const VECTOR3 &v2, const VECTOR3 &omega1,const VECTOR3 &omega2, int i1, int i2);

	Response(const Response &copy);

	~Response(void);

	//velocity correction vector
	VECTOR3 m_vFW1;
	VECTOR3 m_vFW2;

	//Angular velocity update vector
	VECTOR3 m_vOmega1;
	VECTOR3 m_vOmega2;

	int iID1;
	int iID2;
};

}

#endif
