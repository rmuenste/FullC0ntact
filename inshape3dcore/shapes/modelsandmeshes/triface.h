/***************************************************************************
 *   Copyright (C) 2006 by Raphael Münster   *
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

#ifdef WIN32
#pragma once
#endif

#ifndef _TRIFACE_H_
#define _TRIFACE_H_

//===================================================
//					INCLUDES
//===================================================

namespace i3d {

/**
 * @brief A class that stores a triangle as 3 indices into a vertex array
 */  
class TriFace
{
public:
	TriFace(void);

	TriFace(const TriFace& f);

	TriFace(int vertexIndex[3]);

	~TriFace(void);

	int operator[](int number) const
	{
		return m_VertIndices[number];
	};//end operator

	inline void SetIndex(int idx, int number)
	{
		m_VertIndices[idx]=number;
	};

	inline const TriFace& operator=(const TriFace& f)
	{
		
		for(int i=0;i<3;i++)
		{
			m_VertIndices[i]=f[i];
		}//end for

		return *this;
	};//end  operator

	void InitFace(const int vertexIndex[3]);

	void InitFace(int vertexIndex[3]);

  int m_VertIndices[3];
  int m_TexIndices[3];

private:
	
};

}
#endif
