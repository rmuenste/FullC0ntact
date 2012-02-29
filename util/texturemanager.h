/***************************************************************************
 *   Copyright (C) 2006-2009 by Raphael Münster   *
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

#ifndef _TEXTUREMANAGER_H_
#define _TEXTUREMANAGER_H_

//====================================================
//					INCLUDES
//====================================================

//====================================================
//					GLOBALS
//====================================================


//====================================================
//					DEFINES
//====================================================

#define MAX_TEX 100

class C3DModel;

//====================================================
//					CLASS DEFINITION
//====================================================

/** \brief A brief description of MyClass.
 *
 * A more extensive description of MyClass.
 */

class CTextureManager
{
public:

  /** \brief A brief description of CTextureManager().
   *
   * A more extensive description of myProcedure().
   * \param aParameter A brief description of aParameter.
   */
	CTextureManager(void);

  /** \brief A brief description of CTextureManager().
   *
   * A more extensive description of myProcedure().
   */
	~CTextureManager(void);

  /** \brief A brief description of BuildModelTex().
   *
   * A more extensive description of BuildModelTex().
   * \param aParameter A brief description of aParameter.
   * \return A brief description of what BuildModelTex() returns.
   */
	int BuildModelTex(C3DModel &pModel);

  /** \brief A brief description of CreateTexture().
   *
   * A more extensive description of CreateTexture().
   * \param aParameter A brief description of aParameter.
   * \return A brief description of what CreateTexture() returns.
   */
	void CreateTexture(const char* strFileName);

  /** \brief A brief description of CreateTextureSimple().
   *
   * A more extensive description of CreateTextureSimple().
   * \param aParameter A brief description of aParameter.
   * \return A brief description of what CreateTextureSimple() returns.
   */
	void CreateTextureSimple(const char* strFileName);

  /** \brief A brief description of GetTexture().
   *
   * A more extensive description of GetTexture().
   * \param aParameter A brief description of aParameter.
   * \return A brief description of what GetTexture() returns.
   */
	int GetTexture(int iTexID);


	unsigned int m_iTextures[MAX_TEX];
	int m_iCount;

};

#endif