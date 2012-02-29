/***************************************************************************
 *   Copyright (C) 2009 by Raphael Münster   *
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


#ifndef _BITMAP_24_H
#define _BITMAP_24_H

//===================================================
//					INCLUDES
//===================================================
#include <windows.h>

//===================================================
//					DEFINES	
//===================================================
#define WRONG_IMAGE_SIZE 0
#define SUCCESS          1
#define WRONG_FILE_SIZE  2

/** \brief A brief description of CBitmap24.
 *
 * A more extensive description of CBitmap24.
 */
class CBitmap24
{
public:
	/** \brief A brief description of CBitmap24().
	 *
	 * A more extensive description of CBitmap24().
	 * \param aParameter A brief description of aParameter.
	 * \return A brief description of what CBitmap24() returns.
	 */
	CBitmap24(void);
	/** \brief A brief description of CBitmap24(const char* strFileName)().
	 *
	 * A more extensive description of CBitmap24(const char* strFileName)().
	 * \param aParameter A brief description of aParameter.
	 * \return A brief description of what CBitmap24(const char* strFileName)() returns.
	 */
	CBitmap24(const char* strFileName);
	/** \brief A brief description of ~CBitmap24().
	 *
	 * A more extensive description of ~CBitmap24().
	 * \param aParameter A brief description of aParameter.
	 * \return A brief description of what ~CBitmap24() returns.
	 */
	~CBitmap24(void);

public:
	/** \brief A brief description of LoadFromFile().
	 *
	 * A more extensive description of LoadFromFile().
	 * \param aParameter A brief description of aParameter.
	 * \return A brief description of what LoadFromFile() returns.
	 */
	int LoadFromFile(const char* strFileName);
    /** \brief A brief description of m_Colors.
     *
     * A more extensive description of m_Colors.
     */
	RGBQUAD* m_Colors;
    /** \brief A brief description of m_PixelData.
     *
     * A more extensive description of m_PixelData.
     */
	BYTE*    m_PixelData;
    /** \brief A brief description of m_bLoaded.
     *
     * A more extensive description of m_bLoaded.
     */
	bool     m_bLoaded;
    /** \brief A brief description of m_lWidth.
     *
     * A more extensive description of m_lWidth.
     */
	LONG     m_lWidth;
    /** \brief A brief description of m_lHeight.
     *
     * A more extensive description of m_lHeight.
     */
	LONG     m_lHeight;

private:
	BITMAPFILEHEADER bmfh;
	BITMAPINFOHEADER bmih;

};

#endif