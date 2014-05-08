/***************************************************************************
 *   Copyright (C) 2006 by Raphael M�nster   *
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

#include "vector3.h"
#include "matrix4x4.h"
#include "vector4.h"
#include "quaternion.h"
#include "Torus.h"


int main()
{

	CVector2f vec2(4,3);

	Vector3f vec3(2,2,1);

	Vector3f vec31(1,1,1);

	vec3 += vec31;

	vec3.x = 0.0;

	cout<<vec3 * vec31<<endl;

	cout << vec2;

	cout <<vec2.mag()<<endl;

	cout<<vec2 + CVector2f(4,1);

	cout << vec3;

	cout <<vec3.mag()<<endl;

	cout<<vec3 + Vector3f(1,0,0);

	Quaternionf q;

	CMatrix4x4f matrix;

	q.CreateMatrix(matrix);

	q.AxisAngleToQuat(1,0,0, 45);

	CTorus torus(50,50);
	cout<<torus.Size()<<endl;

	


	return 1;
}