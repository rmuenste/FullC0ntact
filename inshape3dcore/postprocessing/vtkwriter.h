/****************************************************************************
**
** Copyright (C) 2005-2007 Trolltech ASA. All rights reserved.
**
** This file is part of the example classes of the Qt Toolkit.
**
** This file may be used under the terms of the GNU General Public
** License version 2.0 as published by the Free Software Foundation
** and appearing in the file LICENSE.GPL included in the packaging of
** this file.  Please review the following information to ensure GNU
** General Public Licensing requirements will be met:
** http://www.trolltech.com/products/qt/opensource.html
**
** If you are unsure which license is appropriate for your use, please
** review the following information:
** http://www.trolltech.com/products/qt/licensing.html or contact the
** sales department at sales@trolltech.com.
**
** This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
** WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
**
****************************************************************************/

#ifndef _CVTKWRITER_H
#define _CVTKWRITER_H


#include "unstructuredgrid.h"
#include <3dmodel.h>
#include <vector>
#include <list>
#include <collisioninfo.h>
#include <response.h>
#include <boundingvolumetree3.h>
#include <traits.h> 
#include <rigidbody.h>
#include <limits>

namespace i3d {

//Forward declaration
template<class BV, class T, class Traits> class CBoundingVolumeNode3;

/**
 * @brief The CVtkWriter class outputs simulation data in several vtk-formats
 */
class CVtkWriter
{
public:

	/** \brief Standard constructor CVtkWriter().
	 *
	 */
	CVtkWriter(void);

	/** \brief Deconstructor ~CVtkWriter().
	 *
	 */
	~CVtkWriter(void);

	/** \brief WriteUnstr produces a .vtk file for Paraview
	 *
	 * Writes a vtk legacy file.
	 * \param Grid The grid to be written
	 * \param strFileName Specifies the filename
	 */
	void WriteUnstr(CUnstrGrid &Grid, const char *strFileName);
  
  void WriteUnstr(CUnstrGrid &Grid,std::vector<Real> &element, const char *strFileName);  
	
	void WriteUnstrXML(CUnstrGrid &Grid, const char * strFileName);

	//thir routine writes a pvtu xml file.
	//this file format is used to combine the single pieces of a parallel simulation
	//into a whole
	void WritePVTU(int iSubNodes, int iTimestep);

	//this routine writes an xml vtu file for paraview:
	//this output format is used to output the solution of the nodes in a parallel simulation
  void WritePUXML(int iNEL,int iNVT,int iKVERT[][8],double dcorvg[][3],double vu[],double vv[],double vw[],double vp[],double dist[],int inode, int iTimestep);
	
	/** \brief A brief description of myProcedure().
	 *
	 * WriteModel() writes a single Model to a vtk poly file in legacy format.
	 * \param pModel A brief description of aParameter.
	 * \param strFileName A brief description of aParameter.
	 */
	void WriteModel(C3DModel &pModel,const char *strFileName);

	void WriteModels(std::vector<C3DModel> &pModels,const char *strFileName);
	
	void WriteRigidBodies(std::vector<CRigidBody*> &pRigidBodies,const char *strFileName);

	void WriteRigidBodiesEx(std::vector<COBB3r*> &pRigidBodies,const char *strFileName);

	void WriteSolids(std::vector<C3DModel> &pSolids,const char *strFileName);

	void WriteModels(std::vector<C3DModel> &pModels,std::list<CCollisionInfo> &vCollInfo,
									 std::vector<VECTOR3> &vVel,std::list<CResponse> &Responses, const char *strFileName);
									 
	void WriteTreeLevel(std::vector<CBoundingVolumeNode3<CAABB3r,Real,CTraits> *> &vec, const char *strFileName); 
	
	void WriteBasf(std::vector<C3DModel> &pModels,const char *strFileName);

  void WriteContacts(std::vector<CContact> & vContacts, const char* strFileName);

  void WriteVTKVertexArrayScalar();
  
  void WriteVTK22(int *NEL,int *NVT,int iKVERT[][8],double dcorvg[][3],double dmon1[],double dmon2[],double df[],double du[],double dgradx[],double dgrady[],double dgradz[],double *dt, double *ddt,int ivl, int imst, int itst,int ismst);

  void WriteVTK23(int *NEL,int *NVT, int iKVERT[][8],double dcorvg[][3],
double dmon[],double dsize[],double dratio[],double *DT,double *DDT,int ivl,int imst,int itst,int ismst);
  
  void WriteTriFile(int NEL, int NVT, int iKVERT[][8], double dcorvg[][3], int id);
  
};

}

#endif
