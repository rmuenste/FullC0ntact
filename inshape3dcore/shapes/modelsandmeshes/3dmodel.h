#ifndef _3DMODEL_H_
#define _3DMODEL_H_

//===================================================
//			INCLUDES
//===================================================


#include "3dmesh.h"
#include <vector>
#include <cstdlib>
#include <vector3.h>
#include <dynamicarray.h>
#include <aabb3.h>

namespace i3d {

  typedef std::vector<Mesh3D>::iterator MeshIter;

  ///@cond HIDDEN_SYMBOLS
  //forward definition of the mesh class
  //class C3DMesh;
  // This holds the information for a material.  It may be a texture map of a color.
  // Some of these are not used, but I left them because you will want to eventually
  // read in the UV tile ratio and the UV tile offset for some models.
  struct tMaterialInfo
  {
    char  strName[255];						// The texture name
    char  strFile[255];						// The texture file name (If this is set it's a texture map)
    unsigned char  color[3];				// The color of the object (R, G, B)
    int   texureId;							// the texture ID
    double uTile;							// u tiling of texture  (Currently not used)
    double vTile;							// v tiling of texture	(Currently not used)
    double uOffset;						    // u offset of texture	(Currently not used)
    double vOffset;							// v offset of texture	(Currently not used)
  };
  ///@cond 

  /** \brief A 3D triangular mesh that supports sub-objects
   *
   * A 3D triangular mesh that supports sub-objects, the sub-objects themselves are
   * instances of the C3DMesh class.
   *
   */
  class Model3D
  {
    public:

      /** \brief Creates an empty Model3D
       *
       * Creates an empty Model3D
       */
      Model3D(void);

      /**
       *
       * Copies a Model3D
       *
       */
      Model3D(const Model3D &pModel);

      /** 
       *
       */
      ~Model3D(void);

      void CreateFrom(std::vector<VECTOR3 > &vVertices,  std::vector<TriFace> &vFaces);

      /** \brief Returns the number of materials.
       *
       * Returns the number of materials
       * \return Number of materials
       */
      inline void IncNumMaterials(){numMaterials_++;};

      /** \brief A brief description of myProcedure().
       *
       * A more extensive description of myProcedure().
       * \param aParameter A brief description of aParameter.
       * \return A brief description of what myProcedure() returns.
       */
      inline void BuildVertexNormals()
      {
        for(int i=0;i<(int)meshes_.size();i++)
        {
          meshes_[i].calcVertexNormals();
        }//end for
      };

      /** 
       *
       * Generates a bounding box that contains all sub-objects
       *
       */
      void GenerateBoundingBox();

      /** \brief Returns the bounding box.
       * \return Returns the bounding box..
       */
      inline const AABB3r& GetBox() {return box_;}

      /** \brief A brief description of myProcedure().
       *
       * A more extensive description of myProcedure().
       * \param aParameter A brief description of aParameter.
       * \return A brief description of what myProcedure() returns.
       */
      void AddMaterial(tMaterialInfo& pMatInfo);

      // Outputs the most important data of the model in a structered way
      /** \brief A brief description of myProcedure().
       *
       * A more extensive description of myProcedure().
       * \param aParameter A brief description of aParameter.
       * \return A brief description of what myProcedure() returns.
       */
      void OutputModelInfo(void);
      void BuildVertexArrays(void);

      // Generates a triangle vector
      /** \brief The routine builds a vector that contains the coordinates of each triangle
       *
       * A more extensive description of myProcedure().
       * \param aParameter A brief description of aParameter.
       * \return A brief description of what myProcedure() returns.
       */
      std::vector<Triangle3r> GenTriangleVector();


      MeshIter begin();
      MeshIter end();

      std::vector<tMaterialInfo> materials_;
      int                        numMaterials_;
      std::vector<Mesh3D>       meshes_;
      Real                       radius_; //Radius of a bounding sphere
      AABB3r                    box_;


  };

}

#endif
