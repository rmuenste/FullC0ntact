#ifndef _CTRIANGULATOR_H_
#define _CTRIANGULATOR_H_

#include <sphere.h>
#include <mymath.h>
#include <ellipsoid.h>
#include <obb3.h>
#include <3dmodel.h>

namespace i3d {

/**
 * @brief A class that creates surfaces triangulations for different shapes
 */  
template<class T,class shape>
class CTriangulator
{
public:
  CTriangulator(void);
  ~CTriangulator(void);

  Model3D Triangulate(const shape &pShape);

};

template<class T,class shape>
CTriangulator<T,shape>::CTriangulator(void)
{
}

template<class T,class shape>
CTriangulator<T,shape>::~CTriangulator(void)
{
}

}

#endif
