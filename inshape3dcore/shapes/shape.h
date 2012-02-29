#ifndef SHAPE_H_
#define SHAPE_H_

//===================================================
//					INCLUDES
//===================================================
#include <limits>
#include <vector3.h>
#include <mymath.h>
#include <aabb3.h>
#include <matrix3x3.h>

namespace i3d {

/** @brief Base class for all shapes used in a rigid body simulation
 *
 * Base class for all shapes used in a rigid body simulation
 */    
template<class T>
class CShape
{
public:
  
	CShape(void);

	virtual ~CShape(void);

  /**
   * Returns an axis-aligned bounding box for the shape
   */
	virtual CAABB3<T> GetAABB() = 0;

  /**
   * Returns the volume of the shape
   */
	virtual T Volume() const = 0;
  
  /**
   * Returns true if the query point is located on the inside of the shape
   * @param vQuery The point that is going to be tested
   */
  virtual bool PointInside(const CVector3<T> &vQuery) const = 0;

};

typedef CShape<float> CShapef;
typedef CShape<double> CShaped;
typedef CShape<Real> CShaper;

}

#endif
