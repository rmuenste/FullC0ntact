#ifndef ELLIPSOID_H_
#define ELLIPSOID_H_

//===================================================
//					INCLUDES
//===================================================
#include <limits>
#include <vector3.h>
#include <mymath.h>

namespace i3d {

/** @brief A class for an ellipsoid
 *
 * A class for an ellipsoid and various ellipsoid-related functions
 */  
template<class T>
class Ellipsoid
{
public:
	Ellipsoid(void);

	Ellipsoid(const Vector3<T> &vCenter,T vA,T vB,T vC): 
	 m_vCenter(vCenter),m_vA(vA),m_vB(vB),m_vC(vC)
	{
	}//end

	~Ellipsoid(void);

	inline Vector3<T> eval(T phi, T theta) const
	{
	  return Vector3<T>(m_vCenter.x+m_vA*cos(phi)*cos(theta), 
						 m_vCenter.y+m_vB*cos(phi)*sin(theta), 
						 m_vCenter.z+m_vC*sin(phi));
	}


/** \brief A brief description of Center().
 *
 * A more extensive description of Center().
 * \param aParameter A brief description of aParameter.
 * \return A brief description of what Center() returns.
 */
	inline Vector3<T>& Center() {return m_vCenter;};

/** \brief A brief description of Center().
 *
 * A more extensive description of Center().
 * \param aParameter A brief description of aParameter.
 * \return A brief description of what Center() returns.
 */
	inline const Vector3<T>& Center() const {return m_vCenter;};

/** \brief A brief description of Volume().
 *
 * A more extensive description of Volume().
 * \param aParameter A brief description of aParameter.
 * \return A brief description of what Volume() returns.
 */
	inline T Volume() {return (T)4.0/(T)3.0*CMath<T>::SYS_PI*m_vA*m_vB*m_vC;};

	inline bool Inside(Vector3<T> vQuery)
	{

	  T value=(T)(vQuery.x/m_vA)*(vQuery.x/m_vA)+(vQuery.y/m_vB)*(vQuery.y/m_vB)+(vQuery.z/m_vC)*(vQuery.z/m_vC)-1.0;
	  if(value < 0.0)
	  {
		return true;
	  }
	  else
		return false;

	};

	T m_vA;
	T m_vB;
	T m_vC;
	Vector3<T> m_vCenter;

};

typedef Ellipsoid<float> Ellipsoidf;

}

#endif
