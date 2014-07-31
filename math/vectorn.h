#if !defined(VECTORN_H)
#define VECTORN_H

#include "mymath.h"
#include <string.h>

namespace i3d {

/**
* @brief A class for a nd vector and various nd vector operations
*
* A class for a nd vector and various nd vector operations, intended for use with n > 4
*/    
template<class T>
class VectorN
{
public:
	VectorN(void);

	VectorN(int n, const T *pData);
	VectorN(int n);

	~VectorN(void);

  VectorN(const VectorN<T> &copy)
  {
   if(m_Data != NULL)
     m_Data = new T[copy.m_n];
   else
   {
     delete[] m_Data;
     m_Data = new T[copy.m_n];
   }
   memcpy(m_Data,copy.m_Data,copy.m_n*sizeof(T));
   m_n = copy.m_n;
  }
  
  const VectorN& operator=(const VectorN& v)
  {    
   if(m_Data != NULL)
     m_Data = new T[v.m_n];
   else
   {
     delete[] m_Data;
     m_Data = new T[v.m_n];
   }
   memcpy(m_Data,v.m_Data,v.m_n*sizeof(T));
   m_n = v.m_n;
   return *this;
  }//end  operator  
  
	inline T& operator() (unsigned int n)
	{
		return m_Data[n];
	}//end operator()

  inline void invert() 
  {

    for(int j=0;j<m_n;j++)
      m_Data[j]=-m_Data[j];
  }//end operator

	inline static T CompNorm(VectorN &v0, VectorN &v1)
	{
    T norm = T(0);
    for(int j=0;j<v0.m_n;j++)
    {
      T temp = v0(j)-v1(j);
      temp = temp * temp;
      norm = norm + temp;
    }
		return sqrt(norm);
	}//end  operator

	inline T norm()
	{
    T norm = T(0);
    for(int j=0;j<m_n;j++)
    {
      T temp = m_Data[j] * m_Data[j];
      norm = norm + temp;
    }
		return sqrt(norm);
	}//end  operator

  inline T max()
  {
    T maxElement = fabs(m_Data[0]);

    for(int j=1;j<m_n;j++)
    {
      if( fabs(m_Data[j]) > maxElement )
        maxElement = fabs(m_Data[j]);
    }
    return maxElement;
  }//end  operator


	inline VectorN operator - () 
	{
    
    VectorN<T> cpy = *this;
    for(int j=0;j<m_n;j++)
      cpy.m_Data[j]=-cpy.m_Data[j];
    return cpy;
	}//end operator

  inline void SetZero()
  {
    memset(m_Data,m_n,sizeof(T));
  }

	void OutputVector();

	T *m_Data;
	int m_n;

};

typedef VectorN<double> VectorNd;
typedef VectorN<float>  VectorNf;
typedef VectorN<Real> VectorNr;

}

#endif
