#ifndef TRAITS_H
#define TRAITS_H

#include <triangle3.h>
#include <vector>

namespace i3d {

/**
 * @brief Example class for mesh traits
 */  
class CTraits
{
public:
	CTraits(void);
	~CTraits(void);
	std::vector<CTriangle3r> m_vTriangles;
	int iCollision;
  void Init()
  {
    iCollision=0;
  };
};

}

#endif
