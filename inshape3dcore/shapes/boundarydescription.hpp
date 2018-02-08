#ifndef BOUNDARYDESCRIPTION_HPP_DXJ7F5IK
#define BOUNDARYDESCRIPTION_HPP_DXJ7F5IK
#include <boundaryshape.hpp>
#include <vector>

namespace i3d
{

  template<int geomBackend>
  class BoundaryDescription
  {
  public:

    BoundaryDescription() = default;

    virtual ~BoundaryDescription();

    std::vector< BasicBoundaryShape<geomBackend> > boundaryShapes_;

  private:
    /* data */

  };

#ifdef WITH_CGAL
  template<>
  class BoundaryDescription<cgalKernel>
  {
  public:

    BoundaryDescription() = default;

    virtual ~BoundaryDescription()
    {
      for(auto &s : boundaryShapes_)
      {
        delete s;
      }
    };

    void addBoundaryShape(BasicBoundaryShape<cgalKernel> *shape)
    {
      boundaryShapes_.push_back(shape);
    }

    std::vector<BasicBoundaryShape<cgalKernel>*> boundaryShapes_;

  private:
    /* data */

  };
#endif

} /* i3d */ 

#endif /* end of include guard: BOUNDARYDESCRIPTION_HPP_DXJ7F5IK */
