#ifndef RBODY_HPP_AEZ6CS01
#define RBODY_HPP_AEZ6CS01

#include <managed.hpp>
#include <vector3.h>

namespace i3d
{

  template <typename T, int memory>
    class Rbody : public Managed
    {

      public:

        int nparticles_;
        int indices_[3];

        Vector3<T> com_;
        Vector3<T> vel_;

    };

} /* i3d */ 

#endif /* end of include guard: RBODY_HPP_AEZ6CS01 */
