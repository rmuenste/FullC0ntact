#ifndef GEOM_CONFIG_HPP_Z96BD1LO
#define GEOM_CONFIG_HPP_Z96BD1LO

namespace i3d {

  enum
  {
    defaultKernel,
    cgalKernel
  };


#ifdef WITH_CGAL


const int geom_kernel = cgalKernel;
#else
const int geom_kernel = defaultKernel;
#endif

}


#endif /* end of include guard: GEOM_CONFIG_HPP_Z96BD1LO */
