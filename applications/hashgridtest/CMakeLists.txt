#________________________________________________________________
#
# Inshape3d Library : meshmeshtest folder CMakeLists.txt file
# Author            : Raphael Muenster
#________________________________________________________________
#
# add include directories
include_directories (. ../../math ../../util ../../inshape3dcore ../../inshape3dcore/collision ../../inshape3dcore/distance ../../inshape3dcore/grid ../../inshape3dcore/fortrancppinterface 
../../inshape3dcore/intersection ../../inshape3dcore/physics ../../inshape3dcore/postprocessing ../../inshape3dcore/preprocessing ../../inshape3dcore/shapes 
../../inshape3dcore/shapes/modelsandmeshes ../../libs/rapidxml-1.13 ../../cuda_addon)

# add library directories
link_directories(${CMAKE_SOURCE_DIR}/libs)

if(FC_CUDA_SUPPORT)
  # add a binary
  cuda_add_executable(hashgridtest hashgridtest.cpp)

  if(WIN32)
    # add dependent libraries
    target_link_libraries(hashgridtest Utility Math inshape3dcore difi)
  else(WIN32)
    # add dependent libraries
    target_link_libraries(hashgridtest Utility Math inshape3dcore difi)
  endif(WIN32)

endif(FC_CUDA_SUPPORT)




