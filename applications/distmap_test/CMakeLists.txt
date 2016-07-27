#________________________________________________________________
#
# Inshape3d Library : gpu_test folder CMakeLists.txt file
# Author            : Raphael Muenster
#________________________________________________________________
#
# add include directories
include_directories (. ../../math ../../cuda_addon ../../util ../../inshape3dcore ../../inshape3dcore/collision ../../inshape3dcore/distance ../../inshape3dcore/grid ../../inshape3dcore/fortrancppinterface 
../../inshape3dcore/intersection ../../inshape3dcore/physics ../../inshape3dcore/postprocessing ../../inshape3dcore/preprocessing ../../inshape3dcore/shapes 
../../inshape3dcore/shapes/modelsandmeshes ../../libs/rapidxml-1.13)

# add library directories
link_directories(${CMAKE_SOURCE_DIR}/libs)

# add a binary
cuda_add_executable(distmap_test distmap_test.cpp)

# add dependent libraries
if(WIN32)
  target_link_libraries(distmap_test Utility Math inshape3dcore difi)
else(WIN32)
  target_link_libraries(distmap_test Utility Math inshape3dcore difi)
endif(WIN32)
