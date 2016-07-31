#________________________________________________________________
#
# Inshape3d Library : meshtest folder CMakeLists.txt file
# Author            : Raphael Muenster
#________________________________________________________________
#
# add include directories
include_directories (. ../../math ../../util ../../cuda_addon ../../inshape3dcore ../../inshape3dcore/collision ../../inshape3dcore/distance ../../inshape3dcore/grid ../../inshape3dcore/fortrancppinterface 
../../inshape3dcore/intersection ../../inshape3dcore/physics ../../inshape3dcore/postprocessing ../../inshape3dcore/preprocessing ../../inshape3dcore/shapes 
../../inshape3dcore/shapes/modelsandmeshes ../../libs/rapidxml-1.13)

# add library directories
link_directories(${CMAKE_SOURCE_DIR}/libs)

if(FC_CUDA_SUPPORT)
  # add a binary
  cuda_add_executable(meshtest meshtest.cpp)
else(FC_CUDA_SUPPORT)
  # add a binary
  add_executable(meshtest meshtest.cpp)
endif(FC_CUDA_SUPPORT)

# add a binary

if(WIN32)
  # add dependent libraries
  target_link_libraries(meshtest Utility Math inshape3dcore)
else(WIN32)
  # add dependent libraries
  target_link_libraries(meshtest Utility Math inshape3dcore)
endif(WIN32)





