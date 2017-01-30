#________________________________________________________________
#
# Inshape3d Library : gridgeneration folder CMakeLists.txt file
# Author            : Raphael Muenster
#________________________________________________________________
#
# add include directories
include_directories (. ../../math ../../cuda_addon ../../util ../../inshape3dcore ../../inshape3dcore/collision ../../inshape3dcore/distance ../../inshape3dcore/grid ../../inshape3dcore/fortrancppinterface 
../../inshape3dcore/intersection ../../inshape3dcore/physics ../../inshape3dcore/postprocessing ../../inshape3dcore/preprocessing ../../inshape3dcore/shapes 
../../inshape3dcore/shapes/modelsandmeshes ../../libs/rapidxml-1.13)

# add library directories
link_directories(${CMAKE_SOURCE_DIR}/libs)

if(NOT FC_CUDA_SUPPORT)
# add a binary
add_executable(q2p1prj2vtm multiblockgrid.cpp)
else(NOT FC_CUDA_SUPPORT)
# add a binary
cuda_add_executable(q2p1prj2vtm multiblockgrid.cpp)
endif(NOT FC_CUDA_SUPPORT)

if(WIN32)
  # add dependent libraries
  target_link_libraries(q2p1prj2vtm Utility Math inshape3dcore)
else(WIN32)
  # add dependent libraries
  target_link_libraries(q2p1prj2vtm Utility Math inshape3dcore)
endif(WIN32)






