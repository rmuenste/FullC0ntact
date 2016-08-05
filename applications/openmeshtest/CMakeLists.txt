#________________________________________________________________
#
# Inshape3d Library : velocitybased folder CMakeLists.txt file
# Author            : Raphael Muenster
#________________________________________________________________
#

# add include directories
include_directories (. ../../math ../../cuda_addon ../../util ../../inshape3dcore ../../inshape3dcore/collision ../../inshape3dcore/distance ../../inshape3dcore/grid ../../inshape3dcore/fortrancppinterface 
../../inshape3dcore/intersection ../../inshape3dcore/physics ../../inshape3dcore/postprocessing ../../inshape3dcore/preprocessing ../../inshape3dcore/shapes 
../../inshape3dcore/shapes/modelsandmeshes ../../libs/rapidxml-1.13 ../../libs/OpenMesh-6.2/src)

# add library directories
link_directories(${CMAKE_SOURCE_DIR}/libs)

if(FC_CUDA_SUPPORT)
  # add a binary
  cuda_add_executable(openmeshtest openmeshtest.cpp)
else(FC_CUDA_SUPPORT)
  # add a binary
  add_executable(openmeshtest openmeshtest.cpp)
endif(FC_CUDA_SUPPORT)

if(WIN32)
  # add dependent libraries
  target_link_libraries(openmeshtest OpenMeshCore OpenMeshTools Utility Math inshape3dcore)
else(WIN32)
  # add dependent libraries
  target_link_libraries(openmeshtest OpenMeshCore OpenMeshTools Utility Math inshape3dcore)
endif(WIN32)





