
# add library directories
link_directories(${CMAKE_SOURCE_DIR}/libs)

if(NOT FC_CUDA_SUPPORT)
# add a binary
add_executable(gridgeneration gridgeneration.cpp)
else(NOT FC_CUDA_SUPPORT)
# add a binary
cuda_add_executable(gridgeneration gridgeneration.cpp)
endif(NOT FC_CUDA_SUPPORT)

if(WIN32)
  # add dependent libraries
  target_link_libraries(gridgeneration Utility Math inshape3dcore difi)
else(WIN32)
  # add dependent libraries
  target_link_libraries(gridgeneration Utility Math inshape3dcore difi)
endif(WIN32)

# add include directories
target_include_directories (gridgeneration PUBLIC . ../../math ../../cuda_addon ../../util ../../inshape3dcore ../../inshape3dcore/collision ../../inshape3dcore/distance ../../inshape3dcore/grid ../../inshape3dcore/fortrancppinterface 
../../inshape3dcore/intersection ../../inshape3dcore/physics ../../inshape3dcore/postprocessing ../../inshape3dcore/preprocessing ../../inshape3dcore/shapes 
../../inshape3dcore/shapes/modelsandmeshes ../../libs/rapidxml-1.13)

