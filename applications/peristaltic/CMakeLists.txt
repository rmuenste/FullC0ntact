
# add library directories
link_directories(${CMAKE_SOURCE_DIR}/libs)

if(FC_CUDA_SUPPORT)
  # add a binary
  cuda_add_executable(peristaltic peristaltic.cpp)
else(FC_CUDA_SUPPORT)
  # add a binary
  add_executable(peristaltic peristaltic.cpp)
endif(FC_CUDA_SUPPORT)

if(WIN32)
  # add dependent libraries
  target_link_libraries(peristaltic OpenMeshCore OpenMeshTools Utility Math inshape3dcore)
  target_compile_definitions(peristaltic PUBLIC _USE_MATH_DEFINES)
else(WIN32)
  # add dependent libraries
  target_link_libraries(peristaltic OpenMeshCore OpenMeshTools Utility Math inshape3dcore)
endif(WIN32)

target_include_directories (peristaltic PUBLIC . ../../math ../../cuda_addon ../../util ../../inshape3dcore ../../inshape3dcore/collision ../../inshape3dcore/distance ../../inshape3dcore/grid ../../inshape3dcore/fortrancppinterface 
../../inshape3dcore/intersection ../../inshape3dcore/physics ../../inshape3dcore/postprocessing ../../inshape3dcore/preprocessing ../../inshape3dcore/shapes 
../../inshape3dcore/shapes/modelsandmeshes ../../libs/rapidxml-1.13 ../../libs/OpenMesh/src)




