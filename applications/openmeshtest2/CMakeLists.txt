
# add library directories
link_directories(${CMAKE_SOURCE_DIR}/libs)

if(FC_CUDA_SUPPORT)
  # add a binary
  cuda_add_executable(openmeshtest2 openmeshtest2.cpp)
else(FC_CUDA_SUPPORT)
  # add a binary
  add_executable(openmeshtest2 openmeshtest2.cpp)
endif(FC_CUDA_SUPPORT)

if(WIN32)
  # add dependent libraries
  target_link_libraries(openmeshtest2 OpenMeshCore OpenMeshTools Utility Math inshape3dcore)
  target_compile_definitions(openmeshtest2 PUBLIC _USE_MATH_DEFINES)
else(WIN32)
  # add dependent libraries
  target_link_libraries(openmeshtest2 OpenMeshCore OpenMeshTools Utility Math inshape3dcore)
endif(WIN32)

target_include_directories(openmeshtest2 PUBLIC ${FC_APP_INCLUDE_DIRS})


