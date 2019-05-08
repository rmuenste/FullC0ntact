
# add library directories
link_directories(${CMAKE_SOURCE_DIR}/libs)

if(FC_CUDA_SUPPORT)
  # add a binary
  cuda_add_executable(openmeshtest3 openmeshtest3.cpp)
else(FC_CUDA_SUPPORT)
  # add a binary
  add_executable(openmeshtest3 openmeshtest3.cpp)
endif(FC_CUDA_SUPPORT)

if(WIN32)
  # add dependent libraries
  target_link_libraries(openmeshtest3 OpenMeshCore OpenMeshTools Utility Math inshape3dcore)
  target_compile_definitions(openmeshtest3 PUBLIC _USE_MATH_DEFINES)
else(WIN32)
  # add dependent libraries
  target_link_libraries(openmeshtest3 OpenMeshCore OpenMeshTools Utility Math inshape3dcore)
endif(WIN32)

target_include_directories(openmeshtest3 PUBLIC ${FC_APP_INCLUDE_DIRS})


