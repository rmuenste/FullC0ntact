
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

target_include_directories(peristaltic PUBLIC ${FC_APP_INCLUDE_DIRS})


