
# add library directories
link_directories(${CMAKE_SOURCE_DIR}/libs)

if(FC_CUDA_SUPPORT)
  # add a binary
  cuda_add_executable(openmeshtest4 openmeshtest4.cpp)
else(FC_CUDA_SUPPORT)
  # add a binary
  add_executable(openmeshtest4 openmeshtest4.cpp)
endif(FC_CUDA_SUPPORT)

if(WIN32)
  # add dependent libraries
  target_link_libraries(openmeshtest4 OpenMeshCore OpenMeshTools Utility Math inshape3dcore ${CGAL_LIBRARIES} ${GMP_LIBRARIES} ${MPFR_LIBRARIES})
  target_compile_definitions(openmeshtest4 PUBLIC _USE_MATH_DEFINES)
else(WIN32)
  # add dependent libraries
  target_link_libraries(openmeshtest4 OpenMeshCore OpenMeshTools Utility Math inshape3dcore ${CGAL_LIBRARIES} ${GMP_LIBRARIES} ${MPFR_LIBRARIES})
endif(WIN32)

target_include_directories(openmeshtest4 PUBLIC ${FC_APP_INCLUDE_DIRS})



