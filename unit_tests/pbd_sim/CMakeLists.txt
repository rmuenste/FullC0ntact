
# add library directories
link_directories(${CMAKE_SOURCE_DIR}/libs)

add_executable(pbd_unit_sim position_based_dynamics.cpp)

if(WIN32)
  # add dependent libraries
  target_link_libraries(pbd_unit_sim OpenMeshCore OpenMeshTools Utility Math inshape3dcore ${CGAL_LIBRARIES} ${GMP_LIBRARIES} ${MPFR_LIBRARIES})
  target_compile_definitions(pbd_unit_sim PUBLIC _USE_MATH_DEFINES)
else(WIN32)
  # add dependent libraries
  target_link_libraries(pbd_unit_sim OpenMeshCore OpenMeshTools Utility Math inshape3dcore ${CGAL_LIBRARIES} ${GMP_LIBRARIES} ${MPFR_LIBRARIES})
endif(WIN32)

target_include_directories(pbd_unit_sim PUBLIC ${FC_APP_INCLUDE_DIRS})
