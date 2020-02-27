
# add library directories
link_directories(${CMAKE_SOURCE_DIR}/libs)

add_executable(pbd_unit_tests position_based_dynamics.cpp mesh_creation.cpp mesh_creation.hpp constraints.hpp)

if(WIN32)
  # add dependent libraries
  target_link_libraries(pbd_unit_tests OpenMeshCore OpenMeshTools Utility Math inshape3dcore ${CGAL_LIBRARIES} ${GMP_LIBRARIES} ${MPFR_LIBRARIES})
  target_compile_definitions(pbd_unit_tests PUBLIC _USE_MATH_DEFINES)
else(WIN32)
  # add dependent libraries
  target_link_libraries(pbd_unit_tests OpenMeshCore OpenMeshTools Utility Math inshape3dcore ${CGAL_LIBRARIES} ${GMP_LIBRARIES} ${MPFR_LIBRARIES})
endif(WIN32)

target_include_directories(pbd_unit_tests PUBLIC ${FC_APP_INCLUDE_DIRS})