
# add library directories
link_directories(${CMAKE_SOURCE_DIR}/libs)

add_executable(pbd_unit_sim position_based_dynamics.cpp mesh_creation.cpp mesh_creation.hpp constraints.hpp)

if(WIN32)
  # add dependent libraries
  target_link_libraries(pbd_unit_sim OpenMeshCore OpenMeshTools Utility Math inshape3dcore)
  target_compile_definitions(pbd_unit_sim PUBLIC _USE_MATH_DEFINES)
else(WIN32)
  # add dependent libraries
  target_link_libraries(pbd_unit_sim OpenMeshCore OpenMeshTools Utility Math inshape3dcore)
endif(WIN32)

target_include_directories(pbd_unit_sim PUBLIC ${FC_APP_INCLUDE_DIRS})
