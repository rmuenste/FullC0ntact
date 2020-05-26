# add library directories
link_directories(${CMAKE_SOURCE_DIR}/libs)

add_executable(ovm simple_mesh.cpp)

if(WIN32)
  # add dependent libraries
  target_link_libraries(ovm OpenVolumeMesh)
  target_compile_definitions(ovm PUBLIC _USE_MATH_DEFINES)
else(WIN32)
  # add dependent libraries
  target_link_libraries(ovm OpenVolumeMesh)
endif(WIN32)

target_include_directories(ovm PUBLIC ${FC_APP_INCLUDE_DIRS})
