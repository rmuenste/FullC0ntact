# add library directories
link_directories(${CMAKE_SOURCE_DIR}/libs)

add_executable(elastic_hexahedron elastic_hexahedron.cpp)

if(WIN32)
  # add dependent libraries
  target_link_libraries(elastic_hexahedron OpenVolumeMesh Utility Math inshape3dcore)
  target_compile_definitions(elastic_hexahedron PUBLIC _USE_MATH_DEFINES)
else(WIN32)
  # add dependent libraries
  target_link_libraries(elastic_hexahedron OpenVolumeMesh Utility Math inshape3dcore)
endif(WIN32)

target_include_directories(elastic_hexahedron PUBLIC ${FC_APP_INCLUDE_DIRS})
