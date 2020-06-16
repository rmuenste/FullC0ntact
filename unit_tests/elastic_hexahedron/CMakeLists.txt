# add library directories
link_directories(${CMAKE_SOURCE_DIR}/libs)

add_executable(elastic_hexahedron elastic_hexahedron.cpp ovm_vtk_writer.cpp)

file(COPY "elastic_hexahedron.json" DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

if(WIN32)
  # add dependent libraries
  target_link_libraries(elastic_hexahedron OpenVolumeMesh)
  target_link_libraries(elastic_hexahedron ${CGAL_LIBRARIES} ${GMP_LIBRARIES} ${MPFR_LIBRARIES})
  target_compile_definitions(elastic_hexahedron PUBLIC _USE_MATH_DEFINES INCLUDE_TEMPLATES)
else(WIN32)
  # add dependent libraries
  target_link_libraries(elastic_hexahedron OpenVolumeMesh)
  target_link_libraries(elastic_hexahedron ${CGAL_LIBRARIES} ${GMP_LIBRARIES} ${MPFR_LIBRARIES} stdc++fs)
  target_compile_definitions(elastic_hexahedron PUBLIC _USE_MATH_DEFINES INCLUDE_TEMPLATES)
endif(WIN32)

target_include_directories(elastic_hexahedron PUBLIC ${FC_APP_INCLUDE_DIRS})
