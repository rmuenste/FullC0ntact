# add library directories
link_directories(${CMAKE_SOURCE_DIR}/libs)

add_executable(mesh_converter mesh_converter.cpp)

#file(COPY "elastic_hexahedron.json" DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

# add dependent libraries
target_link_libraries(mesh_converter OpenVolumeMesh Utility Math inshape3dcore)

target_compile_definitions(mesh_converter PUBLIC _USE_MATH_DEFINES INCLUDE_TEMPLATES)

target_include_directories(mesh_converter PUBLIC ${FC_APP_INCLUDE_DIRS})
