
# add library directories
link_directories(${CMAKE_SOURCE_DIR}/libs)

# add a binary
add_executable(boxmesher boxmesher.cpp)

target_link_libraries(boxmesher ${FC_APP_LIBRARIES})

# add include directories
target_include_directories(boxmesher PUBLIC ${FC_APP_INCLUDE_DIRS})

