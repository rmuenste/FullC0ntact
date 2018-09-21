
# add library directories
link_directories(${CMAKE_SOURCE_DIR}/libs)

# add a binary
add_executable(taylor_line taylor_line.cpp)

target_link_libraries(taylor_line ${FC_APP_LIBRARIES})

# add include directories
target_include_directories(taylor_line PRIVATE ${FC_APP_INCLUDE_DIRS})





