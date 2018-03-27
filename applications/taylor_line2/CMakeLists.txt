
# add library directories
link_directories(${CMAKE_SOURCE_DIR}/libs)

# add a binary
add_executable(taylor_line2 taylor_line2.cpp)

target_link_libraries(taylor_line2 ${FC_APP_LIB_DIRS})

# add include directories
target_include_directories(taylor_line2 PRIVATE ${FC_APP_INCLUDE_DIRS})





