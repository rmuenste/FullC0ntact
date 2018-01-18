
# add library directories
link_directories(${CMAKE_SOURCE_DIR}/libs)

# add a binary
add_executable(test0 test0.cpp)

target_link_libraries(test0 ${FC_APP_LIB_DIRS})

# add include directories
target_include_directories(test0 PUBLIC ${FC_APP_INCLUDE_DIRS})


