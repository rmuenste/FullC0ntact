
# add library directories
link_directories(${CMAKE_SOURCE_DIR}/libs)

# add a binary
add_executable(test1 test1.cpp)

target_link_libraries(test1 ${FC_APP_LIBRARIES})

# add include directories
target_include_directories(test1 PUBLIC ${FC_APP_INCLUDE_DIRS})



