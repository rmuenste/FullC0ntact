
# add library directories
link_directories(${CMAKE_SOURCE_DIR}/libs)

# add a binary
add_executable(friction friction.cpp)

target_link_libraries(friction ${FC_APP_LIBRARIES})

# add include directories
target_include_directories(friction PUBLIC ${FC_APP_INCLUDE_DIRS})



