
# add library directories
link_directories(${CMAKE_SOURCE_DIR}/libs)

# add a binary
add_executable(pardynamics pardynamics.cpp)

target_link_libraries(pardynamics ${FC_APP_LIB_DIRS})

# add include directories
target_include_directories(pardynamics PUBLIC ${FC_APP_INCLUDE_DIRS})



