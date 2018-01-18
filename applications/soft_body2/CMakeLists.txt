#

# add library directories
link_directories(${CMAKE_SOURCE_DIR}/libs)

# add a binary
add_executable(soft_body2 soft_body2.cpp)

target_link_libraries(soft_body2 ${FC_APP_LIB_DIRS})

# add include directories
target_include_directories(soft_body2 PRIVATE ${FC_APP_INCLUDE_DIRS})





