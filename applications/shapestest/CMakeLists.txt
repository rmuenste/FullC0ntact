
# add library directories
link_directories(${CMAKE_SOURCE_DIR}/libs)

# add a binary
add_executable(shapestest shapestest.cpp)

target_link_libraries(shapestest ${FC_APP_LIB_DIRS})

# add include directories
target_include_directories(shapestest PUBLIC ${FC_APP_INCLUDE_DIRS})


