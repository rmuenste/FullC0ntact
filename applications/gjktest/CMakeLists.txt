
# add library directories
link_directories(${CMAKE_SOURCE_DIR}/libs)

# add a binary
add_executable(gjktest gjktest.cpp)

target_link_libraries(gjktest ${FC_APP_LIBRARIES})

# add include directories
target_include_directories(gjktest PUBLIC ${FC_APP_INCLUDE_DIRS})




