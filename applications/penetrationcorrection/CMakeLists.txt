
# add library directories
link_directories(${CMAKE_SOURCE_DIR}/libs)

# add a binary
add_executable(penetrationcorrection penetrationcorrection.cpp)

target_link_libraries(penetrationcorrection ${FC_APP_LIBRARIES})

# add include directories
target_include_directories(penetrationcorrection PUBLIC ${FC_APP_INCLUDE_DIRS})

