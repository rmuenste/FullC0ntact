
# add library directories
link_directories(${CMAKE_SOURCE_DIR}/libs)

# add a binary
add_executable(fbm_test2 fbm_test2.cpp)

target_link_libraries(fbm_test2 ${FC_APP_LIBRARIES})

# add include directories
target_include_directories(fbm_test2 PUBLIC ${FC_APP_INCLUDE_DIRS})

