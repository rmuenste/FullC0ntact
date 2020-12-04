
# add library directories
link_directories(${CMAKE_SOURCE_DIR}/libs)

# add a binary
add_executable(fbm_test3 fbm_test3.cpp)

target_link_libraries(fbm_test3 ${FC_APP_LIBRARIES})

# add include directories
target_include_directories(fbm_test3 PUBLIC ${FC_APP_INCLUDE_DIRS})

