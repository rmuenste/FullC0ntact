
# add library directories
link_directories(${CMAKE_SOURCE_DIR}/libs)

# add a binary
add_executable(inclinedplane inclinedplane.cpp)

target_link_libraries(inclinedplane ${FC_APP_LIB_DIRS})

# add include directories
target_include_directories(inclinedplane PUBLIC ${FC_APP_INCLUDE_DIRS})

