
# add library directories
link_directories(${CMAKE_SOURCE_DIR}/libs)

# add a binary
add_executable(trypanosome trypanosome.cpp)

target_link_libraries(trypanosome ${FC_APP_LIBRARIES})

# add include directories
target_include_directories(trypanosome PRIVATE ${FC_APP_INCLUDE_DIRS})




