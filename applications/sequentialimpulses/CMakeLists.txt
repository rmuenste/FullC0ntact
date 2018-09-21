# add library directories
link_directories(${CMAKE_SOURCE_DIR}/libs)

# add a binary
add_executable(sequentialimpulses sequentialimpulses.cpp)

target_link_libraries(sequentialimpulses ${FC_APP_LIBRARIES})

# add include directories
target_include_directories (sequentialimpulses PRIVATE ${FC_APP_INCLUDE_DIRS}) 






