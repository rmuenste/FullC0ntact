
# add library directories
link_directories(${CMAKE_SOURCE_DIR}/libs)

# add a binary
add_executable(flagella flagella.cpp)

target_link_libraries(flagella ${FC_APP_LIBRARIES})

# add include directories
target_include_directories(flagella PRIVATE ${FC_APP_INCLUDE_DIRS})





