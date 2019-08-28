# add a binary
add_executable(json_reader json_reader.cpp)

# add dependent libraries
target_link_libraries(json_reader Utility Math inshape3dcore)


# add include directories
target_include_directories(json_reader PUBLIC ${FC_APP_INCLUDE_DIRS})



