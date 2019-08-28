# add a binary
add_executable(xml_reader xml_reader.cpp)

# add dependent libraries
target_link_libraries(xml_reader Utility Math inshape3dcore)


# add include directories
target_include_directories(xml_reader PUBLIC ${FC_APP_INCLUDE_DIRS})



