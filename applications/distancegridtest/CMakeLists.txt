
# add a binary
add_executable(distancegridtest distancegridtest.cpp)

# add dependent libraries
target_link_libraries(distancegridtest ${FC_APP_LIBRARIES})

# add include directories
target_include_directories(distancegridtest PRIVATE ${FC_APP_INCLUDE_DIRS})



