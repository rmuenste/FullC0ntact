# add a binary
add_executable(integration integration.cpp)

# add dependent libraries
target_link_libraries(integration Utility Math inshape3dcore)


# add include directories
target_include_directories(integration PUBLIC ${FC_APP_INCLUDE_DIRS})

ADD_TEST(NAME integration COMMAND ./integration)
