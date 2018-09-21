#________________________________________________________________
#
# Inshape3d Library : dem_sim_test0 folder CMakeLists.txt file
# Author            : Raphael Muenster
#________________________________________________________________
#

# add library directories
link_directories(${CMAKE_SOURCE_DIR}/libs)

# add a binary
add_executable(dem_sim_test0 dem_sim_test0.cpp)

if(WIN32)
  # add dependent libraries
  target_link_libraries(dem_sim_test0 ${FC_APP_LIBRARIES})
else(WIN32)
  # add dependent libraries
  target_link_libraries(dem_sim_test0 ${FC_APP_LIBRARIES})
endif(WIN32)

# add include directories
target_include_directories(dem_sim_test0 PRIVATE ${FC_APP_INCLUDE_DIRS})




