
# add library directories
link_directories(${CMAKE_SOURCE_DIR}/libs)

# add a binary
add_executable(ode-app ode-app.cpp)

if(WIN32)
  # add dependent libraries
  target_link_libraries(ode-app Utility Math inshape3dcore ode)
else(WIN32)
  # add dependent libraries
  target_link_libraries(ode-app Utility Math inshape3dcore ode pthread)
endif(WIN32)

# add include directories
target_include_directories(ode-app PRIVATE ${FC_APP_INCLUDE_DIRS})

set(ODE_DIR ${CMAKE_SOURCE_DIR}/libs/ode-cmake)

target_include_directories(ode-app PRIVATE ${ODE_DIR} 
  ${ODE_DIR}/include ${ODE_DIR}/ou/include  
  ${ODE_DIR}/OPCODE ${ODE_DIR}/ode/src 
  ${ODE_DIR}/libccd/src)
target_compile_definitions(ode-app PRIVATE dDOUBLE=1 CCD_DOUBLE=1 WITH_ODE=1)

