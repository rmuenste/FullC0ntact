
# add library directories
link_directories(${CMAKE_SOURCE_DIR}/libs)

# add a binary
add_executable(ode-test ode-test.cpp)

if(WIN32)
  # add dependent libraries
  target_link_libraries(ode-test Utility Math inshape3dcore ode)
else(WIN32)
  # add dependent libraries
  target_link_libraries(ode-test Utility Math inshape3dcore ode pthread)
endif(WIN32)

# add include directories
target_include_directories(ode-test PRIVATE ${FC_APP_INCLUDE_DIRS})

set(ODE_DIR ${CMAKE_SOURCE_DIR}/libs/ode-0.13-cmake)

target_include_directories(ode-test PRIVATE ${ODE_DIR} 
  ${ODE_DIR}/include ${ODE_DIR}/ou/include  
  ${ODE_DIR}/OPCODE ${ODE_DIR}/ode/src 
  ${ODE_DIR}/libccd/src)
target_compile_definitions(ode-test PRIVATE dDOUBLE=1 CCD_DOUBLE=1 WITH_ODE=1)

