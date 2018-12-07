# cmake version
cmake_minimum_required(VERSION 3.2)
CMAKE_POLICY(SET CMP0054 NEW)

# name of the project
PROJECT(FullC0ntact)

# allow easier if-else syntax
SET(CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS TRUE)

# check if the project is compiled as a sub-project
if (${CMAKE_PROJECT_NAME} STREQUAL ${PROJECT_NAME})
  MESSAGE(STATUS "Configuring FullC0ntact as a Standalone-Project")
  include(./cmake_scripts/ConfigureAsProject.cmake)
else()
  MESSAGE(STATUS "Configuring FullC0ntact as a Sub-Project")
  include(./cmake_scripts/ConfigureAsSubProject.cmake)
endif ()

