
IF(NOT FC_BUILD_ID)
  # A build id is composed of:
  # ${cpu_type}-${os}-${compiler}-${build_type}
  # Example: nehalem-linux-intel-release

  # We do not have a user defined build id
  # so we select a default build

  message(STATUS "No build id selected... trying to determine machine type and compiler settings")

  # which compiler is found
  message(STATUS "C++ Compiler ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
  message(STATUS "C Compiler ${CMAKE_C_COMPILER_ID} ${CMAKE_C_COMPILER_VERSION}")

  message(STATUS "Processor ${CMAKE_SYSTEM_PROCESSOR}")

  set(_vendor_id)
  set(_cpu_family)
  set(_cpu_model)
  set(_cpu_flags)

  set(FC_CPU_TYPE)
  set(FC_OS)
  set(FC_DEFAULT_BUILD)
  set(FC_COMPILER_ID)

  # set the compiler
  IF("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    set(FC_COMPILER_ID "gcc")
  ELSEIF("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Intel")
    set(FC_COMPILER_ID "intel")
  ELSEIF("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    set(FC_COMPILER_ID "clang")
  ELSEIF("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
  ELSE()
    message(FATAL_ERROR "Compiler :<${CMAKE_CXX_COMPILER_ID}> is not tested with this library and hence not supported.")  
  ENDIF()


  IF(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
    set(FC_DEFAULT_BUILD "release")
  ELSE(NOT CMAKE_BUILD_TYPE)
    string(TOLOWER "${CMAKE_BUILD_TYPE}" FC_DEFAULT_BUILD)
  ENDIF(NOT CMAKE_BUILD_TYPE)

  # For a windows system cmake will generate visual studio files
  # with all possible build types and we are done. For a linux system
  # we want to select compiler settings that are useful for the hardware
  # we are using  
  IF(CMAKE_SYSTEM_NAME MATCHES "Linux")
    set(FC_OS "linux")
    file(READ "/proc/cpuinfo" _cpuinfo)
    string(REGEX REPLACE ".*vendor_id[ \t]*:[ \t]+([a-zA-Z0-9_-]+).*" "\\1" _vendor_id "${_cpuinfo}")
    string(REGEX REPLACE ".*cpu family[ \t]*:[ \t]+([a-zA-Z0-9_-]+).*" "\\1" _cpu_family "${_cpuinfo}")
    string(REGEX REPLACE ".*model[ \t]*:[ \t]+([a-zA-Z0-9_-]+).*" "\\1" _cpu_model "${_cpuinfo}")
    string(REGEX REPLACE ".*flags[ \t]*:[ \t]+([ \ta-zA-Z0-9_-]+)(.*)" "\\1" _cpu_flags "${_cpuinfo}")

    message(STATUS "vendor_id ${_vendor_id}")
    message(STATUS "family ${_cpu_family}")
    message(STATUS "model ${_cpu_model}")
    message(STATUS "flags ${_cpu_flags}")

    IF(${_vendor_id} MATCHES "AuthenticAMD")
      IF(${_cpu_family} EQUAL 16)
        IF(${_cpu_model} EQUAL 4)
          SET(FC_CPU_TYPE "phenomIIx4")
        ENDIF()
      ELSEIF(${_cpu_family} EQUAL 15)
        IF(${_cpu_model} EQUAL 33)
          SET(FC_CPU_TYPE "opteron")
        ELSEIF(${_cpu_model} EQUAL 65 OR ${_cpu_model} EQUAL 66 )
          SET(FC_CPU_TYPE "opteronx2")
        ENDIF()
      ELSE()
        message(FATAL_ERROR "Unknown CPU, cannot set up default configuration")
      ENDIF()
    ELSEIF(${_vendor_id} MATCHES "GenuineIntel")
      IF(${_cpu_family} EQUAL 6)
        IF("15 21 22 23 29" MATCHES ${_cpu_model})
          SET(FC_CPU_TYPE "core2duo")
        ELSEIF("26 30 37 44 45 42 46" MATCHES ${_cpu_model})
          SET(FC_CPU_TYPE "nehalem")
        ELSEIF("60 94" MATCHES ${_cpu_model})
          SET(FC_CPU_TYPE "xeon")
        ELSEIF("13" MATCHES ${_cpu_model})
          SET(FC_CPU_TYPE "generic")
        ENDIF()
      ENDIF()
    ELSE()
      message(FATAL_ERROR "CPU could not be identified.")
    ENDIF()

    SET(FC_BUILD_ID "${FC_CPU_TYPE}-${FC_OS}-${FC_COMPILER_ID}-${FC_DEFAULT_BUILD}")

    include(./cmake_scripts/buildids.cmake)
    IF(NOT FC_BUILD_ID_FOUND)
      message(FATAL_ERROR "Build id:<${FC_BUILD_ID}> was not found.")
    ENDIF(NOT FC_BUILD_ID_FOUND)
    message(STATUS "Configuring for build id:<${FC_BUILD_ID}>")

  endif(CMAKE_SYSTEM_NAME MATCHES "Linux")  
ELSE(NOT FC_BUILD_ID)

  include(./cmake_scripts/buildids.cmake)
  IF(NOT FC_BUILD_ID_FOUND)
    message(FATAL_ERROR "Build id:<${FC_BUILD_ID}> was not found.")
  ENDIF(NOT FC_BUILD_ID_FOUND)
  message(STATUS "Configuring for build id:<${FC_BUILD_ID}>")

ENDIF(NOT FC_BUILD_ID)
