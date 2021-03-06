set(FC_BUILD_ID_FOUND false)

# Intel compiler builds

IF(FC_BUILD_ID STREQUAL "nehalem-linux-intel-release")
  # set the compiler
  SET (CMAKE_C_COMPILER "icc")
  SET (CMAKE_CXX_COMPILER "icpc")

  SET(CMAKE_BUILD_TYPE "Release")
  SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -xSSE4.2 -funroll-loops -fp-model precise -no-prec-div")
  SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -xSSE4.2 -funroll-loops -fp-model precise -no-prec-div")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -lrt")

  SET(FC_BUILD_ID_FOUND true)
ENDIF(FC_BUILD_ID STREQUAL "nehalem-linux-intel-release")

IF(FC_BUILD_ID STREQUAL "core2duo-linux-intel-release")
  # set the compiler
  SET (CMAKE_C_COMPILER "icc")
  SET (CMAKE_CXX_COMPILER "icpc")

  SET(CMAKE_BUILD_TYPE "Release")

  SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -xSSE3 -funroll-loops -fp-model precise -no-prec-div")
  SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -xSSE3 -funroll-loops -fp-model precise -no-prec-div")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -lrt")

  SET(FC_BUILD_ID_FOUND true)
ENDIF(FC_BUILD_ID STREQUAL "core2duo-linux-intel-release")

IF(FC_BUILD_ID STREQUAL "opteron-linux-intel-release")
  # set the compiler
  SET (CMAKE_C_COMPILER "icc")
  SET (CMAKE_CXX_COMPILER "icpc")

  SET(CMAKE_BUILD_TYPE "Release")

  SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -funroll-loops -fp-model precise -no-prec-div")
  SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -funroll-loops -fp-model precise -no-prec-div")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -lrt")

  SET(FC_BUILD_ID_FOUND true)
ENDIF(FC_BUILD_ID STREQUAL "opteron-linux-intel-release")

IF(FC_BUILD_ID STREQUAL "opteronx2-linux-intel-release")
  # set the compiler
  SET (CMAKE_C_COMPILER "icc")
  SET (CMAKE_CXX_COMPILER "icpc")

  SET(CMAKE_BUILD_TYPE "Release")
  SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -funroll-loops -fp-model precise -no-prec-div")
  SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -funroll-loops -fp-model precise -no-prec-div")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -lrt")

  SET(FC_BUILD_ID_FOUND true)
ENDIF(FC_BUILD_ID STREQUAL "opteronx2-linux-intel-release")

IF(FC_BUILD_ID STREQUAL "phenomIIx4-linux-intel-release")
  # set the compiler
  SET (CMAKE_C_COMPILER "icc")
  SET (CMAKE_CXX_COMPILER "icpc")

  SET(CMAKE_BUILD_TYPE "Release")

  SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -funroll-loops -fp-model precise -no-prec-div")
  SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -funroll-loops -fp-model precise -no-prec-div")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -lrt")

  SET(FC_BUILD_ID_FOUND true)
ENDIF(FC_BUILD_ID STREQUAL "phenomIIx4-linux-intel-release")

#===============================================================================================================
#                                              GCC builds
#===============================================================================================================

IF(FC_BUILD_ID STREQUAL "xeon-linux-gcc-release")
  # set the compiler
  SET(CMAKE_BUILD_TYPE "Release")
  SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -std=c++17 -msse -msse2 -mfpmath=sse -ffast-math -Wall")
  SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -msse -msse2 -mfpmath=sse -ffast-math")
  SET(FC_BUILD_ID_FOUND true)
ENDIF(FC_BUILD_ID STREQUAL "xeon-linux-gcc-release")

IF(FC_BUILD_ID STREQUAL "nehalem-linux-gcc-release")
  # set the compiler
  SET(CMAKE_BUILD_TYPE "Release")
  SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -std=c++17 -msse -msse2 -mfpmath=sse -ffast-math")
  SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -msse -msse2 -mfpmath=sse -ffast-math")
  SET(FC_BUILD_ID_FOUND true)
ENDIF(FC_BUILD_ID STREQUAL "nehalem-linux-gcc-release")

IF(FC_BUILD_ID STREQUAL "generic-linux-gcc-release")
  # set the compiler
  SET(CMAKE_BUILD_TYPE "Release")
  SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -std=c++17 -Wall")
  SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3")
  SET(FC_BUILD_ID_FOUND true)
ENDIF(FC_BUILD_ID STREQUAL "generic-linux-gcc-release")

IF(FC_BUILD_ID STREQUAL "phenomIIx4-linux-gcc-release")
  # set the compiler
  SET(CMAKE_BUILD_TYPE "Release")
  SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -std=c++17 -funroll-loops")
  SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -funroll-loops")
  SET(FC_BUILD_ID_FOUND true)
ENDIF(FC_BUILD_ID STREQUAL "phenomIIx4-linux-gcc-release")

IF(FC_BUILD_ID STREQUAL "i7-linux-gcc-release")
  SET(CMAKE_BUILD_TYPE "Release")
  SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -g -std=c++17 -march=native")
  SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -g -march=native")
  SET(FC_BUILD_ID_FOUND true)
ENDIF()

IF(FC_BUILD_ID STREQUAL "skylake-linux-gcc-release")
  SET(CMAKE_BUILD_TYPE "Release")
  SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -g -std=c++17 -march=native")
  SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -g -march=native")
  SET(FC_BUILD_ID_FOUND true)
ENDIF()

IF(FC_BUILD_ID STREQUAL "opteronx2-linux-gcc-release")
  # set the compiler
  SET(CMAKE_BUILD_TYPE "Release")

  SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -std=c++17 -m64 -mmmx -msse -msse2 -m3dnow -mfpmath=sse -ffast-math")
  SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -m64 -mmmx -msse -msse2 -m3dnow -mfpmath=sse -ffast-math")
  SET(FC_BUILD_ID_FOUND true)
ENDIF(FC_BUILD_ID STREQUAL "opteronx2-linux-gcc-release")


#--------------------------------------------------------------------------------------------------------------------------------
#                                                      Debug builds
#--------------------------------------------------------------------------------------------------------------------------------

IF(FC_BUILD_ID MATCHES "debug")
  SET(CMAKE_BUILD_TYPE "Debug")
  SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -Wall --std=c++11")
  SET(FC_BUILD_ID_FOUND true)
ENDIF(FC_BUILD_ID MATCHES "debug")

# IF(FC_BUILD_ID STREQUAL "opteronx2-linux-intel-debug")
#   SET(CMAKE_BUILD_TYPE "Debug")
#   SET(FC_BUILD_ID_FOUND true)
# ENDIF(FC_BUILD_ID STREQUAL "opteronx2-linux-intel-debug")

