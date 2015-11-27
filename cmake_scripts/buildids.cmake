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

# GNU compiler builds

IF(FC_BUILD_ID STREQUAL "phenomIIx4-linux-gcc-release")
  # set the compiler
  SET (CMAKE_C_COMPILER "gcc")
  SET (CMAKE_CXX_COMPILER "g++")

  SET(CMAKE_BUILD_TYPE "Release")

  SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -std=c++11 -march=amdfam10 -msse4a -ffast-math -funroll-loops")
  SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -march=amdfam10 -msse4a -ffast-math -funroll-loops")

  SET(FC_BUILD_ID_FOUND true)
ENDIF(FC_BUILD_ID STREQUAL "phenomIIx4-linux-gcc-release")

IF(FC_BUILD_ID STREQUAL "opteronx2-linux-gcc-release")
  # set the compiler
  SET (CMAKE_C_COMPILER "gcc")
  SET (CMAKE_CXX_COMPILER "g++")

  SET(CMAKE_BUILD_TYPE "Release")

  SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -std=c++11 -m64 -mmmx -msse -msse2 -m3dnow -mfpmath=sse -ffast-math")
  SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -m64 -mmmx -msse -msse2 -m3dnow -mfpmath=sse -ffast-math")
  SET(FC_BUILD_ID_FOUND true)
ENDIF(FC_BUILD_ID STREQUAL "opteronx2-linux-gcc-release")

IF(FC_BUILD_ID STREQUAL "nehalem-linux-gcc-release")
  # set the compiler
  SET (CMAKE_C_COMPILER "gcc")
  SET (CMAKE_CXX_COMPILER "g++")
  SET(CMAKE_BUILD_TYPE "Release")
  SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 --std=c++11 msse -msse2 -mfpmath=sse -ffast-math")
  SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -msse -msse2 -mfpmath=sse -ffast-math")
  SET(FC_BUILD_ID_FOUND true)
ENDIF(FC_BUILD_ID STREQUAL "nehalem-linux-gcc-release")

IF(FC_BUILD_ID STREQUAL "xeon-linux-gcc-release")
  # set the compiler
  SET (CMAKE_C_COMPILER "gcc")
  SET (CMAKE_CXX_COMPILER "g++")
  SET(CMAKE_BUILD_TYPE "Release")
  SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -std=c++11 -msse -msse2 -mfpmath=sse -ffast-math -Wall")
  SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -msse -msse2 -mfpmath=sse -ffast-math")
  SET(FC_BUILD_ID_FOUND true)
ENDIF(FC_BUILD_ID STREQUAL "xeon-linux-gcc-release")


#--------------------------------------------------------------------------------------------------------------------------------
#                                                      MPI builds
#--------------------------------------------------------------------------------------------------------------------------------


IF(FC_BUILD_ID STREQUAL "opteronx2-linux-intelmpi-release")
  # set the compiler
  SET (CMAKE_C_COMPILER "mpicc")
  SET (CMAKE_CXX_COMPILER "mpic++")

  SET(CMAKE_BUILD_TYPE "Release")
  SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -funroll-loops -fp-model precise -no-prec-div")
  SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -funroll-loops -fp-model precise -no-prec-div")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -lrt")

  SET(FC_BUILD_ID_FOUND true)
ENDIF(FC_BUILD_ID STREQUAL "opteronx2-linux-intelmpi-release")

#--------------------------------------------------------------------------------------------------------------------------------
#                                                      Debug builds
#--------------------------------------------------------------------------------------------------------------------------------

IF(FC_BUILD_ID MATCHES "debug")
  SET(CMAKE_BUILD_TYPE "Debug")
  SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")
  SET(FC_BUILD_ID_FOUND true)
ENDIF(FC_BUILD_ID MATCHES "debug")

# IF(FC_BUILD_ID STREQUAL "opteronx2-linux-intel-debug")
#   SET(CMAKE_BUILD_TYPE "Debug")
#   SET(FC_BUILD_ID_FOUND true)
# ENDIF(FC_BUILD_ID STREQUAL "opteronx2-linux-intel-debug")

