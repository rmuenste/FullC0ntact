#________________________________________________________________
#
# Inshape3d Library : Top-Level CMakeLists file
# Author            : Raphael Muenster
# Date of creation  : 26.10.2010
# Data of version   : 09.06.2011
# Version number    : 00.001
#________________________________________________________________
#
#----------------------------------------------------------------
#                                  ,JhJ????ccc                               
#                           ,c??h$!!9?!????9i!LL zJ?!$.                      
#                         z?!!JtiJ??iJC????$$??!!?PLh!9                      
#                       c?i??!!?!h?!i!J???h$??!!!!!?$$!h                     
#                      .$$i!????!!J?9?!i?!i???!ii!?h!$hh                     
#                    .$?J!i$i!!!!!!J$!$!!!!i$$??h9h?h!CFF                    
#                   z$!$J!!i??!???i$i$!J?3P'F""?$!9!$J$!?$.                  
#                  $$!C9!$SJ?!!9i?i?i!9'jF'?`.`.`.?6iLLL!$!h                 
#                <Ci?J!C!F$!!!$?t!!?Jh$.`.`.`.`.`.`.?hhh!?h$                 
#                C?!!!9!h!C!!$$!!$?!?$h.`.`.`.`.`.`.3h!9?$$$                 
#               J!!!i$!!$tC!9?9!$t?"`.`.`__.`.`.`.`.3h?hL?9F                 
#             ,$?!$?!!i?!$h!$!9!!F`.`.`=""'??$h.`.`.?!C$??"                  
#            C$!!$!!J?!??!i$!JC!P.`.`.`.`ccci.`.`.;c$!C$                     
#           J!C!!C!i$!!!$?i?SC!$!.`.`.`.?,`<$F.`.?"'3?9'                     
#           $!C!9!C!h!!C!$!iC3i$!!'.`.`.`."?".`.`J?$J"                       
#           2J!$?$!J!9?!$!9`$3".`.`.`.`.`.`.`.`.`?$F                         
#          $?P!C!h!$!9??i$!!'F`.`.`.`.`.`.`.`.`.`<`?                         
#          $!!J!C?!h!!?!!C?$$?`.`.`.`.`.`.`.3'cc`.PF                         
#          \!9!?!$!!J?i?$h!t?$'.`.`.`.`.`.`.`.`.?"'F                         
#           $?i!?$!!C!!?h?C9;?h.`.`.`.`.`.,cc?$c'.$F	                       
#           ?!9!i$!!$ii!!?$$;;;?c,`.`.`.`."iZ?h9`$?P                         
#            ?!C$9C!!$!$?h$9.`;;;?$c`.`.`.`.`"".$$C                          
#            `?F?J!!ii?!J!J!$'.`.;;;?c,.`.`.`.`$h6$                          
#            J!!?J?!i!i?i?$!9`.`.`;;;;??i.`.`J$iJ"                           
#           .$!$?!J!i!$I9??i$'.`.`.;;;;;;?$F?h??hL                           
#          ,C!$!!J9!J!$hhPJF.`.`.`.`.;;;jF;;;;?????""?????c,                 
#        ,J?!!$!!$!!$!!C?C9`.`.`.`.`.`;;;;;;;;'.`.`.`.`.`.``h                
#  ,c??!J!i??l$??$?C!!J!!C$?.`.`.`.`;;;;;.`.`;;;;.`.`.`.`.`.`?h              
# $?!`.3!9C!iF!!!!C!??!!Jh9'';;;.`.;;.`.`,;;;;;;;.`.`.`.`.`.`.`?.            
#J!!.`.JIi!$9!!!C!i!?C!?ih';;;;.`.`.`.`;;;;;;;;.`.`.`.`.`.`.`.`.?c           
#$!'.`.$P"'$!$i;$!$!J!?Ji'`.`.`.`.`.`.`.`.`.`.`.`.`.`.`.`.`.`.`.``L          
#$!`.`.$'?h$!$;;99??!i?.`.`.`.`.`.`.`.`.`.`.`.`.`.`.`.`.`.`.`.`.`.?,         
#$!`.`.`jl$$i;;i$hj?".`.`.`.`.`.`.`.`.`.`.`.`.`.`.`.`.`.`.`.`.`.`.`?         
#C;`.`.""`$;;;;.`.`.`.`.`.`.`.`.`.`.`.`.`.`.`.`.`.`.`.`.`.`.`.`.`.`.$        
#C;..`.`,J;;;;'.`.`.`.`.`.`.`.`.`.`.`.`.`.`.`.`.`.`.`.`.`.`.`.`.`.`.`L       
#C;..`.`3F;;;;'.`.`.`.`.`.`.`.;;;;`.`.`.`.`.`.`.`.`.`.`.`.`.`.`.`.`.`3.      
#C;;.`.`9C;;;.`.`.`.`.`.`.`.;;;;;.`.`.`.`.`.`.`.`.`.`cJ?;;'.`.`.`.`.``$      
#$;;.`.;9C;;;.`.`.`.`.`.`.`;;;;;'.`.`.`.`.`.`.`.`.c?'?c;;;'.`.`.`.`.`.`$.    
#3;;.`;;$C;;'.`.`.`.`.`.`.;;;;;.`.`.`.`.`.`.`.`.`3?`.``h9;;.`.`.`.`.`.`.?c   
#?;;.;;jC;;.`.`.`.`.`.`.`;;;;;'.`.`._.`.`.`.`.`.`$;'.`.`?h;;;.`.`.`.`.`.``h  
#`;;;;jC?.`.`.`.`.`.`.`.`;;;;.`.`,c???h.`.`.`.`.`C;.`.`.,J"h;;;.`.`.`.`.`.`$ 
# C;;;$iL.`.`.`.`.`.`.`.`C;;;.`.`3?hJ;$.`.`.`.`.;C,`.`.j"   ?i;;;.`.`.`.`.`.$
# h;;$ii9.`.`.`.`.`.`.`.`C;;;.`.`.TjjP'.`.`.`.`;j;.`.`,F     `?;;;;,`.`.`.`.`
# $;th??$.`.`.`.`.`.;;`.`?;;;.`.`.`.`.`.`.`.`.;;$;.`.`$        ?i;;;;,`.`.`.`
# $;;h??'.`.`.`.`.`;;.`.`?;;;;'.`.`.`.`.`.`.`;;J?.`.`;F          ?;;;;'.`.`.`
# $;;$;;;;'.`.`.`.`;;.`.`.?;;;'.`.`.`.`.`,;;;;J?`.`.;P            ?i;;;;`.`.`
# $;;?C;;;;;`.`.`.;i;.`.`.`?i;;;;;`.`;;;;;;;i?`.`.`;9'             `h;;;;,`.`
# $;;;?i;;;;;;;;;;i?;.`.`.`.`?i;;;;;;;;;jj?"`.`.`.;;$                ?;;;;`.`
# $;;;;?h;;;;;;;;$?;`.`.`.`.`.`'"????""'.`.`.`.`.`;t'                 ?h;;;;`
#  $;;;;;;?hijji??;;.`.`;;;;.`.`.`.`.`.`.`.`.`.`.`.;P                   "h;i?`
#  ?;;'.;;;;;$;;;;;'.`.`;;;;;`;;`.`.`.`.`.`.`.`.`.;j'                     $F.`
# cmake version
cmake_minimum_required(VERSION 2.8)

# name of the project
PROJECT(Inshape3d)

# allow easier if-else syntax
SET(CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS TRUE)

# enable testing
ENABLE_TESTING()

# output the system name
MESSAGE(STATUS "Configuring Inshape3d for a ${CMAKE_SYSTEM} system")

#--------------------------------------------------------------------------------------------------------------------
#                                               compiler settings
#--------------------------------------------------------------------------------------------------------------------

IF(CMAKE_SYSTEM_NAME MATCHES "Linux")
	#set the intel compiler
	#set (CMAKE_C_COMPILER /opt/intel/Compiler/11.1/072/bin/intel64/icc)
	#set (CMAKE_CXX_COMPILER /opt/intel/Compiler/11.1/072/bin/intel64/icpc)
  #execute_process(COMMAND 
  #MESSAGE(STATUS "username = $ENV{USERNAME}")
  SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -lrt")
ENDIF(CMAKE_SYSTEM_NAME MATCHES "Linux")

# TODO:check for processor type and choose compiler flags

# set more sophisticated compiler flags for the release version
#IF(CMAKE_BUILD_TYPE STREQUAL "RELEASE")
#	SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -march=native -mtune=native -fomit-frame-pointer -pipe -mmmx -msse2 -msse3")
#ENDIF(CMAKE_BUILD_TYPE STREQUAL "RELEASE")

#--------------------------------------------------------------------------------------------------------------------
#                                               Preprocessor options
#--------------------------------------------------------------------------------------------------------------------

# set the default build type to featflowlib ()
option(FC_FEATFLOW
    "Build the FullC0ntact library for use with the FEATFLOW simulator"
    OFF
)

IF(FC_FEATFLOW)
  add_definitions(-DFEATFLOWLIB)
ENDIF(FC_FEATFLOW)


#--------------------------------------------------------------------------------------------------------------------
#                                               project directories
#--------------------------------------------------------------------------------------------------------------------
IF(CMAKE_SYSTEM_NAME MATCHES "Linux")
include_directories (math util inshape3dcore inshape3dcore/collision inshape3dcore/distance inshape3dcore/grid 
inshape3dcore/fortrancppinterface inshape3dcore/intersection inshape3dcore/physics inshape3dcore/postprocessing
inshape3dcore/preprocessing inshape3dcore/shapes inshape3dcore/shapes/modelsandmeshes
util/googlehash/google util/googlehash/google/sparsehash util/googlehash/windows util/googlehash
util/googlehash/windows/google/sparsehash)
ENDIF(CMAKE_SYSTEM_NAME MATCHES "Linux")
# set the include directories

# add the core subdirectories
ADD_SUBDIRECTORY(math)
ADD_SUBDIRECTORY(util)
ADD_SUBDIRECTORY(inshape3dcore)
ADD_SUBDIRECTORY(path)

# check whether the applications should be build
if(NOT FC_FEATFLOW)
  ADD_SUBDIRECTORY(applications)

  MESSAGE(STATUS "Copying data directories...")

  SET(APP_DIRS basftest gjktest meshtest velocitybased pardynamics broadphasetest contactgenerationtest  hgridtest reactor distancegridtest inclinedplane reactor_fine featflowinterfacetest  mathtest shapestest penetrationcorrection)

  #--------------------------------------------------------------------------------------------------------------------
  #                                               prepare application
  #--------------------------------------------------------------------------------------------------------------------

  foreach(app ${APP_DIRS})
    execute_process(COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/applications/${app}/meshes ${CMAKE_BINARY_DIR}/applications/${app}/meshes)
    
    if(NOT EXISTS ${CMAKE_BINARY_DIR}/applications/${app}/solution)
      file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/applications/${app}/solution)
    endif ()

    if(NOT EXISTS ${CMAKE_BINARY_DIR}/applications/${app}/output)
      file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/applications/${app}/output)
    endif ()

    if(NOT EXISTS ${CMAKE_BINARY_DIR}/applications/${app}/start)
      file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/applications/${app}/start)
    endif ()

    if(NOT EXISTS ${CMAKE_BINARY_DIR}/applications/${app}/start/data.TXT)
    execute_process(COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_SOURCE_DIR}/applications/${app}/start/data.default
    ${CMAKE_BINARY_DIR}/applications/${app}/start/data.TXT)
    endif ()

  endforeach(app)
endif(NOT FC_FEATFLOW)
#--------------------------------------------------------------------------------------------------------------------
#                                                    add tests
#--------------------------------------------------------------------------------------------------------------------
#ADD_TEST(NAME bench1 WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/applications/velocitybased COMMAND velocitybased)
