
set(CMAKE_AUTOMOC ON)
include_directories (. ../../math ../../cuda_addon ../../util ../../inshape3dcore ../../inshape3dcore/collision 
  ../../inshape3dcore/distance ../../inshape3dcore/grid ../../inshape3dcore/fortrancppinterface 
../../inshape3dcore/intersection ../../inshape3dcore/physics ../../inshape3dcore/postprocessing ../../inshape3dcore/preprocessing ../../inshape3dcore/shapes 
../../inshape3dcore/shapes/modelsandmeshes ../../libs/rapidxml-1.13 ../../libs/OpenMesh-6.2/src)

set(my_hdr
  mainwindow.h
  myopenglwidget.h
  )

set(my_src
  main.cpp
  mainwindow.cpp
  myopenglwidget.cpp
  )

set(my_uis
  mainwindow.ui
  )

INCLUDE_DIRECTORIES(. ${GLUT_INCLUDE_DIR})

find_package(Qt5Widgets 5.7 REQUIRED HINTS "/opt/Qt/5.7/gcc_64/lib")

find_package(Qt5OpenGL 5.7 REQUIRED HINTS "/opt/Qt/5.7/gcc_64/lib")

find_package(OpenGL REQUIRED)

find_package(GLUT REQUIRED)

message(STATUS "Qt5 found in : ${Qt5Widgets_LIBRARIES}")

# add library directories
link_directories(${CMAKE_SOURCE_DIR}/libs)

link_directories(/opt/Qt/5.7/gcc_64/lib)

add_executable(QtOpenGL ${my_src})

qt5_use_modules(QtOpenGL Widgets OpenGL)

#message(STATUS "Qt5 found in : ${Qt5Widgets_LIBRARIES}")

set(QT5_LIBRARIES Qt5::Widgets)

#message(STATUS "Qt5 found in : ${QT5_LIBRARIES}")

target_link_libraries(QtOpenGL OpenMeshCore OpenMeshTools Utility Math inshape3dcore -lQt5OpenGL -lQt5Widgets -lQt5Gui -lQt5Core ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES} -lpthread)

