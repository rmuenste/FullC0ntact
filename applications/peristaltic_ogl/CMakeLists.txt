
set(CMAKE_AUTOMOC ON)

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


find_package(Qt5Widgets 5.6 REQUIRED HINTS "/opt/Qt/5.7/gcc_64/lib")

find_package(Qt5OpenGL 5.6 REQUIRED HINTS "/opt/Qt/5.7/gcc_64/lib")

find_package(OpenGL REQUIRED)

find_package(GLUT REQUIRED)

# add library directories
link_directories(${CMAKE_SOURCE_DIR}/libs)

link_directories(/opt/Qt/5.7/gcc_64/lib)

add_executable(peristaltic_ogl ${my_src})

qt5_use_modules(peristaltic_ogl Widgets OpenGL)

#message(STATUS "Qt5 found in : ${Qt5Widgets_LIBRARIES}")

set(QT5_LIBRARIES Qt5::Widgets)

#message(STATUS "Qt5 found in : ${QT5_LIBRARIES}")


target_link_libraries(peristaltic_ogl OpenMeshCore OpenMeshTools Utility Math inshape3dcore -lQt5OpenGL -lQt5Widgets -lQt5Gui -lQt5Core ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES} -lpthread)

# add include directories
target_include_directories(peristaltic_ogl PUBLIC ${FC_APP_INCLUDE_DIRS} ${GLUT_INCLUDE_DIR})

