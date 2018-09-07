
# set the include directories
set(FC_INCLUDE_DIRS . 
  ../math 
  ../cuda_addon 
  ../util 
  collision 
  distance 
  grid 
  fortrancppinterface 
  intersection 
  physics 
  postprocessing 
  preprocessing 
  shapes 
  shapes/modelsandmeshes 
  ../libs/rapidxml-1.13
  ${CUDA_SDK_ROOT_DIR}/common/inc 
  ${CUDA_SDK_ROOT_DIR}/shared/inc 
  ${CUDA_PATH}/include 
  ../libs/OpenMesh/src
  ../libs/eigen
  ${CMAKE_SOURCE_DIR}/extern/libraries/cgal-install-dir/include
)

if(USE_OPTICALTWEEZERS)
  set(FC_INCLUDE_DIRS ${FC_INCLUDE_DIRS} 
  ${CMAKE_SOURCE_DIR}/extern/libraries/opticaltweezers
  ${CMAKE_SOURCE_DIR}/extern/libraries/opticaltweezers/vector
  ${CMAKE_SOURCE_DIR}/extern/libraries/opticaltweezers/strahl	
  ${CMAKE_SOURCE_DIR}/extern/libraries/opticaltweezers/ot	
  ${CMAKE_SOURCE_DIR}/extern/libraries/opticaltweezers/tinyxml	
  )

  set(OPTICALTWEEZERS_LIBRARIES vector strahl ot)

endif(USE_OPTICALTWEEZERS)

# create code groupings and put them into cmake "source_groups"
set(distance_src
distance/distanceobb3obb3.h
distance/distance.h
distance/distanceconvexconvexgjk.h
distance/distancefunctiongrid.h
distance/distancefuncgridmodel.h
distance/distancelineline.h
distance/distancelinerec.h
distance/distancelineseg.h
distance/distancemodelplane.h
distance/distancemeshsphere.h
distance/distancemeshmesh.h
distance/distancemeshpoint.h
distance/distanceobb3plane.h
distance/distancepointcylinder.h
distance/distancepointpline.h
distance/distancepointrec.h
distance/distancepointseg.h
distance/distancepointobb3.h
distance/distancesegrec.h
distance/distancesegseg.h
distance/distancetriangletriangle.h
distance/distancetools.h
distance/distops3.h
distance/distanceaabbaabb.h
distance/distance.cpp 
distance/distanceconvexconvexgjk.cpp
distance/distancefunctiongrid.cpp
distance/distancefuncgridmodel.cpp
distance/distancelineline.cpp
distance/distancepointcylinder.cpp
distance/distancepointpline.cpp
distance/distancelinerec.cpp
distance/distancelineseg.cpp
distance/distancemodelplane.cpp
distance/distancemeshsphere.cpp
distance/distancemeshpoint.cpp
distance/distancemeshmesh.cpp
distance/distanceobb3obb3.cpp
distance/distanceobb3plane.cpp
distance/distancepointobb3.cpp
distance/distancepointrec.cpp
distance/distancepointseg.cpp
distance/distancesegrec.cpp
distance/distancesegseg.cpp
distance/distancetriangle.cpp
distance/distancetools.cpp
distance/distancetriangletriangle.cpp
distance/distops3.cpp
distance/distanceaabbaabb.cpp)

set(intersection_src
intersection/intersector2aabb.h
intersection/intersector2obb3.h
intersection/intersectorcirclecircle.h
intersection/intersectorplaneobb3.h
intersection/intersectoraabbray.h
intersection/intersectormpr.h
intersection/intersectorray3tri3.h
intersection/intersectorspheresphere.h
intersection/intersectortools.h
intersection/intersectorspheresegment.h
intersection/intersector2aabb.cpp
intersection/intersector2obb3.cpp
intersection/intersectorplaneobb3.cpp
intersection/intersectoraabbray.cpp
intersection/intersectormpr.cpp
intersection/intersectorray3tri3.cpp
intersection/intersectorspheresphere.cpp
intersection/intersectorspheresegment.cpp
intersection/intersectorcirclecircle.cpp
intersection/intersectortools.cpp
)

set(postprocessing_src
postprocessing/triangulator.h
postprocessing/vtkwriter.h
postprocessing/rigidbodyio.h 
postprocessing/triangulator.cpp
postprocessing/vtkwriter.cpp
postprocessing/rigidbodyio.cpp 
)

set(collision_src
collision/boundary.h
collision/boundarybox.h
collision/boundarycyl.h
collision/broadphase.h
collision/collisioninfo.h
collision/collisionpipeline.h
collision/basicpipeline.hpp
collision/collidercompoundboundarybox.h
collision/collresponsesi.h
collision/collresponsedem.h
collision/collresponselcp.h
collision/compare.h
collision/contact.h
collision/subdomaincontact.h
collision/timecontrol.h
collision/collider.h
collision/colliderboxplane.h
collision/colliderconvexconvex.h
collision/colliderconvexconvexgjk.h
collision/colliderfactory.h
collision/colliderboxbox.h
collision/colliderboxboxboundary.h
collision/collidersphereboxboundary.h
collision/collidercylinderboundarybox.h
collision/colliderspheresphere.h
collision/colliderspheresubdomain.h
collision/colliderboxsphere.h
collision/collidersphereplane.h
collision/colliderspherecompound.h
collision/collider2compound.h
collision/collidermeshsphere.h
collision/collidermeshmesh.h
collision/collidermeshboundarybox.h
collision/collidercompoundmesh.h
collision/collidercompoundbox.h
collision/colliderspherecylindricalboundary.h
collision/collidercompcylindricalboundary.h
collision/contactgenerator.h
collision/contactgroup.h
collision/contactgenerator2cylinder.h
collision/contactgeneratorboxcylinder.h
collision/contactgeneratorcylindersphere.h
collision/dembasic.h
collision/demfriction1.h
collision/demfriction.h
collision/dempipeline.hpp
collision/broadphasestrategy.h
collision/broadphasestrategygrid.h
collision/broadphasestrategyhgrid.h
collision/broadphasestrategyrmt.h
collision/collisionhash.h
collision/stacklayer.h
collision/collidercompoundboundarybox.cpp
collision/boundary.cpp
collision/boundarybox.cpp
collision/boundarycyl.cpp
collision/broadphase.cpp
collision/collisionhash.cpp
collision/collisioninfo.cpp
collision/colliderboxbox.cpp
collision/colliderboxboxboundary.cpp
collision/collider2compound.cpp
collision/collresponse.cpp
collision/collresponsesi.cpp
collision/collresponsedem.cpp
collision/collresponselcp.cpp
collision/contact.cpp
collision/subdomaincontact.cpp
collision/timecontrol.cpp
collision/collider.cpp
collision/colliderboxplane.cpp
collision/colliderconvexconvex.cpp
collision/colliderconvexconvexgjk.cpp
collision/collidercylinderboundarybox.cpp
collision/colliderfactory.cpp
collision/collidermeshsphere.cpp
collision/collidermeshboundarybox.cpp
collision/collidersphereboxboundary.cpp
collision/colliderspherecylindricalboundary.cpp
collision/collidercompcylindricalboundary.cpp
collision/collidersphereplane.cpp
collision/colliderspherecompound.cpp
collision/collidercompoundmesh.cpp
collision/collidercompoundbox.cpp
collision/colliderspheresphere.cpp
collision/colliderspheresubdomain.cpp
collision/collidermeshmesh.cpp
collision/colliderboxsphere.cpp
collision/contactgenerator.cpp
collision/contactgroup.cpp
collision/contactgenerator2cylinder.cpp
collision/contactgeneratorboxcylinder.cpp
collision/contactgeneratorcylindersphere.cpp
collision/dembasic.cpp
collision/demfriction.cpp
collision/demfriction1.cpp
collision/broadphasestrategy.cpp
collision/broadphasestrategygrid.cpp
collision/broadphasestrategyhgrid.cpp
collision/broadphasestrategyrmt.cpp
collision/stacklayer.cpp
)

set(shapes_src
shapes/boundarydescription.cpp
shapes/boundarydescription.hpp
shapes/boundaryshape.cpp
shapes/boundaryshape.hpp
shapes/obb3.h
shapes/paramline.h
shapes/plane.h
shapes/ray3.h
shapes/rectangle3.h
shapes/segment3.h
shapes/shape.h
shapes/sphere.h
shapes/transform.h
shapes/aabb3.cpp
shapes/cone.cpp
shapes/convexshape.cpp
shapes/cylinder.cpp
shapes/ellipsoid.cpp
shapes/line3.cpp
shapes/obb3.cpp
shapes/paramline.cpp
shapes/plane.cpp
shapes/ray3.cpp
shapes/rectangle3.cpp
shapes/segment3.cpp
shapes/shape.cpp
shapes/sphere.cpp
shapes/transform.cpp
)

set(modelsandmeshes_src
shapes/aabb3.h
shapes/cone.h
shapes/convexshape.h
shapes/cylinder.h
shapes/ellipsoid.h
shapes/line3.h
shapes/meshobject.h
shapes/modelsandmeshes/3dmesh.h
shapes/modelsandmeshes/3dmodel.h
shapes/modelsandmeshes/boundingvolumetree3.h
shapes/modelsandmeshes/subdivisioncreator.h
shapes/modelsandmeshes/traits.h
shapes/modelsandmeshes/triangle3.h
shapes/modelsandmeshes/triface.h
shapes/meshobject.cpp
shapes/modelsandmeshes/subdivisioncreator.cpp
shapes/modelsandmeshes/3dmesh.cpp
shapes/modelsandmeshes/3dmodel.cpp
shapes/modelsandmeshes/traits.cpp
shapes/modelsandmeshes/triangle3.cpp
shapes/modelsandmeshes/triface.cpp
)

set(grid_src
grid/deformparameters.cpp
grid/grid3d.cpp
grid/hspatialhash.cpp
grid/implicitgrid.cpp
grid/unstructuredgrid.cpp
grid/uniformgrid.cpp
grid/huniformgrid.cpp
grid/simplespatialhash.cpp
grid/spatialhash.cpp
grid/ugsizeheuristic.cpp
grid/ugsizeheuristicstd.cpp
grid/ugsizeheuristicgrd.cpp
grid/deformparameters.h
grid/distancemap.h
grid/distancemap.cpp
grid/grid3d.h
grid/hspatialhash.h
grid/implicitgrid.cpp
grid/simplespatialhash.h
grid/spatialhash.h
grid/unstructuredgrid.h
grid/uniformgrid.h
grid/huniformgrid.h
grid/ugsizeheuristic.h
grid/ugsizeheuristicstd.h
grid/ugsizeheuristicgrd.h
)

set(physics_src
  physics/rigidbodymotion.cpp
  physics/motionintegratorsi.cpp
  physics/motionintegratordem.cpp
  physics/rigidbodymotion.h
  physics/motionintegratorsi.h
  physics/motionintegratordem.h
  physics/springconstraint.hpp
  )

set(preprocessing_src
  preprocessing/3dsloader.cpp
  preprocessing/segmentlistreader.cpp
  preprocessing/basicmodelloader.cpp
  preprocessing/genericloader.cpp
  preprocessing/objloader.cpp
  preprocessing/offloader.cpp
  preprocessing/reader.cpp
  preprocessing/tubeloader.cpp
  preprocessing/3dsloader.h
  preprocessing/basicmodelloader.h
  preprocessing/genericloader.h
  preprocessing/objloader.h
  preprocessing/offloader.h
  preprocessing/reader.h
  preprocessing/segmentlistreader.h
  preprocessing/tubeloader.h
  )

set(fortrancppinterface_src
  fortrancppinterface/cppinterface.h
  fortrancppinterface/cppwrapper_windows.h
  fortrancppinterface/cppwrapper_linux.h
  fortrancppinterface/cgal_interface.hpp
  fortrancppinterface/fbm_func.hpp
  fortrancppinterface/init_func.hpp
  fortrancppinterface/io_func.hpp
  fortrancppinterface/set_get_func.hpp
  fortrancppinterface/soft_body_func.hpp
  fortrancppinterface/softbodyinterface.hpp
  fortrancppinterface/interface_default.hpp
  fortrancppinterface/cppinterface.cpp
  )

# assign the source files to the inshape3dcore_src variable
set(inshape3dcore_src 
  contactgraph.cpp
  particlefactory.cpp
  parinfo.cpp
  rigidbody.cpp
  compoundbody.cpp
  subdomain.cpp
  subdomainboundary.cpp
  world.cpp
  worldparameters.cpp
  application.h
  contactgraph.h
  particlefactory.h
  parinfo.h
  rigidbody.h
  compoundbody.h
  subdomain.h
  subdomainboundary.h
  world.h
  worldparameters.h
  fcdefines.hpp
  )

source_group(distance FILES ${distance_src})
source_group(intersection FILES ${intersection_src})
source_group(postprocessing FILES ${postprocessing_src})
source_group(collision FILES ${collision_src})
source_group(shapes FILES ${shapes_src})
source_group(grid FILES ${grid_src})
source_group(physics FILES ${physics_src})
source_group(preprocessing FILES ${preprocessing_src})
source_group(fortrancppinterface FILES ${fortrancppinterface_src})
source_group(modelsandmeshes FILES ${modelsandmeshes_src})

if(FC_CUDA_SUPPORT)
  cuda_add_library (inshape3dcore ${inshape3dcore_src} ${distance_src} ${intersection_src} ${postprocessing_src} ${collision_src}
    ${shapes_src} ${grid_src} ${physics_src} ${preprocessing_src} ${fortrancppinterface_src} ${modelsandmeshes_src}) 
else(FC_CUDA_SUPPORT)
  add_library (inshape3dcore ${inshape3dcore_src} ${distance_src} ${intersection_src} ${postprocessing_src} ${collision_src}
    ${shapes_src} ${grid_src} ${physics_src} ${preprocessing_src} ${fortrancppinterface_src} ${modelsandmeshes_src}) 
endif(FC_CUDA_SUPPORT)

target_compile_options(inshape3dcore PRIVATE ${CXX_FLAGS_FC})
target_include_directories(inshape3dcore PRIVATE ${FC_INCLUDE_DIRS} ${ODE_DIR} ${ODE_DIR}/include)

if(USE_OPENMESH)
  target_compile_definitions(inshape3dcore PUBLIC WITH_OPENMESH=1)
endif(USE_OPENMESH)

if(USE_ODE)
  set(ODE_DIR ../libs/ode-cmake)
  target_compile_definitions(inshape3dcore PUBLIC WITH_ODE=1)
  target_include_directories(inshape3dcore PRIVATE ${ODE_DIR} ${ODE_DIR}/include)
endif(USE_ODE)


if(USE_CGAL)
  target_compile_definitions(inshape3dcore PUBLIC WITH_CGAL=1)
  target_include_directories(inshape3dcore PRIVATE ${CMAKE_SOURCE_DIR}/extern/libraries/cgal-install/include)
  target_include_directories(inshape3dcore PRIVATE ${Boost_INCLUDE_DIRS})
  target_include_directories(inshape3dcore PRIVATE ${CGAL_INCLUDE_DIR} ${GMP_INCLUDE_DIR} ${MPFR_INCLUDE_DIR})
#  message("Boost include directory: ${Boost_INCLUDE_DIRS}")
#  message("Boost library directory: ${Boost_LIBRARY_DIRS}")
#  message("Boost library directory: ${Boost_LIBRARIES}")
endif(USE_CGAL)

if(WIN32)
  if(USE_OPTICALTWEEZERS)
    target_link_libraries(inshape3dcore Utility Math ${OPTICALTWEEZERS_LIBRARIES})
    target_compile_definitions(inshape3dcore PUBLIC _USE_MATH_DEFINES OPTIC_FORCES=1)
  else(USE_OPTICALTWEEZERS)
    target_compile_definitions(inshape3dcore PUBLIC _USE_MATH_DEFINES)
    target_link_libraries(inshape3dcore Utility Math ${CGAL_LIBRARIES} ${GMP_LIBRARIES} ${MPFR_LIBRARIES})

  endif(USE_OPTICALTWEEZERS)
else(WIN32)
  if(USE_OPTICALTWEEZERS)
    target_link_libraries(inshape3dcore stdc++fs Utility Math ${OPTICALTWEEZERS_LIBRARIES})
    target_compile_definitions(inshape3dcore PUBLIC OPTIC_FORCES=1)
  else(USE_OPTICALTWEEZERS)
    target_link_libraries(inshape3dcore stdc++fs Utility Math)
  endif(USE_OPTICALTWEEZERS)
endif(WIN32)


