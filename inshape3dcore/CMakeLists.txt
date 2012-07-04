#________________________________________________________________
#
# Inshape3d Library : inshape3dcore folder CMakeLists.txt file
# Author            : Raphael Muenster
# Date of creation  : 26.10.2010
# Data of version   : 27.10.2010
# Version number    : 00.002
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

# set the include directories
include_directories (. ../math ../util collision distance grid fortrancppinterface intersection physics postprocessing preprocessing shapes shapes/modelsandmeshes
../util/googlehash/google ../util/googlehash/google/sparsehash ../util/googlehash/windows ../util/googlehash
../util/googlehash/windows/google/sparsehash)

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
intersection/intersectorray3tri3.h
intersection/intersectorspheresphere.h
intersection/intersectortools.h
intersection/intersectorspheresegment.h
intersection/intersector2aabb.cpp
intersection/intersector2obb3.cpp
intersection/intersectorplaneobb3.cpp
intersection/intersectoraabbray.cpp
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
collision/broadphase.h
collision/collisioninfo.h
collision/collisionpipeline.h
collision/collresponsesi.h
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
collision/collidermeshsphere.h
collision/collidermeshmesh.h
collision/collidermeshboundarybox.h
collision/contactgenerator.h
collision/contactgroup.h
collision/contactgenerator2cylinder.h
collision/contactgeneratorboxcylinder.h
collision/contactgeneratorcylindersphere.h
collision/broadphasestrategy.h
collision/broadphasestrategygrid.h
collision/broadphasestrategyhgrid.h
collision/broadphasestrategyrmt.h
collision/collisionhash.h
collision/stacklayer.h
collision/boundary.cpp
collision/boundarybox.cpp
collision/broadphase.cpp
collision/collisionhash.cpp
collision/collisioninfo.cpp
collision/collisionpipeline.cpp
collision/colliderboxbox.cpp
collision/colliderboxboxboundary.cpp
collision/collresponse.cpp
collision/collresponsesi.cpp
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
collision/collidersphereplane.cpp
collision/colliderspherecompound.cpp
collision/colliderspheresphere.cpp
collision/colliderspheresubdomain.cpp
collision/collidermeshmesh.cpp
collision/colliderboxsphere.cpp
collision/contactgenerator.cpp
collision/contactgroup.cpp
collision/contactgenerator2cylinder.cpp
collision/contactgeneratorboxcylinder.cpp
collision/contactgeneratorcylindersphere.cpp
collision/broadphasestrategy.cpp
collision/broadphasestrategygrid.cpp
collision/broadphasestrategyhgrid.cpp
collision/broadphasestrategyrmt.cpp
collision/stacklayer.cpp
)

set(shapes_src
shapes/obb3.h
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
grid/simplespatialhash.cpp
grid/spatialhash.cpp
grid/deformparameters.h
grid/grid3d.h
grid/hspatialhash.h
grid/implicitgrid.cpp
grid/simplespatialhash.h
grid/spatialhash.h
grid/unstructuredgrid.h
)

set(physics_src
physics/rigidbodymotion.cpp
physics/motionintegratorsi.cpp
physics/rigidbodymotion.h
physics/motionintegratorsi.h
)

set(preprocessing_src
preprocessing/3dsloader.cpp
preprocessing/basicmodelloader.cpp
preprocessing/genericloader.cpp
preprocessing/objloader.cpp
preprocessing/reader.cpp
preprocessing/tubeloader.cpp
preprocessing/3dsloader.h
preprocessing/basicmodelloader.h
preprocessing/genericloader.h
preprocessing/objloader.h
preprocessing/reader.h
preprocessing/tubeloader.h
)

set(fortrancppinterface_src
fortrancppinterface/cppinterface.h
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
contactgraph.h
particlefactory.h
parinfo.h
rigidbody.h
compoundbody.h
subdomain.h
subdomainboundary.h
world.h
worldparameters.h
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
add_library (inshape3dcore ${inshape3dcore_src} ${distance_src} ${intersection_src} ${postprocessing_src} ${collision_src}
                           ${shapes_src} ${grid_src} ${physics_src} ${preprocessing_src} ${fortrancppinterface_src} ${modelsandmeshes_src}) 


# link libraries
target_link_libraries(inshape3dcore Utility Math)

SET_TARGET_PROPERTIES(inshape3dcore PROPERTIES COMPILE_FLAGS "-fopenmp" LINK_FLAGS "-fopenmp")


