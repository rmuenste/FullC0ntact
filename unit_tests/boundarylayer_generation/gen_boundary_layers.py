
#!/usr/bin/env python
# vim: set filetype=python
"""
Entry point for the boundary layer generation

"""

import os
import sys
import re
import getopt
import shutil
import subprocess
import math

#===============================================================================
#                        usage function
#===============================================================================
def usage():
    """
    Print out usage information
    """
    print("Hello World!")

def generateBoundaryLayers(workingDir, startMesh, numLayers, thickness):

    surfaceMeshName = startMesh

    for i in range(numLayers):

        prismName = "prismLayer" + str(i) + ".vtk"

        #outputLayerName = " " + outputOFF + "layer" + str(i) + ".off"

        nextBaseMesh = "baseMeshLayer" + str(i+1) + ".off"

        # Command for the OpenMesh based BL generation
        command2 = "./boundarylayer_generator 4 %s %s %s %s %f" %(os.path.basename(surfaceMeshName), workingDir,  prismName, nextBaseMesh, thickness)

        subprocess.call([command2], shell=True)

        print("Boundary layer iteration: ", i, " finished.")

        surfaceMeshName = nextBaseMesh

    subprocess.call(['./boundarylayer_generator 3 %s %i %s' %("baseMeshLayer", numLayers, workingDir)], shell=True)

def main():

    blenderBase = "/home/raphael/code/GitHub/FC-2019/bin/unit_tests/openmesh_traits/simpleNetzsch_base.blend"

    #outputOFF = "/home/raphael/code/GitHub/FC-2019/bin/unit_tests/openmesh_traits/tryD.off"
    outputOFF = "/home/raphael/code/GitHub/FC-2019/bin/unit_tests/openmesh_traits/"

    #thickness = -0.0125
    thickness = 0.00033

#    startMesh = "/home/raphael/code/GitHub/FC-2019/bin/unit_tests/openmesh_traits/netzschLayerStart.off"
    startMesh = "/home/raphael/code/GitHub/FC-2019/bin/unit_tests/openmesh_traits/realNetzsch.off"

    method = 2

    numLayers = 2

    try:
        opts, args = getopt.getopt(sys.argv[1:], 's:b:o:t:m:l:h',
                                   ['start-mesh=', 'blender-file=', 'output-off=', 'thickness=', 'method=',
                                    'layers=',
                                    'help'])

    except getopt.GetoptError:
        usage()
        sys.exit(2)

    for opt, arg in opts:
        if opt in ('-h', '--help'):
            usage()
            sys.exit(2)
        elif opt in ('-s', '--start-mesh'):
            startMesh = arg
        elif opt in ('-b', '--blender-file'):
            blenderBase = arg
        elif opt in ('-o', '--output-off'):
            outputOFF = arg
        elif opt in ('-t', '--thickness'):
            thickness = float(arg)
        elif opt in ('-l', '--layers'):
            numLayers = int(arg)
        elif opt in ('-m', '--method'):
            method = int(arg)
        else:
            usage()
            sys.exit(2)

    # 0.15
    layerMeshNames = []
    surfaceMeshName = startMesh
    base = surfaceMeshName.split("/")
    baseName = base[len(base)-1]
    layerMeshNames.append(baseName)
    for i in range(numLayers):

        prismName = "prismLayer" + str(i) + ".vtk"
        outputLayerName = " " + outputOFF + "layer" + str(i) + ".off"

        # Command for the blender BL generation
        command = "/home/raphael/bin/blender-2.79b-linux-glibc219-x86_64/blender --background -P extrude.py " \
                  "-- %s %s" \
                  "%s %f" %(blenderBase, surfaceMeshName, outputLayerName, thickness)
        nextBaseMesh = "baseMeshLayer" + str(i+1) + ".off"

        # Command for the OpenMesh based BL generation
        command2 = "./boundarylayer_generator 4 %s %s %s %s %f" %(os.path.basename(surfaceMeshName), os.path.basename(outputLayerName),  prismName, nextBaseMesh, thickness)
        if method == 1:
          subprocess.call([command], shell=True)
          subprocess.call(['./boundarylayer_generator 2 %s %s %s %s' %(surfaceMeshName, outputLayerName, prismName, nextBaseMesh)], shell=True)
        else:
          subprocess.call([command2], shell=True)

        print("Boundary layer iteration: ", i, " finished.")
        #surfaceMeshName = "surfMeshLayer" + str(i) + ".off"
        surfaceMeshName = nextBaseMesh
        layerMeshNames.append(surfaceMeshName)

    subprocess.call(['./boundarylayer_generator 3 %s %i' %("baseMeshLayer", numLayers)], shell=True)

if __name__ == "__main__":
    main()
