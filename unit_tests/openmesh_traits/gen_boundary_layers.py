
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

def main():

    blenderBase = "/home/raphael/code/GitHub/FC-2019/bin/unit_tests/openmesh_traits/simpleNetzsch_base.blend"

    #outputOFF = "/home/raphael/code/GitHub/FC-2019/bin/unit_tests/openmesh_traits/tryD.off"
    outputOFF = "/home/raphael/code/GitHub/FC-2019/bin/unit_tests/openmesh_traits/"

    #thickness = -0.0125
    thickness = 0.00033

#    startMesh = "/home/raphael/code/GitHub/FC-2019/bin/unit_tests/openmesh_traits/netzschLayerStart.off"
    startMesh = "/home/raphael/code/GitHub/FC-2019/bin/unit_tests/openmesh_traits/realNetzsch.off"

    try:
        opts, args = getopt.getopt(sys.argv[1:], 'b:o:t:h',
                                   ['blender-file=', 'output-off=', 'thickness='
                                    'help'])

    except getopt.GetoptError:
        usage()
        sys.exit(2)

    for opt, arg in opts:
        if opt in ('-h', '--help'):
            usage()
            sys.exit(2)
        elif opt in ('-b', '--blender-file'):
            blenderBase = arg
        elif opt in ('-o', '--output-off'):
            outputOFF = arg
        elif opt in ('-t', '--thickness'):
            thickness = float(arg)
        else:
            usage()
            sys.exit(2)

    method = 2
    layerMeshNames = []
    surfaceMeshName = startMesh
    base = surfaceMeshName.split("/")
    baseName = base[len(base)-1]
    layerMeshNames.append(baseName)
    for i in range(3):
        prismName = "prismLayer" + str(i) + ".vtk"
        outputLayerName = " " + outputOFF + "layer" + str(i) + ".off"
        command = "/home/raphael/bin/blender-2.79b-linux-glibc219-x86_64/blender --background -P extrude.py " \
                  "-- %s %s" \
                  "%s %f" %(blenderBase, surfaceMeshName, outputLayerName, thickness)
        nextBaseMesh = "baseMeshLayer" + str(i+1) + ".off"

        command2 = "./meshtraits 4 %s %s %s %s %f" %(os.path.basename(surfaceMeshName), os.path.basename(outputLayerName),  prismName, nextBaseMesh, thickness)
        if method == 1:
          subprocess.call([command], shell=True)
          subprocess.call(['./meshtraits 2 %s %s %s %s' %(surfaceMeshName, outputLayerName, prismName, nextBaseMesh)], shell=True)
        else:
          subprocess.call([command2], shell=True)

        print("Boundary layer iteration: ", i)
        #surfaceMeshName = "surfMeshLayer" + str(i) + ".off"
        surfaceMeshName = nextBaseMesh
        layerMeshNames.append(surfaceMeshName)

    subprocess.call(['./meshtraits 3 %s %s %s %s' %(layerMeshNames[0], layerMeshNames[1], layerMeshNames[2], layerMeshNames[3])], shell=True)

if __name__ == "__main__":
    main()