import bpy,os,re
import mathutils
import getopt
import sys
import math
import datetime
import numpy as np

#/home/raphael/bin/blender-2.79b-linux-glibc219-x86_64/blender --background -P extrude.py -- /home/raphael/code/GitHub/FC-2019/bin/unit_tests/openmesh_traits/simpleNetzsch_hull_only.blend /home/raphael/code/GitHub/FC-2019/bin/unit_tests/openmesh_traits/tryD.off

#/home/raphael/bin/blender-2.79b-linux-glibc219-x86_64/blender --background -P extrude.py -- /home/raphael/code/GitHub/FC-2019/bin/unit_tests/openmesh_traits/simpleNetzsch_hull_only.blend /home/raphael/code/GitHub/FC-2019/bin/unit_tests/openmesh_traits/tryD.off -0.00125

#===============================================================================
#                        usage function
#===============================================================================
def usage():
    """
    Print out usage information
    """
    print("Usage: python e3d_start.py [options]")
    print("Where options can be:")
    print("[-f', '--project-folder]: Path to project folder containing a setup.e3d file")
    print("[-n', '--num-processors]: Number of processors to use")
    print("[-p', '--periodicity']: Periodicity of the solution (1, 2, 3, ... " +
          "usually the time flight number)")
    print("[-a', '--angle]: The angular step size between two simulations in " +
          "the sim loop (default 10)")
    print("[-c', '--host-conf]: A hostfile as input for the mpirun command")
    print("[-r', '--rank-file]: A rankfile as input for the mpirun command")
    print("[-t', '--time]: Number of time levels to complete a full 360 rotation")
    print("[-h', '--help']: prints this message")
    print("Example: python ./e3d_start.py -f myFolder -n 5 -t 0")


def getObjectByName(name):
    myObjects = bpy.data.objects
    for o in myObjects:
        if re.search(name, o.name):
            return o

def printAllObjects():
    myObjects = bpy.data.objects
    for o in myObjects:
        print("found name: ", o.name)

def makeSelected(myObject):
    myObject.select = True
    bpy.context.scene.objects.active = myObject

def main():
    print("options: ", sys.argv)
    inputFile = sys.argv[5]
    inputOff = sys.argv[6]
    outputFile = sys.argv[7]
    thickness = float(sys.argv[8])

    bpy.ops.wm.open_mainfile(filepath=inputFile)
    bpy.ops.import_mesh.off(filepath=inputOff,
                            axis_forward='Y', axis_up='Z')
    o1 = inputOff.split('.')
    opath = o1[0].split('/')
    objectName = opath[len(opath)-1]
    print("Name", objectName)
    net = getObjectByName(objectName)
    makeSelected(net)
    bpy.ops.object.modifier_add(type='SOLIDIFY')
    bpy.context.object.modifiers["Solidify"].use_even_offset = True
    bpy.context.object.modifiers["Solidify"].use_quality_normals = True
    bpy.context.object.modifiers["Solidify"].thickness = thickness
    bpy.ops.object.modifier_apply(apply_as='DATA', modifier="Solidify")
    bpy.ops.export_mesh.off(filepath=outputFile,
                            axis_forward='Y', axis_up='Z')

if __name__ == "__main__":
    main()