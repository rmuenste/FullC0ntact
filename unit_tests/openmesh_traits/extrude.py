import bpy,os,re
import mathutils
import sys
import math
import datetime
import numpy as np

def getObjectByName(name):
    myObjects = bpy.data.objects
    for o in myObjects:
        if re.search(name, o.name):
            return o

def makeSelected(myObject):
    myObject.select = True
    bpy.context.scene.objects.active = myObject

bpy.ops.wm.open_mainfile(filepath="I:\\work\\book\\netzschreal.blend")

net = getObjectByName("Netzsch1")
makeSelected(net)
bpy.ops.object.modifier_add(type='SOLIDIFY')
bpy.context.object.modifiers["Solidify"].use_even_offset = True
bpy.context.object.modifiers["Solidify"].use_quality_normals = True
bpy.context.object.modifiers["Solidify"].thickness = -0.0000125
bpy.ops.object.modifier_apply(apply_as='DATA', modifier="Solidify")
bpy.ops.export_mesh.stl(filepath="I:\\work\\book\\mesh.stl", 
                        check_existing=False, 
                        use_selection=True, 
                        ascii=False, 
                        use_mesh_modifiers=True, 
                        axis_forward='Y', axis_up='Z')
