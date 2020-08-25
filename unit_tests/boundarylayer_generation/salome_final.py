#./salome -t import_unv1.py
from salome.gui import helper
from salome.smesh import smeshBuilder
import SMESH
from SMESH_mechanic import *
import tempfile
import os
import sys

smesh = smeshBuilder.New()
#smesh.SetEnablePublish( False ) # Set to False to avoid publish in study if not needed or in some particular situations:
                                 # multiples meshes built in parallel, complex and numerous mesh edition (performance)

Cube_Hexa_unv = smesh.CreateMeshesFromUNV(r'%s/myout.unv' %sys.argv[1])

myMesh = Cube_Hexa_unv
boundaryGroup = myMesh.MakeBoundaryElements(SMESH.BND_2DFROM3D, "bndry2")

theNewFaceList = myMesh.GetElementsByType(SMESH.FACE)

lastIdx = len(theNewFaceList) - 1

rangeFilter = smesh.GetFilter(SMESH.FACE, SMESH.FT_RangeOfIds, Threshold="%i-%i" %(theNewFaceList[lastIdx-1], theNewFaceList[lastIdx]))
ids = myMesh.GetIdsFromFilter(rangeFilter)
rangeGroup = myMesh.GroupOnFilter( SMESH.FACE, "FaceOnRotor", rangeFilter)


cr = myMesh.GetGroupByName("CyclicR")
cl = myMesh.GetGroupByName("CyclicL")
cr[0].SetName("oldCyclicR")
cl[0].SetName("oldCyclicL")

# Add final cyclicr group
cr[0].GetName()
cr[0].GetListOfID()
cr[0].GetListOfID()[0]
faceID = cr[0].GetListOfID()[0]
faceID
filter = smesh.GetFilter(SMESH.FACE, SMESH.FT_CoplanarFaces, faceID, Tolerance = 30.0)
ids = myMesh.GetIdsFromFilter(filter)
CyclicR = myMesh.GroupOnFilter( SMESH.FACE, "CyclicR", filter)

# Add final cyclicl group
cl[0].GetName()
cl[0].GetListOfID()
cl[0].GetListOfID()[0]
faceID = cl[0].GetListOfID()[0]
faceID
filter = smesh.GetFilter(SMESH.FACE, SMESH.FT_CoplanarFaces, faceID, Tolerance = 30.0)
ids = myMesh.GetIdsFromFilter(filter)
CyclicL = myMesh.GroupOnFilter( SMESH.FACE, "CyclicL", filter)

# Add final rotor group
faceID = rangeGroup.GetListOfID()[0]
filter = smesh.GetFilter(SMESH.FACE, SMESH.FT_CoplanarFaces, faceID, Tolerance = 30.0)
RotorI = myMesh.GroupOnFilter( SMESH.FACE, "RotorI", filter)


# Add final stator group
statorGroup = myMesh.GetGroupByName("FaceOnStator")
faceID = statorGroup[0].GetListOfID()[0]
filter = smesh.GetFilter(SMESH.FACE, SMESH.FT_CoplanarFaces, faceID, Tolerance = 30.0)
RotorI = myMesh.GroupOnFilter( SMESH.FACE, "StatorI", filter)

allGroups = myMesh.GetGroups()
for group in allGroups:
    if group.GetName() not in ("CyclicR", "CyclicL", "StatorI", "RotorI"):
        myMesh.RemoveGroup(group)

vols = myMesh.NbVolumes()
print("Number of volumes in mesh %i" %vols)

try:
  myMesh.ExportUNV(r'%s' %(sys.argv[2]))
  print('Mesh %s was created successfully.' %sys.argv[2])
  pass
except:
  print('ExportUNV() failed. Invalid file name?')
