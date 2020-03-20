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

boundaryGroup = myMesh.MakeBoundaryElements(SMESH.BND_2DFROM3D, "bndry")

theNewFaceList = myMesh.GetElementsByType(SMESH.FACE)

lastIdx = len(theNewFaceList) - 1
print("New LastIdx: ",lastIdx)

rangeFilter = smesh.GetFilter(SMESH.FACE, SMESH.FT_RangeOfIds, Threshold="%i-%i" %(theNewFaceList[lastIdx-1], theNewFaceList[lastIdx]))
ids = myMesh.GetIdsFromFilter(rangeFilter)
print("Ids: ", str(ids))
rangeGroup = myMesh.GroupOnFilter( SMESH.FACE, "FaceOnStator", rangeFilter)

rotorGroup = myMesh.GetGroupByName("Rotor")
rotorGroup[0].GetName()
rotorGroup[0].GetListOfID()
rotorGroup[0].GetListOfID()[0]
faceID = rotorGroup[0].GetListOfID()[0]
faceID
filter = smesh.GetFilter(SMESH.FACE, SMESH.FT_CoplanarFaces, faceID, Tolerance = 30.0)
ids = myMesh.GetIdsFromFilter(filter)
print (len(ids))
myGroup = myMesh.GroupOnFilter( SMESH.FACE, "group on filter", filter)
#myGroup.Size()
myMesh.Compute()
aPath = "%s/RotorI.dat" %sys.argv[1]
datFile = tempfile.NamedTemporaryFile(suffix=".dat").name
myMesh.ExportDAT(datFile, meshPart=myGroup)
os.rename(datFile, aPath)

try:
  myMesh.ExportUNV(r'%s/outer.unv' %sys.argv[1])
  pass
except:
  print('ExportUNV() failed. Invalid file name?')
