from salome.gui import helper
from salome.smesh import smeshBuilder
import SMESH
from SMESH_mechanic import *
import tempfile
import os

meshRef = helper.getSObjectSelected()[0].GetObject()
smesh = smeshBuilder.New(salome.myStudy)
myMesh = smesh.Mesh(meshRef)
rotorGroup = myMesh.GetGroupByName("Stator")
rotorGroup[0].GetName()
rotorGroup[0].GetListOfID()
rotorGroup[0].GetListOfID()[0]
faceID = rotorGroup[0].GetListOfID()[0]
faceID
filter = smesh.GetFilter(SMESH.FACE, SMESH.FT_CoplanarFaces, faceID, Tolerance = 30.0)
ids = myMesh.GetIdsFromFilter(filter)
print (len(ids))
myGroup = myMesh.GroupOnFilter( SMESH.FACE, "group on filter", filter)
myGroup.Size()
myMesh.Compute()
aPath = "/home/raphael/code/GitHub/FC-2019/FullC0ntact/unit_tests/boundarylayer_generation/StatorI.dat"
datFile = tempfile.NamedTemporaryFile(suffix=".dat").name
myMesh.ExportDAT(datFile, meshPart=myGroup)
os.rename(datFile, aPath)
