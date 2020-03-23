#./salome -t import_unv1.py
from salome.gui import helper
from salome.smesh import smeshBuilder
import SMESH
from SMESH_mechanic import *
import tempfile
import os
import sys

print(sys.argv)

smesh = smeshBuilder.New()
#smesh.SetEnablePublish( False ) # Set to False to avoid publish in study if not needed or in some particular situations:
                                 # multiples meshes built in parallel, complex and numerous mesh edition (performance)

#Cube_Hexa_unv = smesh.CreateMeshesFromUNV(r'/home/rafa/code/GitHub/FC-2020/install/bin/meshtraits/Netsch_Test_2.unv')
Cube_Hexa_unv = smesh.CreateMeshesFromUNV(r'%s' %sys.argv[1])
try:
  Cube_Hexa_unv.ExportUNV(r'%s' %sys.argv[2])
  #Cube_Hexa_unv.ExportUNV(r'/home/rafa/code/GitHub/FC-2020/install/bin/meshtraits/Netsch_Test_2.unv' %sys.argv[1])
  #Cube_Hexa_unv.ExportUNV(r'/home/rafa/code/GitHub/FC-2020/install/bin/meshtraits/start.unv')
  pass
except:
  print('ExportUNV() failed. Invalid file name?')

#meshRef = helper.getSObjectSelected()[0].GetObject()
#smesh = smeshBuilder.New(salome.myStudy)
#myMesh = smesh.Mesh(meshRef)

myMesh = Cube_Hexa_unv

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
#myGroup.Size()
myMesh.Compute()
aPath = "%s/StatorI.dat" %sys.argv[3]
myMesh.ExportDAT(aPath, meshPart=myGroup)
#datFile = tempfile.NamedTemporaryFile(suffix=".dat").name
#myMesh.ExportDAT(datFile, meshPart=myGroup)
#os.rename(datFile, aPath)
