import sys
import os
import getopt
import openmesh as om
import numpy as np

def normalize(v):
    length = np.linalg.norm(v)
    if length == 0:
        return v
    else: 
        return v/length

def getScaledMesh(mesh, inMesh, dist):
    myMesh = om.read_trimesh(inMesh)
    myMesh.request_vertex_normals()
    myMesh.request_face_normals()
    myMesh.update_normals()
    myMesh.update_face_normals()
    myMesh.update_vertex_normals()

    for vh in myMesh.vertices():
        normal = myMesh.normal(vh)
        normal[2] = 0.0
        hqNormal = normalize(normal)
        point = myMesh.point(vh) + dist * hqNormal
        myMesh.set_point(vh, point)
        
    return myMesh 

def main():
    inputMesh = "statori.off"
    print("Input mesh name: ", inputMesh)
    offMeshStart = om.read_trimesh("statori.off")
    print("Number of vertices: ", len(offMeshStart.vertices()))

    scalingFactor = 0.15

    scaledMesh = getScaledMesh(offMeshStart, inputMesh, scalingFactor)
    om.write_mesh('baseMeshLayer1.off', scaledMesh)

if __name__ == "__main__":
    main()