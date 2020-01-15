import sys
import os
import getopt

#===============================================================================
#                        usage function
#===============================================================================
def usage():
    """
    Print out usage information
    """
    print("Usage: python boundarylayer_generation.py [options]")
    print("Where options can be:")
    print("[-u', '--unv-mesh]: path to input .unv file")
    print("[-b', '--base-layer]: path to the .off file of the first boundary layer")
    print("[-o', '--orig-mesh]: path to the .dat file of the layer to which we want to attach the boundary layers")
    print("[-h', '--help']: prints this message")
    print("Example: python3 ./unv_io.py -u netzsch_outer2.unv -b baseMeshLayer1.off -o RotorI3.dat")

def main():
    print("Hello World!")

    unvMesh = ""
    origMesh = ""
    baseLayer = ""

    try:
        opts, args = getopt.getopt(sys.argv[1:], 'u:o:b:t:m:h',
                                   ['unv-mesh=', 'orig-mesh=', 'base-layer=', 'thickness=', 'method='
                                    'help'])

    except getopt.GetoptError:
        usage()
        sys.exit(2)

    for opt, arg in opts:
        if opt in ('-h', '--help'):
            usage()
            sys.exit(2)
        elif opt in ('-u', '--unv-mesh'):
            unvMesh = arg
        elif opt in ('-b', '--base-layer'):
            baseLayer = arg
        elif opt in ('-o', '--orig-mesh'):
            origMesh = arg
        elif opt in ('-t', '--thickness'):
            thickness = float(arg)
        elif opt in ('-m', '--method'):
            method = int(arg)
        else:
            usage()
            sys.exit(2)

#==============================================================================
#    mesh = om.read_trimesh(baseLayer)
#
#    # read the original mesh info
#    (meshInfo, vertexMap) = readDatInfo(origMesh)
#
#    print("Parsing input UNV file")
#    # load the volume mesh we want to merge the layers into 
#    (units, coordSystem, coords, connect, props) = parseUNV2(unvMesh)
#    print("done parsing")
#
#    vertexLayers = []
#    vertexLayers.append("baseMeshLayer2.off")
#    vertexLayers.append("baseMeshLayer3.off")
#
#    # add the vertices
#    print("Adding Vertices")
#    #newCoords, nverts = addVertexList(mesh, coords, vertexLayers)
#    newCoords, nverts = addVertexList2(mesh, coords, vertexLayers)
#    print("done")
#
#    print("Adding Elements")
#    # add the elements
#    #newElements = addElements(mesh, connect, meshInfo, vertexMap, nverts)
#    newElements = addElements2(mesh, connect, meshInfo, vertexMap, nverts)
#    print("done")
#
#    # write the new unv
#    writeUNV2("myout.unv", (units, coordSystem, newCoords, newElements, props))
#    #writeUNV("myout.unv", (units, coordSystem, newCoords, connect, props))

if __name__ == "__main__":
    main()