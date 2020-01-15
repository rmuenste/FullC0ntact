# UNV Dataset 2467
#================================================================================================================
# See also format description:
# http://sdrl.uc.edu/sdrl/referenceinfo/universalfileformats/file-format-storehouse/universal-dataset-number-2467
#================================================================================================================
# 
# 2467
#    group number                                                        #n of entities in group
#         2         0         0         0         0         0         0         4
#(group name)
#Inlet
#  (entity type)(entity id)                (entity type)(entity id) 
#         8         1         0         0         8         2         0         0
#         8         3         0         0         8         4         0         0
#    group number                                                        #n of entities in group
#         3         0         0         0         0         0         0        16
#(group name)
#Wall
#  (entity type)(entity id)                (entity type)(entity id) 
#         8         9         0         0         8        10         0         0
#         8        11         0         0         8        12         0         0
#         8        13         0         0         8        14         0         0
#         8        15         0         0         8        16         0         0
#         8        17         0         0         8        18         0         0
#         8        19         0         0         8        20         0         0
#         8        21         0         0         8        22         0         0
#         8        23         0         0         8        24         0         0
#    group number                                                        #n of entities in group
#         4         0         0         0         0         0         0         4
#(group name)
#Outlet
#  (entity type)(entity id)                (entity type)(entity id) 
#         8         5         0         0         8         6         0         0
#         8         7         0         0         8         8         0         0
#    -1

# UNV Dataset 2412
#================================================================================================================
# See also format description:
# http://sdrl.uc.edu/sdrl/referenceinfo/universalfileformats/file-format-storehouse/universal-dataset-number-2412
#================================================================================================================
# -1
# 2412
#  (entity id) (fe type)                               (#n nodes on element) 
#         1        44         2         1         7         4
#     (vertex0) (vertex1) (vertex2) (vertex3)
#         1        10        21         9
#  (entity id) (fe type)                               (#n nodes on element) 
#         2        44         2         1         7         4
#     (vertex0) (vertex1) (vertex2) (vertex3)
#         9        21        12         2
# -1

# UNV Dataset 2411
#================================================================================================================
# See also format description:
# http://sdrl.uc.edu/sdrl/referenceinfo/universalfileformats/file-format-storehouse/universal-dataset-number-2411
#================================================================================================================
#  2411
#     (node id)                     (color) 
#         1         1         1        11
#     (coordinates x y z) 
#   0.0000000000000000E+00   0.0000000000000000E+00   1.0000000000000000E+00
#     (node id)                     (color) 
#         2         1         1        11
#     (coordinates x y z) 
#   0.0000000000000000E+00   0.0000000000000000E+00   0.0000000000000000E+00
#     (node id)                     (color) 
#         3         1         1        11
#     (coordinates x y z) 
#   0.0000000000000000E+00   1.0000000000000000E+00   1.0000000000000000E+00
#     (node id)                     (color) 
#         4         1         1        11
#     (coordinates x y z) 
#   0.0000000000000000E+00   1.0000000000000000E+00   0.0000000000000000E+00
import sys
import os
import getopt
import openmesh as om

def readPatch(f, line):
    line = f.readline()
    if line.split()[0] == "-1":
        return False
    entries = line.split()
    line = f.readline()
    name = line.split()
    print("Patch name: %s , with ID %s and %s faces" %(name[0], entries[0], entries[7]))
    numEntries = int(entries[7])
    numLines = (numEntries / 2)
    if numLines % 2 != 0:
        numLines = numLines + 1
    print("Predicted #lines: %i" % numLines )
    faceEntries = []

    for i in range(int(numLines)):
        line = f.readline()
        faceEntries.append(line.split())

    print("Faces in patch %s:" % name[0])
    for entry in faceEntries:
        print(entry[1], entry[5])
    
    return True

def readSection(f, line):
    sectionName = line.split()[0]
    if line:
        print("Section ID: ",line)

    if sectionName == "2467":
        print("Section 2467 is the patch section")

        allOK = True
        while line and line.split()[0] != "-1" and allOK:
            allOK = readPatch(f, line)
        
        print("Section %s ended." %sectionName)
    else:
        while line and line.split()[0] != "-1":
            line = f.readline()
        
        print("Section %s ended." %sectionName)


def parseUNV():
    with open("Cube_Hexa_Salome2.unv", "r") as f:
        line = f.readline()
        prevLine = line 
        while line:
            prevLine = line
            prevLine = prevLine.split()
            line = f.readline()
            if prevLine[0] == "-1":
                print("Section marker found")
                readSection(f, line)

def readNextSection(f):
    line = f.readline()

    if line.split()[0] == "-1":
        beginPos = f.tell()
    
    line = f.readline()
    sectionID = line.split()[0]
    print("Section ID %s" % sectionID)

    # Read the whole section
    while line and line.split()[0] != "-1":
        line = f.readline()
    endPos = f.tell()

#    if sectionID not in ("2411", "2467", "2412"):
#        return
    
    # Go back to the beginning of the section
    f.seek(beginPos, os.SEEK_SET)
    toRead = endPos - beginPos

    # Read and return the content of the section
    content = f.read(toRead)
    content = "    -1\n" + content
    print("Section begin %s, section end %s" %(str(beginPos), str(endPos)))
    #print(content)
    return content

def parseUNV2(fileName):
    beginPos = -1 
    endPos = -1 
    unitSection = []
    coordsysSection = []
    coordSection = []
    connectivitySection = []
    propertiesSection = []

    with open(fileName, "r") as f:
        unitSection = readNextSection(f)
        coordsysSection = readNextSection(f)
        coordSection = readNextSection(f)
        connectivitySection = readNextSection(f)
        propertiesSection = readNextSection(f)
    return (unitSection, coordsysSection, coordSection, connectivitySection, propertiesSection)

def writeVertexMap(vertexMap):
    with open("vertexMap", "w+") as out:
        for key, value in vertexMap.items():
            out.write(str(value - 1) + " " + str(key) + "\n")


def convertDatToOff(fileName, outputFile):
    vertexMap = {} 
    with open(fileName, "r") as f:
        line = f.readline()
        words = line.split()
        print(words[0], words[1])
        nverts = int(words[0])
        nfaces = int(words[1])
        vcount = 1
        for i in range(nverts):
            line = f.readline()
            words = line.split()
            vertexMap[words[0]] = vcount
            vcount = vcount + 1

    with open(fileName, "r") as f:
        line = f.readline()
        with open(outputFile, "w+") as out:
            out.write("OFF\n")
            words = line.split()
            print(words[0], words[1])
            out.write(words[0] + " " + words[1] + " 0\n")
            nverts = int(words[0])
            nfaces = int(words[1])
            vcount = 1
            for i in range(nverts):
                line = f.readline()
                words = line.split()
                out.write(words[1] + " " + words[2] + " " + words[3] + "\n")
                vcount = vcount + 1
            for i in range(nfaces):
                line = f.readline()
                words = line.split()
                ids = [i for i in words]
                ids = ids[2:5]
                ids = [vertexMap[i] for i in ids]
                ids = [i-1 for i in ids]
                out.write("3 " + str(ids[0]) + " " + str(ids[1]) + " " + str(ids[2]) + "\n")
        #print(line)

def readDatFile(fileName):
    vertexMap = {}
    meshInfo = {}
    with open(fileName, "r") as f:
        line = f.readline()
        words = line.split()
        #print(words[0], words[1])
        nverts = int(words[0])
        nfaces = int(words[1])
        vcount = 1
        for i in range(nverts):
            line = f.readline()
            words = line.split()
            vertexMap[words[0]] = vcount
            vcount = vcount + 1

    with open(fileName, "r") as f:
        line = f.readline()
        words = line.split()

        nverts = int(words[0])
        nfaces = int(words[1])

        meshInfo['nverts'] = nverts
        meshInfo['nfaces'] = nfaces
        meshInfo['faces'] = []
        meshInfo['vertices'] = []

        for i in range(nverts):
            line = f.readline()
            words = line.split()
            meshInfo['vertices'].append(words[1:4])

        for i in range(nfaces):
            line = f.readline()
            words = line.split()
            meshInfo['faces'].append(words[2:5])

    return (meshInfo, vertexMap)

def readDatInfo(fileName):
    vertexMap = {}
    with open(fileName, "r") as f:
        line = f.readline()
        words = line.split()
        #print(words[0], words[1])
        nverts = int(words[0])
        nfaces = int(words[1])
        vcount = 1
        for i in range(nverts):
            line = f.readline()
            words = line.split()
            vertexMap[words[0]] = vcount
            vcount = vcount + 1

    meshInfo = {}

    with open(fileName, "r") as f:
        line = f.readline()
        words = line.split()

        nverts = int(words[0])
        nfaces = int(words[1])

        meshInfo['nverts'] = nverts
        meshInfo['nfaces'] = nfaces
        meshInfo['faces'] = []

        for i in range(nverts):
            line = f.readline()

        for i in range(nfaces):
            line = f.readline()
            words = line.split()
            meshInfo['faces'].append(words[2:5])

    return (meshInfo, vertexMap)

def parseVertices(coordSection):
    lines = coordSection.splitlines()

    vertices = []

    for i in range(2, len(lines)-2, 2):
        entry = {}
        entry['desc'] = lines[i]
        entry['coords'] = lines[i+1]
        entry['id'] = lines[i].split()[0]
#        print(entry['desc'])
#        print(entry['coords'])
#        print(entry['id'])
        vertices.append(entry)

    print("Number of vertices: ", len(vertices))
#    print(vertices)
    return vertices

def parseElements(connect):
    lines = connect.splitlines()

    elements = []
    i = 2
    while i < len(lines)-2:
        words = lines[i].split()
        entry = {}
        if words[1] == "11":
            entry['type'] = words[1]
            entry['line1'] = lines[i]
            entry['line2'] = lines[i+1]
            entry['line3'] = lines[i+2]
            entry['id'] = words[0]
            i = i + 3
        else:
            entry['type'] = words[1]
            entry['line1'] = lines[i]
            entry['line2'] = lines[i+1]
            entry['id'] = words[0]
            i = i + 2

        elements.append(entry)
    
#    for i in elements:
#        print(i)
    print("Number of FE: ", len(elements))
    return elements

def writeUNV(fileName, data):
    with open(fileName, "w") as out:
        out.write(data[0])
        out.write(data[1])
        out.write(data[2])
        out.write(data[3])
        out.write(data[4])

def writeUNV2(fileName, data):
    with open(fileName, "w") as out:
        out.write(data[0])
        out.write(data[1])
        # Write the vertex data
        out.write("    -1\n" + "    2411\n")

        for entry in data[2]:
            out.write(entry['desc'] + "\n" + entry['coords'] + "\n")
            
        out.write("    -1\n")

        # Write the element data
        out.write("    -1\n" + "    2412\n")
        for entry in data[3]:
            if entry['type'] == "11":
                out.write(entry['line1'] + "\n" + entry['line2'] + "\n" + entry['line3'] + "\n")
            else:
                out.write(entry['line1'] + "\n" + entry['line2'] + "\n")

        out.write("    -1\n")

        # Write the group data
        out.write(data[4])

def convertVerticesToString(vertices):
    outString = ""
    outString = outString + "    -1\n" + "    2411\n"
    for entry in vertices:
#        print(entry)
        outString = outString + entry['desc'] + "\n" + entry['coords'] + "\n"
        
    outString = outString + "    -1\n"
    return outString

def convertElements(elements):
    count = 0
    outString = ""
    outString = outString + "    -1\n" + "    2412\n"
    for entry in elements:
        if entry['type'] == "11":
          outString = outString + entry['line1'] + "\n" + entry['line2'] + "\n" + entry['line3'] + "\n"
        else:
          outString = outString + entry['line1'] + "\n" + entry['line2'] + "\n"
        count = count + 1
        print(count)
        
        
        
    outString = outString + "    -1\n"
    return outString

def readOff(fileName):
    newVertices = []
    with open(fileName, "r") as f:
        f.readline()
        line = f.readline()
        words = line.split()
        nverts = int(words[0])
        print("Number of vertices in off: ", nverts)

        for i in range(nverts):
            line = f.readline()
            newVertices.append(line)
            #print(line)

    return newVertices

def addVertexList(mesh, coords, vertexLayers):
    mycoords = parseVertices(coords)

    nverts = len(mycoords)

    nextId = len(mycoords) + 1
    descLine = mycoords[nextId-2]['desc']

    idx = nextId
    for vh in mesh.vertices():

        newEntry = {}
        newEntry['desc'] = descLine

        desc = newEntry['desc'].split()
        desc[0] = "        " + str(idx)
        desc = "         ".join(desc)

        newEntry['desc'] = desc
        coordinates = [str(x) for x in mesh.point(vh)]
        newEntry['coords'] = " ".join(coordinates)
        mycoords.append(newEntry)

        idx = idx + 1

    for item in vertexLayers:
        layerMesh = om.read_trimesh(item)
        for vh in layerMesh.vertices():

            newEntry = {}
            newEntry['desc'] = descLine

            desc = newEntry['desc'].split()
            desc[0] = "        " + str(idx)
            desc = "         ".join(desc)

            newEntry['desc'] = desc
            coordinates = [str(x) for x in layerMesh.point(vh)]
            newEntry['coords'] = " ".join(coordinates)
            mycoords.append(newEntry)

            idx = idx + 1
    
       
    out = convertVerticesToString(mycoords)
    return (out, nverts)

def addVertexList2(mesh, coords, vertexLayers):
    mycoords = parseVertices(coords)

    nverts = len(mycoords)

    nextId = len(mycoords) + 1
    descLine = mycoords[nextId-2]['desc']

    idx = nextId
    for vh in mesh.vertices():

        newEntry = {}
        newEntry['desc'] = descLine

        desc = newEntry['desc'].split()
        desc[0] = "        " + str(idx)
        desc = "         ".join(desc)

        newEntry['desc'] = desc
        coordinates = [str(x) for x in mesh.point(vh)]
        newEntry['coords'] = " ".join(coordinates)
        mycoords.append(newEntry)

        idx = idx + 1

    for item in vertexLayers:
        layerMesh = om.read_trimesh(item)
        for vh in layerMesh.vertices():

            newEntry = {}
            newEntry['desc'] = descLine

            desc = newEntry['desc'].split()
            desc[0] = "        " + str(idx)
            desc = "         ".join(desc)

            newEntry['desc'] = desc
            coordinates = [str(x) for x in layerMesh.point(vh)]
            newEntry['coords'] = " ".join(coordinates)
            mycoords.append(newEntry)

            idx = idx + 1
    
       
    return (mycoords, nverts)

def addElements(mesh, elements, meshInfo, vertexMap, meshVerts):

    print("Parsing elements")
    elementSection = parseElements(elements)
    print("done")

    count = 0
    for elem in elementSection:
        if elem['type'] == '11':
            count = count + 1

    nextId = len(elementSection) + 1

    print("Adding elements")
    for faceInfo in meshInfo['faces']:
    #for i in range(1):

    #    faceInfo = meshInfo['faces'][i]

        record1 = [str(nextId), "112", "2 1 7 6"]

        record1 = " ".join(record1)

        entry = {}
        entry['type'] = "112"
        entry['line1'] = record1

        # The face in original numbering
        faces1 = [i for i in faceInfo]
#        print(faces1)

        # The corresponding face in patch numbering
        map1 = [int(vertexMap[fidx])  for fidx in faces1]
#+        print(map1)
        faces2 = [int(vertexMap[fidx]) + meshVerts  for fidx in faces1]

        entry['line2'] = "%s %s %s %i %i %i" % (faces1[0], faces1[1], faces1[2], faces2[0], faces2[1], faces2[2])
#        print(entry['line2'])

        entry['id'] = str(nextId)
        elementSection.append(entry)
        nextId = nextId + 1
        print(nextId)

    for lidx in range(0, 2):

        nverts = mesh.n_vertices()

        for fh in mesh.faces():

            faceIndices = []

            # Get the vertex indices on the patch 
            for vh in mesh.fv(fh):
                faceIndices.append(vh.idx())

            record1 = [str(nextId), "112", "2 1 7 6"]

            record1 = " ".join(record1)

            entry = {}
            entry['type'] = "112"
            entry['line1'] = record1

            # Get the vertex indices on the patch 
            faces1 = [(i + 1) + (lidx * nverts) + meshVerts for i in faceIndices]

            # The matching indices are shifted by the number of 
            # vertices in a vertex layer 
            faces2 = [vidx + nverts for vidx in faces1]

            entry['line2'] = "%s %s %s %i %i %i" % (faces1[0], faces1[1], faces1[2], faces2[0], faces2[1], faces2[2])

            entry['id'] = str(nextId)
            elementSection.append(entry)
            nextId = nextId + 1
            print(nextId)

    print("done")

    print("Converting Elements")
    out = convertElements(elementSection)
    print("done")
    #print(vertexMap)
    return out

def addElements2(mesh, elements, meshInfo, vertexMap, meshVerts):

    elementSection = parseElements(elements)

    count = 0
    for elem in elementSection:
        if elem['type'] == '11':
            count = count + 1

    nextId = len(elementSection) + 1

    for faceInfo in meshInfo['faces']:

        record1 = [str(nextId), "112", "2 1 7 6"]

        record1 = " ".join(record1)

        entry = {}
        entry['type'] = "112"
        entry['line1'] = record1

        # The face in original numbering
        faces1 = [i for i in faceInfo]

        # The corresponding face in patch numbering
        map1 = [int(vertexMap[fidx])  for fidx in faces1]
        faces2 = [int(vertexMap[fidx]) + meshVerts  for fidx in faces1]

        entry['line2'] = "%s %s %s %i %i %i" % (faces1[0], faces1[1], faces1[2], faces2[0], faces2[1], faces2[2])

        entry['id'] = str(nextId)
        elementSection.append(entry)
        nextId = nextId + 1

    for lidx in range(0, 2):

        nverts = mesh.n_vertices()

        for fh in mesh.faces():

            faceIndices = []

            # Get the vertex indices on the patch 
            for vh in mesh.fv(fh):
                faceIndices.append(vh.idx())

            record1 = [str(nextId), "112", "2 1 7 6"]

            record1 = " ".join(record1)

            entry = {}
            entry['type'] = "112"
            entry['line1'] = record1

            # Get the vertex indices on the patch 
            faces1 = [(i + 1) + (lidx * nverts) + meshVerts for i in faceIndices]

            # The matching indices are shifted by the number of 
            # vertices in a vertex layer 
            faces2 = [vidx + nverts for vidx in faces1]

            entry['line2'] = "%s %s %s %i %i %i" % (faces1[0], faces1[1], faces1[2], faces2[0], faces2[1], faces2[2])

            entry['id'] = str(nextId)
            elementSection.append(entry)
            nextId = nextId + 1

    return elementSection

def setVertices(infoA, vertexMapA, infoB, vertexMapB, coords):

    nverts = infoA['nverts']
    nfaces = infoA['nfaces']
    # meshInfo['faces'] = []
    # meshInfo['vertices'] = []

    for entry in coords:
        print(entry['coords'])

def unifyCoordinates():
    (units, coordSystem, coords, connect, props) = parseUNV2("outer_exp.unv")

    (meshInfo, vertexMap) = readDatFile("top.dat")
    (meshInfo2, vertexMap2) = readDatFile("bot.dat")

    mycoords = parseVertices(coords)

    setVertices(meshInfo, vertexMap, meshInfo2, vertexMap, mycoords)

    newCoords = convertVerticesToString(mycoords)

def usage():
    print("Hello World!")

def main():

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
    mesh = om.read_trimesh(baseLayer)

    # read the original mesh info
    (meshInfo, vertexMap) = readDatInfo(origMesh)

    print("Parsing input UNV file")
    # load the volume mesh we want to merge the layers into 
    (units, coordSystem, coords, connect, props) = parseUNV2(unvMesh)
    print("done parsing")

    vertexLayers = []
    vertexLayers.append("baseMeshLayer2.off")
    vertexLayers.append("baseMeshLayer3.off")

    # add the vertices
    print("Adding Vertices")
    #newCoords, nverts = addVertexList(mesh, coords, vertexLayers)
    newCoords, nverts = addVertexList2(mesh, coords, vertexLayers)
    print("done")

    print("Adding Elements")
    # add the elements
    #newElements = addElements(mesh, connect, meshInfo, vertexMap, nverts)
    newElements = addElements2(mesh, connect, meshInfo, vertexMap, nverts)
    print("done")

    # write the new unv
    writeUNV2("myout.unv", (units, coordSystem, newCoords, newElements, props))
    #writeUNV("myout.unv", (units, coordSystem, newCoords, connect, props))

if __name__ == '__main__':
    main()
