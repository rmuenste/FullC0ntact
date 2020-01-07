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
    print(content)
    return content

def parseUNV2():
    beginPos = -1 
    endPos = -1 
    unitSection = []
    coordsysSection = []
    coordSection = []
    connectivitySection = []
    propertiesSection = []
    with open("Cube_Hexa_Salome2.unv", "r") as f:

#        beginPos = f.tell()
#        line = f.readline()
#        if line.split()[0] == "-1":
#            line = f.readline()
#            sectionID = line.split()[0]
#            print("Found Section ID %s" % sectionID)
#        else:
#            sys.exit(2)
#        
#        f.seek(beginPos, os.SEEK_SET)

#        while line and line.split()[0] != "-1":
#            line = f.readline()
#        endPos = f.tell()
#        f.seek(beginPos, os.SEEK_SET)
#        toRead = endPos - beginPos
#        content = f.read(toRead)
#        print("Section begin %s, section end %s" %(str(beginPos), str(endPos)))
#        print(content)
    
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
        print(line)

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

#    print("Number of vertices: ", len(vertices))
#    print(vertices)
    return vertices

def parseElements(connect):
    lines = connect.splitlines()

    elements = []
    i = 2
    while i < len(lines)-2:
        words = lines[i].split()
        entry = {}
        print(words[1])
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
#    print("Number of FE: ", len(elements))
    return elements

def writeUNV(fileName, data):
    print("Hello")
    with open(fileName, "w") as out:
        out.write(data[0])
        out.write(data[1])
        out.write(data[2])
        out.write(data[3])
        out.write(data[4])

def convertVerticesToString(vertices):
    outString = ""
    outString = outString + "    -1\n" + "    2411\n"
    for entry in vertices:
#        print(entry)
        outString = outString + entry['desc'] + "\n" + entry['coords'] + "\n"
        
    outString = outString + "    -1\n"
    return outString

def main():
    convertDatToOff("Hull.dat", "cyl.off")
#    (units, coordSystem, coords, connect, props) = parseUNV2()
##    print(type(units))
##    print(units)
##    convertDatToOff("nhull.dat", "out.off")
#    mycoords = parseVertices(coords)
#
#    nextId = len(mycoords) + 1
#    newEntry = {}
#    newEntry['desc'] = mycoords[nextId-2]['desc']
#
#    desc = newEntry['desc'].split()
#    desc[0] = "        " + str(nextId)
#    desc = "         ".join(desc)
#
#    newEntry['desc'] = desc
#    newEntry['coords'] = "   4.9999999999869393E-00   4.9999999999869382E-00   4.9999999999869393E-00"
#
#    mycoords.append(newEntry)
#
#    out = convertVerticesToString(mycoords)
##    print(out)
##    sys.exit(2)
#    parseElements(connect)
#    writeUNV("out.unv", (units, coordSystem, out, connect, props))

if __name__ == '__main__':
    main()