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

def usage():
    print("Hello World!")

def main():
    inputFile = ""
    outputFile = ""

    try:
        opts, args = getopt.getopt(sys.argv[1:], 'i:o:h',
                                   ['input-file=', 'output-file=',
                                    'help'])

    except getopt.GetoptError:
        usage()
        sys.exit(2)

    for opt, arg in opts:
        if opt in ('-h', '--help'):
            usage()
            sys.exit(2)
        elif opt in ('-i', '--input-file'):
            inputFile = arg
        elif opt in ('-o', '--output-file'):
            outputFile = arg
        else:
            usage()
            sys.exit(2)

    convertDatToOff(inputFile, outputFile)

if __name__ == '__main__':
    main()
