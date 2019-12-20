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

def convertDatToOff(fileName):

    with open(fileName, "r") as f:
        line = f.readline()
        with open("out.txt", "w+") as out:
            out.write(line)
        words = line.split()
        print(words[0], words[1])
        nverts = int(words[0])
        nfaces = int(words[1])
        for i in range(nverts):
            line = f.readline()
            with open("out.txt", "a+") as out:
                words = line.split()
                out.write(words[1] + " " + words[2] + " " + words[3] + "\n")
        print(line)
        for i in range(nfaces):
            line = f.readline()
            with open("out.txt", "a+") as out:
                words = line.split()
                out.write(words[2] + " " + words[3] + " " + words[4] + "\n")
        print(line)

def main():
    convertDatToOff("/home/raphael/Documents/IANUS/OpenFOAM/salomeexport/nhull.dat")

if __name__ == '__main__':
    main()