import sys
import os
import getopt
import subprocess
import glob

from dat2off import convertDatToOff
from gen_boundary_layers import generateBoundaryLayers

#===============================================================================
#                             deleteIfExists
#===============================================================================
def deleteIfExists(file):
    if os.path.exists(file):
        os.remove(file)
    else:
        print("The file does not exist")

#===============================================================================
#                                makeClean 
#===============================================================================
def makeClean():
    """
      Clean the working directory
    """
    stlFiles = glob.glob('*.stl')
    vtkFiles = glob.glob('*.vtk')
    offFiles = glob.glob('*.off')
    datFiles = glob.glob('*.dat')

    for item in stlFiles:
        deleteIfExists(item)

    for item in vtkFiles:
        deleteIfExists(item)

    for item in offFiles:
        deleteIfExists(item)

    for item in datFiles:
        deleteIfExists(item)

    deleteIfExists("outer.unv")
    deleteIfExists("myout.unv")

#===============================================================================
#                                makeClean 
#===============================================================================
def cleanWorkingDir(workingDir):
    """
      Clean the working directory
    """
    stlFiles = glob.glob(workingDir + '/*.stl')
    vtkFiles = glob.glob(workingDir + '/*.vtk')
    offFiles = glob.glob(workingDir + '/*.off')
    datFiles = glob.glob(workingDir + '/*.dat')

    for item in stlFiles:
        deleteIfExists(item)

    for item in vtkFiles:
        deleteIfExists(item)

    for item in offFiles:
        deleteIfExists(item)

    for item in datFiles:
        deleteIfExists(item)

    deleteIfExists(workingDir + "/outer.unv")
    deleteIfExists(workingDir + "/myout.unv")
    deleteIfExists(workingDir + "/start.unv")

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
    print("[-d', '--working-dir]: Set the working directory where temporary files are saved")
    print("[-b', '--base-layer]: path to the .off file of the first boundary layer")
    print("[-r', '--orig-mesh]: path to the .dat file of the layer to which we want to attach the boundary layers")
    print("[-o', '--output-file]: name of the output unv file")
    print("[-s', '--salome-path]: path to the salome launcher")
    print("[-c', '--clean]: remote temporary and intermediate files from the working directory")
    print("[-t', '--thickness]: thickness of a boundary layer")
    print("[-l', '--layers]: the total number of boundary layers")
    print("[-h', '--help']: prints this message")
    print("Example: python3 ./unv_io.py -u netzsch_outer2.unv -b baseMeshLayer1.off -r RotorI3.dat")

def main():

    unvMesh = ""
    origMesh = ""
    baseLayer = ""
    workingDir = os.getcwd()
    numLayers = 2
    outputFile = "newmesh.unv"
    salomePath = "/home/rafa/bin/SALOME-9.3.0-UB18.04-SRC/salome"

    try:
        opts, args = getopt.getopt(sys.argv[1:], 'u:d:r:b:t:m:o:s:l:ch',
                                   ['unv-mesh=', 'working-dir=', 'orig-mesh=', 'base-layer=', 'thickness=', 'method=', 'output-file=',
                                    'salome-path=', 'layers=', 'clean',
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
        elif opt in ('-d', '--working-dir'):
            workingDir = arg
        elif opt in ('-b', '--base-layer'):
            baseLayer = arg
        elif opt in ('-r', '--orig-mesh'):
            origMesh = arg
        elif opt in ('-t', '--thickness'):
            thickness = float(arg)
        elif opt in ('-m', '--method'):
            method = int(arg)
        elif opt in ('-o', '--output-file'):
            outputFile = arg
        elif opt in ('-s', '--salome-path'):
            salomePath = arg
        elif opt in ('-l', '--layers'):
            numLayers = int(arg)
        elif opt in ('-c', '--clean'):
            makeClean()
            sys.exit(2)
        else:
            usage()
            sys.exit(2)

    #stitchLayers2Unv(baseLayer, origMesh, unvMesh, outputFile)
    #unvMeshAbsolute = os.getcwd() + '/' + unvMesh
    outsideWorkingDir = os.getcwd()

    fileDir = os.path.dirname(os.path.abspath(__file__))
    os.chdir(fileDir)

    unvMeshAbsolute = unvMesh

    unvMeshOutAbsolute = workingDir + '/start.unv' 

    # Set the current dir
    meshDir = workingDir 

    subprocess.call(['%s --shutdown-servers=1 -t salome_command_dump.py args:%s,%s,%s' % (salomePath, unvMeshAbsolute, unvMeshOutAbsolute, meshDir)], shell=True)

#    print("Corrected unv mesh: %s" %unvMeshOutAbsolute)
#    sys.exit(2)

    # This step uses the Stator
    offName = meshDir + "/statori.off" 
    datName = meshDir + "/StatorI.dat" 

    convertDatToOff(datName, offName)

    generateBoundaryLayers(workingDir, offName, numLayers, thickness)
    print("Status: Finished generating boundary layers")
    convertToUnvCommand = 'python3 ./unv_io.py -u start.unv -b baseMeshLayer1.off -o StatorI.dat -d %s -l %i' %(workingDir, numLayers)
    print("Command: %s" %convertToUnvCommand)

    #subprocess.call(['python3 ./gen_boundary_layers.py -s statori.off -t %f -l %i' %(thickness, numLayers)], shell=True)
    subprocess.call([convertToUnvCommand], shell=True)

    # Call Salome to process the stator
    subprocess.call(['%s --shutdown-servers=1 -t salome_rotor.py args:%s' %(salomePath, meshDir)],shell=True)

    # This step uses the Rotor group
    offName = meshDir + "/rotori.off" 
    datName = meshDir + "/RotorI.dat" 
    convertDatToOff(datName, offName)

    generateBoundaryLayers(workingDir, offName, numLayers, thickness)

    #subprocess.call(['python3 ./gen_boundary_layers.py -s rotori.off -t %f -l %i' %(thickness, numLayers)],shell=True)

    #input("Press to start conversion to unv")
    subprocess.call(['python3 ./unv_io.py -u outer.unv -b baseMeshLayer1.off -o RotorI.dat -d %s -l %i' %(workingDir, numLayers)],shell=True)
        
    #input("Press to start final salome step")
    # Here comes a last Salome step where the final groups are constructed
    subprocess.call(['%s --shutdown-servers=1 -t salome_final.py args:%s,%s' %(salomePath, meshDir, outputFile)],shell=True)

    cleanWorkingDir(workingDir)

    os.chdir(outsideWorkingDir)

if __name__ == "__main__":
    main()
