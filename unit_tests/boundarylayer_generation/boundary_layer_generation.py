import sys
import os
import getopt
import subprocess
from unv_io import stitchLayers2Unv

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
    print("[-r', '--orig-mesh]: path to the .dat file of the layer to which we want to attach the boundary layers")
    print("[-o', '--output-file]: name of the output unv file")
    print("[-h', '--help']: prints this message")
    print("Example: python3 ./unv_io.py -u netzsch_outer2.unv -b baseMeshLayer1.off -r RotorI3.dat")

def main():
    print("Hello World!")

    unvMesh = ""
    origMesh = ""
    baseLayer = ""
    outputFile = "output.unv"

    try:
        opts, args = getopt.getopt(sys.argv[1:], 'u:r:b:t:m:o:h',
                                   ['unv-mesh=', 'orig-mesh=', 'base-layer=', 'thickness=', 'method=', 'output-file='
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
        elif opt in ('-r', '--orig-mesh'):
            origMesh = arg
        elif opt in ('-t', '--thickness'):
            thickness = float(arg)
        elif opt in ('-m', '--method'):
            method = int(arg)
        elif opt in ('-o', '--output-file'):
            outputFile = arg
        else:
            usage()
            sys.exit(2)

    #stitchLayers2Unv(baseLayer, origMesh, unvMesh, outputFile)

# 0.0122222222222
    subprocess.call(['/home/raphael/bin/SALOME-9.3.0-UB18.04-SRC/salome -t salome_command_dump.py'],shell=True)
    subprocess.call(['python3 ./dat2off.py -i StatorI.dat -o statori.off'],shell=True)
    subprocess.call(['python3 ./gen_boundary_layers.py -s statori.off -t 0.01375'],shell=True)
    subprocess.call(['python3 ./unv_io.py -u start.unv -b baseMeshLayer1.off -o StatorI.dat'],shell=True)

    # This step uses the Rotor group
    subprocess.call(['/home/raphael/bin/SALOME-9.3.0-UB18.04-SRC/salome -t salome_rotor.py'],shell=True)
    subprocess.call(['python3 ./dat2off.py -i RotorI.dat -o rotori.off'],shell=True)
    subprocess.call(['python3 ./gen_boundary_layers.py -s rotori.off -t 0.01375'],shell=True)
    subprocess.call(['python3 ./unv_io.py -u outer.unv -b baseMeshLayer1.off -o RotorI.dat'],shell=True)
    subprocess.call(['/home/raphael/bin/SALOME-9.3.0-UB18.04-SRC/salome -t salome_final.py'],shell=True)

    # Here comes a last Salome step where the final groups are constructed

if __name__ == "__main__":
    main()
