#!/usr/bin/env python
# vim: set filetype=python
#=====================================================
# Author: Raphael Muenster
# Description:
#   Python script to plot settling velocites
#   into one figure using python library matplotlib
#=====================================================

# import libraries
import numpy as np
import matplotlib.pyplot as plt
import sys
import getopt

#===============================================================================
#                                Usage
#===============================================================================
def usage():
  print("Usage: pyplot.py [options]")
  print("Where options can be:")
  print("[-h, --help]: prints this message")
  print("[-f, --file]: path to the data file")
  print("[-c, --columns]: path to the data file")

#===============================================================================
#                      Function: parseColumns
#===============================================================================
def parseColumns(argin):
    """
    Parses the input argument <extrusion-layers> that tells us
    the number of extrusions on each level
    """
    ex = argin.split(",")
    ex = [int(x) for x in ex]
    return ex

#===============================================================================
#                        Main Script Function
#===============================================================================
def main():
    try:
        opts, args = getopt.getopt(sys.argv[1:], 'f:c:h', ['file=','columns=', 'help'])
    except getopt.GetoptError:
        usage()
        sys.exit(2)

    file_data = ""

    if len(sys.argv) < 2:
      usage()
      sys.exit(2)

    columns = [1, 2] 

    for opt, arg in opts:
        if opt in ('-h', '--help'):
            usage()
            sys.exit(2)
        elif opt in ('-f', '--file'):
            file_data = arg
        elif opt in ('-c', '--columns'):
            columns = parseColumns(arg)
        else:
            usage()
            sys.exit(2)

    if file_data == "":
      usage()
      sys.exit(2)

    in_data=np.loadtxt(file_data)

    for idx in range(0, len(columns), 2):
        xidx = columns[idx]
        yidx = columns[idx+1]
        plt.plot(in_data[:,xidx],in_data[:,yidx])

    plt.show()

#===============================================================================
#                             Main Boiler Plate
#===============================================================================
if __name__ == "__main__":
    main()
