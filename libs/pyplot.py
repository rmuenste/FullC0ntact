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
import matplotlib.animation as animation
import sys
import getopt
import time

globalFileName = ""
globalColumns = []

fig, ax = plt.subplots()
fig2, ax2 = plt.subplots()

line, = ax.plot([],[], 'ro', animated=True)

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
    Parses the input argument <columns> that tells us
    which columns to use for plotting 
    """
    ex = argin.split(",")
    ex = [int(x) for x in ex]
    return ex

#===============================================================================
#                      Function: updatePlot
#===============================================================================
def plotUpdate(fileName, cols):
    """
    Parses the input argument <extrusion-layers> that tells us
    the number of extrusions on each level
    """
    
    print("Redrawing")
    in_data=np.loadtxt(fileName)

    ax.clear()
    for idx in range(0, len(cols), 2):
        xidx = cols[idx]
        yidx = cols[idx+1]
        ax.plot(in_data[:,xidx],in_data[:,yidx])
#===============================================================================
#                      Function: updatePlot
#===============================================================================
def init():
    """
    Parses the input argument <extrusion-layers> that tells us
    the number of extrusions on each level
    """
    
    print("Init plot")
    inputData=np.loadtxt("observables.log")
    xmax = np.max(inputData[:,0])
    ymax = np.max(inputData[:,1])
    xmin = np.min(inputData[:,0])
    ymin = np.min(inputData[:,1])

    ax.set_xlim([xmin, xmax])
    ax.set_ylim([ymin, ymax])
    ax.grid()
    line, = ax.plot([],[], 'ro', animated=True)
    return line,

#===============================================================================
#                      Function: updatePlot
#===============================================================================
def animate(data):
    """
    Parses the input argument <extrusion-layers> that tells us
    the number of extrusions on each level
    """
    
    ax.clear()
    ax2.clear()
    xx, yy, yy2, yy3 = data


    ymin = np.min([yy, yy2])
    ymax = np.max([yy, yy2])
    #print("min,max:" + str(ymin) + " " + str(ymax))
    ymin2 = np.min([yy3])
    ymax2 = np.max([yy3])

    ax.set_xlim([0, xx[len(xx)-1]])
    ax.set_ylim([ymin, ymax + 0.1 * (ymax - ymin)])

    ax.grid()

    ax2.set_xlim([0, xx[len(xx)-1]])
    ax2.set_ylim([ymin2, ymax2])
    ax.set_ylim([ymin2, ymax2])

    ax2.grid()

    line.set_xdata(xx)
    line.set_ydata(yy2)
#    ax.plot(xx, yy, lw=2, color="blue")
#    ax.plot(xx, yy2, lw=2, color="red")
#    ax2.plot(xx, yy3, lw=2, color="magenta")
    ax.plot(xx, yy3, lw=2, color="magenta")
    return line,

#===============================================================================
#                      Function: updatePlot
#===============================================================================
def data_gen(fileName):
    
    print("Data Gen using " + fileName)
    cnt = 0
    while True: 
        cnt += 1
        inputData=np.loadtxt("observables.log")
        yield inputData[:,0],inputData[:,3],inputData[:,6],inputData[:,15]
        #yield inputData[:,0],inputData[:,3],inputData[:,6],inputData[:,15]

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
            globalFileName = file_data
        elif opt in ('-c', '--columns'):
            columns = parseColumns(arg)
            globalColumns = columns
        else:
            usage()
            sys.exit(2)

    if file_data == "":
      usage()
      sys.exit(2)

    #ani = animation.FuncAnimation(fig, animate, data_gen(file_data), blit=False, interval=10000, repeat = False, init_func = init)
    ani = animation.FuncAnimation(fig, animate, data_gen(file_data), blit=False, interval=10000, repeat = False)
    plt.show()

#===============================================================================
#                             Main Boiler Plate
#===============================================================================
if __name__ == "__main__":
    main()
