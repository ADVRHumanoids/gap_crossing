Implementation of a biped in OMPL. 14 primitives have been defined (rotation around a fixed foot, short forward/backward steps, long forward steps, lateral steps).
A gap and an obstacle are also included.
RRT is used for the solution.

To run you can either:
- compile and execute the cpp code: it will generate a "path.txt" file. This file will be eventually read from python to plot
- download only the files "plot.py" and "path.txt". Then, running the py code, our solution will be plotted.
