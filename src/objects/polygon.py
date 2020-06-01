import matplotlib
import matplotlib.patches
import matplotlib.pyplot as plt
import numpy as np
import math, random
import pypoman


class Polygon(object):

    def __init__(
            self,
            vertices,
            name,
            color
    ):
        # Store arguments
        self.vertices = vertices
        self.name     = name
        self.color    = color


        # halfspace representation of the polygon
        self.A, self.b = pypoman.duality.compute_polytope_halfspaces(self.vertices)


    def plot(self):
        pypoman.polygon.plot_polygon(self.vertices, color=self.color)

