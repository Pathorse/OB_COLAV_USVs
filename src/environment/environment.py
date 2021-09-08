#import irispy
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull



class Environment(object):

    def __init__(
            self,
            obstacles=[],
            lb=[-np.inf, -np.inf],
            ub=[np.inf, np.inf],
            name="Simulation_Environment"

    ):
        # Declare name
        self.name = name

        # Declare obstacles
        self.obstacles = obstacles

        # Declare bounds
        self.lb = lb
        self.ub = ub


    def draw(self, ax):

        # Initiate empty patches list
        patches = []

        # Add obstacles
        if self.obstacles is not None:
            patches += [mpl.patches.Polygon(vertices, True, alpha=0.4, color='blue') for vertices in self.obstacles]

        # Add safe regions
        if self.safe_regions is not None:
            patches += [mpl.patches.Polygon(
                ConvexHull(v).points,
                True,
                alpha=0.4,
                color='green',
                fill=False
            ) for A, b, v in self.safe_regions]


        # Add to plot
        for patch in patches:
            ax.add_patch(patch)
