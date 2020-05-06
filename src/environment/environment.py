import irispy
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull

import pdb # TODO remove



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

        # Find convex regions
        #self.safe_regions = self.get_convex_regions()


    def get_convex_hull(self, start, return_debug=False, type='halfspace'):

        # Define bounds for irispy
        bounds = irispy.Polyhedron.from_bounds(self.lb, self.ub)

        # Define obstacles for irispy
        obstacles = [obstacle.T for obstacle in self.obstacles]

        # Find region
        if return_debug:
            region, debug = irispy.inflate_region(obstacles, start, bounds=bounds, return_debug_data=True)
            debug.animate(pause=0.5, show=True)
        else:
            region = irispy.inflate_region(obstacles, start, bounds=bounds)

        # Return correct type
        if type == 'halfspace':

            # Get halfspace representation
            A = region.polyhedron.getA()
            b = region.polyhedron.getB()

            return A, b

        elif type == 'vertices':

            # Get vertices
            v = region.polyhedron.generatorPoints()

            return v

        elif type == 'all':

            # Get halfspace representation
            A = region.polyhedron.getA()
            b = region.polyhedron.getB()

            # Get vertices
            v = region.polyhedron.generatorPoints()

            return A, b, v

    def get_convex_regions(self, n_seeds=0): # TODO proper randomized seeding

        # Initiate empty safe region list
        convex_regions = []

        # Declare seed points
        seed_points = [[0,0], [1250, 800], [1800, 900], [1300, 1600],
                       [1250, 1900], [1600, 900], [1650, 1400], [2000, 950], [2000, 2000]]


        # Find convex regions
        for i in range(len(seed_points)):

            # Seed
            seed = seed_points[i]

            # Halfspace matrices, and vertices
            A, b, v = self.get_convex_hull(seed, return_debug=False, type='all')

            # Store
            convex_regions.append((A, b, v))

        return convex_regions


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
