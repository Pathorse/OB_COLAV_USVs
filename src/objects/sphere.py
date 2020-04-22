import numpy as np
import matplotlib.pyplot as plt


class Sphere(object):

    def __init__(
            self,
            name,
            color,
            position,
            radius
    ):
        # Store data
        self.name     = name
        self.color    = color
        self.position = position
        self.radius   = radius


    def plot_contour(self):
        # discretize angle
        angle = np.linspace(0, 2*np.pi)

        # plot circle
        plt.plot(
            self.position[0] + self.radius * np.cos(angle),
            self.position[1] + self.radius * np.sin(angle),
            label=self.name,
            color=self.color,
        )

    def plot_filled(self):
        ax = plt.gca()
        circle = plt.Circle(
            (self.position[0], self.position[1]),
            self.radius,
            label=self.name,
            color=self.color
        )
        #ax.add_artist(circle)
