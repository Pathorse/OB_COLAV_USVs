import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt



# ----------------------------------------------------
# Plots a circle centered at the given point
# and with the given radius
# ----------------------------------------------------
def plot_circle_contour(center, radius, *args, **kwargs):
    # discretize angle
    angle = np.linspace(0, 2*np.pi)

    # plot circle
    plt.plot(
        center[0] + radius * np.cos(angle),
        center[1] + radius * np.sin(angle),
        *args,
        **kwargs
    )




# ----------------------------------------------------
# Plots the USV contour along a given trajectory
# ----------------------------------------------------
def plot_usv_contour(ax, x_trj, width=20, height=10, tip_height=30,  *args, **kwargs):
    w = width
    h = height
    l = tip_height
    # Plot contour for all steps
    for n in range(x_trj.shape[0]):
        # Create USV shape
        if (n == 0) or (n == x_trj.shape[0] - 1): # Different color for start/finish and other points
            shape = mpl.patches.Polygon([(-w/2,-h/2), (-w/2, h/2), (w/2, h/2), (l/2,0), (w/2, -h/2)], color='red', fill=False)
        else:
            shape = mpl.patches.Polygon([(-w/2,-h/2), (-w/2, h/2), (w/2, h/2), (l/2,0), (w/2, -h/2)], fill=False)

        # Transform applied at each step
        t = mpl.transforms.Affine2D().rotate_deg_around(0, 0,
                np.rad2deg(x_trj[n,2])).translate(x_trj[n,0], x_trj[n,1]) + ax.transData
        shape.set_transform(t)

        ax.add_patch(shape)



# ----------------------------------------------------
# Reduce the number of waypoints
# ----------------------------------------------------
