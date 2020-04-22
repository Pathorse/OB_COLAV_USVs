import matplotlib
import matplotlib.patches
import matplotlib.pyplot as plt
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



def generatePolygon( ctrX, ctrY, aveRadius, irregularity, spikeyness, numVerts ) :
    '''Start with the centre of the polygon at ctrX, ctrY,
    then creates the polygon by sampling points on a circle around the centre.
    Randon noise is added by varying the angular spacing between sequential points,
    and by varying the radial distance of each point from the centre.

    Params:
    ctrX, ctrY - coordinates of the "centre" of the polygon
    aveRadius - in px, the average radius of this polygon, this roughly controls how large the polygon is, really only useful for order of magnitude.
    irregularity - [0,1] indicating how much variance there is in the angular spacing of vertices. [0,1] will map to [0, 2pi/numberOfVerts]
    spikeyness - [0,1] indicating how much variance there is in each vertex from the circle of radius aveRadius. [0,1] will map to [0, aveRadius]
    numVerts - self-explanatory

    Returns a list of vertices, in CCW order.
    '''

    irregularity = clip( irregularity, 0,1 ) * 2*math.pi / numVerts
    spikeyness = clip( spikeyness, 0,1 ) * aveRadius

    # generate n angle steps
    angleSteps = []
    lower = (2*math.pi / numVerts) - irregularity
    upper = (2*math.pi / numVerts) + irregularity
    sum = 0
    for i in range(numVerts) :
        tmp = random.uniform(lower, upper)
        angleSteps.append( tmp )
        sum = sum + tmp

    # normalize the steps so that point 0 and point n+1 are the same
    k = sum / (2*math.pi)
    for i in range(numVerts) :
        angleSteps[i] = angleSteps[i] / k

    # now generate the points
    points = []
    angle = random.uniform(0, 2*math.pi)
    for i in range(numVerts) :
        r_i = clip( random.gauss(aveRadius, spikeyness), 0, 2*aveRadius )
        x = ctrX + r_i*math.cos(angle)
        y = ctrY + r_i*math.sin(angle)
        points.append( (int(x),int(y)) )

        angle = angle + angleSteps[i]

    return points

def clip(x, min, max):
    if( min > max ) :  return x
    elif( x < min ) :  return min
    elif( x > max ) :  return max
    else :             return x






















def plot_polygons_lines_and_points(
        fig, blue_polygons=None, yellow_polygons=None, red_polygon=None, additional_polygons=None
):

    patches = [] if additional_polygons is None else additional_polygons
    if yellow_polygons is not None:
        patches += [_get_patch(polygon, "yellow") for polygon in yellow_polygons]

    if blue_polygons is not None:
        patches += [_get_patch(polygon, "blue") for polygon in blue_polygons]

    if red_polygon is not None:
        polygon = matplotlib.patches.Polygon(red_polygon, True, alpha=0.4, color="red")
        patches.append(polygon)

    #fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.axis("auto")

    for patch in patches:
        ax.add_patch(patch)

    plt.axis("equal")

    #plt.show()


def _get_patch(p, color):
    return matplotlib.patches.Polygon(p, True, alpha=0.4, color=color)
