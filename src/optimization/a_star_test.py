import numpy as np

from timeit import default_timer

import pdb

class AStar_test:

    #class Node:

    #    def __init__(self, p, parent=None):
    #        #self.p      = np.array(p)
    #        self.p      = p
    #        self.parent = parent

    #        self.g      = 0.0
    #        self.h      = 0.0
    #        self.f      = 0.0

    #    def __eq__(self, other):
    #        #return self.p[0] == other.p[0] and self.p[1] == other.p[1]
    #        return (self.p == other.p)

    def __init__(self,
                 start,
                 goal,
                 obstacle_list,
                 lb,
                 ub,
                 resolution=1
    ):
        self.lb            = lb
        self.ub            = ub
        self.obstacle_list = obstacle_list
        self.resolution    = resolution
        #self.start_node    = self.Node(self.calc_index(start))
        self.istart        = self.calc_index(start)
        #self.goal_node     = self.Node(self.calc_index(goal))
        self.igoal         = self.calc_index(goal)

    def heuristic(self, node, type='euclidean'):

        # Calculate distance from goal in x and y
        dx = np.abs(node.p[0] - self.goal_node.p[0])
        dy = np.abs(node.p[1] - self.goal_node.p[1])

        if type == 'euclidean':
            return np.sqrt(dx**2 + dy**2)

        elif type == 'euclidean_square':
            return dx**2 + dy**2

        elif type == 'diagonal':
            return dx + dy - min(dx, dy)

        raise Exception('Failed to specify correct heuristic type, options are: euclidean, euclidean_square, diagonal')


    def get_motion(self):

        motion = [
            [-1, -1],
            [-1,  0],
            [ 0, -1],
            [-1,  1],
            [ 1, -1],
            [ 0,  1],
            [ 1,  0],
            [ 1,  1]
        ]

        return motion


    def out_of_bounds(self, graph, node):

        # Graph lenght
        Nx = len(graph) - 1
        Ny = len(graph[-1]) - 1

        # Extract index pos
        ix = node.p[0]
        iy = node.p[1]

        # Set conditions
        cond_xmin = ix < 0
        cond_xmax = ix > Nx
        cond_ymin = iy < 0
        cond_ymax = iy > Ny

        return cond_xmin or cond_xmax or cond_ymin or cond_ymax


   # def collision(self, pos):

   #     # Tolerance
   #     epsilon = 5

   #     # Check for collision with all obstacles
   #     for obstacle in self.obstacle_list:
   #         A = obstacle.A
   #         b = obstacle.b

   #         # Check for collision in an
   #         # epsilon ball around the position
   #         angles = np.linspace(0, 2*np.pi, 5)

   #         for theta in angles:

   #             xp = pos[0] + epsilon * np.cos(theta)
   #             yp = pos[1] + epsilon * np.sin(theta)

   #             x = np.array([xp, yp])

   #             results = np.less_equal(A.dot(x), b)

   #             if results.all():
   #                 return True

   #     # No collision
   #     return False


    def collision(self, pos):

        # Check for collision with all obstacles
        for obstacle in self.obstacle_list:
            A = obstacle.A
            b = obstacle.b

            results = np.less_equal(A.dot(pos), b)

            if results.all():
                return True

        # No collision
        return False


    def segment_collision(self, p1, p2, stepsize=0.1):

        # Extract change
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]

        # Num steps
        N_steps = int(dx/stepsize)

        for i in range(N_steps):

            # Current x
            x = p1[0] + i * stepsize

            # Line y - y0 = dy/dx * (x - x0)
            y = dy/dx * (x - p1[0]) + p1[1]

            # Check for collision
            col = self.collision([x, y])

            if col:
                return True

        return False


    def get_graph(self):

        # Parameters
        xmin = self.lb[0]
        xmax = self.ub[0]

        ymin = self.lb[1]
        ymax = self.ub[1]

        res  = self.resolution

        # Calculate steps
        Nx = int(round((xmax - xmin)/res))
        Ny = int(round((ymax - ymin)/res))

        # Initialize graph
        graph = [[0] *  Ny for _ in range(Nx)]

        # for x in range xmin:res:xmax
        for ix in range(Nx):
            for iy in range(Ny):
                # Index pos
                ipos = [ix, iy]

                # True pos
                pos = self.calc_pos(ipos)

                # Check for collision and
                # set graph accordingly
                if self.collision(pos):
                    graph[ix][iy] = 1


        return graph


    def calc_index(self, pos):

        # Parameters
        xmin = self.lb[0]
        xmax = self.ub[0]

        ymin = self.lb[1]
        ymax = self.ub[1]

        res  = self.resolution

        # Extract values
        x = pos[0]
        y = pos[1]

        # Find x and y index from pos
        ix = int(round((x - xmin)/res))
        iy = int(round((y - ymin)/res))

        return [ix, iy]


    def calc_pos(self, ipos):

        # Parameters
        xmin = self.lb[0]
        xmax = self.ub[0]

        ymin = self.lb[1]
        ymax = self.ub[1]

        res  = self.resolution

        # Extract values
        ix = ipos[0]
        iy = ipos[1]

        # Calc position
        x = ix * res + xmin
        y = iy * res + ymin

        return [x, y]


    def plan(self, epsilon=0.5):

        print("Entering A* path planning")

        # Start timer
        astar_start_time = default_timer()

        # Get graph
        graph = self.get_graph()

        print("Graph created")

        # Get start and goal values
        start = self.istart
        goal  = self.igoal

        last_node = search(graph, 1, start, goal)

        path = self.final_path(last_node)

        # End timer
        astar_stop_time = default_timer()

        print("A* Runtime:", astar_stop_time - astar_start_time)

        return path



    def final_path(self, current_node):

        # Initialize path
        path = []

        # Set current
        current = current_node

        # Backtrack
        while current:
            # Index pos
            ipos = current.position
            # True pos
            pos  = self.calc_pos(ipos)

            # Add true pos to path
            path.append(pos)

            current = current.parent

        # Reduce number of uneccesary points
        path = self.reduce_path_points(path)

        return path


    def reduce_path_points(self, path):

        # Initialize reduced path
        reduced_path = []

        # Initalize index
        i = len(path) - 1

        # Initialize wpt
        curr_wpt = path[i]

        # Add to path
        reduced_path.append(curr_wpt)

        while True:

            # Set current wpt
            curr_wpt = path[i]

            for j in range(i - 1):

                # Extract next wpt
                next_wpt = path[j]

                # Check for no collision betwen points
                if not self.segment_collision(curr_wpt, next_wpt):

                    # Add to reduced path
                    reduced_path.append(next_wpt)

                    # Set index
                    i = j

                    break

            if i == 0:
                break

        return reduced_path




# New testing below -----------------------------------------------------------------


class Node:
    """
        A node class for A* Pathfinding
        parent is parent of the current Node
        position is current position of the Node in the maze
        g is cost from start to current Node
        h is heuristic based estimated cost for current Node to end Node
        f is total cost of present node i.e. :  f = g + h
    """

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0
    def __eq__(self, other):
        return self.position == other.position


def return_path(current_node, maze):

    # Initialize path
    path = []

    # Set current
    current = current_node

    # Backtrack
    while current:
        # Index pos
        ipos = current.position
        # True pos
        #pos  = self.calc_pos(ipos)

        # Add true pos to path
        path.append(ipos)

        current = current.parent

    return path


def search(maze, cost, start, end):
    """
        Returns a list of tuples as a path from the given start to the given end in the given maze
        :param maze:
        :param cost
        :param start:
        :param end:
        :return:
    """

    # Create start and end node with initized values for g, h and f
    start_node = Node(None, tuple(start))
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, tuple(end))
    end_node.g = end_node.h = end_node.f = 0

    # Initialize both yet_to_visit and visited list
    # in this list we will put all node that are yet_to_visit for exploration.
    # From here we will find the lowest cost node to expand next
    yet_to_visit_list = []
    # in this list we will put all node those already explored so that we don't explore it again
    visited_list = []

    # Add the start node
    yet_to_visit_list.append(start_node)

    # Adding a stop condition. This is to avoid any infinite loop and stop
    # execution after some reasonable number of steps
    outer_iterations = 0
    max_iterations = (len(maze) // 2) ** 10

    # what squares do we search . serarch movement is left-right-top-bottom
    #(4 movements) from every positon

    move  =  [
        [-1,  0], # go up
        [ 0, -1], # go left
        [ 1,  0], # go down
        [ 0,  1], # go right
        [-1, -1], # go down left
        [ 1, -1], # go down right
        [-1,  1], # go up left
        [ 1,  1]  # go up right
    ]

    """
        1) We first get the current node by comparing all f cost and selecting the lowest cost node for further expansion
        2) Check max iteration reached or not . Set a message and stop execution
        3) Remove the selected node from yet_to_visit list and add this node to visited list
        4) Perofmr Goal test and return the path else perform below steps
        5) For selected node find out all children (use move to find children)
            a) get the current postion for the selected node (this becomes parent node for the children)
            b) check if a valid position exist (boundary will make few nodes invalid)
            c) if any node is a wall then ignore that
            d) add to valid children node list for the selected parent

            For all the children node
                a) if child in visited list then ignore it and try next node
                b) calculate child node g, h and f values
                c) if child in yet_to_visit list then ignore it
                d) else move the child to yet_to_visit list
    """
    #find maze has got how many rows and columns
    no_rows, no_columns = np.shape(maze)

    # Loop until you find the end

    while len(yet_to_visit_list) > 0:

        # Every time any node is referred from yet_to_visit list, counter of limit operation incremented
        outer_iterations += 1


        # Get the current node
        current_node = yet_to_visit_list[0]
        current_index = 0
        for index, item in enumerate(yet_to_visit_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index

        # if we hit this point return the path such as it may be no solution or
        # computation cost is too high
        if outer_iterations > max_iterations:
            print ("giving up on pathfinding too many iterations")
            return return_path(current_node,maze)

        # Pop current node out off yet_to_visit list, add to visited list
        yet_to_visit_list.pop(current_index)
        visited_list.append(current_node)

        # test if goal is reached or not, if yes then return the path
        if current_node == end_node:
            return current_node

        # Generate children from all adjacent squares
        children = []

        for new_position in move:

            # Get node position
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            # Make sure within range (check if within maze boundary)
            if (node_position[0] > (no_rows - 1) or
                node_position[0] < 0 or
                node_position[1] > (no_columns -1) or
                node_position[1] < 0):
                continue

            # Make sure walkable terrain
            if maze[node_position[0]][node_position[1]] != 0:
                continue

            # Create new node
            new_node = Node(current_node, node_position)

            # Append
            children.append(new_node)

        # Loop through children
        for child in children:

            # Child is on the visited list (search entire visited list)
            if len([visited_child for visited_child in visited_list if visited_child == child]) > 0:
                continue

            # Create the f, g, and h values
            child.g = current_node.g + cost
            ## Heuristic costs calculated here, this is using eucledian distance
            child.h = (((child.position[0] - end_node.position[0]) ** 2) +
                       ((child.position[1] - end_node.position[1]) ** 2))

            child.f = child.g + child.h

            # Child is already in the yet_to_visit list and g cost is already lower
            if len([i for i in yet_to_visit_list if child == i and child.g > i.g]) > 0:
                continue

            # Add the child to the yet_to_visit list
            yet_to_visit_list.append(child)


if __name__ == '__main__':

    maze = [[0, 1, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0],
            [0, 1, 0, 1, 0, 0],
            [0, 1, 0, 0, 1, 0],
            [0, 0, 0, 0, 1, 0]]

    start = [0, 0] # starting position
    end   = [4,5] # ending position
    cost  = 1 # cost per movement

    path = search(maze,cost, start, end)
    print(path)
