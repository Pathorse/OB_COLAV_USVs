import numpy as np

import pdb

class AStar:

    class Node:

        def __init__(self, p, parent=None):
            self.p      = np.array(p)
            self.parent = parent

            self.g      = 0.0
            self.h      = 0.0
            self.f      = 0.0

        def __eq__(self, other):
            #return self.p[0] == other.p[0] and self.p[1] == other.p[1]
            return (self.p == other.p).all()

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
        self.start_node    = self.Node(self.calc_index(start))
        self.goal_node     = self.Node(self.calc_index(goal))

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


    def plan(self):

        print("Entering planning")

        # Initialize open and closed list
        open_list   = []
        closed_list = []

        # Add start node
        open_list.append(self.start_node)

        # Get graph
        graph = self.get_graph()


        print("Entering loop")
        # Loop until you find the goal node
        while len(open_list) > 0:

            # Get current node
            current_node  = open_list[0]
            current_i     = 0

            for i, node in enumerate(open_list):
                if node.f < current_node.f:
                    current_node = node
                    current_i    = i

            # Remove current from open list and add to closed list
            open_list.pop(current_i)
            closed_list.append(current_node)

            # Reached goal node
            if current_node == self.goal_node:
                print("Found goal node")
                return self.final_path(current_node)

            # Get motions
            motions = self.get_motion()

            # Initialize children list
            children = []

            # Generate children
            for motion in motions:

                # New node
                new_node = self.Node(
                    p      = [current_node.p[0] + motion[0], current_node.p[1] + motion[1]],
                    parent = current_node
                )

                # Check bounds
                if self.out_of_bounds(graph, new_node):
                    continue

                # Check collision
                if graph[new_node.p[0]][new_node.p[1]] != 0:
                    continue

                # Add new node as child
                children.append(new_node)

            # Loop through children
            for child_node in children:

                # Skip if in the closed list
                for closed_node in closed_list:
                    if closed_node == child_node:
                        continue

                # Set f, g and h values
                child_node.g = current_node.g + 1
                child_node.h = self.heuristic(child_node, type='euclidean_square')
                child_node.f = child_node.g + child_node.h

                # Skip if child already in the open list
                for open_node in open_list:
                    if open_node == child_node and child_node.g > open_node.g:
                        continue

                # Add child to open_list
                open_list.append(child_node)

        raise Exception('Failed to find A* path')


    def final_path(self, current_node):

        # Initialize path
        path = []

        # Set current
        current = current_node

        # Backtrack
        while current:
            # Index pos
            ipos = current.p
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
