import numpy as np

from timeit import default_timer


class AStar:

    class Node:

        def __init__(self, parent=None, position=None, cost=0):

            self.parent   = parent   # parent of the current Node
            self.position = position # current position of the Node
            self.cost     = cost

            self.g = 0 # cost from start to current Node
            self.h = 0 # heuristic based estimated cost for current Node to end Node
            self.f = 0 # total cost of parent node, i.e. f = g + h
       
        # Two Nodes are equal if they have the same position
        def __eq__(self, other):
            return self.position == other.position


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
        self.istart        = self.calc_index(start)
        self.igoal         = self.calc_index(goal)


    def heuristic(self, node, end_node, type='euclidean'):

        # Calculate distance from goal in x and y
        dx = np.abs(node.position[0] - end_node.position[0])
        dy = np.abs(node.position[1] - end_node.position[1])

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


    def out_of_bounds(self, grid, node_position):

        # Get num rows and cols from grid
        no_rows, no_cols = np.shape(grid)

        # Graph lenght
        Nx = no_rows - 1
        Ny = no_cols - 1

        # Extract index pos
        ix = node_position[0]
        iy = node_position[1]

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


    def get_grid(self):

        # Parameters
        xmin = self.lb[0]
        xmax = self.ub[0]

        ymin = self.lb[1]
        ymax = self.ub[1]

        res  = self.resolution

        # Calculate steps
        Nx = int(round((xmax - xmin)/res))
        Ny = int(round((ymax - ymin)/res))

        # Initialize grid
        grid = [[0] *  Ny for _ in range(Nx)]

        # for x in range xmin:res:xmax
        for ix in range(Nx):
            for iy in range(Ny):
                # Index pos
                ipos = [ix, iy]

                # True pos
                pos = self.calc_pos(ipos)

                # Check for collision and
                # set grid accordingly
                if self.collision(pos):
                    grid[ix][iy] = 1

        return grid


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

        print("Entering A* path planning")

        # Start timer
        astar_start_time = default_timer()

        # Get graph
        grid = self.get_grid()

        print("Uniformly Decomposed Grid created")

        # Get start and goal values
        start = self.istart
        goal  = self.igoal

        last_node = self.search(grid, start, goal)

        path = self.final_path(last_node)

        # End timer
        astar_stop_time = default_timer()

        print("A* Runtime:", astar_stop_time - astar_start_time)

        return path


    def search(self, grid, start, end):

        # Initialize start and goal Node
        start_node = self.Node(None, tuple(start))
        end_node   = self.Node(None, tuple(end))

        # Initialize yet to visit and visited lists
        yet_to_visit = []
        visited      = []

        # Add the start node
        yet_to_visit.append(start_node)

        # Add termination condition to avoid infinite loops
        outer_iters = 0
        max_iters   = (len(grid) // 2) ** 10

        # Get movements
        # [dx, dy, cost]
        moves = [
            [-1,  0, 1],          # go up
            [ 0, -1, 1],          # go left
            [ 1,  0, 1],          # go down
            [ 0,  1, 1],          # go right
            [-1, -1, np.sqrt(2)], # go down left
            [ 1, -1, np.sqrt(2)], # go down right
            [-1,  1, np.sqrt(2)], # go up left
            [ 1,  1, np.sqrt(2)]  # go up right
        ]

        # Loop until a solution is found or max number of iterations is exceeded
        while yet_to_visit:

            # Increment
            outer_iters += 1

            # Get current node
            current_node = yet_to_visit[0]
            current_index  = 0
            for index, node in enumerate(yet_to_visit):
                if node.f < current_node.f:
                    current_node = node
                    current_index = index

            # Terminate if exceeded max number of iterations
            if outer_iters > max_iters:
                print("Exceeded maximum numbers of iterations in A* planning")

                return current_node

            # Remove current_node from yet to visit and add to visited list
            yet_to_visit.pop(current_index)
            visited.append(current_node)

            # Check if goal node is reached
            if current_node == end_node:
                return current_node

            # Initialize children list
            children = []

            # Generate children
            for move in moves:

                # Get new node position
                node_position = (current_node.position[0] + move[0], current_node.position[1] + move[1])


                # Check boundary
                if self.out_of_bounds(grid, node_position):
                    continue

                # Check for obstacles
                if grid[node_position[0]][node_position[1]] != 0:
                    continue

                # Create new node
                new_node = self.Node(current_node, node_position, move[2])

                # Add to children list
                children.append(new_node)

            # Loop through children
            for child in children:

                # Check if child is in the visited list
                if len([visited_child for visited_child in visited if visited_child == child]) > 0:
                    continue

                # Create f, g, and h values
                child.g = current_node.g + child.cost
                child.h = self.heuristic(child, end_node, type='euclidean_square')
                child.f = child.g + child.h

                # Check if child is already in the yet to visit list and is of cheaper cost
                if len([i for i in yet_to_visit if child == i and child.g > i.g]) > 0:
                    continue

                # Add child to yet to visit list
                yet_to_visit.append(child)


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
