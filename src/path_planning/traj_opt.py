import numpy as np
import pdb # TODO Remove when finished

# Pydrake imports
from pydrake.all import (Variable, SymbolicVectorSystem, DiagramBuilder,
                         LogOutput, Simulator, ConstantVectorSource,
                         MathematicalProgram, Solve, SnoptSolver, PiecewisePolynomial,
                         OsqpSolver, eq, le, ge)



def add_decision_variables(prog, n_x, n_u, n_po, time_steps):

    # Optimization variables
    x = prog.NewContinuousVariables(rows=time_steps+1, cols=n_x, name='x')
    u = prog.NewContinuousVariables(rows=time_steps, cols=n_u, name='u')

    # Binary variable
    delta = prog.NewContinuousVariables(rows=time_steps + 1, cols=n_po, name='delta')

    return x, u, delta


def set_initial_and_terminal_position(prog, start, goal, decision_variables):

    # Unpack state and input
    x, u = decision_variables[:2]
    #pdb.set_trace()

    # Initial state
    x_0 = np.concatenate((start, np.array([0, 0, 0, 0])))

    # Final state
    x_N = np.concatenate((goal, np.array([0, 0, 0, 0])))

    # Enforce initial constraint
    prog.AddLinearConstraint(eq(x[0], x_0))

    # Enforce final constraint
    prog.AddLinearConstraint(eq(x[-1], x_N))


def set_dynamics(prog, usv, decision_variables, time_interval, time_steps):

    # Unpack state and input
    x, u = decision_variables[:2]

    # Enforce discrete dynamics
    for t in range(time_steps):
        residuals = usv.discrete_dynamics(x[t], x[t+1], u[t], time_interval)
        for residual in residuals:
            prog.AddConstraint(residual == 0)


def set_initial_guess(prog, start, goal, decision_variables, time_interval, time_steps):

    # Unpack state and input
    x, u = decision_variables[:2]

    # initial guess
    p_guess = interpolate_rocket_state(start, goal, time_interval, time_steps)
    prog.SetInitialGuess(x[:,:2], p_guess[:,:2])


def set_circle_obstacles(prog, obstacles, decision_variables, time_steps):

    # Unpack state and input
    x, u = decision_variables[:2]

    # Enforce circular constraints dist(circle)**2 >= r**2
    for t in range(time_steps):
        for circle in obstacles:
            p = x[t,:2] - circle.position
            residual = p.dot(p) - circle.radius**2
            prog.AddConstraint(residual >= 0)


def set_polygon_obstacles(prog, obstacles, decision_variables, start, goal, time_steps, n_po):

    # Unpack state, input and binary
    x, u, delta = decision_variables[:3]

    # big-M vector
    M = get_big_M(start, goal)

    # Enforce polygon constraints Ax >= b
    for t in range(time_steps) :
        for n in range(n_po):
            polygon = obstacles[n]

            A = polygon.A
            b = polygon.b

            n_sides = len(b)

            M_n = np.array(M * n_sides)
            
            prog.AddLinearConstraint(
                ge(polygon.A.dot(x[t,:2]), polygon.b - (1 - delta[t, n])*M_n)
            )


def set_binary(prog, decision_variables, time_steps):

    # Unpack state, input and binary
    x, u, delta = decision_variables[:3]

    # Enforce binary constraint
    for t in range(time_steps + 1):
        sum_delta = delta[t].sum()
        prog.AddLinearConstraint(sum_delta == 1)


def add_cost(prog, decision_variables, time_interval, time_steps):

    # Unpack state and input
    x, u = decision_variables[:2]

    # minimize fuel consumption
    for t in range(time_steps):
        prog.AddCost(time_interval*u[t].dot(u[t]))


def get_big_M(start, goal):
    M = []

    p = goal - start

    dist = p.dot(p)
    M.append(dist)

    return M






# function that interpolates two given positions of the rocket
# velocity is set to zero for all the times
def interpolate_rocket_state(p_initial, p_final, time_interval=0.2, time_steps=100):

    # initial and final time and state
    time_limits = [0., time_steps * time_interval]
    position_limits = np.column_stack((p_initial, p_final))
    state_limits = np.vstack((position_limits, np.zeros((2, 2))))

    # linear interpolation in state
    state = PiecewisePolynomial.FirstOrderHold(time_limits, state_limits)

    # sample state on the time grid
    state_guess = np.vstack([state.value(t * time_interval).T for t in range(time_steps + 1)])

    return state_guess
