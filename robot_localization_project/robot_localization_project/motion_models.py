import numpy as np
from numpy.linalg import inv
import sympy
from sympy import symbols, Matrix

def velocity_mm():
        x, y, theta, v, w, dt = symbols('x y theta v w dt')
        u_t = Matrix([v, w])
        gux = Matrix([
                x,
                y,
                theta,
        ])+\
        Matrix([
                - (v/w) * sympy.sin(theta) + (v/w) * sympy.sin(theta + w * dt),
                + (v/w) * sympy.cos(theta) - (v/w) * sympy.cos(theta + w * dt),
                + w * dt
        ])

        gux_w_zero = Matrix([
                x,
                y,
                theta,
        ])+\
        Matrix([
                - v * dt * sympy.sin(theta),
                + v * dt * sympy.cos(theta),
                0.0
        ])

        # Define the state vector x_t
        x_t = sympy.Matrix([x, y, theta])

        eval_gux = sympy.lambdify((x, y, theta, v, w, dt), gux, 'numpy')
        Gt = gux.jacobian(x_t)
        eval_Gt = sympy.lambdify((x, y, theta, v, w, dt), Gt, 'numpy')
        Vt = gux.jacobian(u_t)
        eval_Vt = sympy.lambdify((x, y, theta, v, w, dt), Vt, 'numpy')

        eval_gux_w_zero = sympy.lambdify((x, y, theta, v, dt), gux_w_zero, 'numpy')
        Gt_w_zero = gux_w_zero.jacobian(x_t)
        eval_Gt_w_zero = sympy.lambdify((x, y, theta, v, dt), Gt_w_zero, 'numpy')
        Vt_w_zero = gux_w_zero.jacobian(u_t)
        eval_Vt_w_zero = sympy.lambdify((x, y, theta, v, dt), Vt_w_zero, 'numpy')

        return eval_gux, eval_Gt, eval_Vt, eval_gux_w_zero, eval_Gt_w_zero, eval_Vt_w_zero

def odometry_mm():
    
    x, y, theta = symbols('x y theta')
    delta_rot1, delta_trasl, delta_rot2 = symbols('delta_rot1 delta_trasl delta_rot2')
    u_t = sympy.Matrix([delta_rot1, delta_trasl, delta_rot2])
    x_t = sympy.Matrix([x, y, theta])
    gux_odom = Matrix([
        x,
        y,
        theta,
    ])+\
    sympy.Matrix([
        + delta_trasl*sympy.cos(delta_rot1 + theta),
        + delta_trasl*sympy.sin(delta_rot1 + theta),
        + delta_rot1 + delta_rot2
    ])
    Gt_odom = gux_odom.jacobian(x_t)
    Vt_odom = gux_odom.jacobian(u_t)
    eval_gux_odom = sympy.lambdify((x, y, theta, delta_rot1, delta_trasl, delta_rot2), gux_odom, 'numpy')
    eval_Gt_odom = sympy.lambdify((x, y, theta, delta_rot1, delta_trasl, delta_rot2), Gt_odom, 'numpy')
    eval_Vt_odom = sympy.lambdify((x, y, theta, delta_rot1, delta_trasl, delta_rot2), Vt_odom, 'numpy')
    return eval_gux_odom, eval_Gt_odom, eval_Vt_odom

def get_odometry_input(x, x_prev):
    x = x.flatten()
    x_prev = x_prev.flatten()
    if x[0]!=x_prev[0]:
        rot1 = sympy.atan2(x[1]-x_prev[1],x[0]-x_prev[0]) -x_prev[2]
    else:
        rot1 = -x_prev[2]
    trasl = sympy.sqrt(((x[0]-x_prev[0])**2) + ((x[1]-x_prev[1])**2))
    rot2 = x[2] - rot1 - x_prev[2]
    return np.array([[rot1, trasl, rot2]]).T
