import numpy as np
from numpy.linalg import inv
import sympy
from sympy import symbols, Matrix, sqrt, atan2
from numpy.random import randn

def landmarks_mm():
    x, y, theta, m_x, m_y = symbols('x y theta m_x m_y')
    x_t = Matrix([x, y, theta])
    hx = Matrix([
        sqrt((m_x - x)**2 + (m_y - y)**2),
        atan2(m_y - y, m_x - x) - theta
    ])
    eval_hx = sympy.lambdify((x, y, m_x, m_y, theta), hx, 'numpy')
    Ht = hx.jacobian(x_t)
    eval_Ht = sympy.lambdify((x, y, m_x, m_y, theta), Ht, 'numpy')
    return eval_hx, eval_Ht

def z_landmark(x, lmark, std_rng=0.5, max_range= 8.0, fov_deg= 45, std_brg=0.5, eval_hx=None):
    if eval_hx == None:
        raise ValueError("eval_hx function must be provided.")
    x = x.flatten().astype('float64')
    z = eval_hx(x= x[0], y= x[1], m_x= lmark[0], m_y= lmark[1], theta = x[2]).astype('float64')
    
    # filter z for a more realistic sensor simulation (add a max range distance and a FOV)
    fov = np.deg2rad(fov_deg)
    if z[0, 0] < max_range and abs(z[1, 0])<fov:
        return z + np.array([[randn() * std_rng**2, randn() * std_brg]]).T
        
    return None

def residual(a, b):
    y = a - b
    y[1] = y[1] % (2 * np.pi)  # force in range [0, 2 pi)
    if y[1] > np.pi:  # move to [-pi, pi)
        y[1] -= 2 * np.pi
    return y


