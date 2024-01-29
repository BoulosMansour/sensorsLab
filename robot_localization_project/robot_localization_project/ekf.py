import numpy as np
from numpy.linalg import inv
import sympy
from sympy import symbols, Matrix

class RobotEKF:
    def __init__(
        self,
        mu = np.zeros((1,1)),
        Sigma = np.eye(1),
        Mt = np.eye(1),
        Qt = np.eye(1),
        dim_x=1,
        eval_gux=None, eval_Gt=None, eval_Vt=None,
        eval_hx=None, eval_Ht=None,
        eval_gux_w_zero=None, eval_Gt_w_zero=None, eval_Vt_w_zero=None,
    ):
        self.mu = mu  # mean state estimate
        self.Sigma = Sigma  # covariance state estimate
        self.Mt = Mt  # process noise
        self.Qt = Qt  # measurement noise

        self.eval_gux = eval_gux
        self.eval_Gt = eval_Gt
        self.eval_Vt = eval_Vt

        self.eval_gux_w_zero = eval_gux_w_zero
        self.eval_Gt_w_zero = eval_Gt_w_zero
        self.eval_Vt_w_zero = eval_Vt_w_zero

        self.eval_hx = eval_hx
        self.eval_Ht = eval_Ht

        self._I = np.eye(dim_x)  # identity matrix used for computations

    def predict(self, u, g_extra_args=()):
        # Update the state prediction evaluating the motion model
        mu = self.mu.flatten()
        if len(u) == 2:
            if u[1] == 0.0:
                self.mu = self.eval_gux_w_zero(v = u[0], x = mu[0], y=mu[1], theta=mu[2], dt= np.array(g_extra_args))
            else:    
                self.mu = self.eval_gux(v = u[0],w = u[1], x = mu[0], y=mu[1], theta=mu[2], dt= np.array(g_extra_args))
            self.mu[2][0]=((self.mu[2][0] + np.pi) % (2*np.pi)) - np.pi
            # Update the covariance matrix of the state prediction, 
            # you need to evaluate the Jacobians Gt and Vt
            Gt = self.eval_Gt(v = u[0],w = u[1], x = mu[0], y=mu[1], theta=mu[2], dt= np.array(g_extra_args))
            Vt = self.eval_Vt(v = u[0],w = u[1], x = mu[0], y=mu[1], theta=mu[2], dt= np.array(g_extra_args))
        elif len(u) == 3:
            self.mu = self.eval_gux(delta_rot1 = u[0],delta_trasl = u[1],delta_rot2=u[2], x = mu[0], y=mu[1], theta=mu[2])
            self.mu[2][0]=((self.mu[2][0] + np.pi) % (2*np.pi)) - np.pi
            # Update the covariance matrix of the state prediction, 
            # you need to evaluate the Jacobians Gt and Vt
            Gt = self.eval_Gt(delta_rot1 = u[0],delta_trasl = u[1],delta_rot2=u[2], x = mu[0], y=mu[1], theta=mu[2]).astype('float64')
            Vt = self.eval_Vt(delta_rot1 = u[0],delta_trasl = u[1],delta_rot2=u[2], x = mu[0], y=mu[1], theta=mu[2]).astype('float64')

        self.Sigma = Gt @ self.Sigma @ Gt.T + Vt @ self.Mt @ Vt.T

    def update(self, z, lmark, residual=np.subtract):
    
        # Convert the measurement to a vector if necessary. Needed for the residual computation
        if np.isscalar(z) and self.dim_z == 1:
            z = np.asarray([z], float)
                
        # Compute the Kalman gain, you need to evaluate the Jacobian Ht
        mu = self.mu.flatten()
        Ht = self.eval_Ht(m_x=lmark[0], m_y = lmark[1], x = mu[0], y= mu[1], theta = mu[2]).astype('float64')
        self.S = ((Ht @ self.Sigma @ Ht.T) + self.Qt).astype('float64')
        self.K = (self.Sigma @ Ht.T @ inv(self.S)).astype('float64')

        # Evaluate the expected measurement and compute the residual, then update the state prediction
        z_hat = self.eval_hx(m_x=lmark[0], m_y = lmark[1], x = mu[0], y= mu[1], theta = mu[2]).astype('float64')
        self.y = (z - z_hat).astype('float64')
        self.mu = (self.mu + (self.K @ self.y)).astype('float64')
        self.mu[2][0]=((self.mu[2][0] + np.pi) % (2*np.pi)) - np.pi

        # P = (I-KH)P(I-KH)' + KRK' is more numerically stable and works for non-optimal K vs the equation
        # P = (I-KH)P usually seen in the literature. 
        # Note that I is the identity matrix.
        I_KH = (np.eye(Ht.shape[1]) - (self.K @ Ht)).astype('float64')
        self.Sigma = ((I_KH @ self.Sigma @ I_KH.T)+ (self.K @ self.Qt @ (self.K).T)).astype('float64')

