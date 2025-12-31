import numpy as np
from scipy.linalg import solve_discrete_are


class lqr_controller:
    def __init__(self, x_ref_, u_ref_, A, B, Ki):
        self.x_ref_ = x_ref_
        self.u_ref_ = u_ref_
        self.A = A
        self.B = B
        self.Ki = Ki

    def get_Kr_(self, A, B, Q, R):
        P = solve_discrete_are(A, B, Q, R)
        return np.linalg.inv(R) @ B.T @ P

    def control_law(self, x_, int_, Kr):
        error_x = (x_ - self.x_ref_).reshape(-1, 1)
        error_int = int_.reshape(-1, 1)
        delta_u = -Kr @ error_x + self.Ki @ error_int
        return self.u_ref_ + delta_u

    def step(self, system, x_, Kr, int_, dt=1.0e-3):
        int_ += (self.x_ref_ - x_) * dt
        u_ = self.control_law(x_, int_, Kr)
        dx_ = system.dynamics(x_, u_)
        return x_ + dx_ * dt, int_
