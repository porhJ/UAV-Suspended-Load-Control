import numpy as np


class PackageDroneIdealDynamics:
    def __init__(self, m_d, m_p, Id_cm, r_d, l, x_ref_, u_ref_, g=9.8):
        self.m_d = m_d
        self.Id_cm = Id_cm
        self.r_d = r_d
        self.m_p = m_p
        self.l = l
        self.x_ref_ = x_ref_
        self.dx_ref_ = np.array([0, 0, 0, 0])
        self.u_ref_ = u_ref_
        self.g = g

    def dynamics(self, x_, u_):
        x, dx, z, dz, theta_d, dtheta_d, theta_p, dtheta_p = x_

        # frm M * ddq + C * dq = tau_g + Bu
        # its actually just newton's 2nd law but fancy
        # M is mass matrix (or intertia if it is in theta coordinate)
        # Thus force = tau_g + Bu - C * dq
        # So F = Ma where a is obviously ddq
        # tau_g + Bu - C * dq = M * ddq

        M = np.array(
            [
                [self.m_p + self.m_d, 0, 0, self.m_p * self.l * np.cos(theta_p)],
                [0, self.m_p + self.m_d, 0, self.m_p * self.l * np.sin(theta_p)],
                [0, 0, self.Id_cm, 0],
                [
                    self.m_p * self.l * np.cos(theta_p),
                    self.m_p * self.l * np.sin(theta_p),
                    0,
                    self.m_p * (self.l**2),
                ],
            ]
        )

        C = np.array(
            [
                [0, 0, 0, -self.l * dtheta_p * np.sin(theta_p)],
                [0, 0, 0, self.l * dtheta_p * np.cos(theta_p)],
                [0, 0, 0, 0],
                [0, 0, 0, 0],
            ]
        )

        tau_g = np.array(
            [
                [
                    0,
                    -(self.m_p + self.m_d) * self.g,
                    0,
                    -self.m_p * self.g * self.l * np.sin(theta_p),
                ]
            ]
        )
        tau_g = tau_g.T
        B = np.array(
            [
                [np.cos(theta_d), np.cos(theta_d)],
                [np.sin(theta_d), np.sin(theta_d)],
                [self.r_d, -self.r_d],
                [0, 0],
            ]
        )

        q_dot = np.array([[dx, dz, dtheta_d, dtheta_p]])
        q_dot = q_dot.T

        force = tau_g + B @ u_ - C @ q_dot
        # we want ddq, thus we use inv(M), but this is lowkey unsafe, so we just gonna use np.linalg.solve, it is safer. source: trust me bro
        ddq = np.linalg.solve(M, force).flatten()
        ddx, ddz, ddtheta_d, ddtheta_p = ddq

        # we gonna return dx_
        return np.array([dx, ddx, dz, ddz, dtheta_d, ddtheta_d, dtheta_p, ddtheta_p])

    def get_jacobian(self, eps=1e-6):
        n = len(self.x_ref_)
        m = 2
        A = np.zeros((n, n))
        B = np.zeros((n, m))
        f0 = self.dynamics(self.x_ref_, self.u_ref_)

        for i in range(n):
            y_perturb = self.x_ref_.copy()
            y_perturb[i] += eps
            f_perturb = self.dynamics(y_perturb, self.u_ref_)
            A[:, i] = (f_perturb - f0) / eps

        for j in range(m):
            u_perturb = self.u_ref_.copy()
            u_perturb[j] += eps
            f_perturb_u = self.dynamics(self.x_ref_, u_perturb)
            B[:, j] = (f_perturb_u - f0) / eps

        return A, B

    def linearized_dynamics(self, x_, u_, A, B):
        delta_x_ = x_ - self.x_ref_
        delta_u_ = u_ - self.u_ref_
        delta_dx_ = A @ delta_x_ + B @ delta_u_
        dx_ = self.dx_ref_ + delta_dx_
        return dx_
