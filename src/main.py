import numpy as np

from controller import lqr_controller
from dynamics import PackageDroneIdealDynamics
from viz import DronePoleAnimator, plot_drone_LQR

m_d = 5
m_p = 2
M = m_d + m_p
g = 9.8
r_d = 0.5
Id_cm = 1 / 12 * m_d * (2 * r_d) ** 2
l = 0.5
x_ref = 7
z_ref = 5
x_ref_ = np.array([x_ref, 0, z_ref, 0, 0.5 * np.pi, 0, 0, 0])
u_ref_ = np.array([[0.5 * M * g, 0.5 * M * g]])
u_ref_ = u_ref_.T
Ki = np.array([[5, 0, 5, 0, 5, 0, 5, 0], [5, 0, 5, 0, 5, 0, 5, 0]])
int_ = 0
int_ = 0
t_terminate = 0
Q = np.diag([40, 20, 50, 20, 100, 50, 100, 50])
R = np.diag([0.1, 0.1])

systemDynamics = PackageDroneIdealDynamics(m_d, m_p, Id_cm, r_d, l, x_ref_, u_ref_)
A, B = systemDynamics.get_jacobian()

lqr = lqr_controller(x_ref_, u_ref_, A, B, Ki)
Kr = lqr.get_Kr_(A, B, Q, R)
x_ = np.array([5, 0, 10, 0, 0.25 * np.pi - 1.0e-3, 0, 0 + 1.0e-3, 0])

# main loop
int_ = np.zeros_like(x_)
steps = 30000
hist = []
x_hist = []
dx_hist = []
z_hist = []
dz_hist = []
theta_d_hist = []
dtheta_d_hist = []
theta_p_hist = []
dtheta_p_hist = []
t_terminate = 0
for step in range(steps):
    x_, int_ = lqr.step(systemDynamics, x_, Kr=Kr, int_=int_)
    hist.append(x_)
    x, dx, z, dz, theta_d, dtheta_d, theta_p, dtheta_p = x_
    theta_d = np.arctan2(np.sin(theta_d), np.cos(theta_d))
    theta_p = np.arctan2(np.sin(theta_p), np.cos(theta_p))
    x_hist.append(x)
    dx_hist.append(dx)
    z_hist.append(z)
    dz_hist.append(dz)
    theta_d_hist.append(theta_d)
    dtheta_d_hist.append(dtheta_d)
    theta_p_hist.append(theta_p)
    dtheta_p_hist.append(dtheta_p)
    if abs(x - x_ref) < 0.001 and abs(z - z_ref) < 0.001 and t_terminate == 0:
        t_terminate = step

states_hist = [
    x_hist,
    dx_hist,
    z_hist,
    dz_hist,
    theta_d_hist,
    dtheta_d_hist,
    theta_p_hist,
    dtheta_p_hist,
]
states_name = ["x", "dx", "z", "dz", "theta_d", "dtheta_d", "theta_p", "dtheta_p"]
states = {}
n = len(states_hist)
for i in range(n):
    states[f"{states_name[i]}"] = [states_hist[i], x_ref_[i]]

plot_drone_LQR(states, t_terminate, Q, R)

animator = DronePoleAnimator(hist, pole_length=systemDynamics.l)
ani = animator.animate()
animator.show()
