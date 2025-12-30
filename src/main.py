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
A, B = systemDynamics.get_jacobian(x_ref_, u_ref_)
C = np.array(
    [
        [1, 0, 0, 0, 0, 0, 0, 0],  # sensor for x
        [0, 0, 1, 0, 0, 0, 0, 0],  # sensor for z
    ]
)
# if using C = [1, 0, 1, 0, 0, 0, 0, 0], it implies that it is one sensor that can measure both x and z, the rank will be insufficient.


lqr = lqr_controller(x_ref_, u_ref_, A, B, Ki)
Kr = lqr.get_Kr_(A, B, Q, R)
x_ = np.array([5, 0, 10, 0, 0.25 * np.pi - 1.0e-3, 0, 0 + 1.0e-3, 0])
x_hat_ = np.array(
    [5, 0, 10, 0, 0.25 * np.pi - 1.0e-3, 0, 0 + 1.0e-3, 0]
)  # initial estmated x
P = np.eye(8) * 0.1
Q_noise = np.eye(8) * 0.001
R_noise = np.array([[0.1, 0.1], [0.1, 0.1]])

# main loop
int_ = np.zeros_like(x_)
steps = 20000
hist = []
x_hist = []
dx_hist = []
z_hist = []
dz_hist = []
theta_d_hist = []
dtheta_d_hist = []
theta_p_hist = []
dtheta_p_hist = []
x_hat_hist = []
z_hat_hist = []
theta_d_hat_hist = []
theta_p_hat_hist = []
t_terminate = 0
dt = 1.0e-3
for step in range(steps):
    # in controller, it shouldnt have access to the world
    int_ += (x_ref_ - x_hat_) * dt
    u_ = lqr.control_law(x_hat_, int_, Kr)
    dx_hat_pred = systemDynamics.dynamics(x_hat_, u_)
    x_hat_pred_ = x_hat_ + dx_hat_pred * dt
    Df_Dx, _ = systemDynamics.get_jacobian(x_hat_, u_)
    F = np.eye(8) + Df_Dx * dt

    P_pred = F @ P @ F.T + Q_noise

    # simulation
    dx_ = systemDynamics.dynamics(x_, u_)
    x_ += dx_ * dt
    x_ += np.random.normal(0, 0.0001, size=8)  # some gust ig idk

    # measure
    y_ = C @ x_ + np.random.normal(0, 0.001, size=2)
    y_err_ = y_ - (C @ x_hat_pred_)
    S = C @ P_pred @ C.T + R_noise
    Kf = P_pred @ C.T @ np.linalg.solve(S, np.eye(len(S)))

    x_hat_ = x_hat_pred_ + Kf @ y_err_
    P = (np.eye(8) - Kf @ C) @ P_pred

    hist.append(x_.copy())
    x, dx, z, dz, theta_d, dtheta_d, theta_p, dtheta_p = x_
    x_hat, _, z_hat, _, theta_d_hat, _, theta_p_hat, _ = x_hat_
    theta_d_hat = np.arctan2(np.sin(theta_d_hat), np.cos(theta_d_hat))
    theta_p_hat = np.arctan2(np.sin(theta_p_hat), np.cos(theta_p_hat))
    theta_d = np.arctan2(np.sin(theta_d), np.cos(theta_d))
    theta_p = np.arctan2(np.sin(theta_p), np.cos(theta_p))
    x_hat_hist.append(x_hat)
    z_hat_hist.append(z_hat)
    theta_d_hat_hist.append(theta_d_hat)
    theta_p_hat_hist.append(theta_p_hat)
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
    x_hat_hist,
    z_hist,
    z_hat_hist,
    theta_d_hist,
    theta_d_hat_hist,
    theta_p_hist,
    theta_p_hat_hist,
]
states_name = [
    "x",
    "x_hat",
    "z",
    "z_hat",
    "theta_d",
    "theta_d_hat",
    "theta_p_hat",
    "theta_p_hat",
]
states = {}
n = len(states_hist)
for i in range(n):
    states[f"{states_name[i]}"] = [states_hist[i], 0]

plot_drone_LQR(states, t_terminate, Q, R)

animator = DronePoleAnimator(hist, pole_length=systemDynamics.l)
ani = animator.animate()
animator.show()
