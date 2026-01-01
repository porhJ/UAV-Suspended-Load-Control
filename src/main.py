import matplotlib.pyplot as plt
import numpy as np

from controller import lqr_controller
from dynamics import PackageDroneIdealDynamics
from viz import DronePoleAnimator, plot_drone_path, plot_state_history

# initializing stuffs
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
Ki = np.array([[0, 0, 5, 0, 0, 0, 0, 0], [0, 0, 5, 0, 0, 0, 0, 0]])
int_ = 0
int_ = 0
t_terminate = 0
Q = np.diag([40, 20, 50, 20, 100, 50, 100, 50])
R = np.diag([0.1, 0.1])
dt = 1.0e-3
systemDynamics = PackageDroneIdealDynamics(m_d, m_p, Id_cm, r_d, l, x_ref_, u_ref_)
A, B = systemDynamics.get_jacobian(x_ref_, u_ref_)
A = np.eye(len(A)) + A * dt
B = B * dt
C = np.array(
    [
        [1, 0, 0, 0, 0, 0, 0, 0],  # sensor for x
        [0, 0, 1, 0, 0, 0, 0, 0],  # sensor for z
        [0, 0, 0, 0, 1, 0, 0, 0],  # imu
    ]
)
# if using C = [1, 0, 1, 0, 0, 0, 0, 0], it implies =Nonethat it is one sensor that can measure both x and z, the rank will be insufficient.
# and that is NOT smart at alllllllllllllllll yoooooooooooooo


lqr = lqr_controller(x_ref_, u_ref_, A, B, Ki)
Kr = lqr.get_Kr_(A, B, Q, R)
x_ = np.array([5, 0, 10, 0, 0.25 * np.pi - 1.0e-3, 0, 0 + 1.0e-3, 0])
x_hat_ = np.array(
    [5, 0, 10, 0, 0.25 * np.pi - 1.0e-3, 0, 0 + 1.0e-3, 0]
)  # initial estmated x
P = np.eye(8) * 0.1
Q_noise = np.eye(8) * 0.001
R_noise = np.diag([0.1, 0.1, 0.01])
# main loop
int_ = np.zeros_like(x_)
steps = 20000
hist = []
hist_hat = []
t_terminate = 0
for step in range(steps):
    # in controller, it shouldnt have access to the world, it should only know what it can percieve by sensors
    int_ += (x_ref_ - x_hat_) * dt
    u_ = lqr.control_law(x_hat_, int_, Kr)
    dx_hat_pred = systemDynamics.dynamics(x_hat_, u_)
    x_hat_pred_ = x_hat_ + dx_hat_pred * dt
    Df_Dx, _ = systemDynamics.get_jacobian(x_hat_, u_)
    F = np.eye(8) + Df_Dx * dt

    P_pred = F @ P @ F.T + Q_noise

    # simulation, this is the real world
    dx_ = systemDynamics.dynamics(x_, u_)
    x_ += dx_ * dt
    x_ += np.random.normal(0, 0.001, size=8)  # some gust ig idk

    # measure
    y_ = C @ x_ + np.random.normal(0, 0.001, size=3)
    y_err_ = y_ - (C @ x_hat_pred_)
    S = C @ P_pred @ C.T + R_noise
    Kf = P_pred @ C.T @ np.linalg.solve(S, np.eye(len(S)))

    x_hat_ = x_hat_pred_ + Kf @ y_err_
    P = (np.eye(8) - Kf @ C) @ P_pred

    # remember the past ahh code
    # karn dern tarng korng chun lae ther kue karn rien ruuuu ahh
    x, dx, z, dz, theta_d, dtheta_d, theta_p, dtheta_p = x_
    x_hat, _, z_hat, _, theta_d_hat, _, theta_p_hat, _ = x_hat_
    hist.append(x_.copy())
    hist_hat.append(x_hat_.copy())

    if abs(x - x_ref) < 0.001 and abs(z - z_ref) < 0.001 and t_terminate == 0:
        t_terminate = step


hist = np.array(hist)
hist_hat = np.array(hist_hat)

hist[:, 4] = np.arctan2(np.sin(hist[:, 4]), np.cos(hist[:, 4]))
hist[:, 6] = np.arctan2(np.sin(hist[:, 6]), np.cos(hist[:, 6]))

hist_hat[:, 4] = np.arctan2(np.sin(hist_hat[:, 4]), np.cos(hist_hat[:, 4]))
hist_hat[:, 6] = np.arctan2(np.sin(hist_hat[:, 6]), np.cos(hist_hat[:, 6]))


states_name = [
    "x",
    "velocity_x",
    "z",
    "velocity_z",
    "theta_d",
    "angular_velocity_d",
    "theta_p",
    "angular_velocity_p",
]
states = {}

plot_state_history(hist, hist_hat, state_names=states_name, plot_error=True)
plot_drone_path(hist[:, 0], hist[:, 2])

animator = DronePoleAnimator(hist, pole_length=systemDynamics.l)
ani = animator.animate()
animator.show()
