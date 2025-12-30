from turtle import color

import matplotlib.pyplot as plt
import numpy as np
from matplotlib import animation
from matplotlib.patches import Polygon


def plot_drone_LQR(states, t_terminate, Q, R):
    fig, ax = plt.subplots(figsize=(12, 8))
    for state_name, state in states.items():
        state_hist, state_eq = state
        line_style = "-"
        if state_name.endswith("_hat"):
            line_style = "-."
        plt.plot(state_hist, line_style, label=f"{state_name}")
        """
        plt.axhline(
            y=state_eq, linestyle="--", label=f"{state_name} equilibrium at {state_eq}"
        )
        """
    plt.title("Slung Load Drone System LQR-Control")
    plt.axvline(
        x=t_terminate,
        color="g",
        linestyle="--",
        label=f"Reach final point time (x={t_terminate})",
    )
    plt.xlabel("Time Steps")
    plt.legend()
    Q_vals = "_".join(f"{v:g}" for v in np.diag(Q))
    R_vals = "_".join(f"{v:g}" for v in np.diag(R))

    plt.legend(bbox_to_anchor=(1.05, 1), loc="upper left", borderaxespad=0.0)
    plt.figtext(
        0.5, 0.01, f"Q = diag({Q_vals})  R = diag({R_vals})", ha="center", fontsize=10
    )
    plt.subplots_adjust(right=0.75, bottom=0.15)
    plt.show()


def plot_drone_path(x, y, x_ref, y_ref):
    plt.plot(x, y, "-o", markersize=0.5)
    plt.xlabel("x")
    plt.ylabel("z")
    plt.axis("equal")
    plt.axhline(y=y_ref, linestyle="--")
    plt.axvline(x=x_ref, linestyle="--")
    plt.scatter(x[-1], y[-1], color="r")
    plt.figtext(
        0.5,
        0.01,
        f"Start@ ({x[0]}, {y[0]}), End@ ({x[-1]}, {y[-1]})",
        ha="center",
        fontsize=10,
    )
    plt.title("Path of the drone")
    plt.show()


class DronePoleAnimator:
    def __init__(
        self,
        hist,
        pole_length,
        drone_width=0.6,
        drone_height=0.1,
        xlim=(-2, 20),
        ylim=(-1, 20),
        frame_step=10,
        interval=20,
    ):
        self.hist = hist
        self.l = pole_length
        self.drone_width = drone_width
        self.drone_height = drone_height
        self.frame_step = frame_step
        self.interval = interval

        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim(*xlim)
        self.ax.set_ylim(*ylim)
        self.ax.set_aspect("equal")
        self.ax.grid()

        self.drone = Polygon(np.zeros((4, 2)), fc="black", closed=True)
        (self.pole,) = self.ax.plot([], [], lw=2)
        self.ax.add_patch(self.drone)

    def init(self):
        self.drone.set_xy(np.zeros((4, 2)))
        self.pole.set_data([], [])
        return self.drone, self.pole

    def update(self, i):
        x, _, z, _, theta_d, _, theta_p, _ = self.hist[i]

        # --- 1. Calculate Drone Corners Manually ---
        # Rotation angle (adjusting for your coordinate system if needed)
        # Your previous code used (theta_d - pi/2), keeping that logic:
        angle = theta_d - np.pi / 2
        cos_t = np.cos(angle)
        sin_t = np.sin(angle)

        # Half dimensions
        w2 = self.drone_width / 2
        h2 = self.drone_height / 2

        # Local corners of the rectangle (centered at 0,0)
        # order: top-left, top-right, bottom-right, bottom-left
        local_x = np.array([-w2, w2, w2, -w2])
        local_z = np.array([h2, h2, -h2, -h2])

        # Rotate and Translate
        # x_global = x_center + (x_local * cos - z_local * sin)
        # z_global = z_center + (x_local * sin + z_local * cos)
        rot_x = x + (local_x * cos_t - local_z * sin_t)
        rot_z = z + (local_x * sin_t + local_z * cos_t)

        # Stack into (4, 2) array for Polygon
        verts = np.column_stack([rot_x, rot_z])
        self.drone.set_xy(verts)

        # --- 2. Pole Geometry (Unchanged) ---
        px = x + self.l * np.sin(theta_p)
        pz = z - self.l * np.cos(theta_p)
        self.pole.set_data([x, px], [z, pz])

        return self.drone, self.pole

    def animate(self):
        self.ani = animation.FuncAnimation(
            self.fig,
            self.update,
            frames=range(0, len(self.hist), self.frame_step),
            init_func=self.init,
            interval=self.interval,
            blit=True,
        )
        return self.ani

    def show(self):
        plt.show()
