import matplotlib.pyplot as plt
import numpy as np
from matplotlib import animation
from matplotlib.patches import Rectangle
from matplotlib.transforms import Affine2D


def plot_drone_LQR(states, t_terminate, Q, R):
    fig, ax = plt.subplots(figsize=(12, 8))
    for state_name, state in states.items():
        state_hist, state_eq = state
        plt.plot(state_hist, label=f"{state_name}")
        plt.axhline(
            y=state_eq, linestyle="--", label=f"{state_name} equilibrium at {state_eq}"
        )
    plt.title("Inverted Pendulum LQR-Control")
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

        self.drone = Rectangle((0, 0), drone_width, drone_height, fc="black")
        (self.pole,) = self.ax.plot([], [], lw=2)
        self.ax.add_patch(self.drone)

    def init(self):
        self.drone.set_xy((-self.drone_width / 2, -self.drone_height / 2))
        self.pole.set_data([], [])
        return self.drone, self.pole

    def update(self, i):
        x, _, z, _, theta_d, _, theta_p, _ = self.hist[i]

        # camera follow
        """
        W = 4.0
        H = 4.0
        self.ax.set_xlim(x - W / 2, x + W / 2)
        self.ax.set_ylim(z - H / 2, z + H / 2)
        """
        # --- Drone transform (rotate about CENTER) ---
        t = (
            Affine2D().rotate_around(x, z, theta_d - np.pi / 2)  # radians OK here
            + self.ax.transData
        )

        self.drone.set_transform(t)
        self.drone.set_xy((x - self.drone_width / 2, z - self.drone_height / 2))

        # --- Pole geometry (unchanged, correct) ---
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
