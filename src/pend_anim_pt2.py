import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

def pend_anim_pt2(filename="data.txt", speed=4.0):
    """
    Reads data from a text file and animates cart + inverted pendulum
    motion in two side-by-side views: 
    x-track and angle vs time and 
    y-track and angle vs time.

    speed : float
        0    → as fast as possible (just minimal pause for GUI).
        1    → real-time (assuming 1s increments).
        2    → 2× faster than real-time.
        0.5  → half-speed.
    """
    # ---------- DATA ----------
    D = np.loadtxt(filename, delimiter=",")

    if D.ndim != 2 or D.shape[1] < 4:
        raise ValueError("Data file must have at least 4 columns: x, y, theta, t.")
    x  = D[:, 0]
    y  = D[:, 1]
    th = D[:, 2]
    t  = D[:, 3]

    # If angles appear to be in degrees, convert to radians
    if np.max(np.abs(th)) > np.pi:
        th = np.deg2rad(th)

    # ---------- GEOMETRY / VISUALS ----------
    L = 1.5
    cw = 0.12
    ch = 0.06
    floorY = 0.0

    # Fixed vertical span tied to x-range
    xRange = float(x.max() - x.min())
    if xRange <= 0:
        xRange = L
    pivotY = floorY + ch
    ymin   = pivotY - xRange / 2.0
    ymax   = pivotY + xRange / 2.0

    # Figure/axes
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 4))
    try:
        fig.canvas.manager.set_window_title("Inverted pendulum — speed dial")
    except Exception:
        pass
    fig.patch.set_facecolor("white")

    xlim1 = (x.min() - L - cw, x.max() + L + cw)
    xlim2 = (y.min() - L - cw, y.max() + L + cw)

    # Left axis: X-track and angle vs time
    ax1.set_xlim(xlim1)
    ax1.set_ylim(ymin, ymax)
    ax1.set_aspect("equal", adjustable="box")
    ax1.set_xlabel("x")
    ax1.set_ylabel("height")
    ax1.set_title("X-track")

    # Right axis: Y-track and angle vs time
    ax2.set_xlim(xlim2)
    ax2.set_ylim(ymin, ymax)
    ax2.set_aspect("equal", adjustable="box")
    ax2.set_xlabel("y")
    ax2.set_ylabel("height")
    ax2.set_title("Y-track (y on horizontal)")

    # Ground lines
    ax1.plot([xlim1[0], xlim1[1]], [floorY, floorY], "k-", linewidth=1)
    ax2.plot([xlim2[0], xlim2[1]], [floorY, floorY], "k-", linewidth=1)

    # Left (x-axis view)
    cart1 = Rectangle((x[0] - cw / 2.0, floorY), cw, ch,
                      linewidth=1, edgecolor="k", facecolor=(0.85, 0.85, 0.85))
    ax1.add_patch(cart1)
    (stick1,) = ax1.plot([0, 0], [0, 0], "-", linewidth=2, color="k")
    (bob1,)   = ax1.plot(0, 0, "o", markersize=10, linewidth=2, color="k")

    # Right (y-axis view)
    cart2 = Rectangle((y[0] - cw / 2.0, floorY), cw, ch,
                      linewidth=1, edgecolor="k", facecolor=(0.85, 0.85, 0.85))
    ax2.add_patch(cart2)
    (stick2,) = ax2.plot([0, 0], [0, 0], "-", linewidth=2, color="k")
    (bob2,)   = ax2.plot(0, 0, "o", markersize=10, linewidth=2, color="k")

    # ---------- Precompute bob positions ----------
    s = np.sin(th)
    c = np.cos(th)

    bx1 = x + L * s
    by1 = pivotY + L * c

    bx2 = y + L * s
    by2 = pivotY + L * c

    # ---------- Animation loop ----------
    n = len(t)

    plt.ion()  # interactive mode on (so the window updates in the loop)
    for k in range(n):
        # Left (x-axis view)
        cart1.set_xy((x[k] - cw / 2.0, floorY))
        stick1.set_data([x[k], bx1[k]], [pivotY, by1[k]])
        bob1.set_data(bx1[k], by1[k])

        # Right (y-axis view)
        cart2.set_xy((y[k] - cw / 2.0, floorY))
        stick2.set_data([y[k], bx2[k]], [pivotY, by2[k]])
        bob2.set_data(bx2[k], by2[k])

        fig.canvas.draw_idle()

        # Timing control
        if speed > 0 and k < n - 1:
            dt = t[k + 1] - t[k]
            if dt > 0:
                delay = max(dt / speed, 1e-3)  # don't let it go to 0
            else:
                delay = 1e-3
        else:
            # speed <= 0 → as fast as possible, but keep GUI responsive
            delay = 1e-3

        plt.pause(delay)

    plt.ioff()
    plt.show()


if __name__ == "__main__":
    # Build a path to data.txt in the same folder as this script
    here = os.path.dirname(os.path.abspath(__file__))
    data_path = os.path.join(here, "data.txt")

    pend_anim_pt2(data_path, speed=4.0)
