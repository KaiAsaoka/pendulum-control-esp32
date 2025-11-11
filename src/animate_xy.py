# animate_xy.py
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

SCRIPT_DIR = Path(__file__).resolve().parent
DATAFILE = SCRIPT_DIR / "data.txt"
ANIMATE = True
FPS = 6     # <-- speed of animation
MAX_FRAMES = 2000      # downsample if very long
TAIL_POINTS = None     # or set an integer to show only a trailing tail

def load_txy(p: Path):
    try:
        raw = np.loadtxt(p, delimiter=None)
    except Exception:
        raw = np.loadtxt(p, delimiter=",")
    if raw.ndim != 2 or raw.shape[1] < 4:
        raise ValueError("Expected at least 4 columns: x, y, <col3>, time")
    x = raw[:, 0].astype(float)
    y = raw[:, 1].astype(float)
    t = raw[:, 3].astype(float)
    if np.nanmax(t) > 1e5:
        t = t * 1e-6
    return t, x, y

def animate_parametric(t, x, y):
    n = len(t)
    if n <= MAX_FRAMES:
        idx = np.arange(1, n)
    else:
        step = int(np.ceil(n / MAX_FRAMES))
        idx = np.arange(step, n, step)

    fig, ax = plt.subplots(figsize=(7, 6))
    line, = ax.plot([], [], lw=2, animated=True)
    dot,  = ax.plot([], [], "o", animated=True)
    txt = ax.text(0.02, 0.98, "", transform=ax.transAxes, va="top", ha="left", animated=True)
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True)

    xmin, xmax = np.nanmin(x), np.nanmax(x)
    ymin, ymax = np.nanmin(y), np.nanmax(y)
    dx = (xmax - xmin) * 0.05 + 1e-12
    dy = (ymax - ymin) * 0.05 + 1e-12
    ax.set_xlim(xmin - dx, xmax + dx)
    ax.set_ylim(ymin - dy, ymax + dy)

    def init():
        line.set_data([], [])
        dot.set_data([], [])
        txt.set_text("")
        return line, dot, txt

    def update(k):
        j = idx[k]
        if TAIL_POINTS is None:
            i0 = 0
        else:
            i0 = max(0, j - TAIL_POINTS)
        line.set_data(x[i0:j+1], y[i0:j+1])
        dot.set_data(x[j], y[j])
        txt.set_text(f"t = {t[j]:.3f} s")
        return line, dot, txt

    ani = FuncAnimation(
        fig, update, frames=len(idx), init_func=init,
        interval=1000 / FPS, blit=True
    )
    return ani

def main():
    t, x, y = load_txy(DATAFILE)
    if ANIMATE:
        global ANI
        ANI = animate_parametric(t, x, y)  # keep a reference
        plt.show()

if __name__ == "__main__":
    print(f"Using data file: {DATAFILE}")
    main()
