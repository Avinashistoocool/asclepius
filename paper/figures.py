"""Figures for coverage_theory.md.  Run from the repository root:  py paper/figures.py"""

import os
import sys

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "swarm_lab"))

from medial_axis import analyze as medial  # noqa: E402
from sweep import compare, Grid, end_caps, geodesic_field, harmonic_field, lawnmower_field, mat_field  # noqa: E402

OUT = os.path.join(HERE, "figures")
os.makedirs(OUT, exist_ok=True)
plt.rcParams.update({"font.size": 10, "axes.titlesize": 11, "figure.dpi": 100, "savefig.bbox": "tight"})
GREEN, RED, LIME, GREY = "#0f9d58", "#d93025", "#7cb342", "#9e9e9e"


class Fourier:
    """Smooth closed boundary r(t) = R + sum a_k cos(kt) + b_k sin(kt), centred at the origin."""

    def __init__(self, R, a=(), b=()):
        self.R, self.a, self.b = R, np.asarray(a, float), np.asarray(b, float)
        self.k = np.arange(2, 2 + len(self.a))

    def r(self, t, der=0):
        t = np.asarray(t, float)
        out = (self.R if der == 0 else 0) + 0 * t
        for k, a, b in zip(self.k, self.a, self.b):
            c, s = np.cos(k * t), np.sin(k * t)
            out = out + [a * c + b * s, k * (-a * s + b * c), -k * k * (a * c + b * s)][der]
        return out

    def __call__(self, t):
        r = self.r(t)
        return np.array([r * np.cos(t), r * np.sin(t)])

    def d(self, t):
        r, dr = self.r(t), self.r(t, 1)
        return np.array([dr * np.cos(t) - r * np.sin(t), dr * np.sin(t) + r * np.cos(t)])


def pair_direction(g, y):
    u, v = y
    q = g(v) - g(u)
    w = np.array([q @ g.d(v), -(q @ g.d(u))])
    return w / np.linalg.norm(w)


def trace(g, start, eps=2e-3, h=.004, steps=6000):
    """Follow the bisecting-curve equation (4.3) from boundary point `start`; returns (u, v, reached_far_side)."""
    y, prev, out = np.array([start - eps, start + eps]), np.array([-1., 1.]), []
    orient = lambda y, ref: (lambda w: w if w @ ref >= 0 else -w)(pair_direction(g, y))
    for _ in range(steps):
        out.append(y.copy())
        k1 = orient(y, prev); k2 = orient(y + h / 2 * k1, k1); k3 = orient(y + h / 2 * k2, k1); k4 = orient(y + h * k3, k1)
        step = (k1 + 2 * k2 + 2 * k3 + k4) / 6
        y, prev = y + h * step, step / np.linalg.norm(step)
        gap = y[1] - y[0]
        if len(out) > 10 and (gap < eps or gap > 2 * np.pi - eps):
            return *np.array(out).T, True
    return *np.array(out).T, False


def outline(ax, g, **kw):
    t = np.linspace(0, 2 * np.pi, 600)
    ax.plot(*g(t), color=kw.pop("color", "black"), lw=kw.pop("lw", 1.6), **kw)


def clean(ax):
    ax.set_aspect("equal")
    ax.set_xticks([]), ax.set_yticks([])
    for side in ax.spines.values():
        side.set_visible(False)


def is_convex(g):
    t = np.linspace(0, 2 * np.pi, 2000)
    r, r1, r2 = g.r(t), g.r(t, 1), g.r(t, 2)
    return bool(np.all(r * r + 2 * r1 * r1 - r * r2 > 0))


# ---------------------------------------------------------------- Figure 1: what "bisecting" means
convex = Fourier(200, [20, 4], [0, 3])
assert is_convex(convex)
u, v, _ = trace(convex, 2.2)
mid = (convex(u) + convex(v)) / 2
fig, (left, right) = plt.subplots(1, 2, figsize=(11, 4.8), gridspec_kw={"width_ratios": [1.15, 1]})
outline(left, convex)
left.plot(*mid, color=LIME, lw=3, label="centre line $p(s)$")
for k in np.linspace(len(u) * .08, len(u) * .92, 9).astype(int):
    a, b = convex(u[k]), convex(v[k])
    left.plot(*np.c_[a, b], color=GREY, lw=1)
    left.plot(*mid[:, k], "o", color="black", ms=4)
    for end in (a, b):
        left.plot(*end, "o", color=GREY, ms=3)
left.legend(handles=[Line2D([], [], color=LIME, lw=3, label="centre line"),
                     Line2D([], [], color=GREY, lw=1, label="chords perpendicular to it"),
                     Line2D([], [], ls="", marker="o", color="black", ms=4, label="chord midpoints (on the line)")],
            loc="lower center", bbox_to_anchor=(.5, -.18), ncol=3, frameon=False, fontsize=9)
left.set_title("(a) A bisecting centre line: every perpendicular\nchord is cut exactly in half")
clean(left)

# (b) one chord close up, with the quantities used in the equations
p, n = np.array([0., 0.]), np.array([0.25, 1.]) / np.hypot(.25, 1)
d_plus, d_minus = 1.0, 1.0
right.annotate("", xy=p + d_plus * n, xytext=p, arrowprops=dict(arrowstyle="<->", color="#1a73e8", lw=1.6))
right.annotate("", xy=p - d_minus * n, xytext=p, arrowprops=dict(arrowstyle="<->", color="#e8710a", lw=1.6))
s = np.linspace(-1.1, 1.1, 50)
tangent = np.array([n[1], -n[0]])
right.plot(*(p[:, None] + tangent[:, None] * s), color=LIME, lw=3)
for sign, curv in ((1, .35), (-1, -.25)):
    base = p + sign * n * 1.0
    xs = np.linspace(-.9, .9, 40)
    wall = base[:, None] + tangent[:, None] * xs + sign * n[:, None] * curv * xs ** 2
    right.plot(*wall, color="black", lw=1.6)
right.plot(*p, "o", color="black", ms=6)
right.text(*(p + .55 * n + .08 * tangent), "$d_+$", color="#1a73e8", fontsize=13)
right.text(*(p - .55 * n + .08 * tangent), "$d_-$", color="#e8710a", fontsize=13)
right.text(*(p - .12 * n - .25 * tangent), "$p$", fontsize=13)
right.annotate("tangent $T$", xy=p + .9 * tangent, xytext=p + 1.1 * tangent - .35 * n, fontsize=10,
               arrowprops=dict(arrowstyle="->", lw=.8))
right.annotate("normal $N$", xy=p + .3 * n, xytext=p + .35 * n + .55 * tangent, fontsize=10,
               arrowprops=dict(arrowstyle="->", lw=.8))
right.text(-1.25, -1.55, "Bisecting means $d_+ = d_-$.\nThe chord width is $w = d_+ + d_-$.", fontsize=10)
right.set_xlim(-1.4, 1.4), right.set_ylim(-1.7, 1.4)
right.set_title("(b) Notation at one point of the line")
clean(right)
fig.savefig(os.path.join(OUT, "fig1_bisecting.png"), dpi=150)
plt.close(fig)

# ---------------------------------------------------------------- Figure 2: tracing works on convex fields, fails on others
wavy = Fourier(150, [30, 25, 18], [10, -15, 12])
fig, axes = plt.subplots(1, 2, figsize=(11, 5.2))
for ax, g, title in ((axes[0], convex, "(a) Convex field"),
                     (axes[1], wavy, "(b) Strongly non-convex field")):
    outline(ax, g)
    ok_count = 0
    starts = np.linspace(0, 2 * np.pi, 8, endpoint=False) + .013
    for k, start in enumerate(starts):
        u, v, ok = trace(g, start)
        ok_count += ok
        m = (g(u) + g(v)) / 2
        ax.plot(*m, color=LIME if ok else RED, lw=2 if ok else .8, alpha=1 if ok else .55, zorder=3 if ok else 2)
        ax.plot(*g(start), "o", color="black", ms=4, zorder=4)
    ax.set_title(f"{title}\n{ok_count} of {len(starts)} traces reached the far side")
    clean(ax)
fig.legend(handles=[Line2D([], [], color=LIME, lw=2, label="trace reached the far boundary (a valid centre line)"),
                    Line2D([], [], color=RED, lw=1, label="trace circles forever (no centre line from this start)"),
                    Line2D([], [], ls="", marker="o", color="black", ms=4, label="start point on the boundary")],
           loc="lower center", ncol=3, frameon=False, fontsize=9, bbox_to_anchor=(.5, -.02))
fig.subplots_adjust(bottom=.1)
fig.savefig(os.path.join(OUT, "fig2_tracing.png"), dpi=150)
plt.close(fig)

# ---------------------------------------------------------------- Figure 3: four sweep plans on one field
t = np.arange(160) / 160 * 2 * np.pi
rng = np.random.default_rng(1)
phases, amps = rng.random(7) * 2 * np.pi, (rng.random(7) * .6 + .4) / np.arange(2, 9) ** 1.3
radius = 160 + sum(95 * .8 * amps[k] * np.cos((k + 2) * t + phases[k]) for k in range(7))
polygon = np.c_[300 + radius * np.cos(t), 300 + radius * np.sin(t)]
mat = medial(polygon.tolist())
grid = Grid(polygon, 200)
caps = end_caps(polygon, np.asarray(mat["path"]))
start, end = caps[0][caps[3]], caps[0][caps[4]]
fields = [("Medial-axis chords", mat_field(grid, polygon, mat["path"])[0]),
          ("Straight sweep (lawnmower)", lawnmower_field(grid, polygon)),
          ("Harmonic", harmonic_field(grid, polygon, caps)),
          ("Geodesic (constant speed)", geodesic_field(grid, caps))]
fig, axes = plt.subplots(1, 4, figsize=(15, 4.6))
for ax, (title, field) in zip(axes, fields):
    shown = np.where(grid.mask & np.isfinite(field), field, np.nan)
    image = ax.imshow(shown, origin="upper", cmap="viridis", extent=(0, 600, 600, 0), vmin=0, vmax=1)
    ax.contour(grid.xy[..., 0], grid.xy[..., 1], np.where(np.isfinite(shown), shown, -1),
               levels=np.linspace(.08, .92, 8), colors="white", linewidths=1.1)
    ax.plot(*np.vstack((polygon, polygon[:1])).T, "k", lw=1.4)
    ax.plot(*start, "o", color=GREEN, ms=9, mec="white")
    ax.plot(*end, "s", color=RED, ms=8, mec="white")
    ax.set_title(title)
    ax.set_xlim(60, 540), ax.set_ylim(540, 60)
    clean(ax)
bar = fig.colorbar(image, ax=axes, fraction=.015, pad=.01)
bar.set_label("time at which the swarm passes (0 = start, 1 = end)")
fig.legend(handles=[Line2D([], [], color="#555", lw=1.5, label="white lines: position of the drone line at 8 moments"),
                    Line2D([], [], ls="", marker="o", color=GREEN, ms=8, label="start"),
                    Line2D([], [], ls="", marker="s", color=RED, ms=8, label="end (geodesic has no end target)")],
           loc="lower center", ncol=3, frameon=False, fontsize=9, bbox_to_anchor=(.45, -.04))
fig.savefig(os.path.join(OUT, "fig3_sweeps.png"), dpi=150)
plt.close(fig)

# ---------------------------------------------------------------- Figure 4: from a sweep to drone assignments
result = compare(polygon.tolist(), drones=8, hfov=70., vfov=55., overlap=.1)
method = result["methods"]["harmonic"]
lanes = np.asarray(method["lanes"]["values"]).reshape(method["lanes"]["height"], method["lanes"]["width"])
colours = plt.get_cmap("tab10")(np.arange(8))
rgba = np.zeros(lanes.shape + (4,))
rgba[lanes >= 0] = colours[lanes[lanes >= 0]] * [1, 1, 1, .45]
fig, (a, b) = plt.subplots(1, 2, figsize=(11, 5.2))
for ax in (a, b):
    ax.imshow(rgba, origin="upper", extent=(0, 600, 600, 0))
    ax.plot(*np.vstack((polygon, polygon[:1])).T, "k", lw=1.4)
    ax.set_xlim(60, 540), ax.set_ylim(540, 60)
    clean(ax)
for i, drone in enumerate(method["drones"]):
    path = np.asarray(drone["path"])
    a.plot(*path.T, color=colours[i], lw=2)
    a.plot(*path[0], "o", color=colours[i], ms=5, mec="black")
a.set_title("(a) Each colour is one drone's area;\nthe line of the same colour is its flight path")
tx, ty = np.tan(np.radians(35)), np.tan(np.radians(27.5))
for i, drone in enumerate(method["drones"]):
    path, heading, alt = np.asarray(drone["path"]), np.radians(drone["heading"]), np.asarray(drone["altitude"])
    for k in range(0, len(path), 9):
        hdir = np.array([np.cos(heading[k]), np.sin(heading[k])])
        ndir = np.array([-hdir[1], hdir[0]])
        corners = [path[k] + sa * alt[k] * ty * hdir + sc * alt[k] * tx * ndir for sa, sc in ((1, 1), (1, -1), (-1, -1), (-1, 1), (1, 1))]
        b.plot(*np.array(corners).T, color=colours[i], lw=.9)
        b.annotate("", xy=path[k] + 9 * hdir, xytext=path[k], arrowprops=dict(arrowstyle="-|>", color="black", lw=.8))
stats = method["stats"]
b.set_title(f"(b) Camera footprints (every 9th photo) and headings;\n"
            f"ground imaged: {stats['cameraCoverage']}%, highest altitude {stats['maxAltitude']}")
fig.savefig(os.path.join(OUT, "fig4_drones.png"), dpi=150)
plt.close(fig)
print("figures written to", OUT)
