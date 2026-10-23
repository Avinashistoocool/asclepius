"""Numerical checks for the derivations in coverage_theory.md, plus the figures.

Run from the repository root:  py paper/verify_derivations.py
"""

import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "swarm_lab"))


from medial_axis import analyze as medial  # noqa: E402
from sweep import Grid, end_caps, geodesic_field, harmonic_field, lawnmower_field, mat_field  # noqa: E402

RESULTS = []


def report(name, value, tolerance):
    ok = value <= tolerance
    RESULTS.append(ok)
    print(f"[{'PASS' if ok else 'FAIL'}] {name}: {value:.3e} (tolerance {tolerance:.0e})")


# ---------------------------------------------------------------- 1. ray-hit derivatives
A, B = 230., 130.


def ellipse_hit(p, n):
    """Positive distance from p along unit n to the ellipse x^2/A^2 + y^2/B^2 = 1."""
    a = (n[0] / A) ** 2 + (n[1] / B) ** 2
    b = 2 * (p[0] * n[0] / A ** 2 + p[1] * n[1] / B ** 2)
    c = (p[0] / A) ** 2 + (p[1] / B) ** 2 - 1
    return (-b + np.sqrt(b * b - 4 * a * c)) / (2 * a)


def ellipse_normal(x):
    g = np.array([x[0] / A ** 2, x[1] / B ** 2])
    return g / np.linalg.norm(g)


rng = np.random.default_rng(0)
worst_p = worst_n = 0.
h = 1e-6
for _ in range(200):
    p = rng.uniform(-.5, .5, 2) * [A, B]
    angle = rng.uniform(0, 2 * np.pi)
    n = np.array([np.cos(angle), np.sin(angle)])
    d = ellipse_hit(p, n)
    nu = ellipse_normal(p + d * n)
    grad_p = -nu / (nu @ n)
    fd = np.array([(ellipse_hit(p + h * e, n) - ellipse_hit(p - h * e, n)) / (2 * h) for e in np.eye(2)])
    worst_p = max(worst_p, np.max(np.abs(fd - grad_p)))
    t = np.array([-n[1], n[0]])
    turned = lambda s: np.array([np.cos(angle + s), np.sin(angle + s)])
    fd_n = (ellipse_hit(p, turned(h)) - ellipse_hit(p, turned(-h))) / (2 * h)
    worst_n = max(worst_n, abs(fd_n - (-d * (nu @ t) / (nu @ n))))
report("ray-hit derivative w.r.t. position, eq. (3.4)", worst_p, 1e-5)
report("ray-hit derivative w.r.t. direction, eq. (3.5)", worst_n, 1e-4)


# ---------------------------------------------------------------- 2. tangential Euler-Lagrange identity
def f(p, N):
    return 1 + .3 * np.sin(p[0] / 40) * np.cos(p[1] / 55) + .2 * (N[0] ** 2 - .5 * N[0] * N[1])


def grad(fun, x):
    return np.array([(fun(x + h * e) - fun(x - h * e)) / (2 * h) for e in np.eye(len(x))])


s = np.linspace(0, 1, 4001)
curve = np.c_[200 * s, 60 * np.sin(3 * s) + 20 * s ** 2]
arc = np.r_[0, np.cumsum(np.linalg.norm(np.diff(curve, axis=0), axis=1))]
d1 = np.gradient(curve, arc, axis=0)
T = d1 / np.linalg.norm(d1, axis=1)[:, None]
N = np.c_[-T[:, 1], T[:, 0]]
kappa = np.gradient(np.unwrap(np.arctan2(T[:, 1], T[:, 0])), arc)
fv = np.array([f(p, n) for p, n in zip(curve, N)])
gv = np.array([t @ grad(lambda m: f(p, m), n) for p, n, t in zip(curve, N, T)])
gp = np.array([t @ grad(lambda q: f(q, n), p) for p, n, t in zip(curve, N, T)])
report("tangential E-L identity, eq. (3.9)", float(np.max(np.abs(np.gradient(fv, arc) + kappa * gv - gp)[50:-50])), 1e-4)


# ---------------------------------------------------------------- 3. boundary-pair ODE
def gamma(u):
    return np.array([A * np.cos(u), B * np.sin(u)])


def gamma_d(u):
    return np.array([-A * np.sin(u), B * np.cos(u)])


def pair_field(_, y):
    u, v = y
    q = gamma(v) - gamma(u)
    w = np.array([q @ gamma_d(v), -(q @ gamma_d(u))])
    return w / np.linalg.norm(w)


def trace_pairs(start, eps=2e-3, h=.004, steps=4000):
    """Oriented RK4 on the line field (4.3): the sign of w is fixed by continuity with the previous step."""
    y, prev, out = np.array([start - eps, start + eps]), np.array([-1., 1.]), []
    orient = lambda y, ref: (lambda w: w if w @ ref >= 0 else -w)(pair_field(0, y))
    for _ in range(steps):
        out.append(y.copy())
        k1 = orient(y, prev); k2 = orient(y + h / 2 * k1, k1); k3 = orient(y + h / 2 * k2, k1); k4 = orient(y + h * k3, k1)
        step = (k1 + 2 * k2 + 2 * k3 + k4) / 6
        y, prev = y + h * step, step / np.linalg.norm(step)
        gap = y[1] - y[0]
        if len(out) > 10 and (gap < eps or gap > 2 * np.pi - eps):
            break
    return np.array(out).T


u, v = trace_pairs(.4)
u, v = u[5:-5], v[5:-5]
w = np.array([pair_field(0, y) for y in zip(u, v)]).T
velocity = .5 * (gamma_d(u) * w[0] + gamma_d(v) * w[1])
chord = gamma(v) - gamma(u)
cosine = np.abs(np.sum(velocity * chord, axis=0)) / np.linalg.norm(velocity, axis=0) / np.linalg.norm(chord, axis=0)
report("pair ODE keeps chord perpendicular to midpoint velocity, eq. (4.2)", float(np.max(cosine)), 1e-9)
dm = np.diff((gamma(u) + gamma(v)) / 2, axis=1)
mid_chord = (chord[:, :-1] + chord[:, 1:]) / 2
drift = np.abs(np.sum(dm * mid_chord, axis=0)) / np.linalg.norm(dm, axis=0) / np.linalg.norm(mid_chord, axis=0)
report("integrated midpoint path stays perpendicular (finite steps, cosine)", float(drift.max()), 1e-3)


# ---------------------------------------------------------------- 4. harmonic and eikonal fields
t = np.arange(160) / 160 * 2 * np.pi
blob_rng = np.random.default_rng(1)
phases, amps = blob_rng.random(7) * 2 * np.pi, (blob_rng.random(7) * .6 + .4) / np.arange(2, 9) ** 1.3
radius = 160 + sum(95 * .8 * amps[k] * np.cos((k + 2) * t + phases[k]) for k in range(7))
polygon = np.c_[300 + radius * np.cos(t), 300 + radius * np.sin(t)]
mat = medial(polygon.tolist())
grid = Grid(polygon, 200)
caps = end_caps(polygon, np.asarray(mat["path"]))
H = harmonic_field(grid, polygon, caps)
mask = grid.mask
pad = np.pad(mask, 1)
interior = mask & pad[:-2, 1:-1] & pad[2:, 1:-1] & pad[1:-1, :-2] & pad[1:-1, 2:]
lap = 4 * H[1:-1, 1:-1] - H[:-2, 1:-1] - H[2:, 1:-1] - H[1:-1, :-2] - H[1:-1, 2:]
report("discrete Laplace residual of the harmonic field, eq. (5.3)", float(np.nanmax(np.abs(lap[interior[1:-1, 1:-1]]))), 1e-9)

edges = []
for dr, dc in ((1, 0), (0, 1)):
    a = mask[:mask.shape[0] - dr, :mask.shape[1] - dc] & mask[dr:, dc:]
    edges.append((a, dr, dc))


def dirichlet(field):
    total = 0.
    for a, dr, dc in edges:
        diff = field[dr:, dc:] - field[:field.shape[0] - dr, :field.shape[1] - dc]
        total += float(np.sum(diff[a] ** 2))
    return total


bump = np.zeros_like(H)
rows, cols = np.nonzero(interior)
centre = np.array([rows.mean(), cols.mean()])
bump[interior] = np.exp(-np.sum((np.c_[rows, cols] - centre) ** 2, axis=1) / 200)
base = dirichlet(np.where(mask, H, 0))
change = min(dirichlet(np.where(mask, H + e * bump, 0)) - base for e in (-1e-2, 1e-2))
report("harmonic field minimises the discrete Dirichlet energy (negated increase)", max(-change, 0.), 1e-9)

G = geodesic_field(grid, caps)
gy, gx = np.gradient(np.where(mask, G, np.nan), grid.spacing)
slope = np.hypot(gx, gy)[interior]
slope = slope[np.isfinite(slope)]
report("eikonal |grad T| is constant, eq. (5.6): relative IQR", float(np.subtract(*np.percentile(slope, [75, 25])) / np.median(slope)), .1)

print(f"\n{sum(RESULTS)}/{len(RESULTS)} checks passed")


# ---------------------------------------------------------------- figures
import matplotlib  # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

out = os.path.join(HERE, "figures")
os.makedirs(out, exist_ok=True)
plt.rcParams.update({"font.size": 9, "axes.titlesize": 10})
fields = [("MAT + chord", mat_field(grid, polygon, mat["path"])[0]), ("Lawnmower (PCA)", lawnmower_field(grid, polygon)),
          ("Harmonic  $\\Delta T = 0$", H), ("Geodesic  $|\\nabla T| = c$", G)]
fig, axes = plt.subplots(1, 4, figsize=(13, 3.6))
for ax, (title, field) in zip(axes, fields):
    shown = np.where(mask & np.isfinite(field), field, np.nan)
    ax.imshow(shown, origin="upper", cmap="viridis", extent=(0, 600, 600, 0))
    ax.contour(grid.xy[..., 0], grid.xy[..., 1], np.where(np.isfinite(shown), shown, -1),
               levels=np.linspace(.05, .95, 13), colors="white", linewidths=.7)
    ax.plot(*np.vstack((polygon, polygon[:1])).T, "k", lw=1.2)
    ax.plot(*np.asarray(mat["path"]).T, "--", color="#10b981", lw=1)
    ax.set_title(title)
    ax.set_xticks([]), ax.set_yticks([])
    ax.set_xlim(60, 540), ax.set_ylim(540, 60)
fig.suptitle("Arrival-time fields $T(x)$ and fronts $\\{T = t\\}$ on one field; dashed: medial trunk", y=.99)
fig.tight_layout()
fig.savefig(os.path.join(out, "fig1_fields.png"), dpi=160)

fig, (a1, a2) = plt.subplots(1, 2, figsize=(10, 4.2))
theta = np.linspace(0, 2 * np.pi, 400)
a1.plot(A * np.cos(theta), B * np.sin(theta), "k", lw=1)
for start in np.linspace(.25, 2.9, 6):
    u, v = trace_pairs(start)
    a1.plot(*((gamma(u) + gamma(v)) / 2), lw=1.4)
    k = len(u) // 3
    a1.plot(*np.c_[gamma(u[k]), gamma(v[k])], ":", color="gray", lw=.8)
a1.set_aspect("equal"), a1.set_title("Bisector curves of eq. (4.3) on an ellipse")
a1.set_xticks([]), a1.set_yticks([])
uu, vv = np.meshgrid(np.linspace(0, 2 * np.pi, 29), np.linspace(0, 2 * np.pi, 29))
W = np.array([pair_field(0, (a, b)) if abs(a - b) > 1e-6 else (0, 0) for a, b in zip(uu.ravel(), vv.ravel())])
a2.quiver(uu, vv, W[:, 0].reshape(uu.shape), W[:, 1].reshape(uu.shape), angles="xy", pivot="mid", scale=40, width=.003)
a2.plot([0, 2 * np.pi], [0, 2 * np.pi], "r", lw=1, label="$u = v$ (degenerate chord)")
a2.set_xlabel("$u$"), a2.set_ylabel("$v$"), a2.set_title("Line field on the torus of boundary pairs")
a2.legend(loc="upper left", fontsize=8)
fig.tight_layout()
fig.savefig(os.path.join(out, "fig2_pair_ode.png"), dpi=160)
print(f"figures written to {out}")
sys.exit(0 if all(RESULTS) else 1)
