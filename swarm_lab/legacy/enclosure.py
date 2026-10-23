"""Adaptive floating-point interval enclosures for smooth normal chords.

These are conservative numerical bounds with explicit roundoff margins, not
formal exact-arithmetic proofs.
"""

import time

import numpy as np
from scipy.interpolate import PPoly

from smooth import TAU


def point(value):
    value = np.asarray(value, dtype=float)
    return value, value


def add(a, b):
    return a[0] + b[0], a[1] + b[1]


def neg(a):
    return -a[1], -a[0]


def sub(a, b):
    return add(a, neg(b))


def mul(a, b):
    values = np.array([a[0] * b[0], a[0] * b[1], a[1] * b[0], a[1] * b[1]])
    return np.min(values, axis=0), np.max(values, axis=0)


def div(a, b):
    if np.any((b[0] <= 0) & (b[1] >= 0)):
        raise ArithmeticError("An interval divisor contains zero.")
    return mul(a, (1 / b[1], 1 / b[0]))


def square(a):
    low = np.where((a[0] <= 0) & (a[1] >= 0), 0, np.minimum(a[0] ** 2, a[1] ** 2))
    return low, np.maximum(a[0] ** 2, a[1] ** 2)


def dot(a, b):
    product = mul(a, b)
    return np.sum(product[0]), np.sum(product[1])


def norm(a):
    squared = square(a)
    return np.sqrt(np.sum(squared[0])), np.sqrt(np.sum(squared[1]))


def rotate(a):
    return np.array([-a[1][1], a[0][0]]), np.array([-a[0][1], a[1][0]])


def maximum_abs(a):
    return float(np.max(np.maximum(np.abs(a[0]), np.abs(a[1]))))


class PairEnclosure:
    def __init__(self, boundary, splines):
        self.boundary = boundary
        self.splines = splines
        self.polynomials = [[PPoly.from_spline(spline).derivative(order) for order in range(4)]
                            for spline in splines]

    def scalar_bounds(self, which, a, b, derivative):
        polynomial = self.polynomials[which][derivative]
        values = []
        for i, (left, right) in enumerate(zip(polynomial.x[:-1], polynomial.x[1:])):
            if right <= left or right < a or left > b:
                continue
            lo, hi = max(a, left) - left, min(b, right) - left
            coefficients = polynomial.c[:, i]
            roots = np.roots(np.polyder(coefficients)) if len(coefficients) > 1 else []
            sites = [lo, hi] + [root.real for root in roots if abs(root.imag) < 1e-10 and lo < root.real < hi]
            values.extend(np.polyval(coefficients, sites))
        margin = 1e-12 * (1 + max(abs(v) for v in values))
        return min(values) - margin, max(values) + margin

    def bounds(self, lo, hi):
        c, d = self.scalar_bounds(0, lo, hi, 0), self.scalar_bounds(1, lo, hi, 0)
        ct, dt = self.scalar_bounds(0, lo, hi, 1), self.scalar_bounds(1, lo, hi, 1)
        ctt, dtt = self.scalar_bounds(0, lo, hi, 2), self.scalar_bounds(1, lo, hi, 2)
        cttt, dttt = self.scalar_bounds(0, lo, hi, 3), self.scalar_bounds(1, lo, hi, 3)
        u, v = sub(c, d), add(c, d)
        ut, vt = sub(ct, dt), add(ct, dt)
        utt, vtt = sub(ctt, dtt), add(ctt, dtt)
        a, b = self.boundary.bounds(*u), self.boundary.bounds(*v)
        av, bv = self.boundary.bounds(*u, 1), self.boundary.bounds(*v, 1)
        aa, ba = self.boundary.bounds(*u, 2), self.boundary.bounds(*v, 2)
        aj, bj = self.boundary.bounds(*u, 3), self.boundary.bounds(*v, 3)
        position = mul(add(a, b), point(.5))
        velocity = mul(add(mul(av, ut), mul(bv, vt)), point(.5))
        acceleration = mul(add(add(mul(aa, square(ut)), mul(av, utt)),
                               add(mul(ba, square(vt)), mul(bv, vtt))), point(.5))
        jerk = mul(add(
            add(add(mul(aj, mul(square(ut), ut)), mul(mul(aa, mul(ut, utt)), point(3.))),
                mul(av, sub(cttt, dttt))),
            add(add(mul(bj, mul(square(vt), vt)), mul(mul(ba, mul(vt, vtt)), point(3.))),
                mul(bv, add(cttt, dttt)))), point(.5))
        midpoint, vmid, amid = self.evaluate((lo + hi) / 2)
        delta = (-(hi - lo) / 2, (hi - lo) / 2)

        def intersect(original, centered):
            low, high = np.maximum(original[0], centered[0]), np.minimum(original[1], centered[1])
            return low - 1e-10, high + 1e-10

        acceleration = intersect(acceleration, add(point(amid), mul(jerk, delta)))
        velocity = intersect(velocity, add(point(vmid), mul(acceleration, delta)))
        position = intersect(position, add(point(midpoint), mul(velocity, delta)))
        return position, velocity, acceleration

    def evaluate(self, t):
        c, d = [s(t) for s in self.splines]
        ct, dt = [s(t, 1) for s in self.splines]
        ctt, dtt = [s(t, 2) for s in self.splines]
        a, b = self.boundary.evaluate(c - d), self.boundary.evaluate(c + d)
        av, bv = self.boundary.evaluate(c - d, 1), self.boundary.evaluate(c + d, 1)
        aa, ba = self.boundary.evaluate(c - d, 2), self.boundary.evaluate(c + d, 2)
        return ((a + b) / 2, (av * (ct - dt) + bv * (ct + dt)) / 2,
                (aa * (ct - dt) ** 2 + av * (ctt - dtt) +
                 ba * (ct + dt) ** 2 + bv * (ctt + dtt)) / 2)

    def endpoint_cap(self, end, width, tolerance):
        lo, hi = (0., width) if end == 0 else (1 - width, 1.)
        p0, _, _ = self.evaluate(float(end))
        _, _, acceleration = self.bounds(lo, hi)
        direction = acceleration
        speed = norm(direction)
        if speed[0] <= 1e-8:
            return False
        tangent = div(direction, speed)
        # Near t=1 velocity points opposite to the endpoint acceleration.
        if end == 1:
            tangent = neg(tangent)
        phase = self.splines[0](end) + self.splines[1](end)
        boundary_tangent = self.boundary.evaluate(phase, 1)
        boundary_tangent /= np.linalg.norm(boundary_tangent)
        inward = np.array([-boundary_tangent[1], boundary_tangent[0]])
        inward_acceleration = dot(acceleration, point(inward))[0]
        if inward_acceleration <= 0:
            return False
        arc_halfwidth = tolerance * .65 / np.linalg.norm(self.boundary.evaluate(phase, 1))
        arc = self.boundary.bounds(phase - arc_halfwidth, phase + arc_halfwidth)
        radius = norm(sub(arc, point(p0)))[1]
        displacement = norm(acceleration)[1] * width ** 2 / 2
        if radius + displacement > tolerance:
            return False
        # Curvature and a local graph bound establish interior entry into a
        # strictly convex boundary patch, not merely closeness to the boundary.
        av = self.boundary.bounds(phase - arc_halfwidth, phase + arc_halfwidth, 1)
        aa = self.boundary.bounds(phase - arc_halfwidth, phase + arc_halfwidth, 2)
        determinant = sub(mul((av[0][0], av[1][0]), (aa[0][1], aa[1][1])),
                          mul((av[0][1], av[1][1]), (aa[0][0], aa[1][0])))
        if determinant[0] <= 0 or dot(av, point(boundary_tangent))[0] <= 0:
            return False
        graph_curvature = 4 * norm(aa)[1] / norm(av)[0] ** 2
        if inward_acceleration <= graph_curvature * norm(acceleration)[1] ** 2 * width ** 2:
            return False
        displacement_box = mul(acceleration, (0., width ** 2 / 2))
        pbox = add(point(p0), displacement_box)
        signs = []
        for s in (phase - arc_halfwidth, phase + arc_halfwidth):
            signs.append(dot(sub(point(self.boundary.evaluate(s)), pbox), tangent))
        # Both local arc ends must be beyond the normal line; then the two
        # first hits are inside the small arc ball, bounding midpoint error.
        return ((signs[0][0] > 0 and signs[1][0] > 0) or
                (signs[0][1] < 0 and signs[1][1] < 0))

    def root_free(self, a, b, position, tangent, depth=0):
        projection = dot(sub(self.boundary.bounds(a, b), position), tangent)
        if projection[0] > 0 or projection[1] < 0:
            return True
        slope = dot(self.boundary.bounds(a, b, 1), tangent)
        if slope[0] > 0 or slope[1] < 0:
            left = dot(sub(point(self.boundary.evaluate(a)), position), tangent)
            right = dot(sub(point(self.boundary.evaluate(b)), position), tangent)
            if min(left[0], right[0]) > 0 or max(left[1], right[1]) < 0:
                return True
        if depth >= 14:
            return False
        mid = (a + b) / 2
        return self.root_free(a, mid, position, tangent, depth + 1) and self.root_free(mid, b, position, tangent, depth + 1)

    def interval_error(self, lo, hi):
        self.failure = "unknown interval obstruction"
        mid = (lo + hi) / 2
        p, v, acceleration0 = self.evaluate(mid)
        speed0 = np.linalg.norm(v)
        if speed0 < 1e-8:
            self.failure = "stationary midpoint"
            return None
        t0 = v / speed0
        n0 = np.array([-t0[1], t0[0]])
        nt0 = -(acceleration0 @ n0) / speed0
        sites, hits = self.boundary.line_hits(p, t0)
        if len(sites) < 2:
            self.failure = "missing line roots"
            return None
        radii = (hits - p) @ n0
        below, above = np.flatnonzero(radii < 0), np.flatnonzero(radii > 0)
        if not len(below) or not len(above) or len(below) % 2 != 1:
            self.failure = "midpoint winding or first-hit signs"
            return None
        negative = below[np.argmax(radii[below])]
        positive = above[np.argmin(radii[above])]
        position, velocity, acceleration = self.bounds(lo, hi)
        speed = norm(velocity)
        if speed[0] <= 1e-9:
            self.failure = "speed enclosure contains zero"
            return None
        tangent = div(velocity, speed)
        tangent_rate = sub(div(acceleration, speed),
                           div(mul(velocity, dot(velocity, acceleration)), mul(square(speed), speed)))
        centered_tangent = add(point(t0), mul(tangent_rate, (-(hi - lo) / 2, (hi - lo) / 2)))
        tangent = (np.maximum(tangent[0], centered_tangent[0]) - 1e-12,
                   np.minimum(tangent[1], centered_tangent[1]) + 1e-12)
        normal = rotate(tangent)
        normal_rate = rotate(tangent_rate)
        nt = dot(normal_rate, tangent)
        brackets = []
        radius_bounds = []
        for i, site in enumerate(sites):
            derivative = self.boundary.evaluate(site, 1)
            denominator = derivative @ t0
            if abs(denominator) < 1e-8:
                self.failure = "tangent boundary contact"
                return None
            motion = abs((speed0 + radii[i] * nt0) / denominator)
            gap = min((site - sites[i - 1]) % TAU, (sites[(i + 1) % len(sites)] - site) % TAU)
            uncertainty = maximum_abs(dot(sub(point(self.boundary.evaluate(site)), position), tangent))
            width = max(1e-7, motion * (hi - lo) * 1.2, 3 * uncertainty / abs(denominator))
            accepted = False
            for _ in range(16):
                if width >= gap * .45:
                    break
                left, right = site - width, site + width
                fl = dot(sub(point(self.boundary.evaluate(left)), position), tangent)
                fr = dot(sub(point(self.boundary.evaluate(right)), position), tangent)
                slope = dot(self.boundary.bounds(left, right, 1), tangent)
                if ((fl[1] < 0 and fr[0] > 0 and slope[0] > 0) or
                        (fl[0] > 0 and fr[1] < 0 and slope[1] < 0)):
                    accepted = True
                    break
                width *= 1.7
            if not accepted:
                self.failure = "root bracket does not enclose all trajectories"
                return None
            brackets.append((left, right))
            radius_bounds.append(dot(sub(self.boundary.bounds(left, right), position), normal))
        if not self.boundary.certified_convex:
            for i, (_, right) in enumerate(brackets):
                left = brackets[(i + 1) % len(brackets)][0] + (TAU if i == len(brackets) - 1 else 0)
                if not self.root_free(right, left, position, tangent):
                    self.failure = "complement may contain additional roots"
                    return None
        if radius_bounds[negative][1] >= 0 or radius_bounds[positive][0] <= 0:
            self.failure = "radius enclosure crosses zero"
            return None
        for i, radius in enumerate(radius_bounds):
            if i not in (negative, positive):
                if radii[i] < 0 and radius[1] >= radius_bounds[negative][0]:
                    self.failure = "negative first-hit order uncertain"
                    return None
                if radii[i] > 0 and radius[0] <= radius_bounds[positive][1]:
                    self.failure = "positive first-hit order uncertain"
                    return None
        rates = []
        for i in (negative, positive):
            boundary_velocity = self.boundary.bounds(*brackets[i], 1)
            ratio = div(dot(boundary_velocity, normal), dot(boundary_velocity, tangent))
            rates.append(mul(ratio, add(speed, mul(radius_bounds[i], nt))))
        midpoint_rate = mul(add(rates[0], rates[1]), point(.5))
        center_error = abs((radii[negative] + radii[positive]) / 2)
        return center_error + maximum_abs(midpoint_rate) * (hi - lo) / 2 + 1e-7


def verify_intervals(boundary, splines, tolerance, seconds=30):
    enclosure = PairEnclosure(boundary, splines)
    deadline = time.perf_counter() + seconds
    cap = next((width for width in (1e-3, 3e-4, 1e-4, 3e-5, 1e-5)
                if all(enclosure.endpoint_cap(end, width, tolerance) for end in (0, 1))), None)
    if cap is None:
        return {"verified": False, "reason": "Endpoint interval caps could not be bounded at this tolerance."}
    breaks = np.unique(np.r_[cap, splines[0].t[(splines[0].t > cap) & (splines[0].t < 1 - cap)], 1 - cap])
    stack = [(a, b, 0) for a, b in zip(breaks[:-1], breaks[1:])]
    count, maximum = 2, 0.
    while stack:
        if time.perf_counter() > deadline:
            return {"verified": False, "reason": "Interval verification reached its time budget.", "intervals": count}
        lo, hi, depth = stack.pop()
        try:
            bound = enclosure.interval_error(lo, hi)
        except ArithmeticError:
            bound = None
        if bound is not None and bound <= tolerance:
            count += 1
            maximum = max(maximum, bound)
            continue
        if depth >= 17 or hi - lo < 1e-9:
            return {"verified": False, "reason": f"Interval [{lo:.9f}, {hi:.9f}]: {enclosure.failure}; bound={bound}.", "intervals": count}
        mid = (lo + hi) / 2
        stack.extend(((lo, mid, depth + 1), (mid, hi, depth + 1)))
    return {"verified": True, "intervals": count, "bound": max(maximum, tolerance),
            "method": "Adaptive interval root isolation, implicit radius-derivative bounds, and convex endpoint caps; floating-point margins, not formal exact arithmetic."}
