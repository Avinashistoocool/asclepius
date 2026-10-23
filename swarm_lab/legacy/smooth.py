"""Smooth boundary functions; polygon samples are never used as their definition."""

import math

import numpy as np
from scipy.interpolate import CubicSpline

from boundary import cross


TAU = 2 * np.pi


def sin_bounds(a, b):
    if b - a >= TAU:
        return -1., 1.
    low, high = min(math.sin(a), math.sin(b)), max(math.sin(a), math.sin(b))
    if math.ceil((a - np.pi / 2) / TAU) <= math.floor((b - np.pi / 2) / TAU):
        high = 1.
    if math.ceil((a + np.pi / 2) / TAU) <= math.floor((b + np.pi / 2) / TAU):
        low = -1.
    return low, high


class SmoothBoundary:
    def __init__(self, points, model=None):
        self.model = model
        self.fourier = model is not None and model.get("kind") == "fourier"
        if self.fourier:
            radius = model.get("radius")
            ac, bs = np.asarray(model.get("cos"), dtype=float), np.asarray(model.get("sin"), dtype=float)
            if (not isinstance(radius, (float, int)) or not math.isfinite(radius)
                    or ac.ndim != 1 or bs.shape != ac.shape or not 2 <= len(ac) <= 13
                    or not np.all(np.isfinite(ac)) or not np.all(np.isfinite(bs))
                    or abs(ac[0]) > 1e-12 or abs(bs[0]) > 1e-12
                    or radius - np.sum(np.hypot(ac, bs)) <= 10):
                raise ValueError("Fourier radii must be finite, positive, and have at most 12 harmonics.")
            self.cos = np.zeros((len(ac) + 1, 2))
            self.sin = np.zeros_like(self.cos)
            self.cos[0] = [300., 300.]
            self.cos[1, 0] = self.sin[1, 1] = radius
            for k in range(1, len(ac)):
                self.cos[k + 1, 0] += ac[k] / 2
                self.cos[k - 1, 0] += ac[k] / 2
                self.sin[k + 1, 0] += bs[k] / 2
                self.sin[k - 1, 0] += bs[k] / 2
                self.sin[k + 1, 1] += ac[k] / 2
                self.sin[k - 1, 1] -= ac[k] / 2
                self.cos[k - 1, 1] += bs[k] / 2
                self.cos[k + 1, 1] -= bs[k] / 2
            expected = self.evaluate(np.arange(len(points)) * TAU / len(points))
            if not np.allclose(expected, points, atol=1e-7, rtol=0):
                raise ValueError("Fourier coefficients do not match the supplied editing handles.")
            self.center = self.cos[0].copy()
            self.radius = float(radius)
            self.offset = 0.
        else:
            lengths = np.linalg.norm(np.roll(points, -1, axis=0) - points, axis=1)
            self.knots = np.r_[0., np.cumsum(lengths)] / np.sum(lengths) * TAU
            self.spline = CubicSpline(self.knots, np.vstack((points, points[0])), bc_type="periodic")
            self.center = np.mean(points, axis=0)
            self.radius = float(np.mean(np.linalg.norm(points - self.center, axis=1)))
            self.offset = float(np.arctan2(*(points[0] - self.center)[::-1]))
        sample = self.evaluate(np.arange(1024) * TAU / 1024)
        if np.any(sample < 2) or np.any(sample > 598):
            raise ValueError("The smooth boundary overshoots the canvas margin. Move its handles inward.")
        if np.min(np.linalg.norm(self.evaluate(np.arange(2048) * TAU / 2048, 1), axis=1)) < 1e-3:
            raise ValueError("The interpolated boundary has a singular tangent. Space its handles more evenly.")
        self._check_simple(sample)
        self.certified_convex = self._convexity()

    def _convexity(self):
        sites = np.arange(256) * TAU / 256
        if np.any(cross(self.evaluate(sites, 1), self.evaluate(sites, 2)) <= 0):
            return False
        stack = [(i * TAU / 32, (i + 1) * TAU / 32, 0) for i in range(32)]
        while stack:
            a, b, depth = stack.pop()
            vl, vh = self.bounds(a, b, 1)
            al, ah = self.bounds(a, b, 2)
            first = min(vl[0] * al[1], vl[0] * ah[1], vh[0] * al[1], vh[0] * ah[1])
            second = max(vl[1] * al[0], vl[1] * ah[0], vh[1] * al[0], vh[1] * ah[0])
            if first - second > 0:
                continue
            if depth >= 8:
                return False
            middle = (a + b) / 2
            stack.extend(((a, middle, depth + 1), (middle, b, depth + 1)))
        return True

    @staticmethod
    def _check_simple(points):
        ends = np.roll(points, -1, axis=0)
        for i, (a, b) in enumerate(zip(points, ends)):
            indices = np.arange(i + 2, len(points))
            if i == 0:
                indices = indices[indices != len(points) - 1]
            c, d = points[indices], ends[indices]
            overlap = np.all(np.maximum(np.minimum(a, b), np.minimum(c, d)) <=
                             np.minimum(np.maximum(a, b), np.maximum(c, d)), axis=1)
            c, d = c[overlap], d[overlap]
            if np.any((cross(b - a, c - a) * cross(b - a, d - a) < 0) &
                      (cross(d - c, a - c) * cross(d - c, b - c) < 0)):
                raise ValueError("The interpolated smooth boundary crosses itself. Adjust its handles.")

    def evaluate(self, parameter, derivative=0, fraction=1.):
        parameter = np.asarray(parameter)
        if self.fourier:
            k = np.arange(len(self.cos))
            power = k ** derivative
            angle = parameter[..., None] * k + derivative * np.pi / 2
            target = (np.cos(angle) * power) @ self.cos + (np.sin(angle) * power) @ self.sin
        else:
            target = self.spline(np.mod(parameter, TAU), derivative)
        if fraction == 1:
            return target
        phase = parameter + self.offset + derivative * np.pi / 2
        base = self.radius * np.stack((np.cos(phase), np.sin(phase)), axis=-1)
        if derivative == 0:
            base += self.center
        return base + fraction * (target - base)

    def bounds(self, a, b, derivative=0):
        if a > b:
            a, b = b, a
        if self.fourier:
            k = np.arange(len(self.cos))

            def ranges(shift):
                left, right = k * a + shift, k * b + shift
                sl, sr = np.sin(left), np.sin(right)
                lower, upper = np.minimum(sl, sr), np.maximum(sl, sr)
                upper = np.where(np.ceil((left - np.pi / 2) / TAU) <= np.floor((right - np.pi / 2) / TAU), 1., upper)
                lower = np.where(np.ceil((left + np.pi / 2) / TAU) <= np.floor((right + np.pi / 2) / TAU), -1., lower)
                return lower[:, None], upper[:, None]

            sl, sh = ranges(derivative * np.pi / 2)
            cl, ch = ranges((derivative + 1) * np.pi / 2)
            ac, bs = self.cos * (k ** derivative)[:, None], self.sin * (k ** derivative)[:, None]
            low = np.sum(np.minimum(ac * cl, ac * ch) + np.minimum(bs * sl, bs * sh), axis=0)
            high = np.sum(np.maximum(ac * cl, ac * ch) + np.maximum(bs * sl, bs * sh), axis=0)
        else:
            low, high = np.full(2, np.inf), np.full(2, -np.inf)
            cursor = a
            while cursor < b + 1e-13:
                wrapped = cursor % TAU
                index = min(np.searchsorted(self.knots, wrapped, side="right") - 1, len(self.knots) - 2)
                end = min(b, cursor + self.knots[index + 1] - wrapped)
                coefficients = self.spline.c[:, index].copy()
                for _ in range(derivative):
                    coefficients = coefficients[:-1] * np.arange(len(coefficients) - 1, 0, -1)[:, None]
                left, right = wrapped - self.knots[index], wrapped - self.knots[index] + end - cursor
                for coordinate in range(2):
                    polynomial = coefficients[:, coordinate]
                    roots = np.roots(np.polyder(polynomial)) if len(polynomial) > 1 else []
                    sites = [left, right] + [float(root.real) for root in roots
                                           if abs(root.imag) < 1e-10 and left < root.real < right]
                    values = np.polyval(polynomial, sites)
                    low[coordinate] = min(low[coordinate], min(values))
                    high[coordinate] = max(high[coordinate], max(values))
                if end >= b - 1e-13:
                    break
                cursor = end + 1e-12
        margin = 1e-12 * (1 + np.maximum(np.abs(low), np.abs(high)))
        return low - margin, high + margin

    def line_hits(self, point, tangent):
        if self.fourier:
            ac, bs = self.cos @ tangent, self.sin @ tangent
            ac = ac.copy()
            ac[0] -= point @ tangent
            degree = len(ac) - 1
            coefficients = np.zeros(2 * degree + 1, dtype=complex)
            coefficients[degree] = ac[0]
            for k in range(1, degree + 1):
                coefficients[degree + k] = (ac[k] - 1j * bs[k]) / 2
                coefficients[degree - k] = (ac[k] + 1j * bs[k]) / 2
            roots = np.polynomial.polynomial.polyroots(np.trim_zeros(coefficients, "b"))
            parameters = np.mod(np.angle(roots[np.abs(np.abs(roots) - 1) < 2e-4]), TAU)
        else:
            parameters = []
            for index in range(len(self.knots) - 1):
                polynomial = self.spline.c[:, index] @ tangent
                polynomial = polynomial.copy()
                polynomial[-1] -= point @ tangent
                width = self.knots[index + 1] - self.knots[index]
                for root in np.roots(polynomial):
                    if abs(root.imag) < 1e-7 and -1e-10 <= root.real <= width + 1e-10:
                        parameters.append(self.knots[index] + min(width, max(0., root.real)))
            parameters = np.asarray(parameters)
        if not len(parameters):
            return np.array([]), np.empty((0, 2))
        for _ in range(3):
            residual = (self.evaluate(parameters) - point) @ tangent
            slope = self.evaluate(parameters, 1) @ tangent
            good = np.abs(slope) > 1e-8
            parameters[good] -= residual[good] / slope[good]
        parameters = np.unique(np.round(np.mod(parameters, TAU), 11))
        hits = self.evaluate(parameters)
        valid = np.abs((hits - point) @ tangent) < 1e-5
        return parameters[valid], hits[valid]

    def export(self):
        if self.fourier:
            return {"type": "fourier", "cos": self.cos.tolist(), "sin": self.sin.tolist()}
        return {"type": "periodic-cubic", "knots": self.knots.tolist(), "coefficients": self.spline.c.tolist()}
