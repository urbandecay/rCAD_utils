# math_engine.py — Catmull-Rom spline math for vertex resampling.

import math

from mathutils import Vector


class CubicSegment:
    def __init__(self, p0, p1, p2, p3):
        t0 = 0.0
        t1 = self._get_t(t0, p0, p1)
        t2 = self._get_t(t1, p1, p2)
        t3 = self._get_t(t2, p2, p3)
        self.t_start = t1
        self.t_end = t2
        self.dt = t2 - t1

        if abs(self.dt) < 1e-6:
            self.d = p1
            self.is_degenerate = True
            self.times = (0, 1, 2, 3)
            return

        self.is_degenerate = False
        self.p0, self.p1, self.p2, self.p3 = p0, p1, p2, p3
        self.times = (t0, t1, t2, t3)

    def _get_t(self, t, p0, p1):
        dist_sq = (p1 - p0).length_squared
        return t + pow(dist_sq, 0.25)

    def eval(self, t_global):
        if self.is_degenerate:
            return self.d
        t0, t1, t2, t3 = self.times
        t = t_global

        if abs(t1 - t0) < 1e-6 or abs(t2 - t1) < 1e-6 or abs(t3 - t2) < 1e-6:
            return self.p1

        a1 = (t1 - t) / (t1 - t0) * self.p0 + (t - t0) / (t1 - t0) * self.p1
        a2 = (t2 - t) / (t2 - t1) * self.p1 + (t - t1) / (t2 - t1) * self.p2
        a3 = (t3 - t) / (t3 - t2) * self.p2 + (t - t2) / (t3 - t2) * self.p3

        b1 = (t2 - t) / (t2 - t0) * a1 + (t - t0) / (t2 - t0) * a2
        b2 = (t3 - t) / (t3 - t1) * a2 + (t - t1) / (t3 - t1) * a3

        return (t2 - t) / (t2 - t1) * b1 + (t - t1) / (t2 - t1) * b2


class CatmullRomSpline:
    def __init__(self, points, is_closed=False):
        self.segments = []
        self.is_closed = is_closed
        clean_pts = []
        if len(points) > 0:
            clean_pts.append(points[0])
            for p in points[1:]:
                if (p - clean_pts[-1]).length_squared > 1e-12:
                    clean_pts.append(p)

        if is_closed and len(clean_pts) > 1:
            if (clean_pts[0] - clean_pts[-1]).length_squared < 1e-12:
                clean_pts.pop()

        if len(clean_pts) < 2:
            return

        if not is_closed:
            start_ghost = clean_pts[0] + (clean_pts[0] - clean_pts[1])
            end_ghost = clean_pts[-1] + (clean_pts[-1] - clean_pts[-2])
            padded_pts = [start_ghost] + clean_pts + [end_ghost]
        else:
            padded_pts = [clean_pts[-1]] + clean_pts + clean_pts[:2]

        for i in range(len(padded_pts) - 3):
            seg = CubicSegment(
                padded_pts[i], padded_pts[i + 1],
                padded_pts[i + 2], padded_pts[i + 3],
            )
            self.segments.append(seg)

    def eval_global(self, t):
        if not self.segments:
            return Vector((0, 0, 0))
        num = len(self.segments)

        if self.is_closed:
            t = t % num
        else:
            t = max(0.0, min(float(num), t))

        p = math.floor(t)
        if abs(t - num) < 1e-5 and not self.is_closed:
            idx = num - 1
            local_t = self.segments[idx].t_end
            return self.segments[idx].eval(local_t)

        frac = t - p
        idx = int(p) % num
        seg = self.segments[idx]
        local_t = seg.t_start + (frac * seg.dt)
        return seg.eval(local_t)

    def find_closest_t(self, co, resolution=100):
        best_t = 0.0
        min_dist = float('inf')
        num_segs = len(self.segments)
        for i in range(resolution + 1):
            t = (i / resolution) * num_segs
            pos = self.eval_global(t)
            d = (pos - co).length_squared
            if d < min_dist:
                min_dist = d
                best_t = t
        return best_t
