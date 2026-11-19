"""Conservative local affine fit for spindle-off work, no extrapolation."""

import itertools
import json
import math

import cv2
import numpy as np

from .client import RobotError


def points(rows):
    array = np.asarray(rows, dtype=float)
    if array.ndim != 2 or array.shape[1] != 4 or not np.isfinite(array).all():
        raise ValueError("samples must be [j1_steps,j2_steps,x_mm,y_mm] rows")
    if not np.all(array[:, :2] == np.rint(array[:, :2])):
        raise ValueError("sample steps must be integers")
    return array


class Planner:
    def __init__(self, fit, checks):
        try:
            fit, checks = points(fit), points(checks)
            if len(fit) < 6 or len(checks) < 3:
                raise ValueError("need at least six fit samples and three independent check samples")
            if (len(set(map(tuple, fit[:, :2]))) != len(fit) or
                    len(set(map(tuple, checks[:, :2]))) != len(checks) or
                    set(map(tuple, fit[:, :2])) & set(map(tuple, checks[:, :2]))):
                raise ValueError("calibration sample step pairs must be distinct")
            self.hull = cv2.convexHull(fit[:, :2].astype(np.float32))
            if cv2.contourArea(self.hull) <= 0:
                raise ValueError("sample joint positions must span a two-dimensional region")
            for pair in checks[:, :2]:
                self.contains(pair)
            self.center = fit[:, :2].mean(axis=0)
            self.scale = np.ptp(fit[:, :2], axis=0)
            design = np.c_[(fit[:, :2]-self.center)/self.scale, np.ones(len(fit))]
            model, _, rank, s = np.linalg.lstsq(design, fit[:, 2:], rcond=None)
            if rank != 3 or s[0]/s[-1] > 20 or np.linalg.cond(model[:2]) > 25:
                raise ValueError("calibration is poorly conditioned; use better-distributed samples")
            self.linear, self.offset = model[:2], model[2]
            self.inverse = np.linalg.inv(self.linear)
            self.residual_mm = float(np.linalg.norm(
                self.forward(checks[:, :2])-checks[:, 2:], axis=1).max())
        except (TypeError, ValueError, cv2.error, np.linalg.LinAlgError) as exc:
            raise RobotError(f"Invalid calibration: {exc}") from exc

    @classmethod
    def load(cls, path, simulated):
        if simulated and path is None:
            fit = [[j1, j2, j1/10, j2/10] for j1 in (0, 1200, 2350) for j2 in (0, 1270)]
            checks = [[j1, j2, j1/10, j2/10] for j1, j2 in ((300, 200), (1000, 900), (2100, 650))]
            return cls(fit, checks)
        if path is None:
            return None
        with open(path) as source:
            config = json.load(source)
        if config.get("version") != 1 or config.get("frame") != "vision_bed":
            raise RobotError("Calibration version/frame does not match this fixture.")
        return cls(config.get("fit_samples"), config.get("check_samples"))

    def forward(self, steps):
        return ((steps-self.center)/self.scale) @ self.linear + self.offset

    def contains(self, pair):
        if cv2.pointPolygonTest(self.hull, tuple(float(n) for n in pair), False) < 0:
            raise RobotError("Point or check sample is outside the calibrated joint region.")

    def plan(self, rows, budget_mm):
        output = []
        for row in rows:
            xy = np.array([float(row[k]) for k in ("x_mm", "y_mm")])
            if not np.isfinite(xy).all():
                raise RobotError("Non-finite target position.")
            estimate = ((xy-self.offset) @ self.inverse)*self.scale+self.center
            self.contains(estimate)
            candidates = []
            for pair in itertools.product(*((math.floor(n), math.ceil(n)) for n in estimate)):
                try:
                    self.contains(pair)
                except RobotError:
                    continue
                achieved = self.forward(np.array(pair))
                candidates.append((float(np.linalg.norm(achieved-xy)), pair, achieved))
            if not candidates:
                raise RobotError(f"{row['id']}: no feasible integer step pair.")
            error, (j1, j2), achieved = min(candidates, key=lambda item: item[0])
            if self.residual_mm + error > budget_mm:
                raise RobotError(f"{row['id']}: calibration check error plus step rounding exceeds "
                                 f"the {budget_mm:g} mm budget.")
            output.append({"id": row["id"], "requested_mm": xy.tolist(),
                           "achievable_mm": achieved.tolist(), "rounding_mm": error,
                           "steps": {"j1": j1, "j2": j2}, "state": "pending"})
        return output
