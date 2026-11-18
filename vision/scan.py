"""Capture, register, measure, and report. This module does not control the robot."""

import csv
import json
import math
from datetime import datetime, timezone
from itertools import product
from pathlib import Path

import cv2
import numpy as np

from . import fixture as f

PX_PER_MM = 8  # Output sampling only; does not increase the camera's resolution.
PLACEMENT_TOLERANCE_MM = 0.5


class ScanError(Exception):
    pass


def capture(camera):
    device = int(camera) if camera.isdecimal() else camera
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    try:
        if not cap.isOpened():
            raise ScanError(f"Cannot open camera {camera}; check the device, permissions and other camera apps.")
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        for _ in range(12):
            ok, frame = cap.read()
            if not ok or frame is None:
                raise ScanError(f"Camera {camera} opened but did not return a frame.")
        return frame
    finally:
        cap.release()


def project(points, matrix):
    points = np.asarray(points, dtype=np.float64).reshape(-1, 1, 2)
    return cv2.perspectiveTransform(points, matrix).reshape(-1, 2)


def bed_to_raster():
    s = PX_PER_MM
    return np.array(((s, 0., 0.), (0., -s, f.PAGE[1]*s), (0., 0., 1.)))


def register(frame):
    aruco = cv2.aruco
    params = (aruco.DetectorParameters_create() if hasattr(aruco, "DetectorParameters_create")
              else aruco.DetectorParameters())
    params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    if hasattr(aruco, "ArucoDetector"):
        corners, ids, _ = aruco.ArucoDetector(f.dictionary(), params).detectMarkers(gray)
    else:
        corners, ids, _ = aruco.detectMarkers(gray, f.dictionary(), parameters=params)
    seen = [] if ids is None else [int(i) for i in ids.ravel() if i in f.TAGS]
    if len(seen) != len(set(seen)):
        raise ScanError("Duplicate fixture IDs in frame. Keep a single fixture in view.")
    if not {0, 1, 2, 3}.issubset(seen):
        raise ScanError(f"Required corner tags 0–3 are not all visible (found {sorted(seen)}). "
                        "Check framing, focus, lighting and that the image is not digitally mirrored.")
    px = np.vstack([c.reshape(4, 2) for c, i in zip(corners, ids.ravel()) if i in f.TAGS])
    world = np.vstack([f.tag_corners(i) for i in seen])
    h, inliers = cv2.findHomography(px, world, cv2.RANSAC, 0.45)
    if h is None or inliers is None:
        raise ScanError("Could not fit a planar registration.")
    mask = inliers.ravel().astype(bool)
    if np.mean(mask) < .8 or any(np.count_nonzero(row) < 3 for i, row in
                                 zip(seen, mask.reshape(-1, 4)) if i < 4):
        raise ScanError("Tag geometry is inconsistent. Flatten the sheet and check for lens distortion.")
    inv = np.linalg.inv(h)
    error = project(world[mask], inv) - px[mask]
    rms = float(np.sqrt(np.mean(np.sum(error**2, axis=1))))
    if rms > 1.0:
        raise ScanError(f"Tag reprojection RMS {rms:.2f} px exceeds 1.0 px.")
    # Estimate worst native resolution in the board ROI using the local differential.
    bx, by = f.BOARD_ORIGIN
    samples = []
    for x, y in product((bx, bx+25, bx+50), (by, by+25, by+50)):
        p, dx, dy = project(((x, y), (x+1, y), (x, y+1)), inv)
        samples.append(np.linalg.svd(np.column_stack((dx-p, dy-p)), compute_uv=False)[-1])
    return h, {"seen_ids": sorted(seen), "inlier_corners": int(mask.sum()),
               "total_corners": len(mask), "reprojection_rms_px": rms,
               "min_native_px_per_mm": float(min(samples)), "image_to_bed_mm": h.tolist()}


def blobs(gray, diameter_min, diameter_max, polarity):
    kind = cv2.THRESH_BINARY_INV if polarity == "dark" else cv2.THRESH_BINARY
    _, binary = cv2.threshold(gray, 0, 255, kind | cv2.THRESH_OTSU)
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    results = []
    for c in contours:
        x, y, width, height = cv2.boundingRect(c)
        if x == 0 or y == 0 or x+width >= gray.shape[1] or y+height >= gray.shape[0]:
            continue
        area, perimeter = cv2.contourArea(c), cv2.arcLength(c, True)
        if not area or not perimeter:
            continue
        circularity = 4*math.pi*area/perimeter**2
        diameter = 2*math.sqrt(area/math.pi)/PX_PER_MM
        if not diameter_min <= diameter <= diameter_max or circularity < 0.65:
            continue
        moments = cv2.moments(c)
        results.append((moments["m10"]/area, moments["m01"]/area, diameter, circularity))
    return results


def check_fixture(gray):
    checks = []
    radius = 5*PX_PER_MM
    for bx, by in f.CHECKS:
        x, y = np.rint(project(((bx, by),), bed_to_raster())[0]).astype(int)
        roi = gray[y-radius:y+radius, x-radius:x+radius]
        candidates = blobs(roi, 2.5, 5, "dark")
        center = min(candidates, key=lambda b: math.hypot(b[0]-radius, b[1]-radius),
                     default=None)
        error = None if center is None else math.hypot(center[0]-radius, center[1]-radius)/PX_PER_MM
        checks.append({"x_bed_mm": bx, "y_bed_mm": by, "error_mm": error})
    return checks


def detect(gray, polarity, diameter_min, diameter_max):
    s = PX_PER_MM
    bx, by = f.BOARD_ORIGIN
    # Ignore the printed border and tape outside the nominal 50 mm insert.
    margin = 2
    x = round((bx+margin)*s)
    y = round((f.PAGE[1]-by-f.BOARD_SIZE+margin)*s)
    size = round((f.BOARD_SIZE-2*margin)*s)
    results = [{"x_mm": margin+cx/s, "y_mm": f.BOARD_SIZE-margin-cy/s,
                "diameter_mm": diameter, "circularity": circularity}
               for cx, cy, diameter, circularity in
               blobs(gray[y:y+size, x:x+size], diameter_min, diameter_max, polarity)]
    return sorted(results, key=lambda p: (p["y_mm"], p["x_mm"]))


def read_reference(path):
    with open(path, newline="") as source:
        reader = csv.DictReader(source)
        if not set(f.CSV_FIELDS).issubset(reader.fieldnames or ()):
            raise ScanError(f"{path}: expected columns {', '.join(f.CSV_FIELDS)}.")
        try:
            points = [((r["id"] or "").strip(), *(float(r[k]) for k in f.CSV_FIELDS[1:]))
                      for r in reader]
        except (TypeError, ValueError) as exc:
            raise ScanError(f"{path}: reference rows need numeric X, Y and diameter.") from exc
    if not points or len({p[0] for p in points}) != len(points) or any(
            not label or not (0 <= x <= 50 and 0 <= y <= 50 and 0 < d <= 50)
            for label, x, y, d in points):
        raise ScanError("Reference CSV needs unique IDs and valid board-frame positions and diameters.")
    return points


def compare(points, reference):
    # One-to-one nearest pairs within the demo fixture's 3 mm association gate.
    # The nominal marks are well separated; larger offsets are missing + extra.
    pairs = sorted((math.hypot(p["x_mm"]-x, p["y_mm"]-y), i, j)
                   for i, (_, x, y, _) in enumerate(reference) for j, p in enumerate(points))
    matched, used = {}, set()
    for error, i, j in pairs:
        if error > 3:
            break
        if i not in matched and j not in used:
            matched[i] = j
            used.add(j)
    rows = []
    for i, (label, x, y, diameter) in enumerate(reference):
        row = {"id": label, "expected_x_mm": x, "expected_y_mm": y, "status": "MISSING"}
        if i in matched:
            p = points[matched[i]]
            p["id"] = label
            dx, dy = p["x_mm"]-x, p["y_mm"]-y
            error = math.hypot(dx, dy)
            row.update(dx_mm=dx, dy_mm=dy, radial_mm=error,
                       diameter_error_mm=p["diameter_mm"]-diameter,
                       status="PASS" if error <= PLACEMENT_TOLERANCE_MM else "OFFSET")
        rows.append(row)
    extra = 0
    for j, p in enumerate(points):
        if j not in used:
            extra += 1
            p["id"] = f"EXTRA-{extra}"
            rows.append({"id": p["id"], "status": "EXTRA"})
    return rows


def overlay(rectified, points, comparison):
    # Crop to the inspection region (preserving the scale).
    ox, oy = 94, 142
    s = PX_PER_MM
    result = rectified[(f.PAGE[1]-oy)*s:(f.PAGE[1]-74)*s, ox*s:204*s].copy()
    bx, by = f.BOARD_ORIGIN

    def pixel(x, y):
        return (round((x+bx-ox)*s), round((oy-y-by)*s))

    statuses = {row["id"]: row["status"] for row in comparison}
    for row in comparison:
        if row["status"] == "EXTRA":
            continue
        pt = pixel(row["expected_x_mm"], row["expected_y_mm"])
        color = (0, 0, 210) if row["status"] == "MISSING" else (195, 93, 0)
        cv2.drawMarker(result, pt, color, cv2.MARKER_CROSS, 13, 2, cv2.LINE_AA)
        if row["status"] == "MISSING":
            cv2.putText(result, f'{row["id"]} missing', (pt[0]+10, pt[1]-9),
                        cv2.FONT_HERSHEY_SIMPLEX, .45, color, 1, cv2.LINE_AA)
        elif row["status"] == "OFFSET":
            end = pixel(row["expected_x_mm"]+row["dx_mm"], row["expected_y_mm"]+row["dy_mm"])
            cv2.arrowedLine(result, pt, end, (0, 0, 210), 2, cv2.LINE_AA, tipLength=.4)
    for p in points:
        x, y = pixel(p["x_mm"], p["y_mm"])
        color = (0, 125, 0) if statuses.get(p["id"], "PASS") == "PASS" else (0, 0, 210)
        cv2.circle(result, (x, y), max(4, round(p["diameter_mm"]*s/2)),
                   color, 2, cv2.LINE_AA)
        cv2.putText(result, f'{p["id"]} ({p["x_mm"]:.2f}, {p["y_mm"]:.2f})',
                    (x+9, y+20), cv2.FONT_HERSHEY_SIMPLEX, .45, color, 1, cv2.LINE_AA)
    return result


def save_image(path, image):
    if not cv2.imwrite(str(path), image):
        raise ScanError(f"Could not write {path}")


def save_json(path, data):
    path.write_text(json.dumps(data, indent=2, allow_nan=False) + "\n")


def save_csv(path, rows, fields):
    with path.open("w", newline="") as output:
        writer = csv.DictWriter(output, fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows({k: round(v, 4) if isinstance(v, float) else v
                          for k, v in r.items()} for r in rows)


def run(args, frame):
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    directory = Path(args.runs_dir).resolve() / timestamp
    directory.mkdir(parents=True)
    save_image(directory / "raw.png", frame)
    print(f"Run: {directory}", flush=True)
    try:
        success = analyze(args, frame, directory)
    except (ScanError, OSError, ValueError, cv2.error, np.linalg.LinAlgError) as error:
        (directory / "report.md").write_text(f"# Vision scan: failed\n\n{error}\n")
        print(f"FAILED: {error}", flush=True)
        success = False
    return directory, success


def analyze(args, frame, directory):
    h, diagnostics = register(frame)
    s = PX_PER_MM
    rectified = cv2.warpPerspective(frame, bed_to_raster() @ h,
                                    (f.PAGE[0]*s, f.PAGE[1]*s),
                                    borderValue=(255, 255, 255))
    save_image(directory / "rectified.png", rectified)
    gray = cv2.cvtColor(rectified, cv2.COLOR_BGR2GRAY)
    checks = check_fixture(gray)
    diagnostics["independent_checks"] = checks
    save_json(directory / "registration.json", diagnostics)
    if any(row["error_mm"] is None or row["error_mm"] > 1 for row in checks):
        details = ", ".join("missing" if r["error_mm"] is None else
                            f'{r["error_mm"]:.2f} mm' for r in checks)
        raise ScanError(f"Independent fixture checks must all be within 1.0 mm ({details}).")
    native = diagnostics["min_native_px_per_mm"]
    if native < 2.5:
        raise ScanError(f"Board has only {native:.2f} native px/mm; need at least 2.5. "
                        "Reposition the camera, staying in focus.")

    reference = (read_reference(args.reference) if args.reference is not None else
                 f.POINTS if args.preset == "paper" else None)
    points = detect(gray, args.polarity, args.min_diameter, args.max_diameter)
    rows = compare(points, reference) if reference is not None else []
    if reference is None:
        for i, p in enumerate(points, start=1):
            p["id"] = f"P{i:02}"
    for p in points:
        p["x_bed_mm"] = p["x_mm"] + f.BOARD_ORIGIN[0]
        p["y_bed_mm"] = p["y_mm"] + f.BOARD_ORIGIN[1]
    save_csv(directory / "detections.csv", points, (*f.CSV_FIELDS, "x_bed_mm",
                                                    "y_bed_mm", "circularity"))
    save_image(directory / "overlay.png", overlay(rectified, points, rows))
    if reference is not None:
        save_csv(directory / "ground_truth.csv", [dict(zip(f.CSV_FIELDS, p)) for p in reference],
                 f.CSV_FIELDS)
        save_csv(directory / "errors.csv", rows, ("id", "status", "expected_x_mm",
                 "expected_y_mm", "dx_mm", "dy_mm", "radial_mm", "diameter_error_mm"))

    success = bool(points) and all(row["status"] == "PASS" for row in rows)
    if success and not args.verify:
        save_json(directory / "job.json", {
            "schema_version": 1, "units": "mm", "frame": "vision_bed",
            "board_origin_in_bed_mm": f.BOARD_ORIGIN,
            "review_status": "pending", "executable": False,
            "points": [{"id": p["id"], "x": round(p["x_bed_mm"], 4),
                        "y": round(p["y_bed_mm"], 4),
                        "diameter_mm": round(p["diameter_mm"], 4)} for p in points],
        })
    status = ("PASS" if reference is not None else "REVIEW REQUIRED") if success else "FAIL"
    report = [f"# Vision scan: {status}", "",
              f"- Source: {'saved image (replay)' if args.replay else 'live camera'}",
              f"- Mode: {'inspection' if args.verify else 'target detection'} ({args.preset})",
              f"- Native board resolution (worst axis): {native:.2f} px/mm",
              f'- Tags: {diagnostics["seen_ids"]}; inlier reprojection RMS: '
              f'{diagnostics["reprojection_rms_px"]:.3f} px',
              f'- Independent check max error: {max(r["error_mm"] for r in checks):.3f} mm',
              f"- Detected: {len(points)}" + (f" / {len(reference)} nominal" if reference else ""),
              "- No robot-base calibration or robot motion. Review the annotated image.", ""]
    if reference is not None:
        errors = np.array([r["radial_mm"] for r in rows if "radial_mm" in r])
        report += [f"- Matched {len(errors)}; missing {sum(r['status']=='MISSING' for r in rows)}; "
                   f"extra {sum(r['status']=='EXTRA' for r in rows)}."]
        if len(errors):
            report += [f"- Matched radial error: mean {errors.mean():.3f}, "
                       f"population SD {errors.std():.3f}, max {errors.max():.3f} mm."]
        report += ["", "| ID | Result | Radial error (mm) |", "|---|---|---:|"]
        report += [f'| {r["id"]} | {r["status"]} | ' +
                   (f'{r["radial_mm"]:.3f} |' if "radial_mm" in r else "— |") for r in rows]
    (directory / "report.md").write_text("\n".join(report) + "\n")
    print("\n".join(report), flush=True)
    return success
