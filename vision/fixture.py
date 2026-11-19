"""Shared physical layout and print-artifact generator (all dimensions in mm)."""

import csv
import json
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
PRINT_DIR = ROOT / "Docs" / "print"
PAGE = (297, 210)
WIDTH, HEIGHT = 235, 127
PAGE_OFFSET = ((PAGE[0]-WIDTH)/2, (PAGE[1]-HEIGHT)/2)
TAG_SIZE = 30
TAGS = dict(enumerate(((0, 0), (WIDTH, 0), (WIDTH, HEIGHT), (0, HEIGHT))))
POINTS = (("H1", 40, 31, 3), ("H2", 110, 45, 3), ("H3", 186, 30, 3),
          ("H4", 51, 83, 3), ("H5", 128, 94, 3), ("H6", 193, 83, 3))
VERIFY_POINTS = (("H1", 41.5, 31, 3), POINTS[1], POINTS[3], ("H5", 128, 92, 3))
CSV_FIELDS = ("id", "x_mm", "y_mm", "diameter_mm")


def dictionary():
    return cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)


def tag_corners(tag_id):
    x, y = TAGS[tag_id]
    h = TAG_SIZE / 2
    return ((x-h, y+h), (x+h, y+h), (x+h, y-h), (x-h, y-h))


def marker(tag_id):
    aruco = cv2.aruco
    if hasattr(aruco, "generateImageMarker"):
        return aruco.generateImageMarker(dictionary(), tag_id, 6)
    return aruco.drawMarker(dictionary(), tag_id, 6)


def demo_frame():
    """Synthetic camera input for the offline simulation; still goes through CV."""
    scale = 6
    frame = np.full((PAGE[1] * scale, PAGE[0] * scale, 3), 255, dtype=np.uint8)

    def pixel(x, y):
        return round((PAGE_OFFSET[0] + x) * scale), round((PAGE[1] - PAGE_OFFSET[1] - y) * scale)

    for tag_id in TAGS:
        x, y = pixel(*tag_corners(tag_id)[0])
        side = TAG_SIZE * scale
        tag = cv2.resize(marker(tag_id), (side, side), interpolation=cv2.INTER_NEAREST)
        frame[y:y+side, x:x+side] = cv2.cvtColor(tag, cv2.COLOR_GRAY2BGR)
    for _, x, y, diameter in POINTS:
        cv2.circle(frame, pixel(x, y), round(scale * diameter / 2), (0, 0, 0), -1, cv2.LINE_AA)
    return frame


def generate(output=PRINT_DIR):
    from matplotlib import use
    use("Agg")
    from matplotlib import pyplot as plt
    from matplotlib.patches import Circle, Rectangle

    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)

    def sheet(width, height):
        fig = plt.figure(figsize=(width/25.4, height/25.4), facecolor="white")
        ax = fig.add_axes((0, 0, 1, 1), xlim=(0, width), ylim=(0, height))
        ax.axis("off")
        return fig, ax

    def save(fig, name):
        fig.savefig(output / name)
        plt.close(fig)

    for name, points, title in (
        ("aruco_bed_v1.pdf", POINTS, "SCARA VISION | 235 × 127 mm | 6 × Ø3 mm"),
        ("aruco_bed_inspection.pdf", VERIFY_POINTS, "INSPECTION SAMPLE | shifted and missing targets"),
    ):
        fig, ax = sheet(*PAGE)
        left, bottom = PAGE_OFFSET
        ax.text(148.5, 194, title, ha="center", fontsize=12)
        ax.text(148.5, 185, "A4 LANDSCAPE · PRINT AT 100% / ACTUAL SIZE",
                ha="center", fontsize=9)
        for tag_id, (x, y) in TAGS.items():
            for row, col in zip(*((marker(tag_id) == 0).nonzero())):
                cell = TAG_SIZE / 6
                ax.add_patch(Rectangle((left+x-TAG_SIZE/2+col*cell,
                                        bottom+y+TAG_SIZE/2-(row+1)*cell),
                                       cell, cell, fc="black", ec="none", antialiased=False))
        # Leave a white quiet zone around the tags, including where box edges approach.
        gap = 24
        for y in (bottom, bottom+HEIGHT):
            ax.plot([left+gap, left+WIDTH-gap], [y, y], color="black", lw=.6)
        for x in (left, left+WIDTH):
            ax.plot([x, x], [bottom+gap, bottom+HEIGHT-gap], color="black", lw=.6)
        for _, x, y, diameter in points:
            ax.add_patch(Circle((left+x, bottom+y), diameter/2, fc="black", ec="none"))
        ax.plot([98.5, 198.5], [20, 20], color="black", lw=.6)
        for x in (98.5, 198.5):
            ax.plot([x, x], [18, 22], color="black", lw=.6)
        ax.text(148.5, 25, "100 mm — verify with ruler", ha="center", fontsize=8)
        ax.text(148.5, 10, "Origin: center of lower-left tag · X right · Y up",
                ha="center", fontsize=8)
        save(fig, name)

    (output / "tags.json").write_text(json.dumps({
        "units": "mm", "dictionary": "DICT_4X4_50", "bed_origin": "lower_left_tag_center",
        "page_mm": PAGE, "tag_size_mm": TAG_SIZE, "tag_corners_mm": {
            tag_id: tag_corners(tag_id) for tag_id in TAGS},
        "fixture_size_mm": (WIDTH, HEIGHT), "page_offset_mm": PAGE_OFFSET,
    }, indent=2) + "\n")
    with (output / "ground_truth.csv").open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(CSV_FIELDS)
        writer.writerows(POINTS)
    print(f"Print assets: {output}")


if __name__ == "__main__":
    generate()
