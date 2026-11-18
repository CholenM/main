"""Shared physical layout and print-artifact generator (all dimensions in mm)."""

import csv
import json
from pathlib import Path

import cv2

ROOT = Path(__file__).resolve().parents[1]
PRINT_DIR = ROOT / "Docs" / "print"
PAGE = (297, 210)
TAG_SIZE = 40
TAGS = dict(enumerate(((45, 45), (252, 45), (252, 165), (45, 165),
                      (148.5, 38), (148.5, 172))))
BOARD_ORIGIN = (123.5, 80)
BOARD_SIZE = 50
CHECKS = ((102, 82), (102, 128), (195, 82), (195, 128))
POINTS = (("H1", 8, 10, 2.5), ("H2", 20, 8, 3), ("H3", 33, 12, 2.5),
          ("H4", 10, 25, 3), ("H5", 25, 30, 2.5), ("H6", 40, 38, 3))
VERIFY_POINTS = (("H1", 9.5, 10, 2.5), POINTS[1], POINTS[3],
                 ("H5", 25, 28, 2.5))
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

    def print_board(ax, x, y, points):
        ax.add_patch(Rectangle((x, y), BOARD_SIZE, BOARD_SIZE, fill=False, lw=.6))
        for _, bx, by, diameter in points:
            ax.add_patch(Circle((x+bx, y+by), diameter/2, fc="black", ec="none"))

    fig, ax = sheet(*PAGE)
    ax.text(148.5, 199, "SCARA VISION CELL | PRINT AT 100% / ACTUAL SIZE",
            ha="center", fontsize=10)
    for tag_id, (x, y) in TAGS.items():
        for row, col in zip(*((marker(tag_id) == 0).nonzero())):
            cell = TAG_SIZE / 6
            ax.add_patch(Rectangle((x-TAG_SIZE/2+col*cell, y+TAG_SIZE/2-(row+1)*cell),
                                   cell, cell, fc="black", ec="none", antialiased=False))
    print_board(ax, *BOARD_ORIGIN, POINTS)
    ax.text(148.5, 135, "50 × 50 mm target | use as printed or cover with test insert",
            ha="center", fontsize=7)
    for x, y in CHECKS:
        ax.add_patch(Circle((x, y), 2, fc="black", ec="none"))
    ax.plot([88, 188], [68, 68], color="black", lw=.6)
    for x in range(88, 189, 10):
        ax.plot([x, x], [66.5, 69.5], color="black", lw=.6)
    ax.text(138, 71, "100 mm — verify with ruler", ha="center", fontsize=7)
    ax.plot([208, 208], [55, 155], color="black", lw=.6)
    for y in range(55, 156, 10):
        ax.plot([206.5, 209.5], [y, y], color="black", lw=.6)
    ax.text(213, 105, "100 mm", rotation=90, va="center", fontsize=7)
    ax.annotate("+X", xy=(90, 105), xytext=(79, 105), fontsize=7,
                arrowprops={"arrowstyle": "->"})
    ax.annotate("+Y", xy=(85, 117), xytext=(85, 109), fontsize=7,
                arrowprops={"arrowstyle": "->"})
    save(fig, "aruco_bed_v1.pdf")

    for name, points, title in (
        ("board_50x50_v1.pdf", POINTS, "Nominal PCB target"),
        ("board_50x50_drilled.pdf", VERIFY_POINTS, "Inspection sample: shifted and missing marks"),
    ):
        fig, ax = sheet(210, 297)
        ax.text(105, 275, f"{title} | print at 100% / Actual size", ha="center", fontsize=11)
        print_board(ax, 80, 174, points)
        ax.text(105, 165, "Cut out the 50 × 50 mm square and align it to the fixture outline.",
                ha="center", fontsize=8)
        ax.text(105, 149, "Nominal reference (mm), origin at lower-left", ha="center", fontsize=9)
        for i, (label, x, y, diameter) in enumerate(POINTS):
            ax.text(105, 140-i*6, f"{label}     X={x:.2f}     Y={y:.2f}     Ø={diameter:.2f}",
                    ha="center", fontsize=8)
        save(fig, name)

    (output / "tags.json").write_text(json.dumps({
        "units": "mm", "dictionary": "DICT_4X4_50", "bed_origin": "page_lower_left",
        "page_mm": PAGE, "tag_size_mm": TAG_SIZE, "tag_corners_mm": {
            tag_id: tag_corners(tag_id) for tag_id in TAGS},
        "board_origin_bed_mm": BOARD_ORIGIN, "board_size_mm": BOARD_SIZE,
        "check_centers_bed_mm": CHECKS,
    }, indent=2) + "\n")
    with (output / "ground_truth.csv").open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(CSV_FIELDS)
        writer.writerows(POINTS)
    print(f"Print assets: {output}")


if __name__ == "__main__":
    generate()
