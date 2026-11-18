# SCARA
# SCARA PCB vision MVP

The Jetson measures visible drill targets with a Rapoo USB camera and exports XY coordinates
in millimetres. This demo does **not** move the arm, drive the spindle, or claim drilling
accuracy. The team's 235 × 127 mm measured reach is separate from the local A4 vision cell.

## Set up the demo

1. Run `./setup.sh`. It uses the system OpenCV with ArUco and generates the PDFs under
   [`Docs/print/`](Docs/print/). Use `./setup.sh --no-camera` for a print/replay-only setup.
2. Print [`aruco_bed_v1.pdf`](Docs/print/aruco_bed_v1.pdf) on **A4 landscape at 100% / Actual
   size**, not Fit. Use matte paper and check both 100 mm bars with a ruler. Tape the sheet
   flat. The six nominal marks are already in the center: you don't need a second sheet
   for the first scan. Leave the four separate reference dots uncovered. For robot
   integration, place the entire 50 × 50 mm target square comfortably within the
   verified tool-tip reach. The rest of the A4 sheet may extend beyond the reach as
   long as it stays flat and the camera can see the tags and reference dots.
   **Do not shrink the print to fit the reach.**
3. Mount the fixed-focus camera so the whole sheet is sharp and fills most of the frame,
   looking down rather than grazing the surface. The arm should be parked clear of the cell
   by the team before scanning. Use diffuse light. Run `./start.sh --self-test` to check
   capture, tag visibility and native board resolution. Move the camera or adjust lighting
   as needed; it doesn't matter which way the camera is rotated. If the driver or a camera
   app mirrors the raw image, turn off that setting.
4. Run `./start.sh` for a scan or `./start.sh --serve` for a read-only browser view at
   `http://127.0.0.1:8080/`. The viewer shows a **single** snapshot and releases the camera
   before serving. Stop it with Ctrl-C or `./stop.sh`. Use `--bind <Jetson-LAN-IP>` if the
   browser is on another machine on a trusted local network.

## Verification, replay and PCB samples

```bash
./start.sh --verify                           # nominal paper sample
./start.sh --verify --serve                   # review a verification pass or failure
./start.sh --replay runs/<timestamp> --serve    # explicitly replay a saved capture
./start.sh --preset pcb --reference board.csv  # optional known PCB target layout
```

- To demo a failure, print [`board_50x50_drilled.pdf`](Docs/print/board_50x50_drilled.pdf)
  at 100%, cut out the square, and align it over the nominal square. It contains two
  intentional shifts and two missing marks. Run `--verify`, which writes the inspection
  report but no job. The [nominal insert](Docs/print/board_50x50_v1.pdf) is optional.
- The PCB preset is unvalidated on real boards. It expects visible, high-contrast circular
  features in the 50 × 50 mm cell. Try `--polarity bright` for bright features, or set the
  `--min-diameter` and `--max-diameter` filter values in mm. An unmarked blank needs a
  drill file and separate board registration; there is nothing for the detector to find.
  Align the board lower-left with the outline, and physically place the tags **and check
  dots** in the same plane as the PCB top (a recess or shims); a printed outline alone
  does not correct height-induced error. Custom reference columns are
  `id,x_mm,y_mm,diameter_mm`; X and Y are board-local. `--verify` needs a reference for PCB.
- Change devices with `--camera /dev/video2` or `SCARA_CAMERA`. Exposure and white
  balance depend on camera support. If `v4l2-ctl` is available, first check what the driver
  supports with `v4l2-ctl -d /dev/video0 --list-ctrls`.

## Outputs and limits

Each run saves its input, diagnostic report and available artifacts under
`runs/<timestamp>/`. `overlay.png` shows measured centers and their board coordinates.
`detections.csv` includes both board-local and bed coordinates. `errors.csv` compares
matched points against nominal placement (0.5 mm radial tolerance) and flags missing
or extra marks. A passing non-verification run writes `job.json` in the local
`vision_bed` frame, explicitly marked **pending review / not executable**.

The bed origin is the lower-left of the landscape sheet, X right and Y up; the board
origin is `(123.5, 80)` mm in that frame. The ArUco homography corrects planar perspective
and sets scale, not lens distortion. Separate printed check dots test local mapping
but share the printer's scale; the measured bars and a real calibrated target are
needed for absolute accuracy evidence. If corner tags or independent checks fail,
the vision pipeline rejects the scan rather than exporting a job. Fix mount geometry
or calibrate intrinsics next if physical residuals are too high.

Future integration needs repeatable homing, bounded position commands with completion
and fault status, verified limit directions, a measured bed-to-base transform, a dry
run and physical safety sign-off. `ESP32-Motor-Control/main.ino` is unchanged.

Run `python3 -m unittest discover -s tests -v` to regression-test the nominal print
and inspection path from rasterized PDFs (`pdftoppm`, from `poppler-utils`, is required
for these tests).
