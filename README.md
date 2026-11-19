# SCARA
# SCARA PCB vision MVP

The Jetson measures visible drill targets with a Rapoo USB camera and exports XY coordinates
in millimetres. The vision-only demo does **not** move the arm, drive the spindle, or claim
drilling accuracy. The printed 235 × 127 mm box uses the measured overall reach dimensions;
it is not a guarantee that every target is reachable.

## Set up the demo

1. Run `./setup.sh`. It uses the system OpenCV with ArUco and generates the PDFs under
   [`Docs/print/`](Docs/print/). Use `./setup.sh --no-camera` for a print/replay-only setup.
2. Print [`aruco_bed_v1.pdf`](Docs/print/aruco_bed_v1.pdf) on **A4 landscape at 100% / Actual
   size**, not Fit. Check the 100 mm scale bar with a ruler and tape the sheet flat.
   The current fixture has four corner tags and six Ø3 mm printed targets; reprint
   if you still have the older six-tag/50 mm fixture. Keep all four tags visible.
   For robot integration, individually verify the six target positions are reachable.
   The paper margin can extend outside the reach. **Do not shrink the print.**
3. Mount the fixed-focus camera so the whole sheet is sharp and fills most of the frame,
   looking down rather than grazing the surface. The arm should be parked clear of the cell
   by the team before scanning. Use diffuse light. Run `./start.sh --self-test` to check
   capture, tag visibility and native board resolution. Move the camera or adjust lighting
   as needed; it doesn't matter which way the camera is rotated. If the driver or a camera
   app mirrors the raw image, turn off that setting.
4. Run `./start.sh --serve` for the **live dashboard** at `http://127.0.0.1:8080/`.
   Click **Scan** (or press **S**) to measure a full-resolution snapshot and review
   the annotated result. Click **Live** to go back to the preview. Stop with Ctrl-C
   or `./stop.sh`. `./start.sh` without flags is still a one-shot CLI scan; stop the
   live dashboard before using it to scan from the CLI.

   When SSH'd into the Jetson, run the tunnel on your **workstation** and open
   `http://127.0.0.1:8080/` there:

   ```bash
   ssh -N -L 8080:127.0.0.1:8080 scara@<Jetson-SSH-address>
   ```

## Verification, replay and PCB samples

```bash
./start.sh --verify                           # nominal paper sample
./start.sh --verify --serve                   # review a verification pass or failure
./start.sh --replay runs/<timestamp> --serve    # explicitly replay a saved capture
./start.sh --preset pcb --reference board.csv  # optional known PCB target layout
```

- If you need to recreate an offline demo from the printable fixture, render it
  with `pdftoppm -r 152.4 -png -singlefile Docs/print/aruco_bed_v1.pdf /tmp/scara-fixture`
  and run `./start.sh --replay /tmp/scara-fixture.png --serve`. The dashboard marks
  this as replay, not a live camera measurement.
- To demo a failure, print [`aruco_bed_inspection.pdf`](Docs/print/aruco_bed_inspection.pdf)
  at 100% and show it in place of the nominal sheet. It has two intentionally shifted
  and two missing marks. `--verify` writes the inspection report but no job.
- The PCB preset is unvalidated on real boards. It expects visible, high-contrast circular
  features inside the fixture (excluding a 3 mm border and the tag corners).
  Try `--polarity bright` for bright features, or set the
  `--min-diameter` and `--max-diameter` filter values in mm. An unmarked blank needs a
  drill file and separate board registration; there is nothing for the detector to find.
  Align the actual PCB to its known fixture position and place the tags in the same
  plane as the PCB top; the homography does not correct height-induced error. Cover
  the nominal printed targets or use a blank fixture region to avoid false detections.
  Custom reference columns are `id,x_mm,y_mm,diameter_mm` in fixture coordinates.
  `--verify` needs a reference for PCB.
- Change devices with `--camera /dev/video2` or `SCARA_CAMERA`. Exposure and white
  balance depend on camera support. If `v4l2-ctl` is available, first check what the driver
  supports with `v4l2-ctl -d /dev/video0 --list-ctrls`.

## Outputs and limits

Each run saves its input, diagnostic report and available artifacts under
`runs/<timestamp>/`. `overlay.png` shows measured centers and their fixture coordinates.
`detections.csv` gives the same coordinates. `errors.csv` compares
matched points against nominal placement (0.5 mm radial tolerance) and flags missing
or extra marks. A passing non-verification run writes `job.json` in the local
`vision_bed` frame, explicitly marked **pending review / not executable**.

The origin is the center of the lower-left tag, X right and Y up. The ArUco homography
corrects planar perspective and sets scale, not lens distortion. Tag reprojection
error is a fit diagnostic, not an independent accuracy check. Use the printed scale
bar and a calibrated target for physical accuracy evidence. Missing corner tags or
inconsistent registration reject the scan rather than exporting a job.

## Robot workflow

Run **`./start.sh`** and open `http://localhost:8080/`. Connect the ESP32 in
the UI or select **Run simulation** to scan an image and verify simulated
target visits without hardware. See the [command guide](Docs/command-guide.md).

Run `./start.sh --serve --simulate-robot` to try **Homing → Scanning → Planning →
Executing → Done** with the local controller simulator. Approve the plan before
execution. Add `--replay <image-or-run>` for an offline demonstration.
Standard `--serve` remains vision-only. For physical Mode B (home, park, scan
and show coordinates) and the manual setup, Z, drill and pose logging tools, see the
[controller contract and commissioning guide](Docs/robot-contract.md).
The sketch ships status-only until the travel profile is verified; the vision
workflow never enables the drill.

For separate J1/J2 control with live camera preview, open `/manual` on the
robot dashboard. See the [command guide](Docs/command-guide.md) for launch
commands and pre-home bench testing.

Run `python3 -m unittest discover -s tests -v` to regression-test the nominal print
and inspection path from rasterized PDFs (`pdftoppm`, from `poppler-utils`, is required
for these tests).

## Mimicry: separate hand-control dashboard

Run **`./startmimicry.sh --install-model`** once, then **`./startmimicry.sh`**.
Open **http://127.0.0.1:8081/**. The default uses the USB camera with a simulated
robot. Use `./startmimicry.sh --no-camera` to try it with arrow keys and Space
instead. Mimicry doesn't scan fixtures or control the drill. See the
[Mimicry setup guide](Docs/mimicry.md) for gestures, Set Z Zero, and hardware setup.
