# SCARA controller contract and commissioning

The Jetson owns the camera, fixture registration, local calibration, planning, approval
and sequencing. The ESP32 owns pulses, switches, step budgets, target completion
and the motion lease. **The vision workflow remains a dry run: it never turns the
drill on or drills holes.** Manual control can jog a bounded, unreferenced Z axis
and toggle the drill relay.
The bench rig has a bottom Z switch but **no upper Z switch and no Z homing phase**.
`pos.z = 0` is the boot datum, not a measured height.
No USB camera, calibration, or robot is needed to try the simulator workflow.

## Labels and coordinate conventions

| API | Rig labels (motor / driver / switch) | Old code | Positive | STEP / DIR / limit GPIO |
|---|---|---|---|---|
| `z` | `Z / Z-DRV / Z-LIM` | `Z` | Up (unreferenced steps) | 2 / 15 / 27 |
| `j1` | `J1 / J1-DRV / J1-LIM` | `XY` | CCW viewed from above | 16 / 17 / 26 |
| `j2` | `J2 / J2-DRV / J2-LIM` | `EL` | CCW viewed from above | 21 / 22 / 25 |

Rotary position is an integer count of commanded STEP rising edges from the backed-off
home datum, not encoder feedback. Z position is also a commanded step count, but it is
**unreferenced**: `pos.z = 0` is the value at controller boot and resets on every reboot.
Z has a bottom switch, no upper switch, and no homing phase. Positive Z steps must raise
the tool; downward motion stops when the bottom switch closes. The host never chooses
electrical direction levels. Set `positiveLevel` and `homeSign` separately after checking
each axis. Z has no homing phase, so its `homeSign` is unused; leave it at 0. Verify the
Z sign with a few unpowered or raised-tool steps before using it.
The fixture frame is **not** the robot-base frame: X right, Y up from the bottom-left
ArUco tag center, in mm. The A4988 rotary drivers are set to 1/16 and Z to 1/4
per the bench setup; record real pulley tooth counts and Z travel per motor revolution
before changing calibration. GT2 pitch and belt length alone do not determine ratio.

## HTTP `scara-slave/1.0`

ESP32 station mode defaults to `cybernetics-rpi`. Enter the actual site password in
local `secrets.h` and generate a **separate** controller token. Read the IP from
the serial console at 115200; try `scara.local` if mDNS is supported on your LAN.
Every request needs `X-SCARA-Token`. This is plain HTTP: keep it on a trusted isolated
lab network (not campus-wide or the internet). The Jetson hardware dashboard binds
only to loopback; access it through SSH. There is no fallback access point.

| Method | Endpoint | Body | Successful response |
|---|---|---|---|
| GET | `/status` | none | `200` state, position, limits and flags |
| POST | `/home` | `{"boot":"…","command_id":16}` | `202`, async J1 → J2; no Z motion |
| POST | `/move` | `{"boot":"…","command_id":17,"axis":"j1","to":120,"feed":2000}` | `202`, bounded absolute steps; Z does not require homing |
| POST | `/nudge` | `{"boot":"…","command_id":17,"axis":"j2","delta":10,"clearance_confirmed":true}` | `202`, opt-in pre-home bench nudge; see [command guide](command-guide.md) |
| POST | `/heartbeat` | `{"boot":"…","command_id":17}` | `200`, extends that active motion lease and any active drill lease |
| POST | `/drill` | `{"boot":"…","on":true}` | `200`, relay on with a 2.5 s lease renewed by heartbeat; `off` needs no matching boot |
| POST | `/spindle` | `{"on":false}` | `200`; `true` always rejected in this version |
| POST | `/stop` | `{}` | `200`, stops pulses, drill off, latches HOLD |
| POST | `/estop` | `{}` | `200`, network stop latched, drill off, invalidates datum |
| POST | `/reset` | `{}` | `200` from HOLD/FAULT/ESTOP, returns unhomed IDLE with drill off |

POSTs use JSON. Success returns `{"boot":"…","command_id":17}`. Status includes
`fw`, `boot`, `command_id`, `state`, `homed`, `pos`, `target`, `limits`, `spindle`,
`spindle_gpio`, `drill_lease_ms`, `fault`, `commissioned`, and `simulator`, plus
`z_mode: "bounded"` and informational `rssi` (dBm) and `ip` on hardware. `spindle`
is the live drill-relay state, not a promise that the bit or spindle is safe.
`bounds` gives inclusive absolute step ranges for all three axes.
`home_axis` and `home_phase` show the active or failed homing phase. A budget
fault names both, e.g. `HOME_TIMEOUT_J2_SEEK`; it still latches until Reset.
Axis keys are `z`, `j1`, `j2`; limit `0`
means active-low contact and `1` means clear. Non-2xx responses include `error`.
`feed` is a half-period in microseconds (1,000–20,000; scheduler may make it longer).
Homing seeks, releases, re-latches slowly, then steps off the switch. It never goes
to an unknown hard stop to measure travel. Each phase is limited by the local profile.
J1/J2 movement requires rotary homing and bounded joint targets. Z movement is
allowed without homing, within configured `bounds.z`, and updates only the boot-relative
step count. A pressed Z bottom switch rejects downward Z and all J1/J2 moves; it does
not block upward Z, so the tool can be lifted off the contact. A downward Z move that
reaches the switch ends in `IDLE` with `fault: "Z_BOTTOM_LIMIT"` and keeps the rotary
home datum. A clear switch does **not** establish any height, clearance or upper travel
limit, and cannot detect a disconnected switch. There is no upper switch: upward travel
is limited only by `bounds.z` and the operator's stop.

Motion POSTs must match the current boot and last command ID before the controller
accepts them. The Jetson never retries an ambiguous motion POST; it accepts completion only when
boot ID, command ID, state, datum and target agree. Read-only status does **not** renew
the lease. An active command stops on disconnection or two seconds without a matching
heartbeat; re-home after reset. **No automatic retract on link loss in this release.**
The drill has its own 2.5-second lease. Turning it on requires an idle controller and
the current boot; the Jetson renews the lease with `/heartbeat`. The relay drops on
`/stop`, `/estop`, `/reset`, any new home/move/nudge command, Wi-Fi loss, or an expired
lease. The drill has **no interlock**: firmware does not verify spindle power, bit
presence, guards or E-stop wiring. Neither the network stop nor firmware replaces a
hardwired emergency stop that removes hazardous energy.

## Try the UI and commission in order

**Simulator:** `./start.sh --serve --simulate-robot` uses the live webcam; add
`--replay <image>` to use a known-good fixture capture. The dashboard shows
**Homing → Scanning → Planning → Executing → Done**. Click **Start**, inspect the
proposed/rounded positions, then click **Approve & execute dry run**. Parking
and the verification scan still happen. This proves sequencing, not machining.

**Bench setup:**

1. Isolate the spindle and fit a physical emergency stop before powered motion.
   Copy `ESP32-Motor-Control/secrets.example.h` to `secrets.h`, then fill in the Wi-Fi
   password and a random controller token (e.g. from `openssl rand -hex 16`). Both
   local headers are ignored by Git. Open `ESP32-Motor-Control/ESP32-Motor-Control.ino`
   in Arduino, select the actual ESP32 board, install ArduinoJson **7**, then
   compile/upload. The matching sketch file loads the controller in `main.ino`.
2. First flash the **status-only** build (leave `MOTION_COMMISSIONED = false`).
   `GET /status` with the token should work; manually toggle each unpowered axis
   limit and observe that only its labeled status changes. Confirm the relay stays
   off on reset and that the hardware emergency stop works. The pin map is inherited,
   not verification of your wiring or of boot-time relay behavior.

   With `SCARA_ROBOT_TOKEN` set on the Jetson, this checks the link without moving:

   ```bash
   curl -fsS -H "X-SCARA-Token: $SCARA_ROBOT_TOKEN" http://scara.local/status
   ```

   The serial console at 115200 prints one boot block (build time, commissioned flag,
   axis bounds, SSID and token length), then reports every Wi-Fi state change,
   each new address, and a `Link:` line with RSSI and free heap every 30 s. Radio
   power-save is disabled because modem sleep adds over a second of latency and makes
   short HTTP timeouts fail. If no address appears, the stated reason names the fault:
   `SSID not found`, `authentication failed` (wrong password, or a WPA1-only access
   point rejecting the default WPA2 threshold), or repeated `connection lost`
   (supply or signal).
3. With drive power isolated, remove the bit, and keep the spindle physically
   disconnected. Before the first powered test, have the operator verify the hardwired
   stop removes drive power; a relay click, LED or software state is not that test.
   Copy `machine_config.example.h` to `machine_config.h`. Verify **Z, J1 and J2**
   direction levels and switch actuation before setting `MOTION_COMMISSIONED = true`.
   For Z, set `bounds.z` to the measured travel, keep `seek/release/backoff` at zero,
   and confirm positive API steps raise the tool. Test upward travel in small jogs
   because there is no upper switch. Test downward travel only where the bottom switch
   is known to stop it. Verify **J1 and J2** allowable ranges and seek/release/backoff
   budgets from known starting poses. A step budget limits commanded travel; it does
   not prevent a hit if a stop is closer than that budget. Verify the full J1 → J2
   homing sweep and park path clear the fixture and cables. If J2 moves unevenly,
   isolate power and investigate binding, the belt, wiring, and driver current before
   extending the seek or repeating a powered test. The step count describes pulses
   sent, not distance actually moved. An open-loop controller cannot detect missed
   steps; a stop/fault requires reset and re-home.
   With the spindle still unplugged, test `/drill` or the **Drill** button: it should
   switch the relay only while idle, and drop on Stop, Reset, a new motion command,
   Wi-Fi loss, or heartbeat expiry. There is no drill interlock in firmware.
4. Run with `SCARA_ROBOT_TOKEN` set locally:

   ```bash
   ./start.sh --serve --robot-url http://scara.local --park-steps 0 0
   ```

   Supply `0 0` only if the backed-off home pose actually clears the camera. Otherwise
   supply a measured camera-clear J1/J2 step pair. **Start** homes J1/J2, parks,
   then scans and reviews coordinates with no target motion if calibration is absent.
   This is **Mode B**. Leave `--enable-hardware-dry-run` off. The default paper
   preset includes the six nominal targets; no separate reference CSV is required
   for the matching printed fixture.

### Manual setup and pose logging

Open **Manual control** (`/manual`) from the robot dashboard. It keeps the
live camera visible and does not run a scan. To start on this page and disable
the vision workflow, launch:

```bash
./start.sh --serve --robot-url http://scara.local --manual-only
```

Manual controls do not require a park pose, tags, a reference CSV or calibration.
Use **Home J1/J2** to establish the joint step reference for normal jogs, or
follow the [pre-home bench procedure](command-guide.md#pre-home-bench-test).
The controller must be commissioned for motion and have clear limits for J1/J2.
Normal jog commands are single moves of 10, 25, or
50 steps at a fixed feed; there is no hold-to-repeat, and a move outside verified
firmware step bounds is rejected rather than clipped. The limits do not detect
obstacles. **Return to park** moves J1, then J2; verify the entire path before
commanding it. A manual move or changing park clears any pending plan.

**Z − / Z +** sends an unreferenced absolute step target within `bounds.z`. It does
not require homing. **Z +** may lift off a pressed bottom switch; **Z −** is disabled
while the bottom switch is pressed. There is no upper switch, so verify the upward
direction and bound first. A reboot resets `pos.z` to zero without moving the tool.

**Drill** toggles GPIO 23 while the controller is idle. The Jetson renews a 2.5 s
drill lease; the relay drops on Stop, Reset, fault, link loss, heartbeat expiry, or
any new home/move/nudge command. There is no interlock or spindle feedback. Confirm
the tool is clear before toggling, and treat the UI state as the commanded relay
state only.

**Save as park (session)** does not persist across dashboard restarts. Copy the
displayed J1/J2 pair into `--park-steps J1 J2` after confirming camera clearance
and the approach path. The live readout shows commanded steps, not encoder
feedback, and a clear bottom switch does not verify Z height.

**Record pose** saves the current idle, homed step pair from the controller.
Add a label and optional measured fixture XY for observations; both coordinates
are required for a calibration **fit** or **check** row. Records are written to
`runs/poses.jsonl` by default (`--runs-dir` overrides the directory). The CSV
download includes only the current mode (simulator or physical).

### Diagnose a home attempt

Both **Home only** and **Start** save a trace for the home command, including
attempts that end in FAULT. Download **Homing CSV** and **Summary JSON** from
the Robot panel. Older attempts stay under `runs/homing/home-*/`
(`--runs-dir` overrides `runs`). Each row records elapsed time, controller
boot/command IDs, J1/J2 commanded step counts, switch inputs (0 pressed,
1 clear), and the reported state, phase and fault. Sampling happens during the
normal controller poll; it can miss fast transitions and switch bounce.

If the live camera was available, **Before photo** and **After photo** are
saved with the trace. They are ordinary camera frames, not a measurement of
joint angles, tool XY, or Z clearance. Compare the stationary endpoints with
the commanded counts when looking for lost motion. The existing fixture scan
provides XY coordinates for PCB targets only.

## Calibrate after the link is up

Start with the spindle isolated and a safe pointed reference. Measure known
joint-step pairs against tip positions **in the fixture top plane**; don't move
horizontally with the tip touching the surface. A raised tip, or PCB top above the
tag plane, biases the oblique camera. Use measured coplanar samples and retain
independent check points; the calibration workflow does not sample or operate Z.
The plan fits a **local affine** model and rejects extrapolation beyond the
calibration hull. This can fail over a wide workspace; it is not a global SCARA
kinematic solution. Record:

```json
{
  "version": 1,
  "frame": "vision_bed",
  "fit_samples": [],
  "check_samples": []
}
```

Replace the empty arrays with rows `[j1_steps, j2_steps, x_mm, y_mm]`, at least six
distinct fit points and three distinct in-region check points. Do not use fabricated
samples on hardware. The dashboard shows requested XY, nearest achievable XY,
rounding delta, and largest held-out residual. The sum is gated by the placement
budget (default 0.5 mm); it is **not** proof of real placement accuracy or clearance.
`plan.json` is saved beside the approved scan and `dry_run.json` links it to the
verification capture on success.
If measured checks pass and the entire path is clear at the current Z, add
`--calibration /path/to/measured.json --enable-hardware-dry-run`. Use a nominal
fixture/reference CSV. Execution does not command Z; verify the tool height and
clearance before approving. It moves one rotary axis at a time and
requires explicit approval of the current scan. Verification checks the visible
targets again; it does not check drilled holes.
