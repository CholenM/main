# SCARA controller contract and commissioning

The Jetson owns the camera, fixture registration, local calibration, planning, approval
and sequencing. The ESP32 owns pulses, switches, step budgets, target completion
and the motion lease. **This release is spindle-off only, including in firmware.**
The bench rig has a bottom Z switch. This build uses **fixed Z**: the tool axis
must be mechanically secured at a verified clearance height before rotary motion.
No USB camera, calibration, or robot is needed to try the simulator workflow.

## Labels and coordinate conventions

| API | Rig labels (motor / driver / switch) | Old code | Positive | STEP / DIR / limit GPIO |
|---|---|---|---|---|
| `z` | `Z / Z-DRV / Z-LIM` | `Z` | Fixed (bottom contact monitored) | 2 / 15 / 27 |
| `j1` | `J1 / J1-DRV / J1-LIM` | `XY` | CCW viewed from above | 16 / 17 / 26 |
| `j2` | `J2 / J2-DRV / J2-LIM` | `EL` | CCW viewed from above | 21 / 22 / 25 |

Rotary position is an integer count of commanded STEP rising edges from the backed-off
home datum, not encoder feedback. `pos.z = 0` is a placeholder, not a height measurement.
The host never chooses electrical direction
levels. Set `positiveLevel` and `homeSign` separately after checking each axis.
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
only to loopback; access it through SSH. No fallback access point or manual jog page.

| Method | Endpoint | Body | Successful response |
|---|---|---|---|
| GET | `/status` | none | `200` state, position, limits and flags |
| POST | `/home` | `{"boot":"…","command_id":16}` | `202`, async J1 → J2; no Z motion |
| POST | `/move` | `{"boot":"…","command_id":17,"axis":"j1","to":120,"feed":2000}` | `202`, bounded absolute steps |
| POST | `/heartbeat` | `{"boot":"…","command_id":17}` | `200`, extends that active motion lease |
| POST | `/spindle` | `{"on":false}` | `200`; `true` always rejected in this version |
| POST | `/stop` | `{}` | `200`, stops pulses, spindle off, latches HOLD |
| POST | `/estop` | `{}` | `200`, network stop latched, invalidates datum |
| POST | `/reset` | `{}` | `200` from HOLD/FAULT/ESTOP, returns unhomed IDLE |

POSTs use JSON. Success returns `{"boot":"…","command_id":17}`. Status includes
`fw`, `boot`, `command_id`, `state`, `homed`, `pos`, `target`, `limits`, `spindle`,
`fault`, `commissioned`, and `simulator`, plus `z_mode: "fixed"` and informational
`rssi` (dBm) and `ip` on hardware. Axis keys are `z`, `j1`, `j2`; limit `0`
means active-low contact and `1` means clear. Non-2xx responses include `error`.
`feed` is a half-period in microseconds (1,000–20,000; scheduler may make it longer).
Homing seeks, releases, re-latches slowly, then steps off the switch. It never goes
to an unknown hard stop to measure travel. Each phase is limited by the local profile.
Normal movement requires rotary homing and bounded joint targets. All Z moves
are rejected, including a target of zero. A pressed Z bottom switch rejects a
new motion command or latches `Z_BOTTOM_LIMIT` during active motion. A clear
switch does **not** establish Z clearance or detect a disconnected switch.

Motion POSTs must match the current boot and last command ID before the controller
accepts them. The Jetson never retries an ambiguous motion POST; it accepts completion only when
boot ID, command ID, state, datum and target agree. Read-only status does **not** renew
the lease. An active command stops on disconnection or two seconds without a matching
heartbeat; re-home after reset. **No automatic retract on link loss in this release.**
That can be enabled in a later spindle integration only after a separate Z-clearance
and fault validation. Neither the network stop nor firmware replaces a hardwired
emergency stop that removes hazardous energy.

## Try the UI and commission in order

**Simulator:** `./start.sh --serve --simulate-robot` uses the live webcam; add
`--replay <image>` to use a known-good fixture capture. Click **Start**, inspect the
proposed/rounded positions at **Review**, then **Approve dry run**. It returns to
park and compares a second scan. This proves sequencing, not machining.

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
3. With drive power isolated, remove the bit, keep the spindle physically
   disconnected, and have the bench operator mechanically secure Z at a verified
   clearance height. Do not rely on motor holding torque or the bottom switch.
   Before the first powered test, have the operator verify the hardwired stop
   removes drive power; a relay click, LED or software state is not that test.
   Copy `machine_config.example.h` to `machine_config.h`; leave all Z motion
   fields at zero. Verify **J1 and J2** direction levels, switch actuation,
   allowable ranges and seek/release/backoff budgets from known starting poses
   before setting `MOTION_COMMISSIONED = true`. A step budget limits commanded
   travel; it does not prevent a hit if a stop is closer than that budget.
   Verify the full J1 → J2 homing sweep and park path clear the fixture and cables.
   An open-loop controller cannot detect missed steps; a stop/fault requires reset
   and re-home.
4. Run with `SCARA_ROBOT_TOKEN` set locally:

   ```bash
   ./start.sh --serve --robot-url http://scara.local --park-steps 0 0
   ```

   Supply `0 0` only if the backed-off home pose actually clears the camera. Otherwise
   supply a measured camera-clear J1/J2 step pair. **Start** homes J1/J2, parks,
   then scans and reviews coordinates with no target motion if calibration is absent.

## Calibrate after the link is up

Start with the spindle isolated and a safe pointed reference. Measure known
joint-step pairs against tip positions **in the fixture top plane**; don't move
horizontally with the tip touching the surface. A raised tip, or PCB top above the
tag plane, biases the oblique camera. Use measured coplanar samples and retain
independent check points; the dashboard does not automatically sample or operate Z.
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
If measured checks pass and the entire path is clear at safe Z, add
`--calibration /path/to/measured.json --enable-hardware-dry-run`. Use a nominal
fixture/reference CSV. Execution stays at safe Z, one rotary axis at a time, and
requires explicit approval of the current scan. Verification checks the visible
targets again; it does not check drilled holes.
