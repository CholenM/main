# SCARA command guide

Run Jetson commands in `~/main`. The dashboard binds to loopback; do not expose
it directly to the network. The spindle remains off and Z is fixed.

## 1. Open the dashboard

On the Jetson:

```bash
cd ~/main
./start.sh
```

On the workstation, keep this SSH tunnel open:

```bash
ssh -N -L 8080:127.0.0.1:8080 scara@10.42.0.122
```

Open `http://localhost:8080/`. Use **Connect ESP32**, then enter its private
IPv4 address and the token from `secrets.h`. The token stays in the dashboard
process for this session. Connecting reads status without moving the arm.

- **Manual control** (`/manual`): live camera, J1/J2 controls, Home, Reset,
  Network stop, homing traces, and pose recording. No tags, calibration, or park
  pose required. This build locks physical Z and holds the drill relay on
  GPIO 23 off. Virtual Z/drill controls are available on the simulator only.
- **Vision workflow** (`/`): with a verified park pose, Start homes, parks,
  and scans. The default launch does not allow physical target execution.
- **Run simulation** (`/sim/`): choose **Built-in fixture** or **Live camera**.
  Click **Start**, then **Approve & execute dry run** after checking the
  coordinate plan. The simulator accepts joint step commands, reports
  completion, and plots the confirmed target visits. Download the visit log
  with the scan-derived XY, commanded and confirmed steps, and command IDs.
  This verifies the software pipeline, not physical positioning or drilling.
- **Simulation → Manual control** (`/sim/manual`): virtual Z steps and a
  virtual drill toggle, plus the simulated J1/J2 motor controls.

## 2. Pre-home bench test

Normal jogs have a position reference after **Home J1/J2**. Pre-home nudges
are different: they move relative to an unknown position and do not set home.
They are disabled in the supplied firmware profile.

Before enabling this test, an on-site operator must verify that the hardwired
E-stop removes drive power, remove the drill bit, physically disconnect the
spindle, and mechanically secure Z. With drive power isolated, verify switch
wiring and mark a known test pose that has clearance in **both** directions on
both joints for the full pulse budget, including clearance from cables and the
fixture. Do not power-test a joint that is still binding or stalling.

On the flashing workstation, once those checks pass and the measured machine
profile is commissioned, set `SCARA_ENABLE_PREHOME_NUDGE` to `1` in
`ESP32-Motor-Control/machine_config.h`. Flash the updated sketch by opening
`ESP32-Motor-Control.ino` (ESP32 Dev Module, ArduinoJson 7). Check status before
trying a move:

```bash
read -rsp 'ESP32 token: ' SCARA_ROBOT_TOKEN
echo
curl -fsS -H "X-SCARA-Token: $SCARA_ROBOT_TOKEN" \
  http://10.42.0.16/status | python3 -m json.tool
unset SCARA_ROBOT_TOKEN
```

Look for `commissioned: true` and `prehome_nudge.enabled: true`. From the marked
test pose, each J1/J2 click while unhomed sends one 10-step nudge after you confirm
clearance. The controller allows **30 commanded steps total per boot** across
both joints, even if you reverse direction. Reset does not restore the budget.
Do not power-cycle to extend a test from an unknown pose. A clear switch is not
proof of mechanical clearance. Keep the hardwired stop accessible.

## 3. Other commands

Run one dashboard at a time. Stop it only after confirming the arm is idle.
The table lists alternative CLI commands for testing and maintenance; they are
not required for the default UI workflow.

| Task | Jetson command |
|---|---|
| Stop the dashboard | `./stop.sh` |
| Check camera and fixture with dashboard stopped | `./start.sh --self-test --camera /dev/video0` |
| Vision-only live dashboard | `./start.sh --serve --camera /dev/video0` |
| Simulated robot with saved fixture image | `./start.sh --serve --simulate-robot --replay runs/fixture-235-replay.png` |
| Simulated robot with live camera | `./start.sh --serve --simulate-robot` |
| List CLI flags | `./start.sh --help` |
| Run tests | `python3 -m unittest discover -s tests -v` |

For Mode B, set `PARK_J1` and `PARK_J2` to the verified, camera-clear joint
positions in steps and run:

```bash
./start.sh --serve --robot-url http://10.42.0.16 --park-steps "$PARK_J1" "$PARK_J2"
```

Use `0 0` only if the backed-off home pose and its approach are verified clear.
Do not pass `--enable-hardware-dry-run` for Mode B.

## 4. Controller API reference

All controller requests use the token header. Motion requests also require
the current `boot` and `command_id`. Use the dashboard for motion; it renews
the active command's heartbeat and checks completion.

| Endpoint | Purpose |
|---|---|
| `GET /status` | State, commanded joint counts, switches, and pre-home budget |
| `POST /home` | Home J1 then J2 |
| `POST /move` | Bounded absolute J1/J2 move after home |
| `POST /nudge` | One commissioned pre-home nudge |
| `POST /heartbeat` | Renew the matching active command's lease |
| `POST /stop` | Stop motion and latch HOLD |
| `POST /estop` | Latch network stop; not a substitute for the hardwired stop |
| `POST /reset` | Clear a stopped/faulted controller; leaves it unhomed |
| `POST /spindle` | Off only; enabling is rejected |

See the [controller contract](robot-contract.md) for request formats and
commissioning details.
