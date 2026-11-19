# SCARA Mimicry

Mimicry runs separately from the PCB dashboard and is **simulator-only unless**
you pass `--robot-url`. Stop the PCB dashboard with `./stop.sh` before launching.
Use Ctrl-C to stop Mimicry.

```bash
./startmimicry.sh --install-model  # one-time; isolated MediaPipe install and verified model
./startmimicry.sh                 # live USB camera, simulated controller
./startmimicry.sh --no-camera     # simulator: arrows jog, release centers, Space is a fist
```

Open `http://127.0.0.1:8081/` in a current Firefox/Chromium browser. For SSH,
forward `8081:127.0.0.1:8081` on your workstation. The Jetson runs hand recognition
on its USB camera feed and serves the result and preview. It doesn't upload frames
to an external service. Use `--camera /dev/videoX` if needed.
The dashboard shows measured **Tracking FPS** on the Jetson and **Preview FPS**
in the browser, plus inference time and the last frame's age. They can differ:
the camera and inference run independently of the SSH connection. Mimicry
processes the newest available frame rather than queuing old frames.

1. With Z on the bottom switch, select **Set Z Zero**. The simulated Z starts at
   the switch. The real controller only accepts zero if it is idle, the drill
   relay is off, and it reads a stable pressed switch. This button does not move
   the arm.
2. Select **Set Joystick Center**, then hold one open palm steady where you want
   the center. Select **Arm Gestures**, then hold your open palm inside the box.
   Each button gives you up to eight seconds to get into position. In keyboard
   mode, release all keys to center.
3. Move your open palm **up** until Z clears the switch. Center, then select
   **Disarm**. Select **Home J1 + J2** (this explicit setup action moves both
   rotary joints), then arm again.
4. Hold your palm left/right for J1 or up/down for Z. Only the dominant
   direction moves; **gestures never move J2**. If J1 is backwards for your view,
   relaunch with `--reverse-j1`. Neutral stops issuing jogs without disarming.
   A fist, tracking loss, or leaving the tab disarms. Center and select **Arm
   Gestures** again to resume.

## Hardware opt-in

Keep the spindle physically disconnected, operate the camera from outside the
arm's reach, and verify the hardwired emergency stop before powered tests.
The Network E-stop is not a substitute for it. The board-facing camera may need
to be repositioned or replaced with an operator-facing camera.

1. Verify the actual transmission, switch actuation, Z direction and usable
   upper clearance. A directly driven single-start 5/16-24 rod with a 1.8°
   motor at ¼ microstepping gives **136,062 microsteps** for 180 mm (rounded
   down). Use a lower, verified bound if the fixture or carriage needs it.
   Verify that the bottom contact clears within the first 400 upward
   microsteps; otherwise the controller faults instead of allowing
   continued upward travel with an active switch.
2. Follow the [controller commissioning guide](robot-contract.md). In your local
   `ESP32-Motor-Control/machine_config.h`, set `SCARA_MIMIC_Z_MAX_STEPS` to
   the verified upper bound, then compile and flash the updated sketch. This
   option is disabled by default and does not override `MOTION_COMMISSIONED`.
3. Set `SCARA_ROBOT_TOKEN` locally and run:

   ```bash
   ./startmimicry.sh --robot-url http://scara.local
   ```

4. Use your commissioned setup procedure to bring Z to the bottom switch, then
   use **Set Z Zero**. Repeat after reboot, reset, or a latched motion fault.
   If the controller loses its reference after zeroing, bring Z back to the
   switch before jogging again. The cap is based on commanded steps, not
   encoder feedback; lost steps can invalidate the physical position.

Motion gets one jog chunk at a time (up to 8 J1 or 30 Z microsteps). An already
accepted chunk can finish after tracking stops. Firmware gives each accepted jog
a non-renewable 250 ms deadline and never queues the next one. Verify stopping
distance, direction, and the switch at low speed before routine use.
