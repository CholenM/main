**Deliverable 1: ESP32-CAM wiring into SCARA circuit**

**Function & Purpose:** Add the ESP32-CAM as the on-arm/overhead vision source wired into the existing SCARA circuit (power \+ serial data), so the camera streams to the vision host and the arm can be commanded in the same loop.

**Expected Output:** Stable MJPEG stream @ usable FPS; no brownout/reset on motor motion; common ground verified.

**Deliverable 2: ProtoBoard to Etched PCB**

**Function & Purpose:** Move the SCARA control circuit off the protoboard onto a reliable etched PCB so the arm survives transport/demo without intermittent faults.

**Expected Output:** Single etched PCB, labeled, fused, bench-tested; arm runs full homing \+ motion sequence without faults.

**Deliverable 3: Basic functionality remotely**

**Function & Purpose:** Confirm the SCARA arm performs its basic functions (home, move to XYZ, spindle on/off) when commanded remotely from the vision host over the new PCB \+ ESP32-CAM link.

**Expected Output:** Remote command round-trip verified; all basic moves repeatable

---

**Task:**

- [ ] 1\. Power budget   
- [ ] 2\. ESP32-CAM wiring and stream test  
- [ ] 3\. Schematic for etched PCB  
- [ ] 4\. PCB layout \+ etch \+ solder  
- [ ] 5\. Bench \+ functional test on new PCB  
- [ ] 6\. Document pinout/schematic  
- [ ] 7\. Remote homing \+ move demo

---

**Deliverable 4: Discrete Hand-Gesture Command Control**

**Function & Purpose:** Hands-free command of the arm via discrete gestures. **No electromagnet available**, so gesture set is limited to non-grasp motions (home, move-to-pointed, drill, retract, estop). Enables operator command without keyboard/mouse.

**Expected Output:** 4–6 gestures driving the arm

---

**Gesture set (non-grasp command map):**

| \# | Gesture | Hold time | Command |
| :---- | :---- | :---- | :---- |
| G1 | Open palm (held) | 0.5 s to engage | Enable hand tracking — SCARA mimics hand position in real time |
| G2 | Fist | 0.3 s | Stop / hold position |
| G3 | Thumbs up | 0.5 s | Spindle ON |
| G4 | Thumbs down | 0.5 s | Spindle OFF \+ retract Z |

---

**Deliverable 5: Vision-Guided PCB Drilling**

**Function & Purpose:** Use the SCARA arm with a rotary drill end-effector to drill holes in a PCB. SVLM/YOLO reads drill-point callouts (coordinates, diameters) from a board file/drawing; arm drills each point. Automates PCB prototyping with document understanding.

**Expected Output:** Accurate holes at SVLM-extracted drill points; \~±0.5 mm placement; demo of reading a board file and drilling a small PCB; safety checklist signed.

---

**Things we need:**

- [ ] **HARDWARE**  
  - [ ] Drill \+ Printed Mount  
  - [ ] ArUco Bed with Clamps and Fixture to hold PCB inside the tagged region  
  - [ ] ESP32-CAM  
- [ ] **Materials**  
  - [ ] Test PCBs  
  - [ ] Board Drawing (Preferably PNG) with drill-poit callouts

**Task:**

- [ ] **Prepare test PCB**  
    
  - [ ] **Choose test board:** single-sided, 50×50 mm, 4–6 holes across 2 diameters   
  - [ ] **Create board drawing (PNG):** the engineering drawing with drill-point callouts (X, Y, diameter in mm) plus overall board dimensions.  
  - [ ] **Ground-truth CSV:** ID, x\_mm, y\_mm, diameter\_mm (the answer key to validate CV extraction accuracy.)  
  - [ ] **Build the ArUco bed (the physical Cartesian plane)**


- [ ] **Home & retract (safe start)**  
    
  - [ ] SCARA homes all joints to zero.  
  - [ ] Retract Z to safe height; spindle OFF; confirm clear of bed.


- [ ] **Scan & snapshot**  
    
  - [ ] Camera captures a single snapshot of the clamped board.  
  - [ ] Save snapshot with timestamp.

- [ ] **CV marks holes on snapshot (hybrid: VLM/YOLO)**  
    
  - [ ] Run YOLO (MVP) / SVLM (stretch) on snapshot to detect drill points \+ fiducials \+ ArUco tags.  
  - [ ] SVLM reads drill-point callouts from the board file/drawing (X, Y, diameter) and cross-checks them against detected holes.  
  - [ ] Overlay detected holes \+ callout coords on snapshot; human review step (approve before proceeding).


- [ ] **Build board coordinates (digital frame, via ArUco plane)**  
    
  - [ ] Compute per-run ArUco plane (origin/axes/scale) from detected tags.  
  - [ ] Convert each detected hole center (px) → (x\_mm, y\_mm) in the digital cartesian plane.  
  - [ ] Validate against ground-truth CSV (target: 100% count, ≤0.5 mm coord error).  
  - [ ] Export coordinate list (CSV/JSON) that feeds the executor.


- [ ] **Mirror digital to the Physical frame**  
    
  - [ ] Apply homography (camera/digital frame → robot base frame) using fiducials.  
  - [ ] Verify mirror alignment: digital (0,0) maps to physical origin on bed; check a known point.  
  - [ ] Re-check fiducials before each hole to catch board shift.


- [ ] **Drill sequence state machine**  
    
  - [ ] States: IDLE then MOVE TO POINT then SPINDLE ON then Z DOWN (drill) then Z UP then SPINDLE OFF then NEXT (or DONE).  
  - [ ] Logging: per-hole timestamp, position, spindle state.


- [ ] **Dry run (spindle OFF)**  
    
  - [ ] Run the full sequence on the test PCB with the spindle disabled.  
  - [ ] Verify tool tip touches each marked point (paper/pen trace).  
  - [ ] Confirm no collisions; adjust offsets.


- [ ] **Live drill \+ accuracy**  
    
  - [ ] Live drill on 1 test PCB; inspect holes (diameter, position).  
  - [ ] Drill 10 holes across 2–3 boards; measure placement error vs. ground truth.  
  - [ ] Compute mean error \+ std dev; target ±0.5 mm.  
  - [ ] Document failures and mitigations.

