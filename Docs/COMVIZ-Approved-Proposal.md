**Project Name:**		Computer Vision for SCARA Robotic Arm  
**Course:**			ROBO 4107 \- Computer Vision  
**Proponents:**			John Marion Albano  
Moises Clint Balbuena  
John Cholen Makigod   
---

# **Idea 1 \- Discrete Hand-Gesture Command Control**

* **Function & Purpose:** Hands-free control of the arm via discrete gestures \- e.g., open palm \= home, fist \= pick, thumbs-up \= place, point \= move-to-pointed location. Enables an operator to command the robot without a keyboard/mouse.  
* **CV Technique:** MediaPipe Hands \-\> 21 3D landmarks \-\> gesture classifier. Start rule-based (finger curl counts, thumb position), upgrade to a small MLP on landmark features if needed.  
* **System Integration:**  
  * Camera \-\> MediaPipe hand landmarks (real-time, CPU).  
  * Gesture label \-\> command map \-\> target pose.  
  * IK \-\> serial command to ESP32 \-\> motion.  
  * State machine guards against repeated triggers (debounce).  
* **Expected Output:** Hands-free command control; \~10-15 FPS on CPU; demo of 4-6 gestures driving the arm.  
* **Feasibility:** High. Pick/place gestures require an electromagnet.  
* **Risks:** Lighting/background, hand size variation, false triggers. Mitigate with ROI, gesture hold-time threshold, confidence gating.

# **Idea 2 \- Vision-Guided PCB Drilling** 

* **Function & Purpose:** Use the SCARA arm with a rotary drill end-effector to drill holes in PCBs. The SVLM reads drill-point callouts (coordinates, hole diameters) from a board file/drawing and the arm drills each point \- automating PCB prototyping with document understanding.  
* **CV Technique:** **YOLO/SVLM reads drill-point callouts from a board file/drawing** (coordinates \+ hole sizes) \-\> homography maps image coords to robot base frame; YOLO (MVP) for fiducial/point detection \+ board alignment; SVLM (stretch) for callout understanding.  
* **System Integration:**  
  * Board file/drawing fed to SVLM \-\> extracts drill-point list (X, Y, diameter).  
  * PCB fixed on bed \-\> overhead camera aligns board via fiducials (target tracking, Idea 3, keeps it aligned).  
  * For each drill point: IK \-\> serial command to ESP32 \-\> spindle ON \-\> Z down (drill) \-\> Z up \-\> spindle OFF.  
  * Repeat for all points.  
* **Expected Output:** Accurate drilled holes at SVLM-extracted drill points; \~+-0.5 mm placement; demo of reading a board file and drilling a small PCB.  
* **Feasibility:** Medium. SVLM reading callouts is a stretch; YOLO for fiducial/point detection as MVP. Requires drill end-effector (rotary tool \+ mount) and safety shielding. Strong manufacturing/prototyping \+ document-understanding angle.  
* **Risks:** SVLM hallucination on callouts (validate against file); drill vibration; safety (spindle, chips); PCB clamping. Mitigate with validation, rigid mount, guards, clamps, and conservative feed rates.

