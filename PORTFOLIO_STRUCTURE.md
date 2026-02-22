# Portfolio Structure — Parsa Ghasemi
# paarseus.github.io

Generated: 2026-02-21
Purpose: Blueprint for building out index.html project entries

---

## Page Layout

```
[Header]
  Name: Parsa Ghasemi
  Title: R&D Vice President, Autonomous Vehicle Laboratory · Cal Poly Pomona
  Tagline: "Only undergraduate in an executive role at a DoE-funded graduate research lab"
  Links: Email | CV | LinkedIn | GitHub | Google Scholar

[Research & Publications]
  — 3 entries (ASEE, SCAIR, SCCUR)

[Projects]
  — 5 flagship entries (full row with thumbnail + hover video)
  — 2 supporting entries (smaller, text-only or single image)
```

---

## Research & Publications Section

### Paper 1
- **Title:** DRIVE: Development Research Infrastructure for Vehicle Education
- **Venue:** ASEE Zone IV Conference 2026
- **Links:** PDF | Project (→ Entry 1)
- **Note:** Documents full AV platform, $30K vs $200K+ commercial cost

### Paper 2
- **Title:** Simplified Multimodal Imitation Learning for Day–Night Autonomous Driving
- **Venue:** Southern California AI and Robotics Symposium (SCAIR) 2025 — Poster
- **Links:** PDF | Project (→ Entry 4)
- **Note:** PDF already in repo at resume/Simplified-Multimodal-Imitation-Learning-...pdf

### Paper 3
- **Title:** Defense-in-Depth Security Architecture for CAN-Based AV Control Systems
- **Venue:** Southern California Conference for Undergraduate Research (SCCUR) 2025 — Oral Presentation
- **Links:** Slides (if available) | Project (→ Entry 2)
- **Note:** Presented to 500+ attendees, CSU Channel Islands

---

## Projects Section

---

### FLAGSHIP ENTRY 1 — THE ANCHOR
**Section ID:** `drive-platform`
**Title:** DRIVE: Full-Stack Autonomous Vehicle Research Platform
**Repos:** AVROS + av-control-system + AVCONSOLE + AVPCB
**GitHub:** https://github.com/Paarseus/AVROS
**Paper:** ASEE Zone IV 2026

**Thumbnail image:** GPS trail overlaid on satellite map of Cal Poly Pomona campus
**Hover media:** Video of vehicle driving autonomously (field test footage)
**Highlight row:** Yes (bgcolor="#ffffd0" like Jon Barron style)

**Description (2-3 sentences for index.html):**
A complete autonomous vehicle research platform built on a modified Polaris GEM e2,
spanning 19,000+ LOC from bare-metal CAN bus firmware to ROS2 Nav2 navigation to a
real-time mobile WebUI — validated over 50+ hours of field testing with 2,581 telemetry
rows and 784 GPS trail points. The $30K platform achieves sub-centimeter RTK positioning
via NTRIP corrections and integrates Velodyne VLP-16 LIDAR, Intel RealSense RGB-D, and
Xsens MTi-680G IMU with a 6-package ROS2 workspace routing OSMnx campus maps through
Nav2 to 4 distributed Teensy MCUs over CAN at 250 kbps.

**Key metrics to bold:**
- 19,000+ LOC
- 4 MCUs @ 250 kbps CAN
- 100 Hz control loops
- 50+ hours field tested
- 2,581 telemetry rows
- 784 GPS trail points
- Sub-centimeter RTK positioning
- $30K vs $200K+ commercial

**Project page sections (when you build individual pages):**
1. System architecture diagram (full stack: sensors → AVROS → CAN → actuators)
2. Sensor suite (Velodyne + RealSense + Xsens layout diagram)
3. GPS trail visualization + telemetry plots
4. ROS2 package architecture (6 packages, launch file structure)
5. Field testing photos/video
6. ASEE Zone IV paper link
7. Nav2 config highlights (SmacPlannerHybrid, Regulated Pure Pursuit, costmap)

---

### FLAGSHIP ENTRY 2 — EMBEDDED DEPTH
**Section ID:** `can-bus-system`
**Title:** Distributed Drive-by-Wire: 4-MCU CAN Bus Control System
**Repo:** https://github.com/Paarseus/av-control-system
**Paper:** SCCUR 2025 (CAN Cybersecurity)

**Thumbnail image:** CAN bus network topology diagram (Master → Brake/Throttle/Steer/XSENS nodes)
**Hover media:** Oscilloscope trace of CAN traffic OR E-STOP sequence video
**Highlight row:** No

**Description (2-3 sentences for index.html):**
Engineered a safety-critical distributed embedded system coordinating 4 Teensy 4.1
microcontrollers (ARM Cortex-M7 @ 600 MHz) over CAN bus at 250 kbps with 100 Hz
deterministic control loops for brake, throttle, and steering of a drive-by-wire vehicle.
The throttle node required reverse-engineering the vehicle's undocumented ECU by decoding
the 4-channel DAC voltage table (MCP4728 over I2C) that encodes PRND transmission states,
with a safe mode-sequencing protocol to prevent transmission shock. A 4-layer E-STOP
architecture propagates emergency stops across all nodes simultaneously with 100ms watchdog
failsafe — zero safety incidents over 50+ hours of field operation.

**Key metrics to bold:**
- 4 Teensy 4.1 MCUs (ARM Cortex-M7 @ 600 MHz)
- 250 kbps CAN bus
- 100 Hz deterministic loops
- 100ms watchdog timeout
- 4,900+ LOC C++
- Zero safety incidents over 50+ hours

**Project page sections:**
1. CAN message protocol table (0x100 Throttle, 0x200 Steer, 0x300 Brake + status IDs)
2. Node architecture diagram (Master broadcasts, each node autonomous)
3. Throttle DAC reverse-engineering story (the undocumented voltage table)
4. Brake closed-loop position control (ADC feedback, deadband, watchdog)
5. E-STOP state machine diagram
6. SCCUR cybersecurity presentation link

---

### FLAGSHIP ENTRY 3 — UNIQUE DIFFERENTIATOR
**Section ID:** `simchair-ops`
**Title:** Sim Chair Teleoperation & Remote Operations Infrastructure
**Repos:** av-control-system/Simchair + AVCONSOLE/webui_simchair + wireguard
**GitHub:** https://github.com/Paarseus/AVCONSOLE

**Thumbnail image:** Photo of sim chair setup with vehicle visible OR architecture diagram
**Hover media:** Video of operator driving vehicle from sim chair with live camera feed on screen
**Highlight row:** Yes (bgcolor="#ffffd0")

**Description (2-3 sentences for index.html):**
Built a hardware-in-the-loop teleoperation system that connects a physical racing simulator
(steering wheel, pedals, paddle shifters) to the real autonomous vehicle over a Tailscale
VPN mesh — enabling remote vehicle operation from anywhere in the world at 20 Hz with
automatic priority switching and 500ms fallback to phone joystick control. The operator
receives a live 1280×720 WebRTC camera stream from the vehicle's Intel RealSense via
aiortc while the sim chair's pygame inputs (steering axis 0, throttle axis 6, brake axis 1)
are normalized and forwarded over UDP to the Teensy CAN master, with voice control
(OpenAI Whisper on-device, int8 quantized) for E-STOP and mode selection.

**Key metrics to bold:**
- 20 Hz control loop
- 500ms priority fallback
- 1280×720 @ 30fps WebRTC video
- Remote operation over Tailscale VPN
- On-device Whisper ASR (int8 quantized)
- Simchair PC: 100.115.20.7 | Jetson: 100.93.121.3 (Tailscale mesh)

**Architecture diagram to include:**
```
[Sim Chair PC] ──pygame──► simchair_sender.py ──UDP:5006──► server.py (Jetson)
[Phone]        ──WebSocket:8000──────────────────────────► server.py (Jetson)
                                                                │
                                                        Priority Arbitration
                                                         (simchair > phone)
                                                                │
                                                         ──UDP:5005──► Teensy Master
                                                                │       ──CAN──► Brake/Throttle/Steer
                                                         WebRTC video ◄── RealSense D455
```

**Project page sections:**
1. Architecture diagram (above)
2. Priority control system explanation
3. Tailscale VPN mesh diagram (IP map)
4. Sim chair hardware photos
5. WebRTC streaming pipeline
6. Voice command table

---

### FLAGSHIP ENTRY 4 — ML/RESEARCH TRACK
**Section ID:** `imitation-learning`
**Title:** Multimodal Imitation Learning for Illumination-Robust Autonomous Driving
**Repo:** (internal / private)
**Paper:** SCAIR 2025 Poster

**Thumbnail image:** Side-by-side RGB vs thermal camera frame at night (visually striking)
**Hover media:** Training loss curve animation OR day/night comparison video
**Highlight row:** No

**Description (2-3 sentences for index.html):**
Trained an imitation learning policy on 50,000+ synchronized frames of RGB camera (30 Hz),
thermal camera (30 Hz), and real CAN bus telemetry (20 Hz) collected on the DRIVE platform
to learn illumination-robust autonomous driving behavior. The model generalizes from daytime
demonstrations to nighttime operation — validated on a 0.3-mile campus route achieving only
3 manual interventions, an order of magnitude below naive RGB-only baselines. Presented as a
poster at the Southern California AI and Robotics Symposium (SCAIR) 2025.

**Key metrics to bold:**
- 50,000+ training frames
- 3 manual interventions over 0.3-mile route
- RGB @ 30 Hz + Thermal @ 30 Hz + CAN @ 20 Hz
- SCAIR 2025 poster presentation

**Project page sections:**
1. Data collection pipeline diagram (dual camera sync + CAN logging)
2. Day/night visual comparison (RGB vs thermal)
3. Model architecture diagram
4. Training curve
5. Results table (RGB-only vs multimodal interventions)
6. Linked SCAIR poster PDF
7. Connection to DRIVE platform (data collected on Entry 1)

---

### FLAGSHIP ENTRY 5 — RL/SIMULATION TRACK
**Section ID:** `carla-rl`
**Title:** CARLA Gymnasium Environment for Autonomous Driving RL
**Repo:** https://github.com/Paarseus/carla-rl-suite
**Co-author:** Alexander Assal

**Thumbnail image:** CARLA ego-vehicle top-down BEV OR training reward curve
**Hover media:** CARLA simulation video with ego vehicle navigating
**Highlight row:** No

**Description (2-3 sentences for index.html):**
Designed a production-grade Gymnasium environment wrapping the CARLA autonomous driving
simulator, achieving 100 steps/sec training throughput with under 10ms multi-sensor latency
across 10,000 episode runs with zero memory leaks. The 7-manager architecture (Connection,
Hero, Sensor, Traffic, Observation, Action, Reward) uses a factory pattern for sensors and
a dual-queue system separating continuous (camera/LIDAR) from event (collision/lane
invasion) data to ensure atomic per-tick collection. A PPO agent achieves stable lane
following within 50,000 training steps using a 6-component reward function (speed,
lane-keeping, collision, progress, smoothness, alignment) compatible with PPO, SAC, and TD3.

**Key metrics to bold:**
- 100 steps/sec training throughput
- <10ms multi-sensor latency
- 0 memory leaks over 10,000 episodes
- Stable lane-following in 50,000 PPO steps
- PPO / SAC / TD3 compatible
- 7-module manager architecture

**Project page sections:**
1. System architecture diagram (7 managers and their interfaces)
2. Observation space breakdown (10D state + images)
3. Reward function component table (6 components + weights)
4. Training reward curve showing convergence
5. Sensor factory pattern explanation
6. Future work: sim-to-real transfer connection to DRIVE platform

---

### SUPPORTING ENTRY 6
**Section ID:** `legged-robotics`
**Title:** Legged Robotics Lab — 501(c)(3) Nonprofit & K-Bot Arm Control
**Repos:** https://github.com/Paarseus/steveros + robot_arm_control + Quadruped-Robot
**Format:** Smaller entry, single image, 2 sentences max

**Thumbnail image:** Robot arm OR quadruped platform photo
**Hover media:** Arm moving / quadruped walking clip

**Description:**
Founded a 501(c)(3) legged robotics nonprofit at Cal Poly Pomona, growing to 30+ members
with $10K+ in funding, building quadruped and bipedal platforms with forward/inverse
kinematics for walking gait development. Developed steveros, a ros2_control hardware
interface plugin (SystemInterface) for a 5-DOF Robstride brushless motor arm using the
MIT mini-cheetah CAN motor protocol (position/velocity/torque control per joint).

**Key metrics:**
- 501(c)(3) nonprofit
- 30+ members
- $10K+ funding
- ros2_control hardware interface plugin
- MIT motor protocol over CAN

---

### SUPPORTING ENTRY 7
**Section ID:** `avpcb`
**Title:** AVPCB — Custom Electronics Integration Board for AV Platform
**Repo:** https://github.com/Paarseus/AVPCB
**Format:** Smaller entry, single KiCad 3D render image, 1-2 sentences

**Thumbnail image:** KiCad PCB 3D render (export from KiCad: View → 3D Viewer → export)
**Hover media:** Schematic screenshot or component layout

**Description:**
Designing a custom KiCad 9.0 PCB to consolidate the AV platform's electronics: SN65HVD230
CAN transceivers, TXB0104D 3.3V↔5V level shifters, Traco 30W industrial DC-DC converters
(18–75V input for vehicle battery), and BTN8982 half-bridge motor drivers — replacing the
current breadboard/breakout-board prototype with a production-grade integrated board.

**Key components:**
- SN65HVD230 CAN transceiver (TI, 1 Mbps)
- TXB0104D bidirectional level shifter (±15kV ESD)
- THL 30-4812WI Traco 30W DC-DC (18–75V in, 12V out)
- BTN8982 half-bridge motor driver (Infineon)
- TVS diode protection on all CAN lines

---

## Media Checklist (assets needed before building)

### Must Have (blocking)
- [ ] Profile photo — DONE (images/parsa.jpeg)
- [ ] GPS trail map screenshot (from field test data — 784 GPS trail points)
- [ ] Vehicle field test video (30-60 sec autonomous driving clip)
- [ ] CAN bus topology diagram (draw in draw.io or Lucidchart)

### High Priority
- [ ] Simchair photo (chair + screen setup)
- [ ] System architecture diagram (full stack, all layers)
- [ ] RGB vs thermal side-by-side comparison (night scene)
- [ ] CARLA training reward curve (screenshot from training logs)
- [ ] KiCad PCB 3D render (export from KiCad)

### Nice to Have
- [ ] Oscilloscope trace of CAN traffic
- [ ] RViz screenshot (occupancy grid + planned path)
- [ ] Telemetry plot (speed/steering over time during field test)
- [ ] Robot arm or quadruped photo/video

---

## Files to Link (already in repo)

```
resume/resume_parsa_ghasemi.pdf                          → CV link in header
resume/Simplified-Multimodal-Imitation-Learning-...pdf  → Entry 4 paper link
resume/applications/tesla_optimus_cpp_intern_2026/      → (internal, don't link)
resume/projects.bib                                     → BibTeX reference file
```

---

## index.html Edit Plan

When ready to edit index.html:

1. Replace Jon Barron bio with Parsa bio + links
2. Replace "Research" section with 3 papers above
3. Replace all paper entries with 7 project entries (5 flagship + 2 supporting)
4. Flagship entries: full row with image thumbnail + hover swap
5. Supporting entries: smaller row or grouped at bottom
6. Update <title> and <meta> tags (already done: "Parsa Ghasemi")
7. Update favicon (optional)
8. Remove Jon Barron's data/ folder .bib files (not needed)

---

## Entry Priority Order for Building

Build in this order (most impactful first):

1. Header + bio (30 min) — blocks everything else
2. Entry 1: DRIVE Platform (need GPS trail image + video)
3. Entry 2: CAN Bus System (need topology diagram)
4. Publications section (just text, can do now)
5. Entry 3: Simchair (need photo + architecture diagram)
6. Entry 4: Imitation Learning (need RGB/thermal comparison image)
7. Entry 5: CARLA RL (need reward curve screenshot)
8. Entry 6: Legged Robotics (need robot photo)
9. Entry 7: AVPCB (need KiCad 3D render)
