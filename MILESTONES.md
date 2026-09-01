# Milestones

## 2026-03-19 - Milestone 2
Calibration process hardened and verified for practical use.

### Summary
- Added clear step-by-step calibration prompts (MIN -> MAX -> HOME).
- Added direction-aware guidance and live movement feedback during calibration.
- Added pre-save safety summary with PASS/FAIL checks before writing FRAM.
- Reduced feed noise so updates only report meaningful turn changes.
- Set live turn reporting threshold to 0.001 turns from prior reported value.

### Validation
- Calibration completed successfully on Axis 2 with valid ordering (MIN < HOME < MAX).
- Pre-save summary reported PASS.
- Calibration saved to FRAM successfully.
- Firmware rebuilt and uploaded successfully to COM13.

### Why this milestone matters
Reliable calibration is demo-critical and now provides clear physical + digital feedback before save, reducing operator error.

---

## 2026-03-19 - Milestone 3 (Final)
Node 2 motor control via ELRS TX channel verified end-to-end.

### Summary
- Confirmed complete firmware integration:
  - Calibration workflow (serial commands + FRAM persistence).
  - CAN/ODrive comms active and healthy.
  - ELRS/CRSF RX comms active and healthy.
  - Channel control mapping still fully active.
- Node 2 controlled by TX Channel 3 (CH3 -> Node 2).
- Arm gate via TX Channel 5 (CH5 threshold > 1200).
- Return-to-home safety on disarm working.

### Test Results
- Node 2 does not move when disarmed (passed).
- Node 2 follows CH3 smoothly and within calibrated limits when armed (passed).
- Node 2 recenters at CH3 center position (passed).
- Disarm triggers safe return-to-home ramp (passed).

### Why this milestone matters
End-to-end control loop validated: calibration → FRAM persistence → channel control → motor response all working in production code.

---

## 2026-08-31 - ODrive Safety and Diagnostics

### Summary
- Added per-axis freshness tracking for ODrive encoder estimates received over CAN.
- Added ODrive heartbeat monitoring for axis state and reported axis errors.
- Added a safety latch that idles every axis and inhibits motion if feedback becomes stale or an ODrive axis reports an error.
- Added `STATUS` serial output for CRSF, TWAI bus, ODrive feedback, heartbeat, state, and fault details.
- Added guarded `FAULT CLEAR`, which requires the arm to be disarmed and all ODrive axes to report fresh, error-free feedback.
- Changed calibration entry to idle all ODrive axes so the arm is torque-free for manual positioning.

### Validation
- Reinstalled the incomplete PlatformIO ESP32-S3 Xtensa toolchain.
- Completed `pio run -e demo_arm` successfully after the toolchain repair.

### Scope
Encoder feedback remains on the ODrive and is read through CAN encoder estimates; no ESP32-direct encoder support was added.

---

## Ready for Demo
All core functionality tested and working. Ready to pick up tomorrow with multi-axis integration testing.
