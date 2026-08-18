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

## Ready for Demo
All core functionality tested and working. Ready to pick up tomorrow with multi-axis integration testing.
