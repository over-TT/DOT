# Calibration

Before mounting any leg parts, every servo's software 90° must match its physical neutral. Otherwise IK is meaningless.

---

## Required

- Assembled body with all 12 MG90S servos seated in their printed mounts, **horns not yet attached**.
- PCA9685 wired up and powered (see [wiring.md](wiring.md)).
- `firmware/dot_calibrate/` flashed.

---

## Procedure

1. Open `firmware/dot_calibrate/dot_calibrate.ino`. Make sure the top-of-file angle is set to **90°**.
2. Flash and let all 12 servos drive to their neutral position.
3. Without forcing them, tap each servo horn output gently to confirm it's holding. Cut power if any servo is buzzing hard against a mechanical stop — that one's offset is wrong and needs to be corrected before continuing.
4. **Identify reversed servos.** Increase the test angle (e.g. 90 → 120) and reflash. Watch which servos rotate the *wrong* way relative to the leg's intended motion. Add those servo indices to `REVERSED_SERVOS` at the top of the sketch.
5. Reflash with the angle back at 90°. Confirm every servo is at neutral *and* rotating the intended direction when you nudge the angle up.
6. **Mount the legs at 90°.** Upper-leg first, then knee. Tighten the horn screws while the servo is still holding neutral. If you let the horn slip, the leg's zero will be off and you'll be chasing it through `servoOffset[]` forever.
7. Repeat for all four legs.
8. (Optional) Fine-tune `servoOffset[]` in `dot_ik_test/` for any residual asymmetry. Single-digit degrees only — anything more means the horn is on a wrong spline.

When this is done, the software's 90° matches the hardware's 90°, and the IK will mean what it says.

---

## Notes

- `legGeometryOffset[]` exists for the upper-leg and knee servos specifically and is applied on top of the IK output in `moveLegToXYZ()`. Use it for systematic geometric tweaks (e.g. all knees are biased 3° forward), not for fixing a single mismounted horn.
- Servo target angles in `dot_ik_test/` are clipped to **45°–135°**. Anything outside is silently clamped. Keep poses inside that window or change the clamp in `setServoTarget()`.
