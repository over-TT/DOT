# Kinematics

Forward and inverse kinematics for DOT's 3R leg chain.

The working math is also in Desmos:

- [FK](https://www.desmos.com/3d/erax4k9wia)
- [IK](https://www.desmos.com/3d/gpihu2vkmm)

Full written derivation with diagrams is still coming. What follows is the conventions and the IK pipeline as implemented in `firmware/dot_ik_test/`.

---

## Coordinate system

Right-handed, body frame, hip joint as the origin for each leg.

- **X** — forward / back. Positive X points forward.
- **Y** — left / right. Positive Y points outward from the body (away from the spine) once the IK has mirrored the left side. See "Left/right mirroring" below.
- **Z** — up / down. Positive Z points up. Negative Z is where the foot lives.

Units are arbitrary as long as they match `L1` and `L2`. The current sketch uses centimeters.

---

## Leg geometry

Each leg is a 3R chain: hip (ab/ad), upper leg, knee.

- `O` — hip side offset. Distance from the body centerline along Y to the upper-leg pivot. Signed; currently `-0.6`.
- `L1` — upper leg length. From the upper-leg pivot to the knee. Currently `5.0`.
- `L2` — lower leg length. From the knee to the foot tip. Currently `5.0`.

`L1 == L2` simplifies the workspace but is not required.

---

## Servo mapping

Each leg has three servos, indexed in the firmware as `{hip, upper, knee}`:

- `g` — side hip servo. Rotates the upper-leg pivot about the X axis.
- `a` — upper leg servo. Pitches the upper leg about the Y axis at the hip.
- `t` — knee servo. Bends the lower leg about the Y axis at the knee.

All three are MG90S, driven by a PCA9685. Logical angle `90°` is neutral. Some servos are mounted mirrored — those are flipped in software via `REVERSED_SERVOS`.

---

## Left/right mirroring

The IK is solved in a "right-side" frame and then mirrored for the left side.

In code:

```cpp
float yLocal = left ? -y : y;
```

That means the caller can use the same `(x, y, z)` semantics for any leg ("positive Y is outward") and the IK takes care of the sign. The `left` flag is set per leg in the dispatch:

- front left, back left → `isLeft = true`
- front right, back right → `isLeft = false`

---

## IK pipeline

1. **Rotate target into the leg plane.**
   Compute `g` from the Y-Z view using the hip offset `O`:

   ```text
   r1 = sqrt(y² + z²)
   c1 = O / r1            (clamped to [-1, 1])
   P  = atan2(y, z)
   g  = P + acos(c1)
   ```

2. **Project the target into the 2D plane of the leg.**

   ```text
   x2 =  x
   z2 = -y · cos(g) + z · sin(g)
   d  = sqrt(x2² + z2²)
   ```

3. **Check reachability.**
   Bail out if `d > L1 + L2` or `d < |L1 − L2|`.

4. **Solve the 2-link triangle.**

   ```text
   c2 = (L1² + L2² − d²) / (2·L1·L2)
   kneeInner = acos(c2)
   t = π − (kneeInner − π/2)

   c3 = (L1² + d² − L2²) / (2·L1·d)
   beta = acos(c3)
   p    = atan2(z2, x2)
   a    = π − p − beta
   ```

5. **Convert math angles to servo angles.**
   Radians → degrees, then apply mechanical offsets and inversions in `writeServoLogical()`:

   ```text
   logical = mathAngle + servoOffset[i]
   actual  = isReversed(i) ? (180 − logical) : logical
   pulse   = map(actual, 0, 180, SERVO_MIN, SERVO_MAX)
   ```

   `legGeometryOffset[i]` is added on top of the math result for `leg[1]` (upper) and `leg[2]` (knee) to absorb residual mechanical alignment.

---

## Variables

| Symbol | Meaning |
|--------|---------|
| `X, Y, Z` | foot target in the leg's body frame |
| `O`       | hip side offset (signed) |
| `L1`      | upper leg length |
| `L2`      | lower leg length |
| `g`       | ab/ad (side hip) joint angle |
| `a`       | upper-leg pitch joint angle |
| `t`       | knee joint angle |

---

## Status

- [x] FK closed-form
- [x] IK closed-form
- [x] Left/right mirroring in code
- [ ] Validated against hardware end-to-end
- [ ] Jacobian
- [ ] Workspace analysis (straight-leg geometry)
- [ ] Written derivation with diagrams
