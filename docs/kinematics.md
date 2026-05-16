# Kinematics

Forward and inverse kinematics for DOT's 3R leg chain.

Full derivation coming. For now, the working math lives in Desmos:

- [FK](https://www.desmos.com/3d/erax4k9wia)
- [IK](https://www.desmos.com/3d/gpihu2vkmm)

---

## Variables

- `L1` — leg segment length (upper and lower are equal)
- `O` — ab/ad offset (signed)
- `(X, Y, Z)` — foot target in body frame
- `g` — ab/ad joint angle
- `t` — hip pitch joint angle
- `a` — knee joint angle

---

## Status

- [x] FK closed-form
- [x] IK closed-form
- [ ] Validated against hardware
- [ ] Jacobian
- [ ] Workspace analysis (straight-leg geometry)
- [ ] Written derivation with diagrams
