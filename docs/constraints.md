# Sketch Constraints

This document describes every constraint type: what it means geometrically,
what the user must select before applying it, and how it is stored internally.

---

## Selection vocabulary

| Symbol | Meaning |
|---|---|
| **AP** | Active profile (the polyline/arc currently being drawn) |
| **pt** | Point selected from the active profile (`pt_selection`) |
| **seg** | Segment selected from the active profile (`seg_selection`) |
| **CP** | Committed profile (a completed polyline segment, arc, circle, or rectangle) |
| **cp-pt** | Control point selected from a committed profile (`committed_pt_selection`) |
| **cp-seg** | Segment selected from a committed profile (`committed_seg_selection`) |
| **cp-profile** | Whole committed profile selected (`committed_selection`) |
| **ref** | Reference entity selected (Origin, X-axis, Y-axis) |

Constraints stored on the active profile use local point indices.
Constraints stored on a committed profile also use local (0-based) indices into
that profile's `point_indices` array.

---

## Coincident

Makes two points occupy the same position.

**Valid selections:**

| # | Selection | What is applied |
|---|---|---|
| 1 | 2 AP pts | Active-profile `Coincident { pt_a, pt_b }` |
| 2 | 1 AP pt + 1 AP seg | Active-profile `PointOnLine` — point lies on the segment |
| 3 | 1 AP pt + Origin ref | Active-profile `PointOnOrigin` |
| 4 | 1 AP pt + 1 cp-pt | Active-profile `PointFixed` — pins the AP point to the CP point's current position |
| 5 | 1 AP pt + 1 arc/circle cp-profile | Active-profile `PointOnCircle` — AP point lies on the curve |
| 6 | 2 cp-pts, same CP | Per-profile `Coincident { pt_a, pt_b }` on that profile |
| 7 | 2 cp-pts, different CPs | **Structural merge** — the two endpoints share one `global_points` slot; no constraint stored |
| 8 | 1 cp-pt + 1 cp-seg, same CP | Per-profile `PointOnLine` on that profile |

Selection 7 (structural merge) is preferred over a constraint because it
guarantees coincidence unconditionally — dragging either endpoint moves both,
and the solver cannot violate it.

---

## Horizontal

Makes a segment or pair of points level relative to the current view's
horizontal direction.

**Valid selections:**

| # | Selection | What is applied |
|---|---|---|
| 1 | 1+ AP segs | `HorizontalPair` on each selected segment |
| 2 | 2 AP pts | `HorizontalPair` between the two points |
| 3 | 1 AP pt + Origin ref or X-axis ref | `PointOnXAxis` |

"Horizontal" is camera-relative: the constraint records the screen-up direction
at the time it is applied, so it works correctly on oblique sketch planes.

---

## Vertical

Makes a segment or pair of points plumb relative to the current view's
vertical direction.

**Valid selections:**

| # | Selection | What is applied |
|---|---|---|
| 1 | 1+ AP segs | `VerticalPair` on each selected segment |
| 2 | 2 AP pts | `VerticalPair` between the two points |
| 3 | 1 AP pt + Origin ref or Y-axis ref | `PointOnYAxis` |

---

## Parallel

Makes two segments run in the same direction (or 180° opposite).

**Valid selections:**

| # | Selection | What is applied |
|---|---|---|
| 1 | 2 AP segs | Active-profile `Parallel { seg_a, seg_b }` |

---

## Perpendicular

Makes two segments meet at exactly 90°.

**Valid selections:**

| # | Selection | What is applied |
|---|---|---|
| 1 | 2 AP segs | Active-profile `Perpendicular { seg_a, seg_b }` |

---

## Equal Length

Makes two or more segments the same length. The solver converges each pair
toward equal length (distributing the difference).

**Valid selections:**

| # | Selection | What is applied |
|---|---|---|
| 1 | 2+ AP segs | `EqualLength` between seg[0] and each other selected segment |

---

## Angle

Fixes the angle between two segments to a value entered in a dialog.

**Valid selections:**

| # | Selection | What is applied |
|---|---|---|
| 1 | 2 AP segs | Opens angle dialog → `Angle { seg_a, seg_b, degrees }` |

---

## Fixed Length / Point Distance

Locks a distance to a specific value entered in a dialog.

**Valid selections:**

| # | Selection | What is applied |
|---|---|---|
| 1 | 1+ AP segs | Opens length dialog → `FixedLength { seg, value }` per segment |
| 2 | 2 AP pts | Opens length dialog → `PointDistance { pt_a, pt_b, value }` |

---

## Radius

Fixes the radius of a committed arc or circle.

**Valid selections:**

| # | Selection | What is applied |
|---|---|---|
| 1 | 1 arc/circle cp-profile | Opens length dialog → `PointDistance { pt_a: 0, pt_b: 1 }` for circle, `PointDistance { pt_a: 0, pt_b: 2 }` for arc |

The dialog is pre-populated with the current radius.

---

## Symmetry

Constrains a point to be equidistant (perpendicular distance) from two segments.

**Valid selections:**

| # | Selection | What is applied |
|---|---|---|
| 1 | 2 cp-segs + 1 cp-pt | `CommittedCrossConstraint::Symmetric { pi_a, si_a, pi_b, si_b, gi_p }` |

The constraint is solved by a Newton iteration that minimises `d₁² − d₂²` where
`dᵢ` is the signed perpendicular distance from the point to segment i.

---

## Planned / not yet implemented

None at this time — all previously-planned constraints have been implemented.
