# sketch_tools

Per-tool modules for the sketch viewport. Each module describes one drawing or selection tool
available while the user is in sketch mode.

Future non-sketch tools (model tools, assembly tools, etc.) belong in sibling modules at the
crate root, not here.

---

## Tool interface

A sketch tool is not a trait — it is a plain Rust module that optionally exports up to two
functions:

### `add_point(sk: &mut SketchState, p: Point3)`

Called when the user left-clicks in the sketch viewport and the click resolves to a 3D point `p`
on the sketch plane (after snapping). This is the tool's core state-machine step.

- **Polyline** appends `p` to `sk.points` and re-solves constraints.
- **Arc / Rectangle / Circle** advance their `ToolInProgress` state; on the final click they push
  a `CommittedProfile` to `sk.committed_profiles` and clear `tool_in_progress`.

The dispatcher in `mod.rs` matches on `sk.active_tool` and calls the correct module's
`add_point`. The **Pointer** tool never reaches `add_point` — clicks are handled entirely in
`main.rs` (selection) and `editor.rs` (drag).

### `draw_preview(painter, …, proj, stroke)`

Called every frame while the tool has partial state (`tool_in_progress.is_some()`). Draws a
ghost of what will be committed on the next click.

- **Arc** draws a tessellated arc preview and a crosshair at the projected center.
- **Rectangle** draws a four-edge outline.
- **Circle** draws a 64-segment tessellated circle.
- **Polyline** draws a single line segment from the last vertex to the cursor (handled inline in
  `ui.rs`, not via this function).
- **Pointer** has no preview.

The `proj` argument is a `&impl Fn(Point3) -> Option<egui::Pos2>` that maps 3D sketch-plane
points to 2D screen coordinates; it returns `None` for points behind the camera.

---

## ToolInProgress mapping

| Variant | Tool | Meaning |
|---------|------|---------|
| `Arc1 { start }` | Arc | Click 1 done; waiting for end point |
| `Arc2 { start, end_pt }` | Arc | Click 2 done; waiting for center click |
| `RectFirst { corner }` | Rectangle | Click 1 done; waiting for opposite corner |
| `CircleCenter { center }` | Circle | Click 1 done; waiting for radius point |

`tool_in_progress` is always `None` for Polyline and Pointer.

---

## Module list

| Module | Tool | `add_point` | `draw_preview` |
|--------|------|:-----------:|:--------------:|
| `pointer` | Pointer / Select | — | — |
| `polyline` | Polyline | ✓ | — |
| `arc` | Arc | ✓ | ✓ |
| `rectangle` | Rectangle | ✓ | ✓ |
| `circle` | Circle | ✓ | ✓ |
