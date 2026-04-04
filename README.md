# Modeller

A boundary representation (BRep) CAD modelling kernel and interactive desktop application, written in Rust.

## Features

- BRep solid modelling kernel (half-edge topology, analytical geometry)
- Primitives: box, cylinder, sphere, cone
- Boolean operations: union, difference, intersection
- Adaptive tessellation with smooth shading
- Interactive 3D viewport with orbit/pan/zoom
- Orientation cube gizmo with face and corner snap views
- Operation history tree in the object list
- Undo/redo

## Requirements

- Rust (stable, 1.75+) — install via [rustup](https://rustup.rs)
- A GPU with Vulkan, Metal, or DirectX 12 support (wgpu will select the best backend automatically)

## Build and run

```sh
# Clone the repository
git clone https://github.com/ablerman/modeller.git
cd modeller

# Run in debug mode
make run

# Run in release mode (recommended for performance)
make run-release
```

## Other make targets

```sh
make build          # Build all crates (debug)
make build-release  # Build all crates (release)
make test           # Run all tests
make clean          # Remove build artifacts
```

## Controls

| Input | Action |
|-------|--------|
| Left drag | Orbit |
| Shift + left drag | Pan |
| Right / middle drag | Pan |
| Scroll | Zoom toward cursor |
| Click object | Select |
| Shift/Ctrl + click | Multi-select |
| Delete / Backspace | Delete selected |
| Ctrl+Z | Undo |
| Ctrl+Y | Redo |

## Orientation gizmo

The cube in the bottom-right corner shows the current camera orientation.

- **Drag** the gizmo to orbit the viewport
- **Click a face** to snap to that orthogonal view (Front, Right, Top, etc.)
- **Click a corner** to snap to the corresponding isometric view

Snapping animates over 200ms.

## Workspace layout

| Crate | Description |
|-------|-------------|
| `brep-core` | Fundamental types: `EntityId`, tolerances, AABB, math |
| `brep-geom` | Curve and surface geometry (line, circle, B-spline, plane, cylinder, sphere, cone) |
| `brep-topo` | Half-edge DCEL topology, Euler operators, primitives |
| `brep-mesh` | Adaptive tessellation |
| `brep-algo` | BVH acceleration, ray casting, point-in-solid |
| `brep-bool` | Boolean operations (union, difference, intersection) |
| `brep-param` | Parametric feature tree (extrude, revolve, fillet, chamfer, shell) |
| `brep-io` | STEP AP214 reader/writer, STL export |
| `brep-render` | CPU rasterizer and wgpu GPU renderer |
| `brep-app` | Interactive desktop application (egui + wgpu + winit) |
