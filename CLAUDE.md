# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a **Webots R2025a** robotics simulation project implementing a billiards/pool table scene. The simulation uses VRML-based `.wbt` world files and follows the standard Webots project directory layout.

## Running the Simulation

Open the world file in Webots:

```bash
webots worlds/Billiards.wbt
```

## Project Structure

Follows the standard Webots project layout:

- `worlds/` — World definition files (`.wbt`). `Billiards.wbt` is the main scene.
- `controllers/` — Robot controller programs (currently empty — add Python/C/C++ controllers here)
- `protos/` — Custom PROTO node definitions (currently empty)
- `plugins/` — Webots plugins: `physics/`, `remote_controls/`, `robot_windows/` (all currently empty)
- `libraries/` — Shared libraries for controllers (currently empty)

## World Scene

`Billiards.wbt` defines a billiards table with:
- A floor (`Solid` with `Box` geometry), wall barriers around the table edges
- Color-coded corner/side pockets (red, green, blue boxes with matching walls)
- A separate ramp area (tan-colored solid with lower walls)
- Four `Ball` PROTO instances (yellow, purple, magenta, cyan) at `radius 0.375`
- External PROTOs from the Webots R2025a GitHub repository (`TexturedBackground`, `TexturedBackgroundLight`, `Ball`)

## Key Conventions

- The world uses a "factory" textured background and "music_hall" lighting
- Ball positions are on the XY plane; Z is the vertical axis
- Solids use `PBRAppearance` with `roughness 1` and `metalness 0` (matte materials)
- Bounding objects match their visual geometry for physics collision
