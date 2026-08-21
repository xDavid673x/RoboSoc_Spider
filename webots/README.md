# Webots Spider Simulator

This project targets Webots R2025a on macOS. It is separate from the Servo
2040 runtime and does not require the physical controller board.

## Prerequisites

- macOS with Webots R2025a installed (the default app path is
  `/Applications/Webots.app`)
- Python 3 with the repository test dependencies installed
- A checkout opened from the repository root

The simulator does not require a Servo 2040 board or hardware serial port.
The village and uneven-terrain worlds load Webots R2025a external PROTOs, so
the first launch may require network access.

## Demonstration

![Webots spider tripod gait demonstration](assets/demos/spider-tripod-gait.gif)

*Top-down Webots demonstration of the spider's tripod gait moving across uneven
terrain.*

Launch any world directly from a terminal:

```bash
WEBOTS=/Applications/Webots.app/Contents/MacOS/webots
"$WEBOTS" webots/worlds/flat.wbt
```

Replace `flat.wbt` with a world listed below. In the GUI, click the 3D view
before using the keyboard so it receives focus.

## Run

Open one of these worlds in Webots:

- `worlds/flat.wbt`
- `worlds/slope_10.wbt`
- `worlds/slope_20.wbt`
- `worlds/slope_30.wbt`
- `worlds/uneven_terrain_spider.wbt` (R2025a equivalent of the R2019a
  `uneven_terrain.wbt` demo)
- `worlds/realistic_village_spider.wbt` (R2025a Realistic Village integration
  scene)

Start the simulation, click the 3D view so it has keyboard focus, and use:

| Key | Action |
| --- | --- |
| `W` | Walk forward |
| `S` | Walk backward |
| `A` | Move left without turning |
| `D` | Move right without turning |
| `J` | Turn left in place |
| `K` | Turn right in place |
| `Space` | Stop and hold the init stance |
| `R` | Reset body, joints, gait phase, and physics |

`W`/`S` can be combined with `A`/`D` for diagonal translation. Turning takes
precedence when a translation and turn key are held together. The
worlds explicitly use Webots' `NUE` coordinate system: X and Z are horizontal,
Y is up, and the initial forward direction is negative Z.

The uneven-terrain world uses Webots' maintained `UnevenTerrain` PROTO in the
`ENU` frame (X and Y are horizontal, Z is up), so the Spider instance is rotated
into that frame. Its center is flat for a stable reset; moving away from the
center reaches the seeded height field. The linked R2019a sample uses obsolete
`ElevationGrid` fields and a six-wheel controller, so it should not be opened
directly in R2025a.

To launch it directly from macOS:

```bash
/Applications/Webots.app/Contents/MacOS/webots \
  webots/worlds/uneven_terrain_spider.wbt
```

## Model Boundary

`protos/Spider.proto` contains all six CAD leg chains and uses this confirmed
mapping:

| Simulator leg | CAD leg |
| --- | --- |
| `legi` | 7 |
| `legj` | 2 |
| `legn` | 3 |
| `legl` | 4 |
| `legk` | 5 |
| `legm` | 6 |

Tripod A is `legi`, `legk`, and `legm`. Tripod B is `legj`, `legl`, and
`legn`. Each leg has coxa and femur limits of -90 to +90 degrees and a tibia
limit of 0 to 130 degrees.

The CAD-derived gait compensation accounts for `SpiderLeg` using local X/Y as
its horizontal plane while Webots uses local X/Z. This keeps all three stance
feet pushing along one body axis: `W` advances toward Webots negative Z (Fusion
top-view positive Y), and `S` reverses that stride.

For `J` and `K`, the adapter rotates those same alternating tripod sweeps onto
the tangent of each radial leg frame. The stance tripod therefore supplies
torque around the body centre while the swing tripod advances to the next
support position, producing an in-place turn instead of pulling the feet
inward or outward.

The existing `SpiderLeg` and `Tripot_gait` classes remain the source of IK and
gait behavior. They continue to operate in millimetres and degrees. The Webots
adapter removes the physical servo offsets once, then converts motor commands
to radians. CAD mesh scale is converted from millimetres to metres in the
PROTOs. Dense STL files are visual-only; capsules and a box provide collision
geometry.

## Provisional Dynamics

These values are engineering estimates, not calibrated measurements:

| Property | Value |
| --- | --- |
| Total mass | 3.0 kg |
| Body mass | 1.2 kg |
| Each coxa, femur, or tibia link | 0.10 kg |
| Foot static friction estimate | 0.8 |
| Foot dynamic friction estimate | 0.6 |
| Motor torque limit | 2.5 N m |

Webots R2025a uses the configured Coulomb coefficient of 0.8 for the current
contact model. The 0.6 dynamic value is retained as a calibration target; it is
not represented by a separate field in these worlds.

## Verification

From the repository root:

```bash
python3 tools/cad_sync.py check-generated
python3 -m pytest -q
```

`check-generated` rebuilds the attachment hierarchy from the raw Fusion
snapshot and enforces the 0.01 mm / 0.01 degree frame tolerance. It covers all
91 visible CAD bodies, six fixed mount branches, 18 parent-local hinge anchors,
and the reset-zero transform for every joint. The native smoke suite starts
Webots in batch/fast mode. It loads all four worlds, resolves all 18 motors and
sensors, checks each world's first physics step and configured reset transform,
and checks the command path for forward/backward, left/right strafing, in-place
turning, stop, and reset.
The flat-world motion check also rejects lateral drift above 20 percent of the
forward stride or walk yaw above 0.1 radians. Turn checks require opposite,
near-symmetric yaw above 0.5 radians and less than 10 mm of horizontal drift.

Each slope world rotates and translates the initial robot pose with the terrain.
`R` restores that world-specific pose, while the joint reset remains
`[0, 28, 115]` degrees in every world.

The automated smoke suite covers `flat.wbt` and the three slope worlds. Use the
uneven-terrain and village worlds for interactive GUI checks; their external
Webots PROTO dependencies are intentionally outside the deterministic four-world
physics suite.

If the spider appears not to respond, click the 3D view, press `Space` once,
then press `R` to restore the world pose before trying `W`, `A`, `S`,
`D`, `J`, or `K` again.

## Calibration Limits

- The controller uses the committed CAD manifest when present, but currently
  falls back to legacy provisional leg lengths and headings if that file is
  missing. Treat a run without `webots/cad/spider_geometry.v1.json` as
  unverified rather than as an exact CAD simulation.
- The CAD attachment and reset-pose checks are geometric checks, not a claim
  that the current legacy gait is dynamically calibrated. The existing
  `SpiderLeg` gait uses its historical support pose, while the Fusion reset
  stance is represented by `[0, 28, 115]`; the resulting physical stride can
  be below the smoke-test displacement target until a measured stance and
  actuator response are supplied.
- The 10, 20, and 30 degree worlds validate loading and initial contact, not
  measured climbing performance.
- With the provisional fixed-height gait, the 30 degree world slides downhill
  and settles substantially lower than the flat, 10 degree, and 20 degree
  worlds. Mass distribution, foot material, actuator response, and a
  slope-aware stance must be calibrated before claiming 30 degree stability.
- Webots warns that the supplied visual STL files are dense and that the light,
  long tibia inertia is an edge case. Collision remains primitive by design;
  production tuning should use measured link inertia rather than dense meshes.
- Turn rate and absolute walking speed are simulation outputs, not predictions
  of the physical robot, until measured actuator and contact data are supplied.
