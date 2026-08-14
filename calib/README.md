# Force-sensor calibrations

One `calib_data.EEForceSensor` per URDF variant, in the layout mc_rtc expects:
`<calib_dir>/calib_data.<sensor name>`, consumed by `ForceSensorCalibData::loadData` and
applied by `wrenchWithoutGravity`.

**Why these live here.** mc_rtc resolves `calib_dir` from `RobotModule::path`, which for
this module is `kortex_description`. That package neither installs a `calib/` directory nor
tracks one in git, so the files only ever existed in the install tree, created by the GUI's
"Save calibration" button. They survive a rebuild but not a fresh install prefix — and a
missing calibration is silent, mc_rtc only warns. Since the ping-pong policy is trained
against a vendored copy of the racquet's file, losing it would be worse than an error.

`kinovaCalibDir` in `src/kinova.cpp` therefore prefers this directory when it holds a file
for the variant, and falls back to the `kortex_description` default otherwise, so an
in-place recalibration saved by the GUI still wins until it is copied here.

**After running a calibration**, press "Save calibration" as usual, then copy the result
here and commit it:

```sh
cp <install>/share/kortex_description/calib/<variant>/calib_data.EEForceSensor \
   calib/<variant>/
```

## The 13 numbers

`mass`, the rpy of the model-to-actual sensor misalignment, the payload CoM in the parent
frame, then a 6-vector offset that is couple-first (`sva::ForceVecd` ordering).

Two things about them that are not obvious, both established in
`~/devel/ft-calib-investigation/README.md`:

- **`mass` is not the true mass.** The Bota's force channel reads 3.67 % low, and the
  calibration subtracts a *measured* gravity wrench, so the file must hold
  `0.9633 * true_mass`. Anything consuming these files in simulation, where the sensor is
  exact, has to scale its own wrench by the same factor.
- **The offset is the tare.** The sensor is zeroed at the Kinova home pose
  `[0, 15, 180, -130, 0, 55, 90]` deg with the tool attached, so the offset is exactly minus
  the model wrench at that pose. Re-taring at a different pose invalidates the calibration by
  up to `2*m*g`.

## Current files

| variant | mass | origin | notes |
| --- | --- | --- | --- |
| `kinova_bota_plate` | 0.300848 | fitted on the robot, 2026-08-13 | the ping-pong racquet; validated by a check-calibration run |
| `kinova_bota_peg_plate_camera` | 0.824698 | model-derived | 0.345 N / 0.093 N·m on 12 static poses |
| `kinova_bota_peg_plate` | 0.641671 | model-derived | the peg plate has never been weighed, so its mass is CAD; treat as provisional |

A model-derived file is generated from the URDF plus weighed masses by
`~/devel/ft-calib-investigation/analysis/gen_model_calib.py`. It claims no mounting rotation,
which the fitted files do (up to 4°, and it is not real). Fitting buys 16–49 % on the force
residual and nothing structural — the dominant error is the sensor's anisotropy, which
neither file can express.

**Regenerate the model-derived files whenever a tool link changes in the xacro.** They are
derived from the model, so they go stale silently when it moves. The MuJoCo models in
`kinova_mj_description` carry the same inertials and have to move with them.

## Check a file before trusting it

A calib file records no provenance — a mass and a date, and both look plausible whatever
wrote them. Running the calibration controller **in simulation** saves to the same place as
running it on the robot, and that has already happened once here: the
`kinova_bota_peg_plate_camera` file was for a while a MuJoCo run, 52 g light, because in
simulation the sensor is exact and the camera was not yet in the model.

The cheap test is the gain. A file measured on hardware carries `0.9633 * true_mass`, so its
mass should sit about **3.7 % below** the model's payload. One that lands *on* the model
value was taken in simulation.
