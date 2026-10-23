# Test operating points — CCGT and REP 1300 MW(e)

These are **illustrative engineering sanity ranges**, not plant design data, contractual limits or validated initial points. Prefer (1) a measured/reconciled operating point (DSU), (2) values in a user-approved same-version MML4 test / MetroscopiaNPP or MetroscopiaCCGT, then (3) the ranges here. Report which choice was made. Do not fabricate plant-specific measurements. A REP 1300 value depends on circuit, location, load and unit; never use a single generic water/steam boundary condition throughout a nuclear plant.

## Mandatory signal units for generated test models

- Temperature displayed/exchanged at sensor interfaces: `degC` (°C); set `signal_unit="degC"` and `display_unit="degC"` when supported.
- Absolute pressure: `barA`; set `signal_unit="barA"` and `display_unit="barA"` when supported. Do not substitute barG.
- Mass flow: `kg/s`; set `signal_unit="kg/s"` and `display_unit="kg/s"` when supported.
- Use the version's exact supported enum strings; verify sensor declarations. Do not add a `signal_unit` modifier to connectors/components that do not support it.
- **Only these units are allowed for user-facing T/P/Q test signals.** Underlying Modelica thermodynamic quantities may necessarily be in K, Pa, J/kg, W, etc. Respect Modelica/medium SI requirements and built-in sensor conversion; do not write `15` into a variable expecting temperature in K or `1` into pressure requiring Pa. Other physical quantities (efficiency, friction coefficient, electric power) use their proper types/units and are not to be mislabeled degC/barA/kg/s.
- With `Utilities.Interfaces.BoundaryCondition`, `CalibrationInput` and `Observable`, preserve the visual `connect(...)` style. For direct causality calibration sensors remain `sensor_function="Calibration"` and interface `Observable`; `causality` names the calibrated parameter.

## CCGT: indicative ranges / initial test guesses

| Subsystem / signal | Typical illustrative range | Suitable test guess | Notes |
|---|---:|---:|---|
| Compressor ambient air T | -10 to 40 °C | 15 °C | Boundary condition |
| Compressor ambient air P | 0.95 to 1.05 barA | 1.013 barA | Site/weather dependent |
| Large-frame GT compressor air flow Q | 300 to 750 kg/s | 500 kg/s | Strongly turbine and plant dependent |
| Compressor discharge P | 10 to 30 barA | 16 barA | Based on assumed pressure ratio |
| Compressor discharge T | 250 to 550 °C | 400 °C | Check efficiency and pressure ratio |
| Flue gas downstream of turbine T | 450 to 650 °C | 600 °C | Exhaust/HRSG entrance, not combustor inlet |
| HP steam pressure (HRSG) | 80 to 180 barA | 120 barA | Depends on plant design |
| HP steam temperature | 480 to 600 °C | 540 °C | Superheated steam |
| HP steam flow | 50 to 200 kg/s | 100 kg/s | Per train, configuration dependent |
| Condenser absolute pressure | 0.03 to 0.15 barA | 0.06 barA | Vacuum; never use atmospheric 1 barA by default |
| Cooling-water inlet T | 5 to 35 °C | 15 °C | Weather and cooling system |

**Air filter direct-model test (GT inlet):** choose ambient ~15 °C / 1.013 barA and e.g. 500 kg/s as externally imposed BCs; use a **small** pressure loss compared with 1 barA (e.g. about 0.005–0.03 bar, illustrative), compute downstream P as an observable, and derive/check `Kfr` from the actual component equation and density rather than inventing a friction coefficient. Check sign conventions and pressure positivity. `Kfr` is not a pressure and must not be assigned `barA`.

## REP 1300 MW(e): illustrative ranges / initial test guesses

| Location / signal | Typical illustrative range | Suitable test guess | Notes |
|---|---:|---:|---|
| Primary circuit hot leg T | 315 to 330 °C | 325 °C | Approximate full-power PWR |
| Primary circuit cold leg T | 280 to 300 °C | 290 °C | Approximate full-power PWR |
| Primary circuit pressure | 150 to 160 barA | 155 barA | Not secondary steam pressure |
| Primary coolant total flow | 15,000 to 25,000 kg/s | 20,000 kg/s | **Whole primary circuit**; distribute by loops, do not apply to single loop |
| Steam generator main steam pressure | 55 to 75 barA | 65 barA | Secondary side |
| Steam generator main steam T | 270 to 295 °C | 280 °C | Near saturation; check P/T consistency |
| Main steam flow (whole unit) | 1,500 to 2,400 kg/s | 1,900 kg/s | Sum over SGs; divide for individual SG |
| Condenser pressure | 0.03 to 0.12 barA | 0.06 barA | Secondary steam cycle |
| Feedwater T upstream SG | 200 to 240 °C | 225 °C | Plant/load dependent |
| Condensate / cooling-water temperature | 10 to 45 °C | 20 °C | Sensor location matters |

A 1300 MW(e) REP has multiple primary loops / steam generators; never apply whole-unit flows to a single branch. Primary and secondary water are different circuits and pressures. Boiler/steam-generation and saturated conditions require thermodynamic consistency, not independent arbitrary T and P values.

## Required checks before saving a test `.mo`

1. Classify each sensor by **plant type, fluid circuit, subsystem and physical location**. A nominal value cannot be selected before this is known.
2. Choose values from same-version validated model when available; preserve `T_start`, `P_start`, `Q_start` and `start` guesses with correct unit type, and `nominal` where supported. When updating source/sink internals, convert to SI as required by component API.
3. Match `sensor_function`, `causality` and graphical `Utilities.Interfaces.*` according to direct/reverse contract; use `SensorsControl` on user-targeted latest MML4, never silently fallback to `Sensors_Control`.
4. Check pressure loss direction, ranges, nonnegative absolute pressures, plausible T and flow, pressure/temperature compatibility with the medium, and sign of source vs sink Q.
5. Record all guesses and label them illustrative; validate by Dymola translation/simulation if available. Do not claim physical validation merely from these ranges.

## Evidence and scope

- User-approved MML4 AirCompressor direct canonical test: `assets/mml4-reference/AirCompressor_direct.mo` (15 °C, 1 barA, 500 kg/s, compressor ratio 15).
- Numerical estimates are illustrative; use verified target-plant operating data or a validated MML4 test first. External source index: `source-register.md`.
- Numeric ranges beyond the canonical compressor test are broad *engineering starting heuristics* and remain to be confirmed per plant and circuit; they are not presented as Metroscope-certified specifications.

## Mandatory dual sensor initialization (MML4)

For each `SensorsControl` fluid sensor, specify both the reference signal value and internal starting guess. Examples: `PressureSensor(sensor_function="BC", P_start=1.013, P_0=101300, signal_unit="barA", display_unit="barA")`; `TemperatureSensor(sensor_function="Calibration", causality="compressor_eta_is", T_start=400, T_0=673.15, signal_unit="degC", display_unit="degC")`; `FlowSensor(sensor_function="BC", Q_start=500, Q_0=500, signal_unit="kg/s", display_unit="kg/s")`. The `*_0` are initialization parameters (SI), not additional externally imposed BCs. Confirm the actual inherited sensor modifier in the pinned MML4 checkout; select per-circuit physically coherent guesses. For power sensors, use their own supported start parameter (`W_start`) rather than inventing `P_0` / `T_0`.
