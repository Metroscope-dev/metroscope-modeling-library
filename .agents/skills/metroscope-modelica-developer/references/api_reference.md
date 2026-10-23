# AirCompressor MML4 graphical API

For generation, use `references/mml4-aircompressor-gold-standard.md` and the exact source files in `assets/mml4-reference/` as the first point of reference. For other machines inspect selected release API and nearest *MML4* tests before drawing analogies. The MML3 test corpus is unsuitable as an unqualified template.

- Component: `MetroscopeModelingLibrary.FlueGases.Machines.AirCompressor`
- Fluid: `C_in`, `C_out`; shaft: `C_W_in`; calibratable graphical parameter ports: `tau`, `eta_is`.
- Graphical interface types: `Utilities.Interfaces.BoundaryCondition`, `.CalibrationInput`, `.Observable`, `.GenericReal`.
- Sensor signals in canonical test: `T_sensor`, `P_sensor`, `Q_sensor`.
- Sensor class namespace in canonical user file: `SensorsControl.FlueGases.*` (no underscore), NOT the obsolete/other branch namespace `SensorsControl.*`.
- Metadata for calibration: `sensor_function="Calibration"`, `causality="airCompressor_eta_is"` or `"airCompressor_tau"` as appropriate; preserve on reverse and direct.
- Direct values (test): `airCompressor_tau = 15;` and `airCompressor_eta_is = 0.95;`; reverse does not supply fixed equations for these ports.

Do not infer successful compilation or a production-approved version from these files alone.
