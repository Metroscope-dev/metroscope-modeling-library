# MML4 AirCompressor gold-standard models (user-provided)

Priority: **canonical examples for MML4 AirCompressor**, supplied by the Metroscope user 2026-10-16. Reuse bundled exact `.mo` files in `assets/mml4-reference/` for direct, reverse and faulty; do not generate MML3-style code from older tests. They are examples, not evidence of successful Dymola compilation. Do not modify the originals when creating user outputs.

## Essential class and arrangement

- `within MetroscopeModelingLibrary.Tests.FlueGases.Machines;` and model names `AirCompressor_direct`, `AirCompressor_reverse`, `AirCompressor_faulty` in user files. If writing a standalone file in another package, adapt the `within` path and package tree.
- Exactly one process component `MetroscopeModelingLibrary.FlueGases.Machines.AirCompressor airCompressor`; auxiliary `FlueGases.BoundaryConditions.Source`, `.Sink`, `Power.BoundaryConditions.Source`, and inline sensors are needed to close the standalone model.
- User's MML4 sensor namespace is **`SensorsControl.FlueGases.*`**, not `Sensors_Control` (underscore). The user's example uses model-scope short names due to current package imports / resolution; when using another package, verify class resolution against that branch before expanding with prefix.
- BC capteurs: `ambiant_temp_sensor` (`T_start=15`, `signal_unit="degC"`), `ambiant_pressure_sensor` (`P_start=1`, `signal_unit="barA"`), `air_flow_sensor` (`Q_start=500`, `signal_unit="kg/s"`). Their external interfaces are `Utilities.Interfaces.BoundaryCondition` connected by `connect(sensor.T_sensor/P_sensor/Q_sensor, interface)`.
- Calibration capteurs: `discharge_temp_sensor(sensor_function="Calibration", causality="airCompressor_eta_is")`, `discharge_pressure_sensor(sensor_function="Calibration", causality="airCompressor_tau")`. Signal terminals connect to graphical interfaces, NEVER replaced with simple plain top-level scalar I/O. Both sensors keep these metadata on direct and reverse.
- Direct: `Utilities.Interfaces.Observable discharge_temp`, `...Observable discharge_pressure`.
- Reverse: `Utilities.Interfaces.CalibrationInput discharge_temp`, `...CalibrationInput discharge_pressure`.
- Component parameter ports: `Utilities.Interfaces.GenericReal airCompressor_tau`, `...GenericReal airCompressor_eta_is`, each connected to `airCompressor.tau`, `airCompressor.eta_is` using Modelica `connect`. In the direct example, `airCompressor_tau = 15; airCompressor_eta_is = 0.95;` in `equation`. In reverse example those assignments are intentionally omitted, permitting calibration.
- Physical line connection source -> inlet temperature -> inlet pressure -> inlet flow -> compresser -> outlet temperature -> outlet pressure -> sink. Shaft power source -> compressor `C_W_in`. Source composition `source.Xi_out = {0.768,0.232,0.0,0.0,0.0}` in exact reference.
- Preserve `Placement`/`iconTransformation`, diagram lines and `connect`, including parameter signal routes. Do not replace the graphical method with `airCompressor.tau = compression_rate;` and plain `input` / `output` declarations.
- `AirCompressor_faulty` extends the direct model with `airCompressor(faulty=true)` and imposes failure scalars `eta_is_decrease`, `tau_decrease`. Note test fault uses `time` in the imposed trajectories; do **not** conclude this implies time-dependent MML steady-state component physics.

## Version caveat
The branch `fix-bug-reheater` examined earlier had older `Sensors_Control` classes in some files, while user-provided canonical sample uses `SensorsControl`. **Do not silently rewrite one into the other.** Verify the user's actual target checkout/commit and Dymola resolution. If branch/API disagrees with this example, flag the mismatch explicitly, prefer supplied test files for formatting and ask for the intended version only if needed for compile-ready claims.

## Creation/review acceptance checklist
1. Exactly the correct graphical interfaces per role and mode.
2. Calibration capteurs named and associated with exact declared parameter connectors in `causality`.
3. External signal connect and parameter connector connect present, graphically annotated.
4. Direct has numerical parameter equations, reverse does not (unless a documented constraint replaces them).
5. All source/sink/power connectors routed; units and initialization checked.
6. Model name, `within`, `end` and package placement match.
7. Difference from supplied canonical model explicitly justified.
