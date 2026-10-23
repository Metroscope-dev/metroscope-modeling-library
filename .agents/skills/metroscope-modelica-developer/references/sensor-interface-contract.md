# MML4 sensor/interface and causality contract

API snapshot: branch `fix-bug-reheater` at commit `126f351e652e73df0eab808bbb7b29f9cb5e556f` (observed 2026-10-16). Model evidence:
- `MetroscopeModelingLibrary/Examples/Nuclear/MetroscopiaNPP/MetroscopiaNPP_reverse.mo`
- `MetroscopeModelingLibrary/Examples/Nuclear/MetroscopiaNPP/MetroscopiaNPP_direct.mo`
- `MetroscopeModelingLibrary/Examples/CCGT/MetroscopiaCCGT/MetroscopiaCCGT_reverse.mo` and `_direct.mo`
- `MetroscopeModelingLibrary/Utilities/Interfaces/{BoundaryCondition,CalibrationInput,Observable}.mo`
- `MetroscopeModelingLibrary/Partial/Sensors/BaseSensor.mo`

## Three roles: mandatory semantic mapping

| sensor_function | Meaning | reverse model external interface | direct model external interface | causality |
| --- | --- | --- | --- | --- |
| `"BC"` | Real external boundary condition, independent of modeled failures | `Utilities.Interfaces.BoundaryCondition` (`input Real`) | `Utilities.Interfaces.BoundaryCondition` (`input Real`) | ordinarily empty unless a documented exception |
| `"Calibration"` | Measured observable used to identify a component parameter | `Utilities.Interfaces.CalibrationInput` (`input Real`) | `Utilities.Interfaces.Observable` (`output Real`) | **set to exact parameter name calibrated by this sensor** |
| `"Unidentified"` | Ordinary observable, not fixed and not used for calibration | `Utilities.Interfaces.Observable` (`output Real`) | `Utilities.Interfaces.Observable` (`output Real`) | empty |

When `sensor_function` is omitted, inspect the actual inherited default (`"Unidentified"` in the branch's `Partial/Sensors/BaseSensor.mo`) before inferring its role. In MML4 there is a difference between physical `sensor_function` metadata in the graphical sensor and the Modelica causality of its external interface. A `Calibration` sensor stays `Calibration` in the direct model, yet its **external** interface becomes an `Observable`. Do not flip sensor_function to `Unidentified` during reversal.

## Graphical signals, not plain input/output variables

Example observed in NPP reverse:
```modelica
SensorsControl.WaterSteam.PressureSensor HP_heater_P_out_sensor(
  sensor_function="Calibration", causality="HP_heater_Kfr_cold")
  annotation (Placement(transformation(extent={{-636,-10},{-656,10}})));
Utilities.Interfaces.CalibrationInput HP_heater_P_out
  annotation (Placement(transformation(extent={{-4,-4},{4,4}}, origin={-646,20})));
// In equation section:
connect(HP_heater_P_out_sensor.P_sensor, HP_heater_P_out);
```
The matching **direct** example changes only the external declaration to `Utilities.Interfaces.Observable HP_heater_P_out` (while retaining capteur metadata and signal connection). A BC sensor connects e.g. `thermal_power` (`Utilities.Interfaces.BoundaryCondition`) to `thermal_power_sensor.W_sensor`. Exact sensor signal pin depends on sensor type and must be verified (`P_sensor`, `T_sensor`, `W_sensor`, etc.). Keep actual `Placement` and `iconTransformation` annotations from nearest same-version model to preserve Dymola icon and diagram behavior.

## Review algorithm
1. Inventory every `SensorsControl.*` instance; parse `sensor_function` and `causality` modifiers (including multiline declarations), sensor signal connector, and connected graphical `Utilities.Interfaces.*` endpoint.
2. For every sensor, check role against the table for the target variant. Confirm the external interface's name is meaningful and stable across reversal. Record and flag absent/disconnected external interfaces when expected; distinguish legitimate internal-only instrumentation.
3. For every `Calibration` sensor, require a nonempty `causality` naming the **calibrated parameter**, not the observable/sensor label. Check exact target case/name against actual parameter/interface, calibration table, and physical sensitivity. If multiple sensors identify one parameter or vice versa, require explicit coupled-calibration justification rather than pretending uniqueness.
4. For every `BC` sensor, confirm physical fault independence. A fault-sensitive BC can suppress fault propagation in the direct model, even if the Modelica file compiles.
5. Compare reverse/direct pairs: BC remains BC; Calibration input->observable; Unidentified remains observable; component parameter outputs in reverse generally become calibrated parameter inputs/values in direct. Inspect any exceptions and do not infer full mathematical wellposedness from static matching.
6. Preserve sensor_function and causality strings on both sides, annotation geometry, start/nominal values and unit conversion. Explain discrepancies with file names and example line locations.

## Known code quality traps from example snapshots
These are examples, **not a proof of validated models**. Review exact case of targets such as `LPST_CST` and `superheater_kth` against declared parameters and actual use; apparent mismatches require investigation, not automatic rename. Some `Calibration` sensors have no `causality` field in CCGT examples (e.g. opening sensors). Report as incomplete calibration traceability unless documented as deliberate or paired with a nontrivial constraint. The branch is mutable: reverify commit/ref on each invocation.

**Version note:** the snippet is illustrative and user-approved current MML4 uses `SensorsControl`; verify paths in the actual target checkout before compile. Provenance links are in `source-register.md`.
