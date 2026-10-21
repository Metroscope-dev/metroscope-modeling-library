# User-approved MML4 patterns

The user explicitly confirmed these three reference families on 2026-10-20. Approval establishes modeling conventions; it is not evidence of compilation or convergence of the current checkout.

## Locate and select the reference

Paths below are relative to the repository root, except bundled assets relative to this skill:

| Family | Files | Primary use |
| --- | --- | --- |
| AirCompressor | `assets/mml4-reference/AirCompressor_{direct,reverse,faulty}.mo` | Single-component graphical instrument structure; `GenericReal` characteristics; direct/reverse/faulty contracts |
| MetroscopiaCCGT | `MetroscopeModelingLibrary/Examples/CCGT/MetroscopiaCCGT/MetroscopiaCCGT_{direct,reverse}.mo` | CCGT subsystem topology, equipment calibration links, global versus local BCs |
| MetroscopiaNPP | `MetroscopeModelingLibrary/Examples/Nuclear/MetroscopiaNPP/MetroscopiaNPP_{direct,reverse}.mo` | Nuclear subsystem topology, equipment calibration links, explicit graphical fixed hypotheses |

Read the relevant declarations, characteristic assignments and connections in both variants. For a condenser, use the condenser portions of the plant patterns before considering the historical `Tests/WaterSteam/HeatExchangers/Condenser_*` files, which still contain scalar-I/O architecture in the inspected checkout.

## Verified condenser contract in CCGT and NPP

| Quantity | Sensor function | Causality target | Direct interface | Reverse interface |
| --- | --- | --- | --- | --- |
| Steam pressure at condenser admission | Calibration | `condenser_Kth` | Observable | CalibrationInput |
| Cooling-water outlet temperature | Calibration | `condenser_Qv_cold` | Observable | CalibrationInput |
| Cooling-water inlet temperature | BC | empty | BoundaryCondition | BoundaryCondition |
| Cooling-water inlet pressure | BC | empty | BoundaryCondition | BoundaryCondition |

Both families connect `condenser_Kth` to `condenser.Kth` and `condenser_Qv_cold` to `condenser.Qv_cold_in`. Direct fixes those two characteristics by equations; reverse leaves them solved from the two calibration observables. Keep the sensor metadata in both variants. Do not replace the outlet-temperature calibration by a flow calibration merely because a flow sensor is available; that is a distinct modeling decision requiring justification or an explicit user request.

Both assume `Kfr_cold=0` and `C_incond=0` in the inspected examples. NPP expresses these assumptions with `Utilities.Interfaces.RealExpression(y=0)` blocks wired to the component; CCGT uses equations on the component ports. Preserve the selected pattern's distinction between fixed assumptions and identified characteristics. Do not add a third calibrated characteristic or diagnostic sensor merely to fill all available ports.

For an isolated condenser, replace the rest of the plant by necessary steam and cooling-water sources/sinks. Document the new local steam BCs (typically positive inlet mass flow and specific enthalpy for wet steam). These local BCs must not be mistaken for the global boundary conditions of the original closed plant cycle. Do not impose steam pressure as a direct BC when it is the Kth observable.

## Differences to preserve and explain

- AirCompressor uses `GenericReal` for characteristics. CCGT/NPP use `Utilities.Interfaces.RealOutput` for characteristic connectors in the inspected direct and reverse files. `RealOutput` is an actual `output Real` connector in this checkout. These are distinct approved patterns; do not reject the plant examples solely for not using `GenericReal`, and do not silently replace their interface class when extracting a subsystem.
- NPP condenser placement is `extent={{570,40},{670,140}}` (100 by 100), also 100 by 100 in CCGT. Its fluid sensors are 20 by 20; its cooling source/sink are 40 by 40. Preserve the chosen approved pattern's dimensions, orientations and icon transformations when extracting it. A small legacy condenser test's 20 by 18 placement is not the graphical reference for this extraction.
- The examples contain some old defaults and possible inconsistencies. Add supported realistic sensor initialization when creating a new model, as required by the skill; do not treat every numeric start, calibration result or spelling elsewhere in a plant example as universally valid. Recheck exact causality names against declarations.
- Numeric calibration values belong to their original geometry and operating point. Do not transplant CCGT or NPP calibrated Kth/Qv values into a different-sized illustrative condenser and describe the result as calibrated plant data.

Before delivery, list the selected family, preserved conventions, necessary isolation changes, API discrepancies and unexecuted validation. The cross-family interface rules in `mml4-graphics.md` remain applicable.
