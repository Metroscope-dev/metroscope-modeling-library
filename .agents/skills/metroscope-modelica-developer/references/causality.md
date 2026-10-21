# Reverse/direct physical causality — operational method

## Causal decisions start with the process
MML components are acausal, yet how external conditions are imposed determines fault observability. Analyze the complete pressure/flow/enthalpy regulation path and distinguish whole-plant BCs from local assumptions. A true BC must remain independent of the faults to detect.

| Role | Reverse | Direct | Fault exposure |
| --- | --- | --- | --- |
| External BC (`sensor_function="BC"`) | imposed | imposed | must be fault-independent |
| Calibration observable (`"Calibration"`) | imposed through `CalibrationInput` | calculated through `Observable` | must respond to relevant fault |
| Other measured observable (`"Unidentified"`) | calculated through `Observable` | calculated through `Observable` | monitor but not calibrating |
| Tunable component parameter (`GenericReal`) | solved from calibration measurements | imposed at calibrated value | affects observables |

Metadata `sensor_function` and `causality` remain unchanged between reverse and direct. Change only external interface role where applicable. Exact connectors are specified in `sensor-interface-contract.md`.

## How to choose a calibration measurement
For every parameter, map which observable it physically influences. A friction coefficient should be identified by an independent pressure drop or pressure measurement with an appropriate pressure/flow BC. A heat-transfer coefficient needs thermally sensitive measurements. Check sensor availability and uniqueness. Do not force a calibration sensor to be a BC in direct just to close equations: that suppresses fault signatures. Multiple parameters identified by coupled observations need explicit rank/observability justification.

### Small example: filter
With inlet pressure, inlet temperature and air mass flow imposed, `Kfr` prescribed in direct, the outlet pressure is a predicted observable; in reverse, measured outlet pressure allows `Kfr` to be determined. Use actual MML4 `Filter.Kfr` graphical port, not a fictional component parameter. Size Kfr from desired pressure drop and the actual filter equation instead of arbitrary generic test values.

## Reverse-to-direct conversion procedure
Build/reconcile reverse first; run and confirm convergence; export relevant results. Invoke installed MML Python Toolkit `generate_direct_model.py` with reverse model, setup/calibration data and reverse result. Generate ramp variant using `-r` if supported by that checkout; run ramp; transfer appropriate final starts; generate nonramped final direct. Validate observable predictions against measurements and examine faults. Never invent CLI switches absent from the installed toolkit.

## Structural troubleshooting
Equation counts are necessary, not sufficient. Review double imposed quantities (e.g. a BC mass-flow sensor *and* another imposed volumetric cooling flow in condenser). When adding `PressureCut`/`LoopBreaker`, inspect their actual equations and assigned `DP_input`/`loop_flow_error`. Diagnose `+1 equation` or singularity by isolating duplicate physical constraints, not by blindly deleting an arbitrary connection. Distinguish a physical mismatch from poor initial values.

## Required output: reversal table
Columns: component, characteristic, physical effect, calibration sensor, direct input, reverse input, BC assumptions, unit convention, initial/nominal values, source/API verification, identified uncertainty.
