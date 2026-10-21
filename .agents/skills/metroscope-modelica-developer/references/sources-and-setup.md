# Model building: embedded Setup Guide workflow

This is the local operating procedure adapted from the Metroscope Setup Guide (*D - Build Model*, *F - Direct Model*) and the MML library architecture. Routine usage **must not depend on Notion access**. Source URLs and provenance live in `source-register.md`.

## Input dossier and scope
Before model creation collect: plant/process synoptic and equipment mapping, calibrated operating point (or declared realistic defaults), measured tags and units, control/regulation principles, available fault modes, target MML version/commit and Dymola/toolkit version. Record assumptions. Avoid substituting a fictional measurement for an unavailable tag.

## Build the reverse model
1. Select the official MML release for production and document the tag. Never silently use mutable `main`. For development, pin the target feature branch and commit.
2. Divide system into components, subsystems and whole plant. For each component inspect its actual `.mo` declaration, inherited connector definitions, and the closest **MML4** test. Do not treat older MML3 tests as graphical templates.
3. Examine pressure, flow, enthalpy, composition, power and regulation propagation for the *whole process* before deciding causality. Identify external BCs that are truly fault-independent.
4. Build the parameter reversal table: one row per calibrated parameter with physical effect, relevant measured sensor, BC assumptions, reverse/direct interface, initial guess and identifiability caveat. Prefer one independent calibration observable per independently identified parameter.
5. Construct a reverse `.mo` with graphical MML4 connectors. Impose BCs and calibration measurements; leave calibrated parameters solved by process equations. Preserve model-specific modifiers (e.g. geometric area) separately from `GenericReal` tunable parameters.
6. Add realistic initialization and solve iteratively: component first, then subsystem, then plant. Check physical sign conventions, range, connection types and equation/unknown balance; equal counts alone do not imply nonsingularity.
7. Preserve Dymola drawing: unscaled native icons, coherent routes, visible `connect(... ) annotation(Line(points=...))`.

## Direct model and delivery
1. Starting from the validated reverse model, transform causality: global BCs remain inputs; calibrated characteristics become imposed and measured calibration quantities become predictions. Preserve the sensors' `sensor_function="Calibration"` and `causality`.
2. For the formal conversion, inspect and use the selected MML Python Toolkit's `generate_direct_model.py` with reverse `.mo`, calibration setup/results and reverse `.mat`. Use its documented `-r` ramp procedure before deriving stable initialization for final direct without ramp; verify toolkit arguments from the installed version. Avoid naive search-and-replace.
3. Check observable values versus reference measurements, fault sensitivity, pressure/temperature profiles and numerical convergence.
4. Deliver reproducible `.mo` and, when requested and actually produced, `.mat`/`.fmu`, plus parameter reversal table, model/setup details, assumptions, version identifiers and test results. Never claim Dymola validation without running it.

## Core MML concepts
Steady-state 0D thermo-hydraulic modeling, acausal local equations and SI internals. Pressure is a connector potential; mass flow and shaft/thermal power use flow conventions; enthalpy and fluid composition are stream properties. Inspect component inheritance and partial models to understand signs, defaults and `start` parameters. UI sensor units follow `degC`, `barA`, `kg/s` when applicable; SI internal values remain K, Pa and kg/s.

## Routine decision policy
If this local guide and selected-commit API conflict, the verified actual API decides what code can compile; document the difference. Fetch Notion only when critical instructions are missing or when asked to refresh process guidance. Consult `source-register.md` for links.
