---
name: metroscope-dymola-validator
description: Check, compile and simulate new or changed Metroscope Modelica models in real Dymola at the existing sensor initialization point. Use after creating an MML model, for Check Model or simulation requests, and to diagnose Dymola failures; preserve models, sensor starts and graphical interfaces.
---

# Metroscope Dymola Validator

Run real Dymola through its vendor-supplied Python interface. Validate the model at its existing initialization point, preserve diagnostic evidence, and report what actually ran. This skill complements the modeling conventions in `../metroscope-modelica-developer/SKILL.md` and is the default validation step for each new MML model.

## Initialization point, not an input scenario

User policy (2026-10-20): run only the initialization operating point defined by the sensors' `T_start`, `P_start`, `Q_start`, and other supported `*_start` fields. Do not generate `dsu.txt`, input trajectories, ramps, arbitrary external values, or a wrapper fixing the inputs. Simulate the actual model directly. In the verified Dymola/MML configuration, connected sensor signal starts propagate to the aliased input variables and Dymola holds those values when no input file exists.

Verify the propagation in the actual result, not just by assuming it: inventory external `BoundaryCondition` and reverse `CalibrationInput` interfaces, inspect their sensor connections, and map each input to its actual sensor `*_start` parameter in the manifest. Compare actual input values throughout the recorded trajectory to those parameters. Only input values are fixed at the initialization point; predicted observables need not equal their own start guesses.

If a quantity has no suitable sensor, keep its ordinary typed declaration and its equation, as required by the modeling skill. Use its declared start value for simulation; where available, map it to the same existing initialization parameter (e.g. steam enthalpy input start equals `steam_source.h_0`). Do not invent a sensor, graphical interface or a new numerical point. When a model has no external inputs, record `no_external_inputs: true` explicitly. A missing, conflicting or non-propagated start is a model issue to explain and correct, not permission to supply a different value. Never replace a sensor field by a numeric literal simply because result extraction did not find it.

Use a fresh output directory per run so stale `dsu.txt`, results or logs cannot contaminate the point. The runner deliberately creates no input file. Dymola's message that inputs are held at their start values is expected for this workflow; inspect and report other warnings separately.

## Execute

1. Locate the target source, full class name, selected MML checkout/commit and exact Modelica dependency. Inspect sensor APIs/units and the input-to-sensor graph. Fully qualify component paths where a test package can shadow a physical package such as `WaterSteam`.
2. Locate local Dymola, its matching Python-interface wheel, a compatible Python interpreter and the compiler. Start a private, hidden session; do not attach to a user's working session. On Windows/WSL run Windows Python with Windows Dymola. Use the existing session compiler or select GCC via WSL for that session only. Do not silently substitute Modelica 4.1 for a model requiring 4.0.
3. Prepare the small JSON manifest described in [references/usage.md](references/usage.md). Paths and environment are configurable; the runner does not download dependencies or install software. List libraries in dependency order, model source files in load order, input start mappings and equipment-specific physical checks. The script imports outside-package `.mo` files in memory with `setClassText`, preserving their `within` scope; it never saves user libraries.
4. Run `scripts/run_dymola_tests.py --manifest ... --output ... --dymola-exe ... --compiler ...`. It performs `checkModel`, then `simulateModel` (0–1 s, tolerance 1e-6) directly for each model whose check passes. Equal equation/unknown counts alone do not prove convergence. Batch mode retains independent per-model outcomes and exits nonzero if any model fails.
5. Inspect each report and logs, actual inputs versus sensor starts, finite results, time completion and physical checks. Include appropriate pressure/temperature ordering, flow signs, phase consistency, mass/energy balance and fixed assumptions. Do not apply condenser-specific thresholds to other equipment. Inspect solver/initialization warnings even when Dymola returns success. This is a fixed-point test, not proof of dynamic behavior or of an entire operating envelope.
6. On failure classify loading, class resolution, structural balance, compilation, initialization, numerical or physical-result failures before changing code. Preserve the previous logs; make a justified minimal correction and rerun the affected models in a new directory. Do not change boundary conditions, characteristic values or sensor starts just to obtain a green check. If license, environment or inaccessible dependencies prevent execution, report the actual blocker and stop unsupported convergence claims.

## Evidence and delivery

Keep the manifest, source hashes, Dymola version/compiler and library paths/hashes, JSON/CSV summary, per-model Check Model and translation logs, `dslog.txt`, compiler logs, MAT results, extracted CSV trajectories, checked model text, input-start comparisons and physical checks. The runner returns 0 only when the requested checks, simulation and result checks pass; a failure is never hidden behind a printed success message.

Report outcomes concisely: corrections, Check Model result, simulation/convergence, actual point and meaningful predicted results, remaining warnings/limits, and links to model and report. Do not claim a diagram has been visually inspected, that an FMU was produced, or that untested variants converge. If Dymola is unavailable, distinguish an unexecuted validation from a failed model.
