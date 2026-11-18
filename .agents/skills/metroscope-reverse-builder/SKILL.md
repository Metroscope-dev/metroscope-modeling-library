---
name: metroscope-reverse-builder
description: Complete an instrumented MML4 Modelica model into a reverse model after data screening, with frozen sensors and established graphical causality. Add BC, calibration, observable and parameter interfaces and check structural balance with up to three correction iterations. Excludes initialization convergence and reverse-to-direct conversion.
---

# Metroscope Reverse Builder

Complete the supplied instrumented model into a reverse `.mo` model. This is the post-screening interface and structural-balance stage defined by the user on 2026-10-22. Deliver the model and actual structural-check evidence. Initialization convergence is outside this stage.

## Entry contract

- Data screening is complete. The sensor inventory is frozen, including redundant sensors. Preserve every sensor, physical location, connection, `sensor_function`, `causality`, unit, start and graphical size/orientation. Do not add, remove, reclassify or retarget sensors to balance equations.
- Graphical sensor functions and their associated parameter targets are established modeling decisions. Read them as the source of the interface mapping. If causality is incorrect, incomplete or contradictory, diagnose it and propose a concrete correction using the procedure below rather than silently revising it.
- The modeler may have supplied assumptions to compensate for missing sensors. Inventory their equations, fixed values, scope, units and provenance; preserve and implement them as specified. Do not invent measurements, numerical values or physical hypotheses to close the model. If an indispensable assumption is missing, complete independent work and identify the decision needed from the modeler.
- Inspect the model and selected library checkout, including inherited sensor/interface definitions. Record MML ref/commit and Modelica dependency; do not migrate versions here. Preserve topology and physical equations except for justified interface wiring and removal of demonstrably duplicate declarations or constraints introduced by that wiring.

## Focused local references

Read the sibling developer skill's references as needed, without invoking its full creation/simulation workflow:

- [Sensor/interface contract](../metroscope-modelica-developer/references/sensor-interface-contract.md) and [graphical declarations](../metroscope-modelica-developer/references/mml4-graphics.md) for exact roles, signal pins and declaration rules.
- [Approved MML4 families](../metroscope-modelica-developer/references/mml4-approved-patterns.md) for the closest same-version parameter-interface pattern; inspect the actual selected API before coding.
- [Dymola layout](../metroscope-modelica-developer/references/dymola-layout.md) for placements and routing.

This stage's frozen sensor decisions, bounded structural iterations and exclusion of convergence take precedence over broader developer/validator defaults to simulate new models. Do not run the validator's default check-then-simulate runner here. Real Dymola Check Model may use environment-discovery guidance in the [validator skill](../metroscope-dymola-validator/SKILL.md), but stop before simulation.

## Incorrect or incomplete causality

Before wiring interfaces, check that each calibration target exists with exact spelling/case, that the measured quantity physically depends on it under the imposed BCs, and that the selected independent measurements can identify the free characteristics. Inspect whole-process pressure, flow, enthalpy and regulation propagation, existing assumptions, and redundant constraints. Distinguish an interface implementation error from a modeling causality error; matching equation counts alone does not justify a causal choice.

When causality is incorrectly described, **propose a solution**, not merely a blocking finding:

1. Identify the affected sensor/component, current `sensor_function` and `causality`, and the specific inconsistency with physical behavior, declared characteristics or independent constraints.
2. Recommend explicit before/after values for `sensor_function`, `causality` and the corresponding reverse interface, including the exact existing characteristic/port where relevant. Explain why this measurement can identify that characteristic under the existing BCs, or why it should remain a predicted observable. Preserve the full sensor inventory, including redundant sensors.
3. State the expected effect on imposed constraints, free characteristics and structural dependence. Label any unexecuted equation-count or identifiability assessment as a prediction. A redundant calibration measurement may be proposed as an Observable if another independent measurement already identifies the characteristic; justify this physically rather than applying it by default.
4. If the existing sensors cannot support the required identification, propose a specific physical assumption for the modeler to assess, with its scope and consequences, or explain the alternative of leaving the characteristic unidentifiable. Do not invent a numerical value or apply an unapproved assumption. If several causal assignments are plausible, recommend one and explain what missing process/control information distinguishes them.
5. Present a compact correction table (sensor, current role/target, proposed role/target/interface, physical reason, expected structural effect, decision needed). Obtain the modeler's validation before changing established sensor metadata or physical hypotheses, unless that exact correction is already authorized in the conversation. Continue independent interface work while that decision is pending. Once authorized, update metadata and interfaces consistently and resume the bounded structural checks.

These proposals may be prepared as soon as a causality defect is identified; do not spend three structural retries on a known modeling decision that requires the modeler. Numerical initialization remains outside scope.

## Complete the reverse interfaces

Build an audit table before editing: sensor instance, signal connector, `sensor_function`, `causality` target, external interface/name, unit, redundancy status if documented, and associated assumption. Include every frozen sensor; distinguish existing correct interfaces from additions.

| Established sensor role | Reverse interface | Constraint |
| --- | --- | --- |
| `sensor_function="BC"` | `Utilities.Interfaces.BoundaryCondition` | External imposed input; do not also fix it numerically in the model. |
| `sensor_function="Calibration"` | `Utilities.Interfaces.CalibrationInput` | Imposed calibration measurement; resolve the exact associated characteristic from `causality`. |
| `sensor_function="Unidentified"` | `Utilities.Interfaces.Observable` | Predicted measurement; do not impose its measured value as another equation. |

Inspect inherited defaults and the actual API for omitted or other role values. Do not infer a new role from the need for another equation. Redundancy does not justify deleting a sensor or imposing another calibration input: follow its established role. If frozen roles impose dependent constraints, report the dependency and retain the inventory and metadata.

1. Reuse existing correct interfaces; add missing ones and connect directly to verified sensor signal pins (`P_sensor`, `T_sensor`, etc.). Interface instances are the variable declarations. Do not create parallel scalar aliases or assignment equations for the same signal.
2. For each calibrated characteristic, add/reuse the graphical parameter interface supported by the selected approved family (`GenericReal` or `RealOutput`, verified in the target checkout). Preserve the exact name referenced by sensor `causality`, and connect it to the actual component characteristic port. Leave the characteristic solved by reverse equations; do not fix it at a guessed or direct-model value. Distinguish graphical calibrable variables from genuine Modelica `parameter` modifiers such as geometry. Do not impose one free characteristic per sensor by counting alone; retain documented coupled calibration.
3. Keep fixed assumptions separate from solved characteristics. Use supplied equations, modifiers or the supported `RealExpression` pattern as appropriate to verified endpoints. Do not free an assumed value or calibrate a new characteristic to absorb redundant measurements.
4. For a documented BC/quantity without a suitable sensor, retain or add the ordinary typed declaration and required equation according to the modeler's hypothesis. Do not invent a sensor or unconnected graphical interface. Preserve existing starts; `start` is not a physical equation.
5. Place new interfaces near their sensors/parameter ports, retain native/reference icon dimensions, and give every new graphical `connect` a valid `annotation(Line(points=...))` landing on actual transformed connector positions. Preserve existing layout; expand the canvas if necessary.

## Mandatory component-port and connection audit

Before structural checking and after every wiring change, audit **every** `connect` endpoint and its complete connection set against the actual selected-version declarations, including inheritance, redeclarations and connector aliases. Record component/port names, connector class, domain, scalar or record/array structure, `input`/`output`/acausal status, containing-model boundary versus internal-component scope, source location and physical purpose. Check all required component inputs/characteristics for a real connection or defining equation; a `start` is not a binding. Do not require an extra wire for a quantity already determined internally by equations.

- For causal signals between internal components, a producer's `output` must supply a consumer's `input`. Flag two internal outputs driving one set and input-only sets without a producer, external input or explicit defining equation. Inspect the **whole set**, not just pairs. An enclosing model input legitimately forwards to a component input; a component output legitimately exports through an enclosing model output. Do not reject these boundary connections because the two declarations spell `input`/`input` or `output`/`output`.
- `GenericReal` is acausal in the verified MML API; resolve the actual definition rather than guessing from its triangular icon. Parameter connectors and sensor signals can be solved through physical/model equations, without a causal output producer.
- Fluid `Inlet`/`Outlet` and shaft/power connectors with `flow`/`stream` fields are **acausal physical ports**, not Modelica signal inputs/outputs. Verify the intended process direction and sign convention: a simple series path normally joins the upstream outlet to the downstream inlet. Flag same-role two-port series connections for review. Splits and mixing nodes can legitimately join several inlets/outlets in one set; pressure-only taps have zero transport flow. Identify and justify those cases before calling a connection wrong. Several turbine power outlets can legitimately merge into a load inlet.
- Check fluid/media and hot/cold-side compatibility, sensor inlet/outlet order, pressure-tap polarity, branched routes, unexplained bypasses, self-connections and duplicates. Verify that each signal interface reaches the intended measured quantity or characteristic, with compatible units and record/array dimensions. Preserve documented physical junctions and redundant sensors.
- A flipped, rotated or moved icon does not change its declared port direction or the mathematical `connect` endpoints. Verify the code and then recompute line endpoints from the actual transformations; graphical contact alone is not a connection, and a connected line can be drawn to the wrong pin.

Deliver the connection audit and concrete findings with file/line, declared endpoint types, connection-set context and proposed correction. Fix demonstrated implementation wiring errors within established physical intent; request the modeler's decision when the intended physical route or junction is ambiguous. Distinguish definite errors, justified junctions/boundary forwarding, and unresolved cases. Check Model/translation complement this audit: equation balance or a successful type check cannot prove that process routing is physically correct. Do not claim full connector validation if endpoint types, bindings or dimensions were not resolved.

## Structural check and bounded correction loop

Complete the component-port and connection audit above, then run real Dymola `checkModel` on the completed reverse model with the selected dependencies. Retain the source revision/hash and full diagnostics. Record equations, unknowns and their difference when Dymola reports them; do not substitute counts of textual declarations or `connect` statements. Check dependent constraints and structural singularity diagnostics too: equal counts alone do not establish structural solvability or parameter identifiability.

After the initial check, allow **at most three correction iterations**, each comprising a justified minimal change and a new Check Model. Stop early when counts balance and no structural defect is reported. For every iteration record the diagnostic, change, resulting counts (or unavailable counts) and unresolved issues. Correct missing/wrong interface connections, duplicate aliases/constraints and API mistakes within established causality and hypotheses. Do not change starts, assumptions, frozen sensor roles or topology just to pass a check.

After three unsuccessful correction iterations, stop editing and ask for the modeler's opinion with a concrete dossier: current `.mo`, check logs, count history, unresolved variables/constraints, relevant sensors and assumptions, and the precise decision needed. Escalate earlier if continuing requires changing a frozen modeling decision or supplying a missing physical assumption; the iteration allowance does not authorize inventing either. Environment/license/dependency failures are execution blockers, not evidence of imbalance: report them and completed static work without consuming retries on identical unavailable checks or claiming verified counts.

Do not simulate, tune initialization, add ramps/wrappers/input scenarios, or troubleshoot convergence in this stage. Preserve starts even if later simulation may require work. Initialization convergence belongs to a subsequent validation stage.

## Delivery

Deliver the completed reverse `.mo`, sensor/interface/characteristic and connection audits, assumptions used (including missing ones), and compact check history with evidence paths. State whether equation/unknown balance and structural checks were actually verified, how many correction iterations were used, remaining decisions for the modeler, and any proposed causality correction with its validation status. Report initialization convergence as **not evaluated, outside scope**; do not describe a balanced model as numerically validated. Do not generate direct/faulty variants or FMUs here.
