# MML4 graphical coding conventions

## Core architectural change from MML3
MML4 test models are built with graphic sensor connectors and typed external interfaces. Do not reproduce historical `input Real`/`output Real`-only graphical layouts. Graphical calibrable component ports (`GenericReal`, e.g. `AirCompressor.tau`, `.eta_is`, `Filter.Kfr`, heat exchanger `Kth`) are *not* necessarily plain modifiers; other settings such as a geometric surface `S` remain component modifiers if declared as such in selected code. Determine exact names from target class and inherited classes.

## Interface instances are the variable declarations

User clarification, 2026-10-20: declare model quantities through interface instances and connect them directly to sensors and components, minimizing handwritten equations. This applies to variable declarations, not only to the appearance of the diagram.

```modelica
// Declaration section: these instances ARE the model variables.
Utilities.Interfaces.BoundaryCondition CW_T_in;
Utilities.Interfaces.Observable P_cond;
Utilities.Interfaces.RealOutput condenser_Kth; // NPP/CCGT pattern
Utilities.Interfaces.RealExpression condenser_C_incond(y=0);
equation
  condenser_Kth = 2500; // Direct fixed characteristic, as in approved patterns.
  connect(CW_T_in, CW_T_in_sensor.T_sensor);
  connect(P_cond_sensor.P_sensor, P_cond);
  connect(condenser_Kth, condenser.Kth);
  connect(condenser_C_incond.y, condenser.C_incond);
```

The snippet omits placements/line annotations only for readability; generated `.mo` files require them. AirCompressor uses `GenericReal` instead of `RealOutput` for its characteristics. Select the approved family, as documented in `mml4-approved-patterns.md`.

Do not additionally declare `input Real CW_T_in_value`, `Real measured_pressure`, or `parameter Real Kth_ref` and then copy them to/from the interfaces. Likewise, avoid assignments such as `P_cond = P_cond_sensor.P_sensor` or `condenser.Kth = condenser_Kth` when both endpoints are connectors: use `connect`. An ordinary scalar calculation, a physical conversion not handled by a sensor, a composition assumption or a direct fixed-characteristic equation can still be necessary; do not delete real constraints just to minimize the textual equation count.

### Quantity without a suitable sensor

User clarification, 2026-10-20: when the selected API has no suitable sensor for a physical variable, declare that variable normally and use equations. Do not create a graphical interface with no actual sensor endpoint and do not draw a wire to an invented location on a component. The fact that a source internally types its field as a scalar connector does not override this modeling convention.

For the isolated condenser's wet-steam inlet, no enthalpy sensor is available:

```modelica
input Utilities.Units.SpecificEnthalpy steam_enthalpy(start=2300000);
equation
  steam_source.h_out = steam_enthalpy;
```

This scalar is the actual BC declaration, not an alias duplicating an interface. Do not also declare `Utilities.Interfaces.BoundaryCondition steam_enthalpy`, and do not replace the assignment by a drawn `connect` to the source symbol. `start` remains initialization information; for the default initialization-point validation, Dymola holds the ordinary enthalpy input at its declared start. Verify its actual value and keep the source equation.

Review the declaration section together with the equation section: instrumented quantities and graphical characteristics use interfaces connected to their real endpoints; quantities without a suitable sensor use ordinary typed declarations and the necessary equations. Neither path should duplicate the variable. Report necessary explicit equations and their purpose. Reducing manually authored equations and aliases does not imply fewer physical constraints: Modelica expands connection sets into equations during flattening.

## Sensor contract
`sensor_function="BC"` routes to `BoundaryCondition` in both direct and reverse. `sensor_function="Calibration"` routes to `CalibrationInput` in reverse and `Observable` in direct; its `causality` always names the corresponding calibrated parameter. `sensor_function="Unidentified"` routes to `Observable` in both. This is an external interface mapping, **not** a change to `sensor_function` metadata between models. Refer to `sensor-interface-contract.md` for the audit procedure. The latest user-approved MML4 namespace is **`SensorsControl`**, without underscore; `SensorsControl` in an older branch is legacy evidence only. Verify selected checkout.

## Graphical connections and annotations
Every graphical fluid/heat/power connection and supported sensor/interface or graphical characteristic connection requires actual `connect(a,b)` in `equation` with `annotation(Line(points={...}, color={...}))`; without Line annotation it may not be visible in Dymola Diagram view. Set endpoints from transformed connector positions, not bounding-box centers. Keep flow directions and fluid/shaft signal routes coherent. Preserve **all icon sizes**; only translate placement coordinates or expand canvas, never rescale extents. Position sensors just downstream of measured process components, and interfaces immediately by sensor signal pins. Place `GenericReal` adjacent to its parameter port. `references/dymola-layout.md` contains strict drawing acceptance criteria.

## Initialization and units
Specify meaningful `P_start` in `barA`, `T_start` in `degC`, `Q_start` in `kg/s`, plus inherited SI starts (`P_0=1e5*P_start`, `T_0=T_start+273.15`, `Q_0=Q_start`) **only if actual class supports the modifier**. Do not assume a sensor exposes all modifiers. Set `signal_unit`/`display_unit` only with supported enum literals. For steam and gas equipment, estimate pressure/temperature/enthalpy/flow state along every branch of the plant; reference `operating-points-and-units.md`.

## Model interface versus test excitation

Evidence: the user-provided `assets/mml4-reference/AirCompressor_direct.mo` leaves its three `BoundaryCondition` connectors as external inputs, fixes the two `GenericReal` characteristics through numerical equations, and connects the sensor signals using `connect`. Apply this distinction when adapting that pattern to another component, while verifying the other component's physical causality and API.

- A `BoundaryCondition` intended as an external input must not also be fixed by an equation such as `ambient_temp = 15` inside the delivered model. For the default validation requested by the user, simulate the original model directly at the sensor initialization point, with no input file or wrapper. Confirm that input values in the Dymola result equal the connected sensor *_start parameters. External input scenarios or a closed standalone experiment require a distinct user request.
- `start`, `P_start`, `T_start`, `Q_start` and inherited `*_0` values are initialization information. They are not equations imposing a physical boundary value. Dymola can propagate the connected sensor signal starts to aliased inputs and hold them for a no-input-file simulation, as verified on the condenser and AirCompressor patterns. This is execution behavior, not an extra model equation: confirm it in the results using the validation skill. Do not infer that predicted observables must equal their initial guesses.
- Follow the selected reference's direct-characteristic pattern: AirCompressor uses `Utilities.Interfaces.GenericReal`; the approved CCGT/NPP patterns use `Utilities.Interfaces.RealOutput`. Connect the characteristic interface to the component port and fix its calibrated or illustrative value in `equation`. See `mml4-approved-patterns.md` for these verified differences. Keep genuine internal parameters such as area or height as component modifiers. Avoid adding a second top-level `*_ref` parameter layer merely to restate each graphical characteristic; it is an adaptation choice to explain, not a demonstrated library requirement.
- Do not invent a sensor for a quantity absent from the selected API. A wet-steam inlet can need an enthalpy BC rather than a temperature BC. When no suitable sensor exists, declare the enthalpy as a normal typed variable and assign it to the source with an equation, as explained above.
- Make declarations readable like the supplied `.mo` reference: multiline modifiers and placements, recognizable class paths, and complete line annotations. Preserve the reference's component and interface icon dimensions and distinguish `Placement.transformation` from `iconTransformation`.

Before delivery, compare the model with the approved reference and report separately: sensor/interface roles, equations defining external inputs, equations fixing characteristics, initialization, graphics, and compilation/simulation. A check that merely counts `connect` statements or recognizes `GenericReal` declarations does not verify the input contract, diagram readability, flattened equation balance or user-standard compliance.

When the user rejects a model's MML4 conformity without identifying the specific differences, ask which reference or conventions they intend while fixing demonstrated deviations. Do not turn guessed objections into permanent library-wide rules, and do not label the replacement approved until the unresolved conventions are settled.

## Fault variants
Faulty tests must explicitly activate the component's actual supported fault (`faulty=true` plus the documented fault magnitude); assert differences in output observables. Never fabricate arbitrary failure fields.

## Verification
Read actual component and inherited sources, compare three user-supplied canonical `AirCompressor_*.mo` files in `assets/mml4-reference/`, inspect reverse/direct/faulty roles, check `connect` graph and annotation placements, check equation count, then compile/run Dymola if available. No static regex proves Dymola render or convergence.
