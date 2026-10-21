# Mandatory Dymola diagram layout review for MML4

## Layout before coding

1. Create a subsystem drawing plan with clear bands: gas turbine (top), exhaust-gas HRSG (middle), water/steam loop (lower), condenser cooling loop (separate), mechanical/electric power (upper or side).
2. Determine both the visual flow direction and component order; use a local coordinate grid and reserve space for sensors between units.
3. Place every P and T sensor immediately downstream of its respective component, or on an adjacent measurement branch with the physical tapping location unambiguous. Place flowmeters where requested (e.g. ST inlet and feedwater pump outlet). P and T sensors must not obscure unrelated ducts or units.
4. Place each measurement interface 15–35 diagram coordinate units from its sensor, near its `*_sensor` signal pin. `sensor_function`, `causality` and matching connector type must agree, independently of icon layout.
5. Put calibrated/fixed `Utilities.Interfaces.GenericReal` objects next to each owning component's actual parameter port (Kfr, Kth, eta_is, Cst, hn, rh, etc.). Never line up all interfaces in a distant row, which produces giant wire crossings. Keep pairs grouped and label clearly.
6. Distinguish gas, water/steam, cooling water and shaft paths with line colors appropriate to the MML examples. Use 90-degree polyline elbows instead of direct diagonal, long crossing lines where practical. Do not place the water loop on top of the HRSG hot-gas duct.
7. After every component move, recompute **all** affected `annotation(Placement(...))` and `annotation(Line(points=...))`. Account for extents, rotation and `iconTransformation`; connector center must match the first/last drawn point.
8. Run a static audit: every `connect` has `annotation(Line(...))`, no duplicate/missing graphical interfaces, no remote (>~50 diagram units) sensor-signal and parameter wires without rationale, no obviously disconnected ends, no severely overlapped symbols.
9. Open in Dymola Diagram view if available, visually inspect and repair until the process flow is clear. Do not claim visual QA on a headless static check. Translation, solving and graphical QA are separate deliverables.

## Adding a pressure cut / loop breaker

When required immediately before the feedwater pump in a single-pressure Rankine loop, use the actual same-version water/steam implementations `WaterSteam.Pipes.PressureCut` and `WaterSteam.Pipes.LoopBreaker` (after API verification), **in series**:

`condenser liquid outlet -> outlet P/T sensors -> PressureCut -> LoopBreaker -> feedwaterPump.C_in`

Preserve the pump outlet mass-flow measurement. Graph both new components immediately adjacent to pump inlet, not elsewhere. Validate whether both elements introduce correct degrees of freedom / structural equations for the intended circuit and check compilation in Dymola. Never assume a syntactically connected loop converges.

## Blocking examples

- `connect(component.Kth, remoteKth)` drawn across the entire system to a distant bottom-of-page parameter table.
- `connect(sensor.P_sensor, Pout)` with drawn endpoint coordinates not aligned to `Pout`'s declared `Placement`.
- Orphan interfaces or connections that cut through icons.
- Correct Modelica wiring but unreadable or invisible diagram.

## Never change icon size — acceptance gate
- Treat every canonical Modelica `Placement` icon bounding-box **width and height** as immutable, for equipment, sources/sinks, sensors and interfaces. Preserve orientation and `iconTransformation` when available.
- Move a component only by translating the placement coordinates: `(x1,y1),(x2,y2) -> (x1+dx,y1+dy),(x2+dx,y2+dy)`. For `origin=...` variants, change origin only. Do not rescale `extent`, either uniformly or non-uniformly.
- Fix collisions by creating more whitespace / enlarging the **Diagram coordinate system viewport**, not scaling icons. Re-route line annotations to actual connector coordinates after translations.
- Audit size parity against original or golden source; **unexpected resize is blocking** even if diagram otherwise looks good.
