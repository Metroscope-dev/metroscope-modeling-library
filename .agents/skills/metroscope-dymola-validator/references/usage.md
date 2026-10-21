# Runner manifest and local execution

`scripts/run_dymola_tests.py` uses Python's standard library plus the matching Dymola vendor wheel. It supports one or multiple ordinary, dotted Modelica class names. It does not parse or infer arbitrary Modelica topology: the modeling agent reviews the actual source and supplies the small input-start mapping and physical checks. Models using quoted identifiers or a source beginning with comments before `within` require adaptation of the loader; do not guess their namespace.

## Manifest

Paths are absolute or relative to the manifest file, interpreted by the Python process executing Dymola. When running Windows Python from WSL, use Windows paths in the manifest and arguments. Use a new output directory for every attempt.

```json
{
  "unload": ["Modelica", "ModelicaServices", "Complex"],
  "libraries": [
    {"file": "dependencies/ModelicaStandardLibrary-4.0.0/Complex.mo", "version": "4.0.0"},
    {"file": "dependencies/ModelicaStandardLibrary-4.0.0/ModelicaServices/package.mo", "version": "4.0.0"},
    {"file": "dependencies/ModelicaStandardLibrary-4.0.0/Modelica/package.mo", "version": "4.0.0"},
    {"file": "../../MetroscopeModelingLibrary/package.mo", "version": "4.0.0-beta"}
  ],
  "models": [{
    "class_name": "MetroscopeModelingLibrary.Tests.WaterSteam.HeatExchangers.Condenser_direct_MML4",
    "sources": ["Condenser_direct_MML4.mo"],
    "input_starts": {
      "CW_T_in": "CW_T_in_sensor.T_start",
      "CW_P_in": "CW_P_in_sensor.P_start",
      "steam_flow": "steam_flow_sensor.Q_start",
      "steam_enthalpy": "steam_source.h_0"
    },
    "signals": ["P_cond", "CW_T_out", "condenser.W"],
    "checks": [
      {"name": "positive_heat", "expression": "v('condenser.W') > 0"},
      {"name": "temperature_order", "expression": "v('CW_T_in') < v('CW_T_out') and v('CW_T_out') < v('condenser.Tsat') - 273.15"},
      {"name": "heat_sides_balance", "expression": "-1e-5 * v('condenser.W') < v('condenser.cold_side.W') + v('condenser.hot_side.W') < 1e-5 * v('condenser.W')"}
    ]
  }]
}
```

`unload` applies only to the fresh private session, to remove preloaded incompatible libraries. Dymola can return false if one listed class was absent while still unloading the others; inspect its log and subsequent library loads. A library entry's version records the intended pin; confirm it against the actual package source and loading logs rather than treating manifest text alone as proof of the loaded version.

For a model already registered in the loaded libraries, `sources` can be empty. For a separate file with a nonempty `within`, the runner uses `setClassText(parent, source_without_within)`: opening it as a top-level file may otherwise return true while Dymola warns of a mismatching `within` and checks the wrong class. No source file or package.order is saved by the runner.

`input_starts` maps every external BC/calibration input to an existing parameter in the SAME model. Mapping is not an assignment: the script only reads and compares the saved results. No independent numeric input values are introduced. A parameter expression such as `T_start` may itself depend on other parameters; Dymola evaluates it. Read the correct field from the actual API (including power, humidity, etc.), and respect its signal unit. `P_0`/`T_0` remain internal SI guesses, distinct from signal `P_start`/`T_start`.

For a scalar input with no sensor, confirm its declaration contains the intended start and an appropriate source equation. If its literal start has no existing named parameter to compare to, use `"scalar_input_starts": {"steam_enthalpy": 2300000}` with the SAME reviewed declaration value. This is only a result comparison, never a supplied input signal. Keep `input_starts` for instrumented inputs and do not list the same input in both mappings. Do not add a redundant interface or initialization parameter just for the validator.

Physical expressions accept arithmetic, comparisons, booleans and `v('signal')` only. Attributes, indexing, imports and arbitrary function calls are rejected. Every referenced signal is extracted automatically and every check is evaluated at all recorded points. Expected aliases/sensor starts must be extractable from the MAT file: if result storage is configured to omit them, enable their logging or diagnose the missing signal; never silently skip input verification. Avoid check names reserved for the runner (`finite_results`, `completed_time`).

## Commands

Validate the manifest and paths without starting Dymola:

```bash
python run_dymola_tests.py --manifest manifest.json --output new-run --validate-config
```

Windows Dymola, called from WSL (replace interpreter/project paths with the actual locations):

```bash
'/mnt/c/Program Files/Inkscape/bin/python.exe' \
  'C:\Works\project\.agents\skills\metroscope-dymola-validator\scripts\run_dymola_tests.py' \
  --manifest 'C:\Works\project\validation\manifest.json' \
  --output 'C:\Works\project\validation\run-001' \
  --dymola-exe 'C:\Program Files\Dymola 2026x\bin64\Dymola.exe' \
  --compiler wsl
```

The wheel is discovered under the selected Dymola install's `Modelica/Library/python_interface`, or supplied with `--interface PATH`. On native Linux, use Linux Dymola/Python and `--compiler existing`, with its compiler already available. The native-Linux path has not been execution-tested by this skill's initial acceptance test.

## Outputs and failure semantics

- `summary.json` and `summary.csv`: every requested model, final status and report path.
- Per-model `report.json`: stages, source hashes, input mode, actual input/reference values, physical checks, results and exceptions.
- Per-model `check_model.log`, `simulate.log`, `buildlog.txt`, `dslog.txt`, `result.mat`, `trajectory.csv`, `checked_model.mo` when their corresponding stages ran.
- Exit code 0: all requested models passed. Exit code 1/nonzero: a configuration, environment, Check Model, simulation or result-check failure. A failed Check Model stops simulation for that model while other independent models can continue.

The expected `dsu.txt`-absent notice is retained: it documents that Dymola holds model inputs at their starts. Validate their actual values using the saved comparisons. It is not a reason to create input trajectories. Other warnings, including failed assertions or solver recovery, need engineering review even if the return code is successful.

## Acceptance evidence (2026-10-20)

The runner was executed with Dymola 2026x, MSL v4.0.0 and GCC via WSL against the direct condenser and the user-provided AirCompressor direct pattern. Both Check Model and simulation passed, with every external input equal to its sensor start (and ordinary enthalpy start) throughout the result, and no `dsu.*` generated. A deliberately false physical check produced `result_checks_failed` and exit code 1 even though Dymola simulation succeeded. These tests establish runner behavior; they do not validate every future model.

Run the fast logic tests with `python -m unittest discover -s scripts -p 'test_*.py' -v`. These check expression validation, required input inventory and that a failed Check Model prevents simulation. They are not a substitute for the real Dymola test.
