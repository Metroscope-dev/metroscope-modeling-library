#!/usr/bin/env python3
"""Check and simulate MML models at their existing sensor start values.

Uses the installed vendor Dymola Python interface. Does not rewrite models,
create input trajectories, save libraries, or attach to an existing session.
"""
from __future__ import annotations

import argparse
import ast
import csv
import hashlib
import importlib
import json
import math
from pathlib import Path
import re
import sys
import traceback

IDENT = re.compile(r"[A-Za-z_]\w*(?:\.[A-Za-z_]\w*)*")


def expression_signals(expression):
    """Validate the small arithmetic/comparison language used for physical checks."""
    tree = ast.parse(expression, mode="eval")
    allowed = (ast.Expression, ast.Constant, ast.Call, ast.Name, ast.Load,
               ast.BinOp, ast.UnaryOp, ast.Compare, ast.BoolOp,
               ast.Add, ast.Sub, ast.Mult, ast.Div, ast.Pow, ast.Mod,
               ast.USub, ast.UAdd, ast.Not, ast.And, ast.Or,
               ast.Lt, ast.LtE, ast.Gt, ast.GtE, ast.Eq, ast.NotEq)
    signals = set()
    for node in ast.walk(tree):
        if not isinstance(node, allowed):
            raise ValueError(f"Unsupported physical-check syntax: {type(node).__name__}")
        if isinstance(node, ast.Name) and node.id != "v":
            raise ValueError("Only v('signal.name') calls are allowed")
        if isinstance(node, ast.Call):
            if (not isinstance(node.func, ast.Name) or node.func.id != "v"
                    or len(node.args) != 1 or node.keywords
                    or not isinstance(node.args[0], ast.Constant)
                    or not isinstance(node.args[0].value, str)):
                raise ValueError("Use v('literal.signal.name') in physical checks")
            signals.add(node.args[0].value)
    return tree, signals


def load_manifest(path):
    data = json.loads(path.read_text(encoding="utf-8"))
    if not data.get("libraries") or not data.get("models"):
        raise ValueError("Manifest requires nonempty libraries and models")
    for library in data["libraries"]:
        resolve_file(path, library["file"])
    seen = set()
    for model in data["models"]:
        name = model["class_name"]
        if not IDENT.fullmatch(name) or name in seen:
            raise ValueError(f"Invalid or duplicate class_name: {name}")
        seen.add(name)
        for source in model.get("sources", []):
            resolve_file(path, source)
        starts = model.get("input_starts", {})
        scalar_starts = model.get("scalar_input_starts", {})
        if not starts and not scalar_starts and not model.get("no_external_inputs", False):
            raise ValueError(f"{name}: inventory the external inputs in input_starts")
        if not all(isinstance(k, str) and isinstance(v, str) for k, v in starts.items()):
            raise ValueError("input_starts maps input signal names to sensor *_start parameters")
        if not model.get("checks"):
            raise ValueError(f"{name}: provide at least one component-specific physical check")
        if starts.keys() & scalar_starts.keys():
            raise ValueError("An input cannot have both sensor and scalar start expectations")
        if not all(isinstance(k, str) and isinstance(v, (int, float)) and not isinstance(v, bool)
                   and math.isfinite(v) for k, v in scalar_starts.items()):
            raise ValueError("scalar_input_starts must contain the finite declared scalar start values")
        check_names = set()
        for check in model["checks"]:
            if check["name"] in check_names or check["name"] in {"finite_results", "completed_time"}:
                raise ValueError("Duplicate physical check name")
            check_names.add(check["name"])
            expression_signals(check["expression"])
    return data


def resolve_file(manifest, value):
    path = Path(value)
    path = (manifest.parent / path).resolve() if not path.is_absolute() else path.resolve()
    if not path.is_file():
        raise FileNotFoundError(path)
    return path


def write_json(path, data):
    path.write_text(json.dumps(data, indent=2, ensure_ascii=False), encoding="utf-8")


def import_source(dymola, path):
    """Import a .mo in memory even when its file is outside its within package."""
    text = path.read_text(encoding="utf-8-sig")
    match = re.match(r"\s*within\s*([^;]*);", text)
    parent = match[1].strip() if match else ""
    if parent and not IDENT.fullmatch(parent):
        raise ValueError(f"Unsupported within clause in {path}")
    return dymola.setClassText(parent, text[match.end():] if match else text)


def validate_model(dymola, model, manifest_path, out):
    out.mkdir()
    report = {"class_name": model["class_name"], "status": "running", "steps": {},
              "input_mode": "existing_sensor_starts", "sources": []}

    def persist():
        write_json(out / "report.json", report)

    def step(name, call):
        print(f'{model["class_name"]}: {name}', flush=True)
        value = call()
        report["steps"][name] = value
        (out / (name + ".log")).write_text(dymola.getLastErrorLog(), encoding="utf-8")
        persist()
        return value

    try:
        if not dymola.cd(out.as_posix()):
            raise RuntimeError("Dymola could not select the isolated output directory")
        # A fresh output directory ensures no old dsu.txt can change the input point.
        for i, value in enumerate(model.get("sources", [])):
            source = resolve_file(manifest_path, value)
            report["sources"].append({"file": str(source),
                "sha256": hashlib.sha256(source.read_bytes()).hexdigest()})
            if not step(f"load_source_{i}", lambda source=source: import_source(dymola, source)):
                raise RuntimeError(f"Cannot import {source}")
        if not step("check_model", lambda: dymola.checkModel(model["class_name"])):
            report["status"] = "check_failed"
            return report
        ok = step("simulate", lambda: dymola.simulateModel(
            model["class_name"], startTime=0.0, stopTime=1.0,
            numberOfIntervals=10, tolerance=1e-6, resultFile="result"))
        if not ok:
            report["status"] = "simulation_failed"
            return report
        names = {"Time", *model.get("signals", [])}
        starts = model.get("input_starts", {})
        names.update(starts)
        names.update(starts.values())
        names.update(model.get("scalar_input_starts", {}))
        for check in model["checks"]:
            names.update(expression_signals(check["expression"])[1])
        names = ["Time"] + sorted(names - {"Time"})
        count = dymola.readTrajectorySize("result.mat")
        if count < 2:
            raise RuntimeError("Missing or incomplete result trajectory")
        values = dymola.readTrajectory("result.mat", names, count)
        if len(values) != len(names) or any(len(v) != count for v in values):
            raise RuntimeError("Missing signals or inconsistent trajectory dimensions")
        traces = dict(zip(names, values))
        finite = all(math.isfinite(x) for series in values for x in series)
        if not finite:
            raise RuntimeError("Non-finite result values")
        report["results"] = {name: {"initial": series[0], "final": series[-1]}
                             for name, series in traces.items()}
        with (out / "trajectory.csv").open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(names)
            writer.writerows(zip(*values))
        point_checks = {}
        for signal, reference in starts.items():
            expected = traces[reference][0]
            point_checks[signal] = {
                "reference": reference, "expected": expected,
                "actual": traces[signal][0],
                "pass": all(math.isclose(x, expected, rel_tol=1e-6, abs_tol=1e-8)
                            for x in traces[signal])}
        for signal, expected in model.get("scalar_input_starts", {}).items():
            point_checks[signal] = {
                "reference": "reviewed ordinary input start declaration", "expected": expected,
                "actual": traces[signal][0],
                "pass": all(math.isclose(x, expected, rel_tol=1e-6, abs_tol=1e-8)
                            for x in traces[signal])}
        report["input_start_checks"] = point_checks
        checks = {"finite_results": finite,
                  "completed_time": math.isclose(traces["Time"][-1], 1.0, abs_tol=1e-9)}
        # Check the whole saved trajectory, not just its final point.
        for check in model["checks"]:
            code = compile(expression_signals(check["expression"])[0], "physical_check", "eval")
            passed = True
            for index in range(count):
                result = eval(code, {"__builtins__": {}}, {"v": lambda name: traces[name][index]})
                if not isinstance(result, bool):
                    raise ValueError(f'Check {check["name"]} must return a boolean')
                passed = passed and result
            checks[check["name"]] = passed
        report["physical_checks"] = checks
        report["status"] = ("passed" if all(checks.values()) and
                            all(item["pass"] for item in point_checks.values()) else "result_checks_failed")
        log = (out / "dslog.txt").read_text(encoding="utf-8", errors="replace")
        report["runtime_warning_lines"] = [line for line in log.splitlines()
                                            if "warning" in line.lower() or "assert" in line.lower()]
        report["expected_input_default_notice"] = "assumed to be at their start-value" in log
        # Save the exact imported model text for inspection, without saving user libraries.
        text, _ = dymola.getClassText(model["class_name"], includeAnnotations=True)
        (out / "checked_model.mo").write_text(text, encoding="utf-8")
    except Exception:
        report["status"] = "error"
        report["exception"] = traceback.format_exc()
        try:
            (out / "exception.log").write_text(dymola.getLastErrorLog(), encoding="utf-8")
        except Exception:
            pass
    finally:
        persist()
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True, help="New, empty output directory")
    parser.add_argument("--dymola-exe", type=Path)
    parser.add_argument("--interface", type=Path, help="Vendor wheel or directory containing dymola")
    parser.add_argument("--compiler", choices=["existing", "wsl"], default="existing")
    parser.add_argument("--validate-config", action="store_true", help="Validate paths/check syntax without starting Dymola")
    args = parser.parse_args()
    manifest_path = args.manifest.resolve()
    manifest = load_manifest(manifest_path)
    if args.validate_config:
        print(f'Valid configuration: {len(manifest["models"])} model(s)')
        return 0
    output = args.output.resolve()
    if output.exists() and any(output.iterdir()):
        parser.error("Output must be empty: prior logs/results must not be overwritten")
    output.mkdir(parents=True, exist_ok=True)
    if args.interface:
        sys.path.insert(0, str(args.interface.resolve()))
    elif args.dymola_exe:
        vendor = args.dymola_exe.resolve().parent.parent / "Modelica/Library/python_interface"
        wheels = sorted(vendor.glob("dymola-*.whl"))
        if wheels:
            sys.path.insert(0, str(wheels[-1]))
    dymola = None
    summary = {"input_mode": "existing_sensor_starts", "models": [], "libraries": [], "manifest": str(manifest_path), "manifest_sha256": hashlib.sha256(manifest_path.read_bytes()).hexdigest()}
    try:
        interface = importlib.import_module("dymola.dymola_interface")
        print("Starting dedicated Dymola session", flush=True)
        kwargs = {"showwindow": False, "nolibraryscripts": True, "customArgs": "/nosettings" if sys.platform == "win32" else "-nosettings"}
        if args.dymola_exe:
            kwargs["dymolapath"] = str(args.dymola_exe.resolve())
        dymola = interface.DymolaInterface(**kwargs)
        summary["dymola_version"] = dymola.ExecuteCommand("DymolaVersion()")
        if not dymola.cd(output.as_posix()):
            raise RuntimeError("Cannot select output directory")
        if manifest.get("unload"):
            summary["unload_result"] = dymola.eraseClasses(manifest["unload"])
            (output / "unload.log").write_text(dymola.getLastErrorLog(), encoding="utf-8")
        for index, library in enumerate(manifest["libraries"]):
            path = resolve_file(manifest_path, library["file"])
            ok = dymola.openModel(path.as_posix(), changeDirectory=False)
            summary["libraries"].append({**library, "file": str(path), "loaded": ok,
                "sha256": hashlib.sha256(path.read_bytes()).hexdigest()})
            (output / f"library_{index}.log").write_text(dymola.getLastErrorLog(), encoding="utf-8")
            if not ok:
                raise RuntimeError(f"Library failed to load: {path}")
        if args.compiler == "wsl":
            if sys.platform != "win32":
                raise ValueError("WSL compiler selection applies to Windows Dymola")
            dymola.SetDymolaCompiler("wsl", ["CCompiler=WSL", "WSLPath=C:/Windows/System32/wsl.exe"], mergeSettings=True)
        summary["compiler"] = dymola.GetDymolaCompiler()
        write_json(output / "summary.json", summary)
        for index, model in enumerate(manifest["models"]):
            folder = output / f'{index+1:02d}_{model["class_name"].split(".")[-1]}'
            result = validate_model(dymola, model, manifest_path, folder)
            summary["models"].append({"class_name": result["class_name"],
                                       "status": result["status"], "report": str(folder / "report.json")})
            write_json(output / "summary.json", summary)
        summary["status"] = "passed" if all(m["status"] == "passed" for m in summary["models"]) else "failed"
    except Exception:
        summary["status"] = "error"
        summary["exception"] = traceback.format_exc()
        print(summary["exception"], file=sys.stderr, flush=True)
    finally:
        if dymola:
            try:
                dymola.close()
            except Exception:
                summary["close_error"] = traceback.format_exc()
            dymola = None
        write_json(output / "summary.json", summary)
        with (output / "summary.csv").open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=["class_name", "status", "report"])
            writer.writeheader()
            writer.writerows(summary["models"])
    print(json.dumps({"status": summary["status"], "models": summary["models"]}), flush=True)
    return 0 if summary["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
