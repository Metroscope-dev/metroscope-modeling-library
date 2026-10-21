#!/usr/bin/env python3
"""Show collinear fault couples from a Metroscope fault-matrix CSV.

This script intentionally uses only the Python standard library so it can run
from a plain WSL Python installation without numpy, pandas, or a virtual
environment.
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from dataclasses import dataclass
from itertools import combinations
from pathlib import Path


METADATA_FIELDS = {
    "name",
    "id",
    "enabled",
    "activated",
    "active",
    "description",
    "unit",
    "min",
    "max",
    "minimumvalue",
    "maximumvalue",
    "amplitude",
    "magnitude",
    "system",
    "equipmenttype",
    "degradationtype",
    "failurecategory",
    "equipments",
    "suspectedelements",
    "reparability",
}
ACTIVE_VALUES = {"1", "true", "yes", "y", "on", "enabled", "active"}


@dataclass(frozen=True)
class Couple:
    fault_a: str
    fault_b: str
    angle: float

    @property
    def separation(self) -> float:
        return min(self.angle, 180.0 - self.angle)

    @property
    def direction(self) -> str:
        return "Same" if self.angle <= 90.0 else "Opposite"


def normalized(value: object) -> str:
    return "".join(character for character in str(value).lower() if character.isalnum())


def find_column(columns: list[str], names: set[str]) -> str | None:
    return next((column for column in columns if normalized(column) in names), None)


def detect_delimiter(path: Path) -> str:
    with path.open("r", encoding="utf-8-sig", errors="replace", newline="") as stream:
        sample = stream.read(16_384)
    try:
        return csv.Sniffer().sniff(sample, delimiters=";,\t|").delimiter
    except csv.Error:
        return ";"


def read_matrix(path: Path) -> tuple[list[str], list[dict[str, str]]]:
    delimiter = detect_delimiter(path)
    with path.open("r", encoding="utf-8-sig", errors="replace", newline="") as stream:
        reader = csv.DictReader(stream, delimiter=delimiter)
        columns = [column.strip() for column in (reader.fieldnames or []) if column]
        rows = [
            {str(key).strip(): (value or "").strip() for key, value in row.items() if key}
            for row in reader
        ]
    if not columns:
        raise ValueError("the file has no header")
    if not rows:
        raise ValueError("the file contains no fault rows")
    return columns, rows


def select_active(
    columns: list[str], rows: list[dict[str, str]], include_all: bool
) -> list[dict[str, str]]:
    if include_all:
        return rows
    active_column = find_column(columns, {"activated", "active", "enabled"})
    if active_column is None:
        return rows
    return [
        row
        for row in rows
        if row.get(active_column, "").strip().lower() in ACTIVE_VALUES
    ]


def numeric_value(value: str) -> float | None:
    text = value.strip()
    if not text:
        return None
    try:
        result = float(text)
    except ValueError:
        return None
    return result if math.isfinite(result) else None


def observable_columns(columns: list[str], rows: list[dict[str, str]]) -> list[str]:
    observables: list[str] = []
    for column in columns:
        if normalized(column) in METADATA_FIELDS:
            continue
        populated = [row.get(column, "") for row in rows if row.get(column, "").strip()]
        if not populated:
            continue
        numeric_count = sum(numeric_value(value) is not None for value in populated)
        if numeric_count / len(populated) >= 0.95:
            observables.append(column)
    if not observables:
        raise ValueError(
            "no numeric observable columns were found; expected one fault per row "
            "and observable signatures in numeric columns"
        )
    return observables


def calculate(
    columns: list[str], rows: list[dict[str, str]], threshold: float
) -> tuple[list[Couple], int, int]:
    name_column = find_column(
        columns, {"name", "fault", "failure", "faultname", "failurename"}
    )
    if name_column is None:
        name_column = columns[0]
    observables = observable_columns(columns, rows)

    signatures: list[tuple[str, list[float], float]] = []
    for row in rows:
        name = row.get(name_column, "").strip()
        vector = [numeric_value(row.get(column, "")) or 0.0 for column in observables]
        norm = math.sqrt(sum(value * value for value in vector))
        if name and norm > 0.0 and math.isfinite(norm):
            signatures.append((name, vector, norm))

    couples: list[Couple] = []
    for first, second in combinations(signatures, 2):
        name_a, vector_a, norm_a = first
        name_b, vector_b, norm_b = second
        dot_product = sum(a * b for a, b in zip(vector_a, vector_b))
        cosine = max(-1.0, min(1.0, dot_product / (norm_a * norm_b)))
        angle = math.degrees(math.acos(cosine))
        if angle <= threshold or angle >= 180.0 - threshold:
            couples.append(Couple(name_a, name_b, angle))

    couples.sort(key=lambda couple: (couple.separation, couple.fault_a, couple.fault_b))
    return couples, len(signatures), len(observables)


def cell_lines(value: str, width: int) -> list[str]:
    words = value.split()
    if not words:
        return [""]
    lines: list[str] = []
    current = ""
    for word in words:
        if len(word) > width:
            if current:
                lines.append(current)
                current = ""
            while len(word) > width:
                lines.append(word[:width])
                word = word[width:]
        candidate = word if not current else f"{current} {word}"
        if len(candidate) <= width:
            current = candidate
        else:
            lines.append(current)
            current = word
    if current:
        lines.append(current)
    return lines


def print_row(values: list[str], widths: list[int], align_right: set[int] | None = None) -> None:
    align_right = align_right or set()
    wrapped = [cell_lines(value, width) for value, width in zip(values, widths)]
    for line_number in range(max(len(lines) for lines in wrapped)):
        cells: list[str] = []
        for index, (lines, width) in enumerate(zip(wrapped, widths)):
            value = lines[line_number] if line_number < len(lines) else ""
            cells.append(value.rjust(width) if index in align_right else value.ljust(width))
        print("│ " + " │ ".join(cells) + " │")


def display(
    path: Path, couples: list[Couple], threshold: float, faults: int, observables: int
) -> None:
    print(f"\nFault collinearity analysis — {path.name}")
    print(
        f"{faults} faults · {observables} observables · "
        f"threshold ≤ {threshold:g}° from parallel or opposite\n"
    )
    widths = [3, 30, 30, 9, 10, 9]
    headers = ["#", "Fault A", "Fault B", "Angle", "Separation", "Direction"]
    top = "┌" + "┬".join("─" * (width + 2) for width in widths) + "┐"
    divider = "├" + "┼".join("─" * (width + 2) for width in widths) + "┤"
    bottom = "└" + "┴".join("─" * (width + 2) for width in widths) + "┘"
    print(top)
    print_row(headers, widths)
    print(divider)
    for rank, couple in enumerate(couples, 1):
        print_row(
            [
                str(rank),
                couple.fault_a,
                couple.fault_b,
                f"{couple.angle:.2f}°",
                f"{couple.separation:.2f}°",
                couple.direction,
            ],
            widths,
            {0, 3, 4},
        )
    print(bottom)
    print(f"Found {len(couples)} collinear couple(s).")


def save(path: Path, couples: list[Couple]) -> None:
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            ["fault_a", "fault_b", "angle_degrees", "separation_degrees", "direction"]
        )
        for couple in couples:
            writer.writerow(
                [
                    couple.fault_a,
                    couple.fault_b,
                    f"{couple.angle:.3f}",
                    f"{couple.separation:.3f}",
                    couple.direction.lower(),
                ]
            )


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Show collinear fault couples and angles from a fault-matrix CSV."
    )
    parser.add_argument("fault_matrix", type=Path)
    parser.add_argument("-t", "--threshold", type=float, default=30.0)
    parser.add_argument("--all-faults", action="store_true")
    parser.add_argument("-o", "--output", type=Path)
    arguments = parser.parse_args()

    if not 0.0 <= arguments.threshold <= 90.0:
        parser.error("--threshold must be between 0 and 90 degrees")
    if not arguments.fault_matrix.is_file():
        parser.error(f"file not found: {arguments.fault_matrix}")

    try:
        columns, rows = read_matrix(arguments.fault_matrix)
        rows = select_active(columns, rows, arguments.all_faults)
        if not rows:
            raise ValueError("no active faults were found")
        couples, faults, observables = calculate(columns, rows, arguments.threshold)
        display(arguments.fault_matrix, couples, arguments.threshold, faults, observables)
        if arguments.output:
            save(arguments.output, couples)
            print(f"\nSaved: {arguments.output}")
    except (OSError, UnicodeError, csv.Error, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
