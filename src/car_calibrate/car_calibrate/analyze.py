"""不创建运动接口的离线 CSV 分析命令。"""

from __future__ import annotations

import argparse
import csv
from datetime import datetime
from pathlib import Path

import yaml

from .estimator import estimate_rows, sha256_file


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--measured-at")
    args = parser.parse_args()
    with args.input.open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    measured_at = args.measured_at or datetime.now().astimezone().isoformat(timespec="seconds")
    result = estimate_rows(
        rows,
        measured_at=measured_at,
        source_log_sha256=sha256_file(args.input),
    )
    if args.output.exists():
        raise FileExistsError(args.output)
    args.output.write_text(
        yaml.safe_dump(result.calibration, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )
    print(args.output)
