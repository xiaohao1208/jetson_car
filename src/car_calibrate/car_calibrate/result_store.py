"""成功标定历史结果与最新快照的安全发布。"""

from __future__ import annotations

import os
from pathlib import Path
import shutil
import tempfile


CALIBRATION_FILES = ("calibration.yaml", "calibration_raw.csv")


def _fsync_directory(path: Path) -> None:
    descriptor = os.open(path, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def publish_calibration_snapshot(source: Path, destination: Path) -> None:
    """用已完成的历史 run 事务式替换固定的最新标定目录。"""

    source = Path(source).resolve()
    raw_destination = Path(destination).expanduser()
    destination = raw_destination.parent.resolve() / raw_destination.name
    if not source.is_dir():
        raise FileNotFoundError(f"标定历史目录不存在: {source}")
    for name in CALIBRATION_FILES:
        if not (source / name).is_file():
            raise FileNotFoundError(f"标定历史目录缺少 {name}: {source}")

    parent = destination.parent
    parent.mkdir(parents=True, exist_ok=True)
    staging = Path(tempfile.mkdtemp(
        prefix=f".{destination.name}.", suffix=".pending", dir=parent,
    ))
    backup = parent / f".{destination.name}.previous"
    installed = False
    previous_saved = False
    try:
        for name in CALIBRATION_FILES:
            target = staging / name
            shutil.copy2(source / name, target)
            with target.open("rb") as stream:
                os.fsync(stream.fileno())
        _fsync_directory(staging)

        if backup.exists():
            if destination.exists():
                shutil.rmtree(backup)
            else:
                os.replace(backup, destination)
        if destination.exists():
            if not destination.is_dir() or destination.is_symlink():
                raise FileExistsError(f"最新标定路径不是普通目录: {destination}")
            os.replace(destination, backup)
            previous_saved = True
        os.replace(staging, destination)
        installed = True
        _fsync_directory(parent)
    except Exception:
        if installed and destination.exists():
            shutil.rmtree(destination)
        if previous_saved and backup.exists():
            os.replace(backup, destination)
            _fsync_directory(parent)
        raise
    finally:
        if staging.exists():
            shutil.rmtree(staging)

    if backup.exists():
        shutil.rmtree(backup)
        _fsync_directory(parent)
