#!/usr/bin/env python3
# 必选：由 get_model launch 调用，完成模型包校验、准备和原子部署。

import argparse
import datetime
import fcntl
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import shutil
import stat
import subprocess
import sys
import tempfile
import zipfile

import yaml


ARCHIVE_NAME = "car_rl_model.zip"
MAX_ARCHIVE_MEMBERS = 10000
MAX_UNCOMPRESSED_BYTES = 8 * 1024 * 1024 * 1024


class DeploymentError(RuntimeError):
    """模型部署输入或运行状态不满足要求。"""


def sha256_file(path):
    """流式计算文件摘要，避免把大模型一次读入内存。"""
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def read_expected_hash(checksum_path, archive_name):
    """读取标准 sha256sum 文件并核对其中的文件名。"""
    try:
        lines = [
            line.strip()
            for line in checksum_path.read_text(encoding="utf-8").splitlines()
            if line.strip()
        ]
    except (OSError, UnicodeError) as exception:
        raise DeploymentError(f"无法读取模型校验文件：{checksum_path}") from exception
    if len(lines) != 1:
        raise DeploymentError("模型校验文件必须只包含一条 SHA-256 记录")
    fields = lines[0].split()
    if len(fields) != 2 or len(fields[0]) != 64:
        raise DeploymentError("模型校验文件格式无效")
    expected = fields[0].lower()
    if any(character not in "0123456789abcdef" for character in expected):
        raise DeploymentError("模型校验文件中的 SHA-256 无效")
    recorded_name = fields[1].lstrip("*")
    if recorded_name != archive_name:
        raise DeploymentError(
            f"模型校验文件记录的文件名不匹配：{recorded_name}"
        )
    return expected


def verify_archive_pair(archive_path):
    """确认 ZIP 与相邻摘要文件完整且相互匹配。"""
    if archive_path.name != ARCHIVE_NAME:
        raise DeploymentError(f"模型压缩文件名必须是 {ARCHIVE_NAME}")
    checksum_path = Path(f"{archive_path}.sha256")
    if not archive_path.is_file():
        raise DeploymentError(f"模型压缩文件不存在：{archive_path}")
    if not checksum_path.is_file():
        raise DeploymentError(f"缺少相邻校验文件：{checksum_path}")
    expected = read_expected_hash(checksum_path, archive_path.name)
    actual = sha256_file(archive_path)
    if actual != expected:
        raise DeploymentError(f"模型压缩文件 SHA-256 校验失败：{archive_path}")
    return actual, checksum_path


def atomic_copy_pair(source_archive, source_checksum, inbox, digest):
    """先完成临时复制和复核，再替换收件目录中的正式文件。"""
    inbox.mkdir(parents=True, exist_ok=True)
    destination_archive = inbox / ARCHIVE_NAME
    destination_checksum = Path(f"{destination_archive}.sha256")
    if source_archive.resolve() == destination_archive.resolve():
        return destination_archive

    temporary_archive = inbox / f".{ARCHIVE_NAME}.{os.getpid()}.tmp"
    temporary_checksum = inbox / f".{ARCHIVE_NAME}.sha256.{os.getpid()}.tmp"
    try:
        shutil.copyfile(source_archive, temporary_archive)
        if sha256_file(temporary_archive) != digest:
            raise DeploymentError("模型压缩文件复制后摘要发生变化")
        shutil.copyfile(source_checksum, temporary_checksum)
        if read_expected_hash(temporary_checksum, ARCHIVE_NAME) != digest:
            raise DeploymentError("模型校验文件复制后内容发生变化")
        os.replace(temporary_archive, destination_archive)
        os.replace(temporary_checksum, destination_checksum)
    finally:
        temporary_archive.unlink(missing_ok=True)
        temporary_checksum.unlink(missing_ok=True)
    print(f"模型包已复制到收件目录：{destination_archive}", flush=True)
    return destination_archive


def safe_extract(archive_path, destination):
    """在解压前检查全部成员，拒绝路径逃逸和特殊文件。"""
    try:
        archive = zipfile.ZipFile(archive_path, "r")
    except (OSError, zipfile.BadZipFile) as exception:
        raise DeploymentError("模型压缩文件不是有效 ZIP") from exception

    with archive:
        members = archive.infolist()
        if not members or len(members) > MAX_ARCHIVE_MEMBERS:
            raise DeploymentError("模型 ZIP 文件数量为空或超过安全上限")
        total_size = sum(member.file_size for member in members)
        if total_size > MAX_UNCOMPRESSED_BYTES:
            raise DeploymentError("模型 ZIP 解压后大小超过 8 GiB 安全上限")

        seen_names = set()
        checked_members = []
        for member in members:
            name = member.filename
            if not name or "\\" in name or "\x00" in name:
                raise DeploymentError("模型 ZIP 包含无效路径")
            path = PurePosixPath(name)
            if path.is_absolute() or ".." in path.parts:
                raise DeploymentError(f"模型 ZIP 包含不安全路径：{name}")
            normalized_name = path.as_posix()
            if normalized_name in seen_names:
                raise DeploymentError(f"模型 ZIP 包含重复路径：{name}")
            seen_names.add(normalized_name)
            mode = member.external_attr >> 16
            if stat.S_ISLNK(mode):
                raise DeploymentError(f"模型 ZIP 包含符号链接：{name}")
            file_type = stat.S_IFMT(mode)
            if file_type not in (0, stat.S_IFREG, stat.S_IFDIR):
                raise DeploymentError(f"模型 ZIP 包含特殊文件：{name}")
            if member.flag_bits & 0x1:
                raise DeploymentError("模型 ZIP 不允许包含加密文件")
            checked_members.append((member, path))

        for member, relative_path in checked_members:
            target = destination.joinpath(*relative_path.parts)
            if member.is_dir():
                target.mkdir(parents=True, exist_ok=True)
                continue
            target.parent.mkdir(parents=True, exist_ok=True)
            with archive.open(member, "r") as source, target.open("wb") as output:
                shutil.copyfileobj(source, output, length=1024 * 1024)

    bundle_files = [
        path for path in destination.rglob("bundle.yaml") if path.is_file()
    ]
    if len(bundle_files) != 1:
        raise DeploymentError(
            "模型 ZIP 必须只包含一个 bundle.yaml，"
            f"当前数量={len(bundle_files)}"
        )
    return bundle_files[0].parent


def normalize_bundle(bundle_root, destination, model_class):
    """把训练包的类别子目录转换为运行时扁平类别目录。"""
    manifest_path = bundle_root / "bundle.yaml"
    try:
        manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as exception:
        raise DeploymentError("无法读取模型 bundle.yaml") from exception
    if not isinstance(manifest, dict):
        raise DeploymentError("模型 bundle.yaml 根节点必须是映射")
    if model_class != "controller":
        raise DeploymentError(f"当前不支持模型种类：{model_class}")

    metadata_value = manifest.get("controller_metadata")
    if not isinstance(metadata_value, str) or not metadata_value:
        raise DeploymentError("bundle.yaml 缺少 controller_metadata")
    metadata_path = (bundle_root / metadata_value).resolve()
    bundle_root_resolved = bundle_root.resolve()
    if not metadata_path.is_relative_to(bundle_root_resolved):
        raise DeploymentError("controller_metadata 路径超出模型包")
    if not metadata_path.is_file():
        raise DeploymentError("controller_metadata 指向的文件不存在")
    class_source = metadata_path.parent
    if class_source == bundle_root_resolved:
        raise DeploymentError("训练模型包已经是扁平结构，无需再次转换")
    if class_source.parent != bundle_root_resolved:
        raise DeploymentError("controller 模型目录必须直接位于 bundle 根目录下")

    destination.mkdir(parents=True, exist_ok=False)

    def copy_file(source):
        target = destination / source.name
        if target.exists():
            raise DeploymentError(f"模型扁平化时文件名冲突：{source.name}")
        shutil.copy2(source, target)

    for child in bundle_root.iterdir():
        if child == manifest_path or child.resolve() == class_source:
            continue
        if not child.is_file():
            raise DeploymentError(f"模型包包含未知目录：{child.name}")
        copy_file(child)
    for child in class_source.iterdir():
        if not child.is_file():
            raise DeploymentError(f"控制器模型包含未知目录：{child.name}")
        copy_file(child)

    manifest["controller_metadata"] = metadata_path.name
    try:
        (destination / "bundle.yaml").write_text(
            yaml.safe_dump(
                manifest,
                allow_unicode=True,
                sort_keys=False,
                default_flow_style=False,
            ),
            encoding="utf-8",
        )
    except (OSError, yaml.YAMLError) as exception:
        raise DeploymentError("无法生成扁平化 bundle.yaml") from exception
    (destination / ".gitkeep").touch()
    return destination


def run_visible(arguments):
    """运行需要向终端持续输出进度的模型工具命令。"""
    result = subprocess.run(arguments, check=False)
    if result.returncode != 0:
        raise DeploymentError(
            f"模型工具执行失败，退出码={result.returncode}，命令={arguments[1]}"
        )


def read_status(model_tool, bundle=None, show_output=False):
    """读取 model_tool 的稳定 JSON 状态接口。"""
    arguments = [str(model_tool), "status"]
    if bundle is not None:
        arguments.extend(["--bundle", str(bundle)])
    arguments.append("--json")
    result = subprocess.run(
        arguments,
        check=False,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    if show_output and result.stdout.strip():
        print(result.stdout.strip(), flush=True)
    if result.returncode != 0:
        if result.stderr.strip():
            print(result.stderr.strip(), file=sys.stderr, flush=True)
        raise DeploymentError("无法读取模型运行状态")
    try:
        return json.loads(result.stdout)
    except json.JSONDecodeError as exception:
        raise DeploymentError("model_tool 返回了无效状态 JSON") from exception


def load_receipt(receipt_path):
    """读取上次成功部署记录；损坏记录按不存在处理。"""
    try:
        value = json.loads(receipt_path.read_text(encoding="utf-8"))
        return value if isinstance(value, dict) else {}
    except (OSError, UnicodeError, json.JSONDecodeError):
        return {}


def write_receipt(receipt_path, digest, status_value):
    """原子记录成功部署摘要，供重复执行时判定幂等。"""
    value = {
        "schema_version": 1,
        "archive_sha256": digest,
        "bundle_version": status_value.get("bundle_version", ""),
        "backend_name": status_value.get("backend_name", ""),
        "deployed_at": datetime.datetime.now(
            datetime.timezone.utc
        ).isoformat(),
    }
    temporary = receipt_path.with_name(f".{receipt_path.name}.{os.getpid()}.tmp")
    try:
        temporary.write_text(
            json.dumps(value, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        os.replace(temporary, receipt_path)
    finally:
        temporary.unlink(missing_ok=True)


def select_source(project_root, explicit_archive, inbox):
    """按显式路径、同级训练目录、手动收件目录的顺序选择模型。"""
    if explicit_archive:
        path = Path(explicit_archive).expanduser()
        if not path.is_absolute():
            raise DeploymentError("archive 参数必须使用绝对路径")
        return path.resolve()

    repository_root = project_root.parent
    training_archive = (
        repository_root.parent
        / "car_rl_train"
        / "exports"
        / "car_rl_model"
        / ARCHIVE_NAME
    ).resolve()
    training_checksum = Path(f"{training_archive}.sha256")
    if training_archive.exists() or training_checksum.exists():
        if not training_archive.is_file() or not training_checksum.is_file():
            raise DeploymentError(
                f"训练导出目录中的模型包不完整：{training_archive.parent}"
            )
        print(f"使用训练目录模型包：{training_archive}", flush=True)
        return training_archive

    manual_archive = (inbox / ARCHIVE_NAME).resolve()
    print(f"训练目录没有模型包，使用手动收件目录：{manual_archive}", flush=True)
    return manual_archive


def publish_bundle(bundle_root, models_root, model_class, backup_root):
    """把已准备的 bundle 原子发布到对应模型种类目录。"""
    target = models_root / model_class
    incoming = models_root / f".{model_class}.importing.{os.getpid()}"
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    backup = backup_root / timestamp
    backup_root.mkdir(parents=True, exist_ok=True)
    if incoming.exists():
        shutil.rmtree(incoming)
    shutil.copytree(bundle_root, incoming)
    had_target = target.exists() and any(
        child.name != ".gitkeep" for child in target.iterdir()
    )
    try:
        if target.exists():
            if had_target:
                os.replace(target, backup)
            else:
                shutil.rmtree(target)
        os.replace(incoming, target)
    except Exception:
        if incoming.exists():
            shutil.rmtree(incoming)
        if had_target and backup.exists() and not target.exists():
            os.replace(backup, target)
        raise
    return target, backup if had_target else None


def rollback_bundle(target, backup, backup_root):
    """最终状态失败时恢复先前的类别模型。"""
    failed = backup_root / (
        "failed." + datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    )
    if target.exists():
        os.replace(target, failed)
    if backup is not None and backup.exists():
        os.replace(backup, target)
    else:
        target.mkdir(parents=True, exist_ok=True)
        (target / ".gitkeep").touch()


def deploy(arguments):
    """执行完整且可回滚的模型部署事务。"""
    project_root = Path(arguments.project_root)
    model_tool = Path(arguments.model_tool)
    model_class = arguments.model_class.strip()
    if model_class != "controller":
        raise DeploymentError(
            f"当前只支持 controller 模型，class={model_class or '<empty>'}"
        )
    if not project_root.is_absolute() or not project_root.is_dir():
        raise DeploymentError("project-root 必须是存在的绝对目录")
    project_root = project_root.resolve()
    if not (project_root / "src" / "car_rl").is_dir():
        raise DeploymentError("project-root 中不存在 src/car_rl")
    if not model_tool.is_absolute() or not model_tool.is_file():
        raise DeploymentError("model-tool 必须是存在的绝对文件")
    model_tool = model_tool.resolve()

    car_rl_root = project_root / "src" / "car_rl"
    inbox = car_rl_root / "car_rl_model" / model_class
    models_root = car_rl_root / "models"
    data_root = project_root / "data" / "car_rl"
    state_root = data_root / "model_state"
    backup_root = data_root / "model_backups" / model_class
    staging_root = data_root / "model_staging" / model_class
    inbox.mkdir(parents=True, exist_ok=True)
    models_root.mkdir(parents=True, exist_ok=True)
    state_root.mkdir(parents=True, exist_ok=True)
    staging_root.mkdir(parents=True, exist_ok=True)
    lock_path = state_root / f"{model_class}.lock"

    with lock_path.open("a+", encoding="utf-8") as lock_stream:
        fcntl.flock(lock_stream.fileno(), fcntl.LOCK_EX)
        source_archive = select_source(
            project_root, arguments.archive.strip(), inbox
        )
        digest, source_checksum = verify_archive_pair(source_archive)
        print("模型 ZIP 的 SHA-256 校验通过", flush=True)
        archive_path = atomic_copy_pair(
            source_archive, source_checksum, inbox, digest
        )

        receipt_path = state_root / f"{model_class}.json"
        receipt = load_receipt(receipt_path)
        if receipt.get("archive_sha256") == digest:
            current_status = read_status(model_tool, show_output=True)
            if current_status.get("available") is True:
                print("当前模型已经部署且运行时有效，无需重复处理", flush=True)
                return

        with tempfile.TemporaryDirectory(
            prefix="archive.", dir=staging_root
        ) as stage_directory:
            stage_root = Path(stage_directory)
            extracted_root = stage_root / "extracted"
            extracted_root.mkdir()
            bundle_root = safe_extract(archive_path, extracted_root)
            run_visible(
                [str(model_tool), "verify", "--bundle", str(bundle_root)]
            )
            normalized_root = normalize_bundle(
                bundle_root, stage_root / "normalized", model_class
            )
            run_visible(
                [str(model_tool), "verify", "--bundle", str(normalized_root)]
            )

            staged_status = read_status(model_tool, bundle=normalized_root)
            backend_name = staged_status.get("backend_name")
            if backend_name == "tensorrt":
                run_visible(
                    [
                        str(model_tool),
                        "build",
                        "--bundle",
                        str(normalized_root),
                        "--precision",
                        "fp16",
                    ]
                )
            elif backend_name == "onnxruntime":
                run_visible(
                    [str(model_tool), "prepare", "--bundle", str(normalized_root)]
                )
            else:
                raise DeploymentError(
                    f"当前 car_rl 构建没有可用推理后端：{backend_name}"
                )

            target, backup = publish_bundle(
                normalized_root, models_root, model_class, backup_root
            )
            try:
                final_status = read_status(model_tool, show_output=True)
                if final_status.get("available") is not True:
                    raise DeploymentError(
                        "新模型部署后未达到 available=true"
                    )
                write_receipt(receipt_path, digest, final_status)
            except Exception:
                rollback_bundle(target, backup, backup_root)
                raise

            print(f"模型已部署：{target}", flush=True)
            if backup is not None and backup.exists():
                print(f"旧模型已备份：{backup}", flush=True)


def parse_arguments():
    """解析只供 launch 调用的稳定参数。"""
    parser = argparse.ArgumentParser(description="部署 car_rl 模型包")
    parser.add_argument("--project-root", required=True)
    parser.add_argument("--model-tool", required=True)
    parser.add_argument("--class", dest="model_class", default="controller")
    parser.add_argument("--archive", default="")
    return parser.parse_args()


def main():
    """命令行入口。"""
    try:
        deploy(parse_arguments())
    except (DeploymentError, OSError, zipfile.BadZipFile) as exception:
        print(f"强化学习模型部署失败：{exception}", file=sys.stderr, flush=True)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
