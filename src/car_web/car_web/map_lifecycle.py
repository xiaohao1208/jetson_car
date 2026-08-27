import os


def _fsync(path, *, directory=False):
    """在替换前后同步地图文件与所在目录。"""

    flags = os.O_RDONLY | (os.O_DIRECTORY if directory else 0)
    descriptor = os.open(path, flags)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def replace_saved_map(map_path, temporary_prefix, pose_store=None):
    """完整且原子地替换地图YAML和图像"""
    temporary_yaml = temporary_prefix.with_suffix(".yaml")
    temporary_image = temporary_prefix.with_suffix(".pgm")
    if not temporary_yaml.is_file() or not temporary_image.is_file():
        raise RuntimeError("地图保存命令未生成完整文件")
    yaml_lines = temporary_yaml.read_text(
        encoding="utf-8"
    ).splitlines()
    for index, line in enumerate(yaml_lines):
        if line.lstrip().startswith("image:"):
            yaml_lines[index] = (
                f"image: {map_path.with_suffix('.pgm').name}"
            )
            break
    else:
        raise RuntimeError("地图YAML缺少image字段")

    final_image = map_path.with_suffix(".pgm")
    temporary_yaml.write_text(
        "\n".join(yaml_lines) + "\n",
        encoding="utf-8",
    )
    _fsync(temporary_image)
    _fsync(temporary_yaml)
    backup_yaml = temporary_prefix.with_suffix(".backup.yaml")
    backup_image = temporary_prefix.with_suffix(".backup.pgm")
    had_yaml = map_path.is_file()
    had_image = final_image.is_file()
    try:
        if had_yaml:
            os.replace(map_path, backup_yaml)
        if had_image:
            os.replace(final_image, backup_image)
        os.replace(temporary_image, final_image)
        os.replace(temporary_yaml, map_path)
        _fsync(map_path.parent, directory=True)
        if pose_store is not None:
            pose_store.invalidate_map_cache()
    except OSError:
        if map_path.is_file():
            map_path.unlink()
        if final_image.is_file():
            final_image.unlink()
        if had_yaml and backup_yaml.is_file():
            os.replace(backup_yaml, map_path)
        if had_image and backup_image.is_file():
            os.replace(backup_image, final_image)
        raise
    finally:
        for backup in (backup_yaml, backup_image):
            try:
                backup.unlink()
            except FileNotFoundError:
                pass


def remove_temporary_map(temporary_prefix):
    """清理一次地图保存产生的临时文件"""
    for suffix in (".yaml", ".pgm"):
        try:
            temporary_prefix.with_suffix(suffix).unlink()
        except FileNotFoundError:
            pass
