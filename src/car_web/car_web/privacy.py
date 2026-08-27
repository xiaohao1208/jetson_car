"""限制浏览器响应中暴露本机文件系统信息。"""

from os import PathLike
import re


_PRIVATE_KEYS = frozenset(
    {
        "directory",
        "file",
        "path",
        "result_directory",
        "observations_csv",
        "metadata_yaml",
        "calibration_yaml",
        "calibration_csv",
        "coverage_json",
    }
)

_LOCAL_PATH_PATTERNS = (
    re.compile(r"file://[^\s,，；;：\"'<>()[\]{}]+", re.IGNORECASE),
    re.compile(r"(?<![A-Za-z0-9_])[A-Za-z]:[\\/][^\s,，；;：\"'<>()[\]{}]+"),
    re.compile(r"\\{2,}[^\s\\/]+[\\/][^\s,，；;：\"'<>()[\]{}]+"),
    re.compile(
        r"(?<![A-Za-z0-9_:/])/(?:home|root|Users|tmp|var|etc|opt|usr|"
        r"mnt|media|run|dev|proc|sys|workspace)"
        r"(?:/[^\s,，；;：\"'<>()[\]{}]*)?"
    ),
)


def public_text(value):
    """把可能包含本机绝对路径的文本转换为可公开文本。"""

    text = str(value)
    for pattern in _LOCAL_PATH_PATTERNS:
        text = pattern.sub("[路径已隐藏]", text)
    return text


def public_value(value):
    """递归过滤 Web JSON 中的路径字段和路径文本。"""

    if isinstance(value, dict):
        return {
            key: public_value(item)
            for key, item in value.items()
            if not _private_key(key)
        }
    if isinstance(value, list):
        return [public_value(item) for item in value]
    if isinstance(value, tuple):
        return [public_value(item) for item in value]
    if isinstance(value, PathLike):
        return "[路径已隐藏]"
    if isinstance(value, str):
        if value.startswith("data:image/"):
            return value
        return public_text(value)
    return value


def _private_key(key):
    """识别直接或按命名约定携带文件位置的 JSON 字段。"""

    name = str(key).lower()
    return name in _PRIVATE_KEYS or name.endswith(
        ("_path", "_directory", "_file")
    )
