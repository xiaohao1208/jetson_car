import base64
import hashlib
import io
import json
import math
from pathlib import Path

from PIL import Image
import yaml

from car_web.atomic_io import atomic_write_text


class MapPoseStore:
    """把初始位姿绑定到具体地图内容，防止换图后误用旧位置"""

    def __init__(self, map_yaml):
        """建立地图、位姿文件和网页图像缓存路径"""
        self.map_yaml = Path(map_yaml)
        self.pose_file = self.map_yaml.with_suffix(".pose.json")
        self._image_cache_id = None
        self._image_cache = None
        self._map_signature = None
        self._map_id_cache = None

    def _source_files(self):
        """返回地图 YAML 和它声明的图像路径。"""

        yaml_bytes = self.map_yaml.read_bytes()
        config = yaml.safe_load(yaml_bytes.decode("utf-8")) or {}
        if not isinstance(config, dict):
            raise ValueError("地图YAML顶层必须是对象")
        image = Path(config.get("image", ""))
        if not image.is_absolute():
            image = self.map_yaml.parent / image
        return yaml_bytes, image

    @staticmethod
    def _signature(path):
        """生成足以识别原子替换和内容写入的文件签名。"""

        stat = path.stat()
        return (stat.st_dev, stat.st_ino, stat.st_size, stat.st_mtime_ns)

    def map_id(self):
        """计算地图 YAML 与图像内容共同决定的 SHA256 标识"""
        if not self.map_yaml.is_file():
            return None
        try:
            yaml_bytes, image = self._source_files()
            signature = (
                self._signature(self.map_yaml),
                self._signature(image) if image.is_file() else None,
            )
        except (OSError, UnicodeError, ValueError, yaml.YAMLError):
            return None
        if signature == self._map_signature:
            return self._map_id_cache
        digest = hashlib.sha256(yaml_bytes)
        try:
            if image.is_file():
                digest.update(image.read_bytes())
        except OSError:
            return None
        self._map_signature = signature
        self._map_id_cache = digest.hexdigest()
        return self._map_id_cache

    def save(self, pose):
        """将有限地图位姿与当前地图标识原子保存"""
        map_id = self.map_id()
        if map_id is None:
            raise RuntimeError("地图不存在，不能保存初始位姿")
        payload = {
            "map_id": map_id,
            "x": float(pose["x"]),
            "y": float(pose["y"]),
            "yaw": float(pose["yaw"]),
        }
        self.pose_file.parent.mkdir(parents=True, exist_ok=True)
        atomic_write_text(
            self.pose_file,
            json.dumps(payload, ensure_ascii=False, indent=2),
        )
        return payload

    def load(self):
        """只返回仍与当前地图内容匹配的已保存位姿"""
        if not self.pose_file.is_file():
            return None
        try:
            payload = json.loads(self.pose_file.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return None
        if payload.get("map_id") != self.map_id():
            return None
        try:
            return {
                "x": float(payload["x"]),
                "y": float(payload["y"]),
                "yaw": float(payload["yaw"]),
            }
        except (KeyError, TypeError, ValueError):
            return None

    def map_payload(self):
        """读取保存地图并转换为网页可以直接显示的PNG数据"""
        map_id = self.map_id()
        if map_id is None:
            return {"available": False, "reason": "地图文件不存在"}
        if map_id == self._image_cache_id and self._image_cache is not None:
            return dict(self._image_cache)
        try:
            config = yaml.safe_load(
                self.map_yaml.read_text(encoding="utf-8")
            ) or {}
            resolution = float(config["resolution"])
            origin = config["origin"]
            origin_x = float(origin[0])
            origin_y = float(origin[1])
            origin_yaw = float(origin[2])
            image_path = Path(config["image"])
            if not image_path.is_absolute():
                image_path = self.map_yaml.parent / image_path
            if (
                not math.isfinite(resolution)
                or resolution <= 0.0
                or not math.isfinite(origin_x)
                or not math.isfinite(origin_y)
                or not math.isfinite(origin_yaw)
            ):
                raise ValueError("地图元数据包含无效数值")
            if abs(origin_yaw) > 1.0e-6:
                raise ValueError("网页暂不支持带旋转原点的地图")
            with Image.open(image_path) as source:
                image = source.convert("L")
                width, height = image.size
                buffer = io.BytesIO()
                image.save(buffer, format="PNG")
        except (KeyError, OSError, TypeError, ValueError, yaml.YAMLError) as error:
            return {
                "available": False,
                "reason": f"保存地图读取失败：{error}",
            }
        payload = {
            "available": True,
            "source": "saved",
            "image": "data:image/png;base64," + base64.b64encode(
                buffer.getvalue()
            ).decode("ascii"),
            "width": width,
            "height": height,
            "resolution": resolution,
            "origin": {"x": origin_x, "y": origin_y},
        }
        self._image_cache_id = map_id
        self._image_cache = dict(payload)
        return payload

    def invalidate_map_cache(self):
        """地图文件替换后清除保存图像缓存"""
        self._image_cache_id = None
        self._image_cache = None
        self._map_signature = None
        self._map_id_cache = None
