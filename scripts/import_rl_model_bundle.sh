#!/usr/bin/env bash

set -euo pipefail

# 输出命令帮助
show_help()
{
  printf '%s\n' \
    '用法：import_rl_model_bundle.sh --archive <模型zip路径> [--project-root <目录>] [--verify-only]' \
    '默认 Orin 项目目录：/home/seeed/ros2_car'
}

# Windows 导出的模型包 ZIP
archive_path=''
# Orin 上的项目根目录
project_root='/home/seeed/ros2_car'
# 是否仅校验而不安装
verify_only='false'

while (($# > 0)); do
  case "$1" in
    --archive)
      if (($# < 2)); then
        show_help
        exit 2
      fi
      archive_path=$2
      shift 2
      ;;
    --project-root)
      if (($# < 2)); then
        show_help
        exit 2
      fi
      project_root=$2
      shift 2
      ;;
    --verify-only)
      verify_only='true'
      shift
      ;;
    --help|-h)
      show_help
      exit 0
      ;;
    *)
      printf '未知参数，参数=%s\n' "$1" >&2
      show_help
      exit 2
      ;;
  esac
done

if [[ -z "$archive_path" || ! -f "$archive_path" ]]; then
  printf '模型包不存在，文件=%s\n' "$archive_path" >&2
  exit 1
fi

# 模型包 ZIP 的绝对路径
archive_path=$(cd -- "$(dirname -- "$archive_path")" && pwd)/$(basename -- "$archive_path")
# 相邻的 ZIP SHA-256 文件
archive_hash="$archive_path.sha256"

# 加载 model_tool 运行时依赖的 ROS 和工作区环境
if [[ ! -f /opt/ros/humble/setup.bash ||
  ! -f "$project_root/jetson_car/install/setup.bash" ]]
then
  printf '%s\n' 'ROS 2 或 Orin 项目安装环境不存在，请先构建 car_rl' >&2
  exit 1
fi
set +u
source /opt/ros/humble/setup.bash
source "$project_root/jetson_car/install/setup.bash"
set -u

for command_name in unzip zipinfo sha256sum; do
  if ! command -v "$command_name" >/dev/null 2>&1; then
    printf '缺少导入依赖，命令=%s\n' "$command_name" >&2
    exit 1
  fi
done

if [[ -f "$archive_hash" ]]; then
  (
    cd -- "$(dirname -- "$archive_path")"
    sha256sum --check --status "$(basename -- "$archive_hash")"
  )
  printf '%s\n' '模型 ZIP 的 SHA-256 校验通过'
else
  printf '缺少相邻校验文件，文件=%s\n' "$archive_hash" >&2
  exit 1
fi

# ZIP 条目路径在解压前必须确认不会逃逸临时目录
if zipinfo -1 "$archive_path" |
  grep -Eq '(^/|(^|/)\.\.(/|$)|\\)'
then
  printf '%s\n' '模型 ZIP 包含不安全路径' >&2
  exit 1
fi

# ZIP 中不允许携带符号链接
if zipinfo -l "$archive_path" | awk '$1 ~ /^l/ {found=1} END {exit !found}'; then
  printf '%s\n' '模型 ZIP 包含符号链接' >&2
  exit 1
fi

# 本次校验和安装使用的临时目录
stage_root=$(mktemp -d)
# 脚本退出时清理临时目录
cleanup()
{
  rm -rf -- "$stage_root"
}
trap cleanup EXIT

unzip -q "$archive_path" -d "$stage_root"

# 解压后的模型包清单候选
mapfile -t bundle_files < <(find "$stage_root" -type f -name bundle.yaml -print)
if ((${#bundle_files[@]} != 1)); then
  printf '模型 ZIP 必须只包含一个 bundle.yaml，当前数量=%d\n' "${#bundle_files[@]}" >&2
  exit 1
fi
# 模型包根目录
bundle_root=$(dirname -- "${bundle_files[0]}")

# car_rl 模型校验工具
model_tool="$project_root/jetson_car/install/car_rl/lib/car_rl/model_tool"
if [[ ! -x "$model_tool" ]]; then
  printf '未找到 model_tool，请先在 Orin 构建 car_rl，文件=%s\n' "$model_tool" >&2
  exit 1
fi

"$model_tool" verify --bundle "$bundle_root"
printf '模型契约校验通过，目录=%s\n' "$bundle_root"

if [[ "$verify_only" == 'true' ]]; then
  printf '%s\n' '仅校验模式结束，未更改 active 模型'
  exit 0
fi

# car_rl 模型目录
models_root="$project_root/jetson_car/src/car_rl/models"
# 原子替换前的临时模型目录
incoming_root="$models_root/.active.importing.$$"
# 原 active 模型的时间戳备份目录
backup_root="$models_root/active.backup.$(date +%Y%m%d_%H%M%S)"

mkdir -p "$models_root"
if [[ -e "$incoming_root" ]]; then
  printf '临时安装目录已存在，目录=%s\n' "$incoming_root" >&2
  exit 1
fi
cp -a "$bundle_root" "$incoming_root"

if [[ -e "$models_root/active" ]]; then
  mv -- "$models_root/active" "$backup_root"
fi
mv -- "$incoming_root" "$models_root/active"

printf '模型已安装，目录=%s\n' "$models_root/active"
if [[ -e "$backup_root" ]]; then
  printf '旧模型已备份，目录=%s\n' "$backup_root"
fi
printf '%s\n' '下一步在 Orin 构建 TensorRT engine'
printf 'ros2 run car_rl model_tool build --bundle %s --precision fp16\n' \
  "$project_root/jetson_car/src/car_rl/models/active"
