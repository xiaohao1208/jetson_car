#!/usr/bin/env bash

set -euo pipefail

# 输出命令帮助
show_help()
{
  printf '%s\n' \
    '用法：export_rl_windows_handoff.sh --output <zip路径> [--force]' \
    '示例：./jetson_car/scripts/export_rl_windows_handoff.sh --output /media/hao/USB/car_rl_windows_handoff.zip'
}

# 解析后的交接包输出路径
output_path=''
# 是否允许覆盖同名交接包
force='false'

while (($# > 0)); do
  case "$1" in
    --output)
      if (($# < 2)); then
        show_help
        exit 2
      fi
      output_path=$2
      shift 2
      ;;
    --force)
      force='true'
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

if [[ -z "$output_path" || "$output_path" != *.zip ]]; then
  printf '%s\n' '输出路径必须使用 .zip 扩展名' >&2
  exit 2
fi

# 当前脚本所在目录
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
# jetson_car 工作区根目录
jetson_root=$(cd -- "$script_dir/.." && pwd)
# 项目仓库根目录
project_root=$(cd -- "$jetson_root/.." && pwd)
# 输出目录的绝对路径
output_dir=$(cd -- "$(dirname -- "$output_path")" && pwd)
# 输出文件名
output_name=$(basename -- "$output_path")
# 最终 ZIP 绝对路径
output_zip="$output_dir/$output_name"
# ZIP 外部校验文件路径
output_hash="$output_zip.sha256"

# 加载 xacro 查找 ROS 包和运行库需要的环境
if [[ ! -f /opt/ros/humble/setup.bash || ! -f "$jetson_root/install/setup.bash" ]]; then
  printf '%s\n' 'ROS 2 或 jetson_car 安装环境不存在，请先完成工作区构建' >&2
  exit 1
fi
set +u
source /opt/ros/humble/setup.bash
source "$jetson_root/install/setup.bash"
set -u

if [[ -e "$output_zip" || -e "$output_hash" ]]; then
  if [[ "$force" != 'true' ]]; then
    printf '输出文件已存在，使用 --force 才能覆盖，文件=%s\n' "$output_zip" >&2
    exit 1
  fi
  rm -f -- "$output_zip" "$output_hash"
fi

for command_name in zip sha256sum xacro; do
  if ! command -v "$command_name" >/dev/null 2>&1; then
    printf '缺少导出依赖，命令=%s\n' "$command_name" >&2
    exit 1
  fi
done

# 已安装的契约导出程序
contract_tool="$jetson_root/install/car_rl/lib/car_rl/contract_tool"
if [[ ! -x "$contract_tool" ]]; then
  printf '%s\n' '未找到 contract_tool，请先在 jetson_car 中构建 car_rl' >&2
  printf '%s\n' '命令：colcon build --packages-select car_rl' >&2
  exit 1
fi

# 本次导出使用的临时目录
stage_parent=$(mktemp -d)
# 脚本退出时清理临时目录
cleanup()
{
  rm -rf -- "$stage_parent"
}
trap cleanup EXIT

# ZIP 内固定的顶层目录
stage_root="$stage_parent/car_rl_windows_handoff"
mkdir -p \
  "$stage_root/source" \
  "$stage_root/assets" \
  "$stage_root/parameters/jetson_car" \
  "$stage_root/parameters/esp32_car" \
  "$stage_root/contract" \
  "$stage_root/docs" \
  "$stage_root/tools" \
  "$stage_root/source/car_description"

cp -a "$jetson_root/src/car_rl" "$stage_root/source/car_rl"
cp -a "$jetson_root/src/car_description/urdf" "$stage_root/source/car_description/urdf"
cp -a \
  "$jetson_root/src/car_navigation/config/nav2_params.yaml" \
  "$jetson_root/src/car_navigation/config/navigation_modes.yaml" \
  "$jetson_root/src/car_move/config/move.yaml" \
  "$jetson_root/src/car_base/config/base.yaml" \
  "$stage_root/parameters/jetson_car/"
cp -a \
  "$project_root/esp32_car/include/config/car_config.hpp" \
  "$project_root/esp32_car/include/control/motion_controller.hpp" \
  "$project_root/esp32_car/include/control/wheel_pid_controller.hpp" \
  "$stage_root/parameters/esp32_car/"
cp -a \
  "$project_root/ARCHITECTURE.md" \
  "$project_root/CONFIGURATION.md" \
  "$stage_root/docs/"
cp -a \
  "$jetson_root/src/car_rl/handoff/WINDOWS_HANDOFF.md" \
  "$stage_root/WINDOWS_HANDOFF.md"
cp -a \
  "$jetson_root/scripts/import_rl_model_bundle.sh" \
  "$stage_root/tools/import_rl_model_bundle.sh"
cp -a \
  "$jetson_root/src/car_rl/handoff/verify_handoff.ps1" \
  "$stage_root/verify_handoff.ps1"

# 临时 ament 索引保证 xacro 使用当前源码而不是旧安装副本
source_overlay="$stage_parent/source_overlay"
mkdir -p \
  "$source_overlay/share/ament_index/resource_index/packages" \
  "$source_overlay/share/car_description"
touch "$source_overlay/share/ament_index/resource_index/packages/car_description"
cp -a \
  "$jetson_root/src/car_description/urdf" \
  "$source_overlay/share/car_description/urdf"
AMENT_PREFIX_PATH="$source_overlay:$AMENT_PREFIX_PATH" xacro \
  "$jetson_root/src/car_description/urdf/car.urdf.xacro" \
  > "$stage_root/assets/car_training.urdf"
"$contract_tool" export \
  --output "$stage_root/contract/golden_vectors.json"

# 源文件快照清单用于定位某次训练对应的精确输入
(
  cd -- "$stage_root"
  find source parameters assets contract docs tools -type f -print0 |
    sort -z |
    xargs -0 sha256sum > SOURCE_SHA256SUMS
)
# 源文件快照清单自身的 SHA-256
source_fingerprint=$(sha256sum "$stage_root/SOURCE_SHA256SUMS" | awk '{print $1}')

{
  printf '%s\n' \
    'schema_version: 1' \
    'handoff_type: car_rl_windows_training_input' \
    'bundle_schema_version: 2' \
    'observation_contract_version: 1' \
    'observation_size: 86' \
    'action_size: 2' \
    "source_fingerprint_sha256: \"$source_fingerprint\"" \
    'machines:' \
    '  ubuntu_source_root: /home/hao/ros2_car_gpt' \
    '  windows_isaaclab_root: D:\IsaacLab' \
    '  windows_training_root: D:\car_rl_training' \
    '  windows_handoff_root: D:\car_rl_handoff\car_rl_windows_handoff' \
    '  orin_project_root: /home/seeed/ros2_car' \
    'transfer:' \
    '  transport: removable_storage' \
    '  network_required: false'
} > "$stage_root/manifest.yaml"

# 完整清单用于在 Windows 解压后检查每个文件
(
  cd -- "$stage_root"
  find . -type f ! -name SHA256SUMS -print0 |
    sort -z |
    xargs -0 sha256sum > SHA256SUMS
)

if find "$stage_root" -type f \
  \( -name '*.engine' -o -name '*.onnx' -o -path '*/build/*' -o \
  -path '*/install/*' -o -path '*/log/*' \) |
  grep -q .
then
  printf '%s\n' '交接包包含禁止的构建产物或模型文件' >&2
  exit 1
fi

(
  cd -- "$stage_parent"
  zip -q -r "$output_zip" car_rl_windows_handoff
)
(
  cd -- "$output_dir"
  sha256sum "$output_name" > "$output_name.sha256"
)

printf 'Windows 离线交接包已生成，文件=%s\n' "$output_zip"
printf 'ZIP 校验文件已生成，文件=%s\n' "$output_hash"
printf '源文件指纹=%s\n' "$source_fingerprint"
