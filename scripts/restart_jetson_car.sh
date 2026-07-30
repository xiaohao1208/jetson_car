#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(
  cd -- "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1
  pwd
)"
source "${SCRIPT_DIR}/.jetson_car_control.sh"

jetson_car_acquire_lock
jetson_car_stop_locked
jetson_car_start_locked
