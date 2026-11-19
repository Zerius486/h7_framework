#!/usr/bin/env bash
set -euo pipefail

if [ "$#" -lt 1 ]; then
  echo "Usage: $0 <test_file.c> [extra gcc flags or extra .c files...]" >&2
  exit 2
fi

TEST_SRC="$1"
shift

if [ ! -f "$TEST_SRC" ]; then
  echo "Error: Test source '$TEST_SRC' not found." >&2
  exit 2
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SKILL_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
REPO_ROOT="$(cd "$SKILL_DIR/../../.." && pwd)"
STUBS_DIR="$SKILL_DIR/resources/stubs"

OUT_BIN="/tmp/h7_host_test_bin_$$"
cleanup() {
  rm -f "$OUT_BIN"
}
trap cleanup EXIT

cd "$REPO_ROOT"

gcc -std=gnu11 -O1 -g -Wall -Wextra -Werror \
  -ffunction-sections -fdata-sections \
  -I"$STUBS_DIR" \
  -ISkywalker/Algorithm/Inc \
  -ISkywalker/BSP/Inc \
  -ISkywalker/Device/Inc \
  -ISkywalker/Application/Inc \
  "$TEST_SRC" \
  "$STUBS_DIR/h7_host_stubs.c" \
  Skywalker/Algorithm/Src/*.c \
  Skywalker/BSP/Src/bsp_stream.c \
  Skywalker/Device/Src/dev_peripheral.c \
  Skywalker/Device/Src/dev_dji_motor.c \
  Skywalker/Device/Src/dev_dm_motor.c \
  Skywalker/Device/Src/dev_lk_motor.c \
  Skywalker/Device/Src/dev_dji_dt7.c \
  Skywalker/Application/Src/app_referee.c \
  "$@" \
  -Wl,--gc-sections \
  -lm \
  -o "$OUT_BIN"

"$OUT_BIN"
