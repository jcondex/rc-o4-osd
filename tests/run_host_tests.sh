#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="${TMPDIR:-/tmp}/rc_o4_osd_host_tests"

c++ -std=c++17 -DRC_O4_OSD_HOST_TEST \
  -I"${ROOT_DIR}/src" \
  "${ROOT_DIR}/tests/host_tests.cpp" \
  "${ROOT_DIR}/src/msp.cpp" \
  "${ROOT_DIR}/src/nav.cpp" \
  "${ROOT_DIR}/src/gps.cpp" \
  "${ROOT_DIR}/src/bmp390.cpp" \
  "${ROOT_DIR}/src/calibration_flash.cpp" \
  "${ROOT_DIR}/src/compass_qmc5883l.cpp" \
  "${ROOT_DIR}/src/displayport.cpp" \
  "${ROOT_DIR}/src/state_machine.cpp" \
  -o "${OUT}"

"${OUT}"
