#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir="$(mktemp -d)"
trap 'rm -rf "${build_dir}"' EXIT

cd "${repo_root}"

cxx="${CXX:-g++}"
if [[ -n "${NODE:-}" ]]; then
    node_bin="${NODE}"
elif command -v node >/dev/null 2>&1; then
    node_bin="node"
elif command -v node.exe >/dev/null 2>&1; then
    node_bin="node.exe"
else
    echo "node/node.exe is required for JavaScript host tests" >&2
    exit 1
fi
common_flags=(-std=c++17 -Wall -Wextra -Werror -Ihost_tests/stubs -IgugaPI)

build_and_run() {
    local name="$1"
    shift
    echo "[host-test] ${name}"
    "${cxx}" "${common_flags[@]}" "$@" -o "${build_dir}/${name}"
    "${build_dir}/${name}"
}

build_and_run config_store_v16 \
    host_tests/config_store_v13_test.cpp \
    gugaPI/app/config_store.cpp
build_and_run dm_g6220_protocol \
    host_tests/dm_g6220_protocol_test.cpp \
    gugaPI/drivers/dm_g6220/dm_g6220.cpp
build_and_run dm_g6220_controller \
    host_tests/dm_g6220_controller_test.cpp \
    gugaPI/app/dm_g6220_controller.cpp \
    gugaPI/drivers/dm_g6220/dm_g6220.cpp
build_and_run app_can_bus \
    host_tests/app_can_bus_test.cpp
build_and_run grayscale_processing \
    host_tests/grayscale_processing_test.cpp \
    gugaPI/drivers/grayscale/grayscale_processing.cpp
build_and_run infrared_line_protocol \
    host_tests/infrared_line_protocol_test.cpp \
    gugaPI/drivers/infrared_line/infrared_line_protocol.cpp
build_and_run infrared_line_dma_cursor \
    host_tests/infrared_line_dma_cursor_test.cpp
build_and_run infrared_calibration \
    host_tests/infrared_calibration_test.cpp \
    gugaPI/app/app_infrared_sensor.cpp \
    gugaPI/drivers/infrared_line/infrared_line_protocol.cpp
build_and_run grayscale_road \
    host_tests/grayscale_road_test.cpp \
    gugaPI/app/grayscale_road.cpp
build_and_run heading_arc_math \
    host_tests/heading_arc_math_test.cpp
build_and_run heading_lock_math \
    host_tests/heading_lock_math_test.cpp
build_and_run imu_bias_estimator \
    host_tests/imu_bias_estimator_test.cpp \
    gugaPI/app/imu_bias_estimator.cpp
build_and_run linefollow_road_handoff \
    host_tests/linefollow_road_handoff_test.cpp
build_and_run mode_switch_chord \
    host_tests/mode_switch_chord_test.cpp
build_and_run road_event_controller \
    host_tests/road_event_controller_test.cpp \
    gugaPI/app/road_event_controller.cpp
build_and_run action_runner \
    host_tests/action_runner_test.cpp \
    gugaPI/app/action.cpp \
    gugaPI/app/seq_store.cpp
build_and_run oled_framebuffer \
    host_tests/oled_framebuffer_test.cpp \
    gugaPI/drivers/oled/oled_ssd1306.cpp
build_and_run large_timer \
    host_tests/large_timer_test.cpp \
    gugaPI/app/app_large_timer.cpp

echo "[host-test] shell_command_catalog"
"${node_bin}" host_tests/shell_command_catalog_test.js

echo "[host-test] parameter_catalog"
"${node_bin}" host_tests/parameter_catalog_test.js

echo "[host-test] sequence_core"
"${node_bin}" host_tests/sequence_core_test.js

echo "[host-test] help_catalog"
"${node_bin}" host_tests/help_catalog_test.js

echo "[host-test] dashboard_core"
"${node_bin}" host_tests/dashboard_core_test.js

echo "[host-test] all tests passed"
