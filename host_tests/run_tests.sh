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
common_flags=(-std=c++17 -Wall -Wextra -Werror -IgugaPI)

build_and_run() {
    local name="$1"
    shift
    echo "[host-test] ${name}"
    "${cxx}" "${common_flags[@]}" "$@" -o "${build_dir}/${name}"
    "${build_dir}/${name}"
}

build_and_run config_store_v15 \
    host_tests/config_store_v13_test.cpp \
    gugaPI/app/config_store.cpp
build_and_run grayscale_processing \
    host_tests/grayscale_processing_test.cpp \
    gugaPI/drivers/grayscale/grayscale_processing.cpp
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
build_and_run road_event_controller \
    host_tests/road_event_controller_test.cpp \
    gugaPI/app/road_event_controller.cpp

echo "[host-test] shell_command_catalog"
"${node_bin}" host_tests/shell_command_catalog_test.js

echo "[host-test] dashboard_core"
"${node_bin}" host_tests/dashboard_core_test.js

echo "[host-test] all tests passed"
