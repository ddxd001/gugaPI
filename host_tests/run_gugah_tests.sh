#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir="$(mktemp -d)"
trap 'rm -rf "${build_dir}"' EXIT
cd "${repo_root}"

cxx="${CXX:-g++}"
flags=(-std=c++17 -Wall -Wextra -Werror -Ihost_tests/stubs -IgugaH)

"${cxx}" "${flags[@]}" \
    host_tests/gugah_core_test.cpp \
    gugaH/app/h_app.cpp \
    gugaH/config/h_config.cpp \
    gugaH/control/ball_control.cpp \
    gugaH/control/course_control.cpp \
    gugaH/control/line_control.cpp \
    gugaH/control/vision_state.cpp \
    gugaH/drivers/ball_vision/ball_vision_protocol.cpp \
    gugaH/drivers/grayscale/grayscale_processing.cpp \
    -o "${build_dir}/gugah_core"

"${build_dir}/gugah_core"
