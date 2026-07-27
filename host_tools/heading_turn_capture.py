#!/usr/bin/env python3
"""Capture one bounded gugaPI heading turn and plot its control response.

This script commands real chassis motion. Put the car in a safe test area.
It always sends ``heading stop`` before releasing the serial port.
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import math
from pathlib import Path
import time

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import serial

from linefollow_capture import EXPECTED_FIELDS, parse_data_line, send_command


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Capture and plot one gugaPI relative heading turn"
    )
    parser.add_argument("--port", default="COM14")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--degrees", type=int, required=True)
    parser.add_argument("--period-ms", type=int, default=50)
    parser.add_argument("--capture-s", type=float, default=6.0)
    parser.add_argument("--output-dir", type=Path, default=Path("captures"))
    args = parser.parse_args()
    if not -180 <= args.degrees <= 180 or args.degrees == 0:
        parser.error("--degrees must be in -180..-1 or 1..180")
    if not 50 <= args.period_ms <= 5000:
        parser.error("--period-ms must be in 50..5000")
    if args.capture_s <= 0.0:
        parser.error("--capture-s must be positive")
    return args


def unwrap_degrees(values: list[float]) -> list[float]:
    if not values:
        return []
    result = [values[0]]
    for value in values[1:]:
        candidate = value
        while candidate - result[-1] > 180.0:
            candidate -= 360.0
        while candidate - result[-1] < -180.0:
            candidate += 360.0
        result.append(candidate)
    return result


def align_targets(targets: list[float], yaw: list[float]) -> list[float]:
    result: list[float] = []
    for target, current in zip(targets, yaw):
        while target - current > 180.0:
            target -= 360.0
        while target - current < -180.0:
            target += 360.0
        result.append(target)
    return result


def write_csv(path: Path, records: list[list[float]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(EXPECTED_FIELDS)
        writer.writerows(records)


def plot_capture(path: Path, records: list[list[float]]) -> None:
    columns = {
        name: [row[index] for row in records]
        for index, name in enumerate(EXPECTED_FIELDS)
    }
    start_ms = columns["t"][0]
    seconds = [(value - start_ms) / 1000.0 for value in columns["t"]]
    yaw = unwrap_degrees(columns["yaw"])
    target = align_targets(columns["yaw_tgt"], yaw)

    figure, axes = plt.subplots(4, 1, figsize=(11, 10), sharex=True)
    axes[0].plot(seconds, target, "--", label="Target yaw")
    axes[0].plot(seconds, yaw, label="Measured yaw")
    axes[0].set_ylabel("Yaw (deg)")
    axes[0].legend(loc="best")

    axes[1].plot(seconds, columns["head_err"], label="Heading error")
    axes[1].plot(
        seconds,
        [value / 1000.0 for value in columns["head_turn_brake_mdeg"]],
        "--",
        label="Dynamic brake threshold",
    )
    axes[1].axhline(0.0, linewidth=0.8)
    axes[1].set_ylabel("Angle (deg)")
    axes[1].legend(loc="best")

    axes[2].plot(seconds, columns["L_tgt"], "--", label="Left target")
    axes[2].plot(seconds, columns["L_act"], label="Left actual")
    axes[2].plot(seconds, columns["R_tgt"], "--", label="Right target")
    axes[2].plot(seconds, columns["R_act"], label="Right actual")
    axes[2].set_ylabel("Wheel RPM")
    axes[2].legend(loc="best", ncol=2)

    axes[3].plot(
        seconds,
        [value / 1000.0 for value in columns["head_turn_rate_mdps"]],
        label="Gyro Z rate",
    )
    axes[3].step(
        seconds,
        columns["head_turn_phase"],
        where="post",
        label="Turn phase",
    )
    axes[3].set_ylabel("deg/s / phase")
    axes[3].set_xlabel("Time (s)")
    axes[3].legend(loc="best")

    for axis in axes:
        axis.grid(True, alpha=0.25)
    figure.suptitle("gugaPI heading-turn telemetry")
    figure.tight_layout()
    figure.savefig(path, dpi=160)
    plt.close(figure)


def main() -> int:
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    stamp = dt.datetime.now().strftime("%Y%m%d-%H%M%S")
    csv_path = args.output_dir / f"heading-turn-{stamp}.csv"
    png_path = args.output_dir / f"heading-turn-{stamp}.png"
    records: list[list[float]] = []
    shell_messages: list[str] = []

    with serial.Serial(
        args.port,
        args.baud,
        timeout=0.1,
        write_timeout=0.5,
    ) as port:
        time.sleep(0.2)
        port.reset_input_buffer()
        send_command(port, f"telem on {args.period_ms}")
        time.sleep(0.3)
        send_command(port, f"heading turn {args.degrees}")
        deadline = time.monotonic() + args.capture_s
        try:
            while time.monotonic() < deadline:
                raw = port.readline()
                if not raw:
                    continue
                line = raw.decode("ascii", errors="replace").strip()
                if not line:
                    continue
                values = parse_data_line(line)
                if values is not None and all(math.isfinite(v) for v in values):
                    records.append(values)
                elif not line.startswith("#"):
                    shell_messages.append(line)
        finally:
            send_command(port, "heading stop")
            time.sleep(0.1)
            send_command(port, "telem off")
            time.sleep(0.1)

    for message in shell_messages:
        print(message)
    if not records:
        raise RuntimeError("No telemetry rows received; verify firmware and port")

    write_csv(csv_path, records)
    plot_capture(png_path, records)
    print(f"samples={len(records)}")
    print(f"csv={csv_path.resolve()}")
    print(f"plot={png_path.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
