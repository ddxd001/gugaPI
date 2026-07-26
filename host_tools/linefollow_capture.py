#!/usr/bin/env python3
"""Capture gugaPI line-follow telemetry from a serial port and plot it.

The script never resets the MCU. When --start-rpm is supplied it starts one
bounded line-follow run and always sends ``lf stop`` before releasing the port.
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


EXPECTED_FIELDS = [
    "t",
    "mode",
    "step",
    "L_tgt",
    "L_act",
    "R_tgt",
    "R_act",
    "yaw_tgt",
    "yaw",
    "head_err",
    "head_corr",
    "gray_pos",
    "gray_strength",
    "gray_conf",
    "gray_valid",
    "gray_state",
    "lf_err",
    "lf_corr",
    "lf_weak",
    "lf_invalid",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Capture and plot gugaPI line-follow telemetry"
    )
    parser.add_argument("--port", default="COM14", help="serial port")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument(
        "--period-ms",
        type=int,
        default=50,
        metavar="50..5000",
    )
    parser.add_argument(
        "--capture-s",
        type=float,
        default=None,
        help="capture duration; default is run duration plus 1 second",
    )
    parser.add_argument(
        "--start-rpm",
        type=int,
        default=None,
        help="start one bounded line-follow run at this base RPM",
    )
    parser.add_argument("--run-ms", type=int, default=6000)
    parser.add_argument(
        "--enable-oled",
        action="store_true",
        help="enable the processed grayscale/chassis OLED page at 100 ms",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("captures"),
    )
    args = parser.parse_args()
    if args.capture_s is None:
        args.capture_s = (args.run_ms / 1000.0 + 1.0)
    if args.capture_s <= 0.0:
        parser.error("--capture-s must be positive")
    if not 50 <= args.period_ms <= 5000:
        parser.error("--period-ms must be in 50..5000")
    if not 0 <= args.run_ms <= 30000:
        parser.error("--run-ms must be in 0..30000")
    return args


def send_command(port: serial.Serial, command: str) -> None:
    port.write((command + "\r\n").encode("ascii"))
    port.flush()


def parse_data_line(line: str) -> list[float] | None:
    parts = line.strip().split(",")
    if len(parts) != len(EXPECTED_FIELDS):
        return None
    try:
        values = [float(part) for part in parts]
    except ValueError:
        return None
    if not all(math.isfinite(value) for value in values):
        return None
    return values


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

    figure, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)

    axes[0].plot(seconds, columns["gray_pos"], label="Interpolated position")
    axes[0].plot(seconds, columns["lf_err"], "--", label="Controller error")
    invalid_x = [
        seconds[index]
        for index, valid in enumerate(columns["gray_valid"])
        if valid < 0.5
    ]
    invalid_y = [
        columns["gray_pos"][index]
        for index, valid in enumerate(columns["gray_valid"])
        if valid < 0.5
    ]
    if invalid_x:
        axes[0].scatter(invalid_x, invalid_y, marker="x", label="Invalid frame")
    axes[0].axhline(0.0, linewidth=0.8)
    axes[0].set_ylabel("Position (mpos)")
    axes[0].legend(loc="upper right")
    axes[0].grid(True, alpha=0.25)

    axes[1].plot(seconds, columns["L_tgt"], "--", label="Left target")
    axes[1].plot(seconds, columns["L_act"], label="Left actual")
    axes[1].plot(seconds, columns["R_tgt"], "--", label="Right target")
    axes[1].plot(seconds, columns["R_act"], label="Right actual")
    axes[1].set_ylabel("Wheel speed (RPM)")
    axes[1].legend(loc="upper right", ncol=2)
    axes[1].grid(True, alpha=0.25)

    axes[2].plot(seconds, columns["lf_corr"], label="Steering correction")
    axes[2].axhline(0.0, linewidth=0.8)
    axes[2].set_ylabel("Correction (RPM)")
    axes[2].set_xlabel("Time (s)")
    axes[2].legend(loc="upper right")
    axes[2].grid(True, alpha=0.25)

    figure.suptitle("gugaPI line-follow telemetry")
    figure.tight_layout()
    figure.savefig(path, dpi=160)
    plt.close(figure)


def main() -> int:
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    stamp = dt.datetime.now().strftime("%Y%m%d-%H%M%S")
    csv_path = args.output_dir / f"linefollow-{stamp}.csv"
    png_path = args.output_dir / f"linefollow-{stamp}.png"
    records: list[list[float]] = []
    shell_messages: list[str] = []
    started_motion = args.start_rpm is not None

    with serial.Serial(
        args.port,
        args.baud,
        timeout=0.1,
        write_timeout=0.5,
    ) as port:
        time.sleep(0.2)
        port.reset_input_buffer()
        if args.enable_oled:
            send_command(port, "gray oled on 100")
            time.sleep(0.1)
        send_command(port, f"telem on {args.period_ms}")
        time.sleep(0.2)
        if started_motion:
            send_command(port, f"lf start {args.start_rpm} {args.run_ms}")

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
                if values is not None:
                    records.append(values)
                elif not line.startswith("#"):
                    shell_messages.append(line)
        finally:
            if started_motion:
                send_command(port, "lf stop")
                time.sleep(0.1)
            send_command(port, "telem off")
            time.sleep(0.1)

    for message in shell_messages:
        print(message)
    if not records:
        raise RuntimeError(
            "No telemetry rows received; verify the new firmware and close VOFA+"
        )

    write_csv(csv_path, records)
    plot_capture(png_path, records)
    print(f"samples={len(records)}")
    print(f"csv={csv_path.resolve()}")
    print(f"plot={png_path.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
