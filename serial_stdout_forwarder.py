#!/usr/bin/env python3
import argparse
import subprocess
import sys
from pathlib import Path

try:
    import serial
except ImportError as exc:  # pragma: no cover
    raise SystemExit("pyserial is required. Install it with: pip install pyserial") from exc


def parse_args():
    parser = argparse.ArgumentParser(
        description="Run a command, mirror stdout/stderr to console, and forward it to a serial port."
    )
    parser.add_argument("--port", required=True, help="Target serial port, e.g. COM10")
    parser.add_argument("--baudrate", type=int, default=115200, help="Serial baudrate")
    parser.add_argument(
        "--encoding",
        default="utf-8",
        help="Encoding used for forwarding process output to serial",
    )
    parser.add_argument(
        "--log",
        help="Optional log file path for saving forwarded output",
    )
    parser.add_argument(
        "command",
        nargs=argparse.REMAINDER,
        help="Command to run, for example: -- pc_radar_algo_float_v2\\build\\Release\\pc_radar_algo_float_v2.exe 2dfft_array AT_sy_r10_1.txt",
    )
    args = parser.parse_args()
    if args.command and args.command[0] == "--":
        args.command = args.command[1:]
    if not args.command:
        parser.error("missing command to run; pass it after '--'")
    return args


def main():
    args = parse_args()

    log_path = Path(args.log) if args.log else None
    log_fh = log_path.open("a", encoding=args.encoding) if log_path else None

    try:
        with serial.Serial(args.port, args.baudrate, timeout=1) as ser:
            proc = subprocess.Popen(
                args.command,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                encoding=args.encoding,
                errors="replace",
                bufsize=1,
            )

            assert proc.stdout is not None
            for line in proc.stdout:
                sys.stdout.write(line)
                sys.stdout.flush()

                if log_fh is not None:
                    log_fh.write(line)
                    log_fh.flush()

                ser.write(line.encode(args.encoding, errors="replace"))
                ser.flush()

            return proc.wait()
    finally:
        if log_fh is not None:
            log_fh.close()


if __name__ == "__main__":
    raise SystemExit(main())
