#!/usr/bin/env python3
"""GELLO leader arm reader (Arduino + 7x AS5600L magnetic encoders, RAW firmware).

Relays raw GELLO firmware lines, UNPROCESSED, from a local serial port for
input_agent.py to forward to robot/arm_agent.py's WebSocket. Calibration
math (unwrap the 0/360 seam, clip to measured range, smoothing, direction/
scale/offset) happens in exactly ONE place now: the REAL lerobot
GelloAs5600RawLeader class, running server-side in arm_agent.py -- not a
reimplementation here. See that file's module docstring for the full
story: an earlier version of this file DID reimplement that math, and had
a real bug (ported the wrong lerobot teleoperator class entirely, no
angle-unwrap). Relaying raw, uninterpreted data removes that whole class
of bug -- this file is now just a thin serial reader.

The RAW GELLO firmware streams plain ASCII over serial, one line every
~16ms:

    t<ms> J1:<deg> J2:<deg> ... J7:<deg>

<deg> is the ABSOLUTE sensor angle in [0, 360) -- no firmware zero, no
serial command interface.
"""
import os
import threading
import time

import serial

PORT_ENV = "GELLO_PORT"
BAUDRATE = 115200
FIRMWARE_RESET_SETTLE_SEC = 2.5  # opening the port resets the Arduino (DTR)


class GelloReader:
    """Background serial reader: keeps the latest raw line, untouched."""

    def __init__(self, port: str, baudrate: int = BAUDRATE):
        self.ser = serial.Serial(port, baudrate=baudrate, timeout=0.2)
        self._lock = threading.Lock()
        self._latest_line: str | None = None
        self._stop = threading.Event()

        # Opening the port resets the Arduino (DTR); let the firmware boot
        # before reading. The RAW firmware has no serial command interface
        # (no zero/recalibrate prompt to answer, unlike the older
        # EEPROM-zeroed GELLO firmware) -- it just starts streaming.
        time.sleep(FIRMWARE_RESET_SETTLE_SEC)
        self.ser.reset_input_buffer()

        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                raw = self.ser.readline()
            except Exception:
                continue
            if not raw:
                continue
            line = raw.decode("ascii", errors="ignore").strip()
            if "J" not in line:  # cheap filter for garbage/partial reads
                continue
            with self._lock:
                self._latest_line = line

    def latest_line(self) -> str | None:
        with self._lock:
            return self._latest_line

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=1.0)
        self.ser.close()


_reader: GelloReader | None = None
_init_failed = False


def read_gello_raw_line() -> str | None:
    """Entry point imported by input_agent.py.

    Returns the latest raw GELLO firmware line (untouched -- see
    robot/arm_agent.py's on_arm_ws() for what parses it), or None if the
    GELLO isn't usable at all: GELLO_PORT unset, the port failed to open,
    or no line has arrived yet. Port-open failure is sticky for this
    process's lifetime (no retry loop) -- restart input_agent.py once the
    GELLO is actually plugged in.
    """
    global _reader, _init_failed
    if _init_failed:
        return None
    if _reader is None:
        port = os.environ.get(PORT_ENV)
        if not port:
            print(f"[gello_reader] {PORT_ENV} not set -- GELLO disabled.")
            _init_failed = True
            return None
        try:
            _reader = GelloReader(port)
            print(f"[gello_reader] connected on {port}")
        except Exception as exc:
            print(f"[gello_reader] failed to open {port}: {exc}")
            _init_failed = True
            return None

    return _reader.latest_line()


if __name__ == "__main__":
    print(f"gello_reader standalone test ({PORT_ENV}={os.environ.get(PORT_ENV)!r}) -- "
          "move the GELLO, Ctrl+C to stop.")
    try:
        while True:
            line = read_gello_raw_line()
            print(f"{line or 'no reading yet...':<80}", end="\r")
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\nstopped.")
