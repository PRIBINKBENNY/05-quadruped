"""Where the pulses go.

- PCA9685Backend: Raspberry Pi (or any Linux SBC) talking I2C to a PCA9685 board directly.
- SerialBackend:  an ESP32/Arduino running firmware/servo_bridge.ino, which drives the PCA9685.
- DryRunBackend:  prints or records pulses. Use it before you power the servos.
"""
from __future__ import annotations

import time


class DryRunBackend:
    def __init__(self, echo: bool = False):
        self.echo = echo
        self.frames: list[dict[int, int]] = []

    def write(self, pulses: dict[int, int]) -> None:
        self.frames.append(dict(pulses))
        if self.echo:
            print(" ".join(f"{ch}:{us}" for ch, us in sorted(pulses.items())))


class PCA9685Backend:
    def __init__(self, address: int = 0x40, freq_hz: int = 50):
        from adafruit_servokit import ServoKit  # pip install adafruit-circuitpython-servokit

        self.kit = ServoKit(channels=16, address=address, frequency=freq_hz)
        self.period_us = 1_000_000 / freq_hz

    def write(self, pulses: dict[int, int]) -> None:
        for ch, us in pulses.items():
            self.kit._pca.channels[ch].duty_cycle = int(us / self.period_us * 0xFFFF)

    def relax(self) -> None:
        for ch in range(16):
            self.kit._pca.channels[ch].duty_cycle = 0


def encode_serial(pulses: dict[int, int]) -> bytes:
    """Line protocol understood by firmware/servo_bridge.ino:  S ch:us ch:us ...\\n"""
    return ("S " + " ".join(f"{ch}:{us}" for ch, us in sorted(pulses.items())) + "\n").encode()


class SerialBackend:
    def __init__(self, port: str = "/dev/ttyUSB0", baud: int = 115200):
        import serial  # pip install pyserial

        self.ser = serial.Serial(port, baud, timeout=0.1)
        time.sleep(2.0)  # many boards reset when the port opens

    def write(self, pulses: dict[int, int]) -> None:
        self.ser.write(encode_serial(pulses))

    def relax(self) -> None:
        self.ser.write(b"R\n")
