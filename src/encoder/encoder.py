"""Read a quadrature encoder through a Counter Click (LS7366R).

The defaults match wspec2026.kicad_sch:

* SPI0 SCK/MISO/MOSI: BCM GPIO11/GPIO9/GPIO10
* SPI0 CS0: BCM GPIO27 (configured by a Device Tree overlay)
* Counter enable: BCM GPIO26
* Interrupt: BCM GPIO19 (not used by this polling driver)
"""

from __future__ import annotations

import argparse
import sys
import time

import spidev
from gpiozero import DigitalOutputDevice


class Encoder:
    """Driver for the LS7366R 32-bit quadrature counter."""

    WRITE_MDR0 = 0x88
    WRITE_MDR1 = 0x90
    READ_MDR0 = 0x48
    READ_MDR1 = 0x50
    READ_CNTR = 0x60
    READ_STR = 0x70
    CLEAR_CNTR = 0x20

    DEFAULT_COUNT_ENABLE_GPIO = 26
    INTERRUPT_GPIO = 19
    DEFAULT_PPR = 5120

    def __init__(
        self,
        count_enable_gpio: int = DEFAULT_COUNT_ENABLE_GPIO,
        bus: int = 0,
        device: int = 0,
        spi_speed: int = 500_000,
        quadrature: int = 4,
    ) -> None:
        """Initialize the counter.

        ``device=0`` expects SPI0 CS0 to be assigned to BCM GPIO27 with
        ``dtoverlay=spi0-1cs,cs0_pin=27``.  The mikroBUS RST pin on Counter
        Click carries the LS7366R CNT_EN signal; it is not a reset input.
        """

        quadrature_modes = {1: 0x01, 2: 0x02, 4: 0x03}
        if quadrature not in quadrature_modes:
            raise ValueError("quadrature must be 1, 2, or 4")
        if bus < 0 or device < 0:
            raise ValueError("SPI bus and device must be non-negative")
        if spi_speed <= 0:
            raise ValueError("spi_speed must be positive")

        self.quadrature = quadrature
        self._closed = False

        # Keep counting disabled while configuring the LS7366R. GPIO26 is
        # connected to Counter Click RST, which is the active-high CNT_EN pin.
        self._count_enable = DigitalOutputDevice(
            count_enable_gpio,
            active_high=True,
            initial_value=False,
        )
        self._spi = spidev.SpiDev()

        try:
            self._spi.open(bus, device)
            self._spi.max_speed_hz = spi_speed
            self._spi.mode = 0
            self._spi.bits_per_word = 8
            self._spi.lsbfirst = False

            # MDR0: selected quadrature multiplier, free-running mode,
            # index disabled, filter clock divisor 1.
            expected_mdr0 = quadrature_modes[quadrature]

            # MDR1: 4-byte (32-bit) counter, software counting enabled,
            # interrupt flags disabled.
            expected_mdr1 = 0x00

            self._spi.xfer2([self.WRITE_MDR0, expected_mdr0])
            self._spi.xfer2([self.WRITE_MDR1, expected_mdr1])

            actual_mdr0, actual_mdr1 = self.read_registers()
            if (actual_mdr0, actual_mdr1) != (
                expected_mdr0,
                expected_mdr1,
            ):
                raise RuntimeError(
                    "LS7366R communication failed: "
                    f"expected MDR0/MDR1=0x{expected_mdr0:02X}/"
                    f"0x{expected_mdr1:02X}, received "
                    f"0x{actual_mdr0:02X}/0x{actual_mdr1:02X}"
                )

            self._count_enable.on()
            if not self.read_status() & 0x08:
                raise RuntimeError("LS7366R counting is not enabled")
        except BaseException:
            self.close()
            raise

    def read_registers(self) -> tuple[int, int]:
        """Return the MDR0 and MDR1 configuration registers."""

        mdr0 = self._spi.xfer2([self.READ_MDR0, 0x00])[1]
        mdr1 = self._spi.xfer2([self.READ_MDR1, 0x00])[1]
        return mdr0, mdr1

    def read_status(self) -> int:
        """Return the LS7366R status register."""

        return self._spi.xfer2([self.READ_STR, 0x00])[1]

    def read_count(self) -> int:
        """Return the current counter value as a signed 32-bit integer."""

        response = self._spi.xfer2([self.READ_CNTR, 0, 0, 0, 0])
        return int.from_bytes(
            bytes(response[1:]),
            byteorder="big",
            signed=True,
        )

    def reset_count(self) -> None:
        """Clear the counter value to zero using the SPI command."""

        self._spi.xfer2([self.CLEAR_CNTR])

    @staticmethod
    def count_difference(current: int, previous: int) -> int:
        """Return a count difference with signed 32-bit wraparound."""

        return ((current - previous + 2**31) % 2**32) - 2**31

    def close(self) -> None:
        """Release the SPI and GPIO resources."""

        if self._closed:
            return

        try:
            self._count_enable.off()
        finally:
            try:
                self._spi.close()
            finally:
                self._count_enable.close()
                self._closed = True

    def __enter__(self) -> Encoder:
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="LS7366R encoder test")
    parser.add_argument(
        "--enable-gpio",
        type=int,
        default=Encoder.DEFAULT_COUNT_ENABLE_GPIO,
        help="BCM GPIO connected to Counter Click CNT_EN (default: 26)",
    )
    parser.add_argument("--interval", type=float, default=0.1)
    parser.add_argument(
        "--ppr",
        type=int,
        default=Encoder.DEFAULT_PPR,
        help="encoder pulses per revolution (default: 5120 for AMT10E2-V)",
    )
    parser.add_argument("--zero", action="store_true")
    parser.add_argument(
        "--quadrature",
        type=int,
        choices=(1, 2, 4),
        default=4,
    )
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    if args.interval <= 0:
        parser.error("--interval must be positive")
    if args.ppr <= 0:
        parser.error("--ppr must be positive")

    try:
        with Encoder(
            count_enable_gpio=args.enable_gpio,
            quadrature=args.quadrature,
        ) as encoder:
            if args.zero:
                encoder.reset_count()

            mdr0, mdr1 = encoder.read_registers()
            print("Encoder initialized")
            print(f"MDR0: 0x{mdr0:02X}")
            print(f"MDR1: 0x{mdr1:02X}")
            print("Press Ctrl+C to stop")
            print("-" * 72)

            previous_count = encoder.read_count()
            previous_time = time.monotonic()

            while True:
                time.sleep(args.interval)
                count = encoder.read_count()
                now = time.monotonic()
                elapsed = now - previous_time
                delta = encoder.count_difference(count, previous_count)
                counts_per_second = delta / elapsed

                message = (
                    f"Count: {count:11d} | "
                    f"Delta: {delta:8d} | "
                    f"Count/s: {counts_per_second:11.1f}"
                )

                counts_per_revolution = args.ppr * encoder.quadrature
                rpm = counts_per_second * 60 / counts_per_revolution
                message += f" | RPM: {rpm:9.2f}"

                print(message)
                previous_count = count
                previous_time = now
    except KeyboardInterrupt:
        print("\nEncoder test finished")
        return 0
    except (OSError, RuntimeError) as error:
        print(f"Encoder error: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
