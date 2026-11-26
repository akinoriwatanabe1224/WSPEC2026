#!/usr/bin/env python3
"""
VESC USB communication helper for Raspberry Pi / Linux.

VESCのUSB端子はLinux上では /dev/ttyACM0 などのUSB CDC仮想シリアルとして見える。
通信プロトコル自体はUART版と同じVESC packet protocolを使う。

注意:
- set_duty_percent(10.0) は duty 10% = VESC内部のduty比 0.10 を送る。
- set_duty_ratio(0.10) も同じ意味。
"""
from __future__ import annotations

import glob
import math
import struct
import time
from dataclasses import dataclass
from typing import Optional, Dict, Any, Iterable

import serial
from serial.tools import list_ports


COMM_SET_DUTY = 5
COMM_ALIVE = 30
COMM_GET_VALUES = 4
COMM_GET_IMU_DATA = 65


FAULT_CODES = {
    0: "FAULT_CODE_NONE",
    1: "FAULT_CODE_OVER_VOLTAGE",
    2: "FAULT_CODE_UNDER_VOLTAGE",
    3: "FAULT_CODE_DRV",
    4: "FAULT_CODE_ABS_OVER_CURRENT",
    5: "FAULT_CODE_OVER_TEMP_FET",
    6: "FAULT_CODE_OVER_TEMP_MOTOR",
    7: "FAULT_CODE_GATE_DRIVER_OVER_VOLTAGE",
    8: "FAULT_CODE_GATE_DRIVER_UNDER_VOLTAGE",
    9: "FAULT_CODE_MCU_UNDER_VOLTAGE",
    10: "FAULT_CODE_BOOTING_FROM_WATCHDOG_RESET",
    11: "FAULT_CODE_ENCODER_SPI",
    12: "FAULT_CODE_ENCODER_SINCOS_BELOW_MIN_AMPLITUDE",
    13: "FAULT_CODE_ENCODER_SINCOS_ABOVE_MAX_AMPLITUDE",
    14: "FAULT_CODE_FLASH_CORRUPTION",
    15: "FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_1",
    16: "FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_2",
    17: "FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_3",
    18: "FAULT_CODE_UNBALANCED_CURRENTS",
    19: "FAULT_CODE_BRK",
    20: "FAULT_CODE_RESOLVER_LOT",
    21: "FAULT_CODE_RESOLVER_DOS",
    22: "FAULT_CODE_RESOLVER_LOS",
    23: "FAULT_CODE_FLASH_CORRUPTION_APP_CFG",
    24: "FAULT_CODE_FLASH_CORRUPTION_MC_CFG",
    25: "FAULT_CODE_ENCODER_NO_MAGNET",
    26: "FAULT_CODE_ENCODER_MAGNET_TOO_STRONG",
    27: "FAULT_CODE_PHASE_FILTER",
    28: "FAULT_CODE_ENCODER_FAULT",
    29: "FAULT_CODE_LV_OUTPUT_FAULT",
    30: "FAULT_CODE_ENCODER_SLIP",
    31: "FAULT_CODE_OVERSPEED",
    32: "FAULT_CODE_UNDERSPEED",
    33: "FAULT_CODE_ABS_OVERSPEED",
}


@dataclass
class VescValues:
    temp_fet_c: Optional[float] = None
    temp_motor_c: Optional[float] = None

    current_motor_a: Optional[float] = None
    current_in_a: Optional[float] = None
    current_id_a: Optional[float] = None
    current_iq_a: Optional[float] = None

    duty_now: Optional[float] = None  # VESC内部のduty比。例: 0.10 = 10%
    erpm: Optional[float] = None
    voltage_in_v: Optional[float] = None

    amp_hours: Optional[float] = None
    amp_hours_charged: Optional[float] = None
    watt_hours: Optional[float] = None
    watt_hours_charged: Optional[float] = None

    tachometer: Optional[int] = None
    tachometer_abs: Optional[int] = None

    fault_code: Optional[int] = None
    fault_name: Optional[str] = None

    pid_pos_now_deg: Optional[float] = None
    controller_id: Optional[int] = None

    temp_mos1_c: Optional[float] = None
    temp_mos2_c: Optional[float] = None
    temp_mos3_c: Optional[float] = None

    vd_v: Optional[float] = None
    vq_v: Optional[float] = None

    status: Optional[int] = None

    def as_dict(self) -> Dict[str, Any]:
        return self.__dict__.copy()


@dataclass
class VescImuData:
    roll_rad: Optional[float] = None
    pitch_rad: Optional[float] = None
    yaw_rad: Optional[float] = None

    acc_x: Optional[float] = None
    acc_y: Optional[float] = None
    acc_z: Optional[float] = None

    gyro_x_rad_s: Optional[float] = None
    gyro_y_rad_s: Optional[float] = None
    gyro_z_rad_s: Optional[float] = None

    mag_x: Optional[float] = None
    mag_y: Optional[float] = None
    mag_z: Optional[float] = None

    q0: Optional[float] = None
    q1: Optional[float] = None
    q2: Optional[float] = None
    q3: Optional[float] = None

    controller_id: Optional[int] = None

    def as_dict(self) -> Dict[str, Any]:
        return self.__dict__.copy()


class VescUsb:
    def __init__(
        self,
        port: str = "/dev/ttyACM0",
        baudrate: int = 115200,
        timeout: float = 0.3,
    ) -> None:
        # USB CDCではbaudrateは実質重要でない場合が多いが、pyserial API上は指定する。
        self.port = resolve_port(port)
        self.baudrate = baudrate
        self.timeout = timeout

        self.ser = serial.Serial(
            port=self.port,
            baudrate=baudrate,
            timeout=timeout,
            write_timeout=timeout,
        )

    def close(self) -> None:
        self.ser.close()

    def __enter__(self) -> "VescUsb":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    @staticmethod
    def crc16(data: bytes) -> int:
        """
        VESC packet CRC16.
        Polynomial: 0x1021
        Initial value: 0
        """
        crc = 0

        for b in data:
            crc ^= b << 8

            for _ in range(8):
                if crc & 0x8000:
                    crc = ((crc << 1) ^ 0x1021) & 0xFFFF
                else:
                    crc = (crc << 1) & 0xFFFF

        return crc

    @classmethod
    def make_packet(cls, payload: bytes) -> bytes:
        """
        VESC packetを作る。
        payload length <= 255:
          0x02, len, payload, crc high, crc low, 0x03
        payload length <= 65535:
          0x03, len high, len low, payload, crc high, crc low, 0x03
        """
        crc = cls.crc16(payload)

        if len(payload) <= 255:
            return (
                bytes([2, len(payload)])
                + payload
                + crc.to_bytes(2, "big")
                + bytes([3])
            )

        if len(payload) <= 65535:
            return (
                bytes([3])
                + len(payload).to_bytes(2, "big")
                + payload
                + crc.to_bytes(2, "big")
                + bytes([3])
            )

        raise ValueError("Payload too long")

    def send_payload(self, payload: bytes) -> None:
        packet = self.make_packet(payload)
        self.ser.write(packet)
        self.ser.flush()

    def _read_exact(self, n: int) -> bytes:
        buf = bytearray()
        deadline = time.monotonic() + self.timeout

        while len(buf) < n:
            chunk = self.ser.read(n - len(buf))

            if chunk:
                buf.extend(chunk)
                continue

            if time.monotonic() > deadline:
                raise TimeoutError(f"USB serial timeout: needed {n}, got {len(buf)}")

        return bytes(buf)

    def read_packet(self) -> bytes:
        """
        VESC packetを1つ読み、payloadだけを返す。
        """
        start = None
        deadline = time.monotonic() + self.timeout

        while time.monotonic() < deadline:
            b = self.ser.read(1)

            if not b:
                continue

            if b[0] in (2, 3):
                start = b[0]
                break

        if start is None:
            raise TimeoutError("No VESC packet start byte received")

        if start == 2:
            payload_len = self._read_exact(1)[0]
        else:
            payload_len = int.from_bytes(self._read_exact(2), "big")

        payload = self._read_exact(payload_len)
        received_crc = int.from_bytes(self._read_exact(2), "big")
        end = self._read_exact(1)[0]

        if end != 3:
            raise ValueError(f"Invalid VESC packet end byte: {end}")

        calc_crc = self.crc16(payload)

        if received_crc != calc_crc:
            raise ValueError(
                f"CRC mismatch: received=0x{received_crc:04X}, calc=0x{calc_crc:04X}"
            )

        return payload

    def get_values(self) -> VescValues:
        """
        VESCの基本データを取得する。
        電圧、電流、ERPM、温度、faultなど。
        """
        self.ser.reset_input_buffer()
        self.send_payload(bytes([COMM_GET_VALUES]))
        payload = self.read_packet()

        if len(payload) < 1:
            raise ValueError("Empty payload")

        if payload[0] != COMM_GET_VALUES:
            raise ValueError(f"Unexpected response command: {payload[0]}")

        return self.parse_get_values(payload)

    def set_duty_percent(self, duty_percent: float) -> None:
        """
        dutyを%で指定してVESCに送る。

        例:
          10.0  -> duty比 0.10
          -5.0  -> duty比 -0.05

        VESCのCOMM_SET_DUTYは内部的に duty_ratio * 100000 のint32を送る。
        """
        if duty_percent < -100.0 or duty_percent > 100.0:
            raise ValueError("duty_percent must be between -100.0 and 100.0")

        self.set_duty_ratio(duty_percent / 100.0)

    def set_duty_ratio(self, duty_ratio: float) -> None:
        """
        duty比を-1.0〜+1.0で指定してVESCに送る。

        例:
          0.10 -> 10%
          1.00 -> 100%
        """
        if duty_ratio < -1.0 or duty_ratio > 1.0:
            raise ValueError("duty_ratio must be between -1.0 and 1.0")

        duty_int = int(duty_ratio * 100000.0)

        payload = bytes([COMM_SET_DUTY]) + duty_int.to_bytes(
            4,
            byteorder="big",
            signed=True,
        )

        self.send_payload(payload)

    def set_duty(self, duty_percent: float) -> None:
        """
        互換用: 既存コードに合わせ、set_duty(10.0) = 10% として扱う。
        新規コードでは set_duty_percent() または set_duty_ratio() を推奨。
        """
        self.set_duty_percent(duty_percent)

    def send_alive(self) -> None:
        """
        VESCの通信タイムアウト対策用。
        set_dutyを周期送信している場合は必須ではないが、必要なら使う。
        """
        self.send_payload(bytes([COMM_ALIVE]))

    def stop_motor(self) -> None:
        """
        モータ停止用。duty=0%を送る。
        """
        self.set_duty_percent(0.0)

    @staticmethod
    def parse_get_values(payload: bytes) -> VescValues:
        """
        payload[0] = COMM_GET_VALUES

        ファームウェア差分にある程度対応するため、存在するところまで読む。
        """
        v = VescValues()
        i = 1

        def can_read(n: int) -> bool:
            return i + n <= len(payload)

        def get_i16(scale: float) -> Optional[float]:
            nonlocal i
            if not can_read(2):
                return None
            raw = struct.unpack_from(">h", payload, i)[0]
            i += 2
            return raw / scale

        def get_i32(scale: float) -> Optional[float]:
            nonlocal i
            if not can_read(4):
                return None
            raw = struct.unpack_from(">i", payload, i)[0]
            i += 4
            return raw / scale

        def get_int32() -> Optional[int]:
            nonlocal i
            if not can_read(4):
                return None
            raw = struct.unpack_from(">i", payload, i)[0]
            i += 4
            return raw

        def get_u8() -> Optional[int]:
            nonlocal i
            if not can_read(1):
                return None
            raw = payload[i]
            i += 1
            return raw

        v.temp_fet_c = get_i16(10.0)
        v.temp_motor_c = get_i16(10.0)

        v.current_motor_a = get_i32(100.0)
        v.current_in_a = get_i32(100.0)

        v.current_id_a = get_i32(100.0)
        v.current_iq_a = get_i32(100.0)

        v.duty_now = get_i16(1000.0)
        v.erpm = get_i32(1.0)
        v.voltage_in_v = get_i16(10.0)

        v.amp_hours = get_i32(10000.0)
        v.amp_hours_charged = get_i32(10000.0)
        v.watt_hours = get_i32(10000.0)
        v.watt_hours_charged = get_i32(10000.0)

        v.tachometer = get_int32()
        v.tachometer_abs = get_int32()

        v.fault_code = get_u8()
        if v.fault_code is not None:
            v.fault_name = FAULT_CODES.get(
                v.fault_code,
                f"UNKNOWN_FAULT_{v.fault_code}",
            )

        v.pid_pos_now_deg = get_i32(1_000_000.0)
        v.controller_id = get_u8()

        v.temp_mos1_c = get_i16(10.0)
        v.temp_mos2_c = get_i16(10.0)
        v.temp_mos3_c = get_i16(10.0)

        v.vd_v = get_i32(1000.0)
        v.vq_v = get_i32(1000.0)

        v.status = get_u8()

        return v

    @staticmethod
    def _decode_float32_auto_from_uint32(raw: int) -> float:
        """
        VESC buffer_get_float32_auto 相当のデコード。
        IMUデータのfloat復号に使う。
        """
        e = (raw >> 23) & 0xFF
        sig_i = raw & 0x7FFFFF
        neg = raw & (1 << 31)

        sig = 0.0

        if e != 0 or sig_i != 0:
            sig = sig_i / (8388608.0 * 2.0) + 0.5
            e -= 126

        if neg:
            sig = -sig

        return math.ldexp(sig, e)

    @staticmethod
    def _get_float32_auto(payload: bytes, idx: list[int]) -> Optional[float]:
        if idx[0] + 4 > len(payload):
            return None

        raw = struct.unpack_from(">I", payload, idx[0])[0]
        idx[0] += 4

        return VescUsb._decode_float32_auto_from_uint32(raw)

    def get_imu_data(self, mask: int = 0x01C7) -> VescImuData:
        """
        VESCのIMUデータを取得する。

        デフォルトmask = 0x01C7
          bit0: roll
          bit1: pitch
          bit2: yaw
          bit6: gyro_x
          bit7: gyro_y
          bit8: gyro_z
        """
        if not (0 <= mask <= 0xFFFF):
            raise ValueError("IMU mask must be 0x0000 to 0xFFFF")

        self.ser.reset_input_buffer()

        payload = bytes([
            COMM_GET_IMU_DATA,
            (mask >> 8) & 0xFF,
            mask & 0xFF,
        ])

        self.send_payload(payload)
        response = self.read_packet()

        if len(response) < 3:
            raise ValueError(f"IMU response too short: {len(response)} bytes")

        if response[0] != COMM_GET_IMU_DATA:
            raise ValueError(f"Unexpected IMU response command: {response[0]}")

        return self.parse_imu_data(response)

    @staticmethod
    def parse_imu_data(payload: bytes) -> VescImuData:
        """
        payload[0] = COMM_GET_IMU_DATA
        payload[1:3] = mask
        その後、maskで指定した値が順番に入る。
        """
        imu = VescImuData()
        idx = [1]

        mask = struct.unpack_from(">H", payload, idx[0])[0]
        idx[0] += 2

        def read_if(bit: int) -> Optional[float]:
            if mask & (1 << bit):
                return VescUsb._get_float32_auto(payload, idx)
            return None

        imu.roll_rad = read_if(0)
        imu.pitch_rad = read_if(1)
        imu.yaw_rad = read_if(2)

        imu.acc_x = read_if(3)
        imu.acc_y = read_if(4)
        imu.acc_z = read_if(5)

        imu.gyro_x_rad_s = read_if(6)
        imu.gyro_y_rad_s = read_if(7)
        imu.gyro_z_rad_s = read_if(8)

        imu.mag_x = read_if(9)
        imu.mag_y = read_if(10)
        imu.mag_z = read_if(11)

        imu.q0 = read_if(12)
        imu.q1 = read_if(13)
        imu.q2 = read_if(14)
        imu.q3 = read_if(15)

        if idx[0] < len(payload):
            imu.controller_id = payload[idx[0]]

        return imu


def list_serial_ports() -> list[str]:
    """現在見えているシリアルポートを説明付きで返す。"""
    lines = []
    for p in list_ports.comports():
        lines.append(f"{p.device}\t{p.description}\t{p.hwid}")
    return lines


def resolve_port(port: str) -> str:
    """
    port='auto' の場合は /dev/serial/by-id, /dev/ttyACM*, /dev/ttyUSB* の順で候補を選ぶ。
    VESC以外のUSBシリアルも混ざる場合は、明示的に --port で指定すること。
    """
    if port != "auto":
        return port

    candidates: list[str] = []

    patterns: Iterable[str] = (
        "/dev/serial/by-id/*VESC*",
        "/dev/serial/by-id/*vesc*",
        "/dev/serial/by-id/*STMicroelectronics*",
        "/dev/serial/by-id/*STMicro*",
        "/dev/ttyACM*",
        "/dev/ttyUSB*",
    )

    for pattern in patterns:
        candidates.extend(sorted(glob.glob(pattern)))

    # 重複削除しつつ順序維持
    seen = set()
    unique = []
    for c in candidates:
        if c not in seen:
            unique.append(c)
            seen.add(c)

    if not unique:
        raise FileNotFoundError(
            "No USB serial port found. Try --list-ports or specify --port /dev/ttyACM0"
        )

    return unique[0]


# 既存コードとの互換用エイリアス。from vesc_usb import VescUart でも動く。
VescUart = VescUsb
