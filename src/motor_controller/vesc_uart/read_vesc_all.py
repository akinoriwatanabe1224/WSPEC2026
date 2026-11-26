#!/usr/bin/env python3
import argparse
import csv
import math
import time
from datetime import datetime

from vesc_uart import VescUart


def fmt(x, digits=2):
    if x is None:
        return "N/A"
    return f"{x:.{digits}f}"


def rad_to_deg(x):
    if x is None:
        return None
    return x * 180.0 / math.pi


def main():
    parser = argparse.ArgumentParser(
        description="Read VESC voltage, current, RPM and IMU data via UART"
    )

    parser.add_argument("--port", default="/dev/serial0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--hz", type=float, default=20.0)

    # Scorpionなど14極モータなら極対数は7
    parser.add_argument("--motor-pole-pairs", type=float, default=7.0)

    parser.add_argument("--csv", default="")

    args = parser.parse_args()

    # roll, pitch, yaw + gyro_x, gyro_y, gyro_z
    IMU_MASK = 0x01C7

    csv_file = None
    writer = None

    if args.csv:
        csv_file = open(args.csv, "w", newline="")
        writer = csv.writer(csv_file)

        writer.writerow([
            "time",
            "voltage_in_v",
            "current_in_a",
            "current_motor_a",
            "duty_now",
            "erpm",
            "motor_mechanical_rpm",
            "temp_fet_c",
            "temp_motor_c",
            "fault_code",
            "fault_name",
            "tachometer",
            "tachometer_abs",
            "roll_rad",
            "pitch_rad",
            "yaw_rad",
            "roll_deg",
            "pitch_deg",
            "yaw_deg",
            "gyro_x_rad_s",
            "gyro_y_rad_s",
            "gyro_z_rad_s",
            "gyro_x_deg_s",
            "gyro_y_deg_s",
            "gyro_z_deg_s",
        ])

    dt_sleep = 1.0 / args.hz

    with VescUart(
        port=args.port,
        baudrate=args.baud,
        timeout=0.3,
    ) as vesc:

        print("Start reading VESC data")
        print(f"UART port : {args.port}")
        print(f"Baudrate  : {args.baud}")
        print(f"Rate      : {args.hz} Hz")
        print(f"Pole pairs: {args.motor_pole_pairs}")
        print("Ctrl+C to stop")

        try:
            while True:
                now_text = datetime.now().isoformat(timespec="milliseconds")

                values = vesc.get_values()
                imu = vesc.get_imu_data(IMU_MASK)

                motor_mechanical_rpm = None
                if values.erpm is not None and args.motor_pole_pairs > 0:
                    motor_mechanical_rpm = values.erpm / args.motor_pole_pairs

                roll_deg = rad_to_deg(imu.roll_rad)
                pitch_deg = rad_to_deg(imu.pitch_rad)
                yaw_deg = rad_to_deg(imu.yaw_rad)

                gyro_x_deg_s = rad_to_deg(imu.gyro_x_rad_s)
                gyro_y_deg_s = rad_to_deg(imu.gyro_y_rad_s)
                gyro_z_deg_s = rad_to_deg(imu.gyro_z_rad_s)

                print(
                    f"{now_text} | "
                    f"Vin={fmt(values.voltage_in_v)} V | "
                    f"Iin={fmt(values.current_in_a)} A | "
                    f"Imotor={fmt(values.current_motor_a)} A | "
                    f"Duty={fmt(values.duty_now, 3)} | "
                    f"ERPM={fmt(values.erpm, 0)} | "
                    f"MechRPM={fmt(motor_mechanical_rpm, 0)} | "
                    f"FET={fmt(values.temp_fet_c)} °C | "
                    f"Motor={fmt(values.temp_motor_c)} °C | "
                    f"Roll={fmt(roll_deg, 1)} deg | "
                    f"Pitch={fmt(pitch_deg, 1)} deg | "
                    f"Yaw={fmt(yaw_deg, 1)} deg | "
                    f"Gyro=({fmt(imu.gyro_x_rad_s, 3)}, "
                    f"{fmt(imu.gyro_y_rad_s, 3)}, "
                    f"{fmt(imu.gyro_z_rad_s, 3)}) rad/s | "
                    f"Fault={values.fault_name}"
                )

                if writer is not None:
                    writer.writerow([
                        now_text,
                        values.voltage_in_v,
                        values.current_in_a,
                        values.current_motor_a,
                        values.duty_now,
                        values.erpm,
                        motor_mechanical_rpm,
                        values.temp_fet_c,
                        values.temp_motor_c,
                        values.fault_code,
                        values.fault_name,
                        values.tachometer,
                        values.tachometer_abs,
                        imu.roll_rad,
                        imu.pitch_rad,
                        imu.yaw_rad,
                        roll_deg,
                        pitch_deg,
                        yaw_deg,
                        imu.gyro_x_rad_s,
                        imu.gyro_y_rad_s,
                        imu.gyro_z_rad_s,
                        gyro_x_deg_s,
                        gyro_y_deg_s,
                        gyro_z_deg_s,
                    ])

                    csv_file.flush()

                time.sleep(dt_sleep)

        except KeyboardInterrupt:
            print("\nStopped")

        finally:
            if csv_file is not None:
                csv_file.close()


if __name__ == "__main__":
    main()
