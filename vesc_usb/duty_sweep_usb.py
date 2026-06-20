#!/usr/bin/env python3
import argparse
import time

from vesc_usb import VescUsb, list_serial_ports


vesc = None


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def set_duty_percent(duty_percent: float, max_duty_percent: float) -> None:
    """VESCへdutyを%で送る。10.0なら10%。"""
    global vesc

    duty_percent = clamp(duty_percent, -max_duty_percent, max_duty_percent)
    vesc.set_duty_percent(duty_percent)


def hold_duty_percent(duty_percent: float, hold_time: float, step_interval: float, max_duty_percent: float) -> None:
    """指定したdutyを一定時間保持する。保持中も周期的に送信する。"""
    if hold_time <= 0:
        return

    hold_count = int(hold_time / step_interval)

    for _ in range(hold_count):
        set_duty_percent(duty_percent, max_duty_percent)
        print(f"duty = {duty_percent:+.2f} % hold")
        time.sleep(step_interval)


def sweep_once(max_duty_percent: float, duty_div: int, step_interval: float, hold_time: float, zero_hold_time: float) -> None:
    """
    1周期分のduty掃引。
    0 -> +MAX
    +MAX保持
    +MAX -> -MAX
    -MAX保持
    -MAX -> 0
    """

    # Duty上昇 0 -> +MAX_DUTY
    for i in range(0, duty_div + 1):
        duty = (i / duty_div) * max_duty_percent
        set_duty_percent(duty, max_duty_percent)
        print(f"duty = {duty:+.2f} %")
        time.sleep(step_interval)

    # +MAX_DUTYで保持
    hold_duty_percent(max_duty_percent, hold_time, step_interval, max_duty_percent)

    # Duty下降 +MAX_DUTY -> -MAX_DUTY
    for i in range(duty_div, -duty_div - 1, -1):
        duty = (i / duty_div) * max_duty_percent
        set_duty_percent(duty, max_duty_percent)
        print(f"duty = {duty:+.2f} %")
        time.sleep(step_interval)

    # -MAX_DUTYで保持
    hold_duty_percent(-max_duty_percent, hold_time, step_interval, max_duty_percent)

    # Duty負 -> 0
    for i in range(-duty_div, 1):
        duty = (i / duty_div) * max_duty_percent
        set_duty_percent(duty, max_duty_percent)
        print(f"duty = {duty:+.2f} %")
        time.sleep(step_interval)

    hold_duty_percent(0.0, zero_hold_time, step_interval, max_duty_percent)


def safe_stop() -> None:
    """念のため複数回0% dutyを送る。"""
    global vesc

    if vesc is None:
        return

    for _ in range(5):
        try:
            vesc.stop_motor()
            print("duty = +0.00 % stop")
            time.sleep(0.05)
        except Exception:
            pass


def main() -> None:
    global vesc

    parser = argparse.ArgumentParser(description="VESC duty sweep via USB CDC serial")
    parser.add_argument("--port", default="/dev/ttyACM0", help="例: /dev/ttyACM0, /dev/serial/by-id/..., auto")
    parser.add_argument("--baud", type=int, default=115200, help="USB CDCでは実質重要でないがpyserial用に指定")
    parser.add_argument("--timeout", type=float, default=0.3)
    parser.add_argument("--list-ports", action="store_true")

    parser.add_argument("--max-duty", type=float, default=5.0, help="最大duty [%]。最初は5〜10程度で確認")
    parser.add_argument("--duty-div", type=int, default=10, help="duty分割数")
    parser.add_argument("--step-interval", type=float, default=0.05, help="dutyを1段階変える間隔 [s]")
    parser.add_argument("--hold-time", type=float, default=2.0, help="最大/最小dutyで保持する時間 [s]")
    parser.add_argument("--zero-hold-time", type=float, default=0.0, help="0%に戻ったときの保持時間 [s]")
    parser.add_argument("--once", action="store_true", help="1周期だけ実行して終了")

    args = parser.parse_args()

    if args.list_ports:
        ports = list_serial_ports()
        if not ports:
            print("No serial ports found")
        else:
            for line in ports:
                print(line)
        return

    if args.max_duty < 0.0 or args.max_duty > 100.0:
        raise ValueError("--max-duty must be between 0.0 and 100.0 [%]")

    if args.duty_div <= 0:
        raise ValueError("--duty-div must be positive")

    print("VESC duty sweep step mode over USB")
    print(f"PORT          = {args.port}")
    print(f"BAUDRATE      = {args.baud}")
    print(f"MAX_DUTY      = {args.max_duty} %")
    print(f"DUTY_DIV      = {args.duty_div}")
    print(f"STEP_INTERVAL = {args.step_interval} s")
    print(f"HOLD_TIME     = {args.hold_time} s")
    print("Ctrl+C to stop")

    with VescUsb(port=args.port, baudrate=args.baud, timeout=args.timeout) as v:
        vesc = v
        print(f"Resolved port = {vesc.port}")

        try:
            safe_stop()
            time.sleep(0.5)

            while True:
                sweep_once(
                    max_duty_percent=args.max_duty,
                    duty_div=args.duty_div,
                    step_interval=args.step_interval,
                    hold_time=args.hold_time,
                    zero_hold_time=args.zero_hold_time,
                )

                if args.once:
                    break

        except KeyboardInterrupt:
            print("\nKeyboardInterrupt detected")

        finally:
            safe_stop()
            print("Motor stopped")


if __name__ == "__main__":
    main()
