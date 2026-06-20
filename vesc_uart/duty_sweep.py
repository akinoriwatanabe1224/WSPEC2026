#!/usr/bin/env python3
import time
from vesc_uart import VescUart


# =========================
# ここを変更する
# =========================

PORT = "/dev/serial0"
BAUDRATE = 115200

# 実際にVESCへ送る最大duty
# 最初は 0.05 や 0.1 で試すこと
MAX_DUTY = 10

# dutyの分割数
# 10なら 0.00, 0.05, 0.10, ... 0.50
DUTY_DIV = 10

# dutyを1段階変える間隔 [s]
STEP_INTERVAL = 0.05

# 最大duty・最小dutyで保持する時間 [s]
HOLD_TIME = 2.0

# 0に戻ったときに保持する時間 [s]
ZERO_HOLD_TIME = 0.0

# Trueなら無限ループ
LOOP_FOREVER = True


# =========================
# 以下は基本変更不要
# =========================

vesc = None


def clamp_duty(duty: float) -> float:
    if duty > MAX_DUTY:
        return MAX_DUTY
    if duty < -MAX_DUTY:
        return -MAX_DUTY
    return duty


def set_duty(duty: float) -> None:
    """
    VESCへdutyを送る。
    """
    global vesc

    duty = clamp_duty(duty)
    vesc.set_duty(duty)


def hold_duty(duty: float, hold_time: float) -> None:
    """
    指定したdutyを一定時間保持する。
    保持中も周期的に送信する。
    """
    if hold_time <= 0:
        return

    hold_count = int(hold_time / STEP_INTERVAL)

    for _ in range(hold_count):
        set_duty(duty)
        print(f"duty = {duty:+.3f} hold")
        time.sleep(STEP_INTERVAL)


def sweep_once() -> None:
    """
    1周期分のduty掃引。
    0 -> +MAX
    +MAX保持
    +MAX -> -MAX
    -MAX保持
    -MAX -> 0
    """

    # Duty上昇 0 -> +MAX_DUTY
    for i in range(0, DUTY_DIV + 1):
        duty = (i / DUTY_DIV) * MAX_DUTY
        set_duty(duty)
        print(f"duty = {duty:+.3f}")
        time.sleep(STEP_INTERVAL)

    # +MAX_DUTYで保持
    hold_duty(MAX_DUTY, HOLD_TIME)

    # Duty下降 +MAX_DUTY -> -MAX_DUTY
    for i in range(DUTY_DIV, -DUTY_DIV - 1, -1):
        duty = (i / DUTY_DIV) * MAX_DUTY
        set_duty(duty)
        print(f"duty = {duty:+.3f}")
        time.sleep(STEP_INTERVAL)

    # -MAX_DUTYで保持
    hold_duty(-MAX_DUTY, HOLD_TIME)

    # Duty負 -> 0
    for i in range(-DUTY_DIV, 1):
        duty = (i / DUTY_DIV) * MAX_DUTY
        set_duty(duty)
        print(f"duty = {duty:+.3f}")
        time.sleep(STEP_INTERVAL)

    # 0で保持したい場合
    hold_duty(0.0, ZERO_HOLD_TIME)


def safe_stop() -> None:
    """
    念のため複数回0 dutyを送る。
    """
    global vesc

    if vesc is None:
        return

    for _ in range(5):
        try:
            vesc.stop_motor()
            print("duty = +0.000 stop")
            time.sleep(0.05)
        except Exception:
            pass


def main() -> None:
    global vesc

    if MAX_DUTY < 0.0 or MAX_DUTY > 100.0:
        raise ValueError("MAX_DUTY must be between 0.0 and 1.0")

    if DUTY_DIV <= 0:
        raise ValueError("DUTY_DIV must be positive")

    print("VESC duty sweep step mode")
    print(f"PORT          = {PORT}")
    print(f"BAUDRATE      = {BAUDRATE}")
    print(f"MAX_DUTY      = {MAX_DUTY}")
    print(f"DUTY_DIV      = {DUTY_DIV}")
    print(f"STEP_INTERVAL = {STEP_INTERVAL} s")
    print(f"HOLD_TIME     = {HOLD_TIME} s")
    print("Ctrl+C to stop")

    with VescUart(port=PORT, baudrate=BAUDRATE, timeout=0.3) as v:
        vesc = v

        try:
            # 最初は必ず停止
            safe_stop()
            time.sleep(0.5)

            while True:
                sweep_once()

                if not LOOP_FOREVER:
                    break

        except KeyboardInterrupt:
            print("\nKeyboardInterrupt detected")

        finally:
            safe_stop()
            print("Motor stopped")


if __name__ == "__main__":
    main()
