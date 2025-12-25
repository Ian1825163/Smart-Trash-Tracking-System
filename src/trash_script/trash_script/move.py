#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import lgpio
import math

# ---------------- Parameters ----------------
MOTORS = {
    "M1": {"IN1": 12, "IN2": 16},  # Right Front (RF)
    "M2": {"IN1": 20, "IN2": 21},  # Right Rear (RR)
    "M3": {"IN1": 4,  "IN2": 9},   # Left Front (LF)
    "M4": {"IN1": 11, "IN2": 5},   # Left Rear (LR)
}

MOTOR_SIGN = {"M1": +1, "M2": +1, "M3": -1, "M4": -1}
GAIN_RIGHT = {"M1": 0.996, "M2": 1.025, "M3": 1.000, "M4": 1.108} #gain for right movement
GAIN_LEFT  = {"M1": 0.95, "M2": 1.100, "M3":0.60 , "M4": 1.40} #gain for left movement

PWM_FREQ = 2000
MAX_DUTY = 100
BRAKE_DUTY = 25
BRAKE_TIME = 0.08
TIME_SCALE = 0.83

def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


class MotorNoPWMpin:
    def __init__(self, chip, in1, in2):
        self.chip = chip
        self.in1 = in1
        self.in2 = in2
        lgpio.gpio_claim_output(chip, in1, 0)
        lgpio.gpio_claim_output(chip, in2, 0)

        self.active_pin = None   # in1 or in2
        self.last_duty = 0.0
        self.stop()

    def set(self, signed_cmd, duty_limit=MAX_DUTY):
        signed_cmd = clamp(signed_cmd, -1.0, 1.0)
        duty = min(abs(signed_cmd) * 100.0, float(duty_limit))

        if duty <= 0.0:
            self.stop()
            return

        # choose direction pin
        want_pin = self.in1 if signed_cmd > 0 else self.in2
        other_pin = self.in2 if want_pin == self.in1 else self.in1

        # if direction changed: stop both first
        if self.active_pin != want_pin:
            lgpio.tx_pwm(self.chip, self.in1, PWM_FREQ, 0)
            lgpio.tx_pwm(self.chip, self.in2, PWM_FREQ, 0)
            lgpio.gpio_write(self.chip, self.in1, 0)
            lgpio.gpio_write(self.chip, self.in2, 0)
            self.active_pin = want_pin
            self.last_duty = -1.0  # force update

        # ensure other pin is low
        lgpio.tx_pwm(self.chip, other_pin, PWM_FREQ, 0)
        lgpio.gpio_write(self.chip, other_pin, 0)

        # update pwm only if duty changed meaningfully
        if abs(duty - self.last_duty) > 0.5:
            lgpio.tx_pwm(self.chip, want_pin, PWM_FREQ, duty)
            self.last_duty = duty

    def stop(self):
        lgpio.tx_pwm(self.chip, self.in1, PWM_FREQ, 0)
        lgpio.tx_pwm(self.chip, self.in2, PWM_FREQ, 0)
        lgpio.gpio_write(self.chip, self.in1, 0)
        lgpio.gpio_write(self.chip, self.in2, 0)
        self.active_pin = None
        self.last_duty = 0.0


class MoveBurstNode(Node):
    def __init__(self):
        super().__init__('move_burst_node')

        self.sub = self.create_subscription(String, '/burst_cmd', self.cb, 10)

        self.chip = lgpio.gpiochip_open(0)
        self.motors = {name: MotorNoPWMpin(self.chip, cfg["IN1"], cfg["IN2"]) for name, cfg in MOTORS.items()}

        self.state = "IDLE"   # IDLE, MOVING, BRAKING
        self.end_time = 0.0
        self.current_cmds = {}
        self.current_gains = {name: 1.0 for name in MOTORS}   # ← 新增
        self.timer = self.create_timer(0.02, self.loop)  # 50Hz
        self.get_logger().info("MoveBurstNode ready (50Hz).")

    def mecanum_mix(self, vx, vy, wz=0.0):
        fl = vx - vy - wz
        fr = vx + vy + wz
        rl = vx + vy - wz
        rr = vx - vy + wz
        raw = {"M3": fl, "M1": fr, "M4": rl, "M2": rr}
        m = max(1.0, max(abs(v) for v in raw.values()))
        return {k: v / m for k, v in raw.items()}

    def cb(self, msg: String):
        try:
            direction, dur_s = msg.data.split(',')
            direction = direction.strip().upper()
            duration = float(dur_s)

            if direction == 'STOP':
                self.stop_motors()
                self.state = "IDLE"
                return

            vx, vy = 0.0, 0.0
            if direction in ['F', 'FORWARD']:
                vx, vy = 1.0, 0.0
                gains = GAIN_RIGHT
            elif direction in ['B', 'BACK']:
                vx, vy = -1.0, 0.0
                gains = GAIN_RIGHT
            elif direction in ['L', 'LEFT']:
                vx, vy = 0.0, -1.0
                gains = GAIN_LEFT
            elif direction in ['R', 'RIGHT']:
                vx, vy = 0.0, +1.0
                gains = GAIN_RIGHT
            elif direction == 'FR':
                vx, vy = 1.0, +1.0
                gains = GAIN_RIGHT
            elif direction == 'FL':
                vx, vy = 1.0, -1.0
            elif direction == 'BR':
                vx, vy = -1.0, +1.0
                gains = GAIN_RIGHT
            elif direction == 'BL':
                vx, vy = -1.0, 1.0
            else:
                self.get_logger().warn(f"Unknown direction: {direction}")
                return

            self.current_cmds = self.mecanum_mix(vx, vy)
            self.current_gains = gains   # ← 新增：把目前方向要用的 gain 存起來


            now = self.get_clock().now().nanoseconds / 1e9
            self.state = "MOVING"
            self.end_time = now + duration * TIME_SCALE

            self.get_logger().info(f"Burst: {direction},{duration:.2f}s")

        except Exception as e:
            self.get_logger().error(f"Parse error: {e}")

    def loop(self):
        now = self.get_clock().now().nanoseconds / 1e9

        if self.state == "IDLE":
            return

        if self.state == "MOVING":
            if now < self.end_time:
                for name, cmd in self.current_cmds.items():
                    out = cmd * self.current_gains.get(name, 1.0) * MOTOR_SIGN[name]  # ← 改這行
                    self.motors[name].set(out, duty_limit=MAX_DUTY)
            else:
                self.state = "BRAKING"
                self.end_time = now + BRAKE_TIME

        elif self.state == "BRAKING":
            if now < self.end_time:
                for name, base_cmd in self.current_cmds.items():
                    if abs(base_cmd) > 1e-6:
                        out = -math.copysign(BRAKE_DUTY / 100.0, base_cmd)
                        out *= MOTOR_SIGN[name] 
                        self.motors[name].set(out, duty_limit=100)
            else:
                self.stop_motors()
                self.state = "IDLE"

    def stop_motors(self):
        for m in self.motors.values():
            m.stop()

    def destroy_node(self):
        self.stop_motors()
        lgpio.gpiochip_close(self.chip)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MoveBurstNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
