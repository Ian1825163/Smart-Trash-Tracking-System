#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
import math


class TrajectoryNode(Node):
    def __init__(self):
        super().__init__('trajectory_node')

        self.sub = self.create_subscription(PointStamped, '/target_coord', self.cb, 10)
        self.pub = self.create_publisher(String, '/burst_cmd', 10)

        # ===== Follow (追蹤用，可錄影片) =====
        self.x_set = 0.0
        self.y_set = 0.25

        self.dead_x = 0.18
        self.dead_y = 0.15

        self.need_cnt_x = 3
        self.need_cnt_y = 3
        self.follow_dt = 0.25          # 越小越會抖；越大越lag
        self.tmin = 0.06
        self.tmax = 0.18
        self.k_burst = 0.12

        # ===== Throw (丟出即觸發，搶時間) =====
        self.throw_speed = 1.2         # 速度門檻（你要現場調）
        self.alpha = 0.35              # 速度低通
        self.throw_cooldown = 0.7      # 避免連續誤觸發（秒）
        self.last_trigger_t = -1e9

        # 備援：如果太快看不到，仍然可以用 “不見了” 觸發
        self.armed_window = 0.5
        self.miss_trigger = 0.05       # 0.15 真的太慢，改小（1~2 frame）
        self.vx_lr_th = 0.35
        self.intercept_duration = 0.70

        # 丟出後暫停 follow，避免抖動/亂追
        self.lock_until = 0.0

        # ===== state =====
        self.last_pos = None
        self.last_t = None
        self.vx_f = 0.0
        self.vy_f = 0.0

        self.last_seen_t = 0.0
        self.last_cmd_t = 0.0
        self.exceed_x_cnt = 0
        self.exceed_y_cnt = 0

        self.armed_until = 0.0
        self.intercept_sent = False
        self.throw_vx = 0.0
        self.throw_vy = 0.0

        # latency print
        self.last_latency_print_t = 0.0
        self.lat_print_dt = 0.5

        # watchdog 頻率加快（縮短 miss_trigger 的效果）
        self.timer = self.create_timer(0.01, self.watchdog)  # 100Hz
        self.get_logger().info("Trajectory node started (EARLY throw + follow lock + latency).")

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def burst_from_err(self, e: float) -> float:
        return max(self.tmin, min(self.tmax, self.k_burst * abs(e)))

    def send(self, direction: str, duration: float):
        # 夾一個 timestamp 給 move 算 traj->move latency
        t_send = self.now_s()
        cmd = String()
        cmd.data = f"{direction},{duration:.2f},{t_send:.6f}"
        self.pub.publish(cmd)

    def decide_direction(self, vx: float, vy: float) -> str:
        # 你原本是 vy>0 => B, 否則 F（依你座標定義）
        fb = "B" if vy > 0 else "F"

        lr = ""
        if vx > self.vx_lr_th:
            lr = "R"
        elif vx < -self.vx_lr_th:
            lr = "L"

        return fb + lr if lr else fb

    def is_armed(self, t: float) -> bool:
        return t < self.armed_until

    def cb(self, msg: PointStamped):
        now = self.now_s()
        self.last_seen_t = now

        x = float(msg.point.x)
        y = float(msg.point.y)

        # ---- latency vision->traj ----
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        vision_to_traj = now - stamp
        if now - self.last_latency_print_t > self.lat_print_dt:
            self.get_logger().info(f"latency vision->traj: {vision_to_traj*1000:.1f} ms")
            self.last_latency_print_t = now

        # ---- velocity ----
        vx, vy = 0.0, 0.0
        if self.last_pos is not None and self.last_t is not None:
            dt = now - self.last_t
            if dt > 1e-3:
                vx = (x - self.last_pos[0]) / dt
                vy = (y - self.last_pos[1]) / dt

        self.vx_f = (1 - self.alpha) * self.vx_f + self.alpha * vx
        self.vy_f = (1 - self.alpha) * self.vy_f + self.alpha * vy

        self.last_pos = (x, y)
        self.last_t = now

        # ===== EARLY THROW TRIGGER（看到超速就立刻衝）=====
        speed = math.hypot(self.vx_f, self.vy_f)

        # 1) cooldown 避免連續觸發
        if (now - self.last_trigger_t) > self.throw_cooldown:
            if speed > self.throw_speed:
                self.last_trigger_t = now

                direction = self.decide_direction(self.vx_f, self.vy_f)
                self.get_logger().warn(
                    f"THROW EARLY! speed={speed:.2f} vx={self.vx_f:.2f} vy={self.vy_f:.2f} -> {direction}"
                )

                # 立刻送 burst
                self.send(direction, self.intercept_duration)

                # 鎖住 follow 一段時間，避免抖
                self.lock_until = now + self.intercept_duration + 0.15

                # 同時把備援 armed 設起來（如果中途又消失，也不會再觸發第二次）
                self.armed_until = now + self.armed_window
                self.intercept_sent = True
                self.throw_vx = self.vx_f
                self.throw_vy = self.vy_f
                return

        # ===== FOLLOW（錄追蹤影片用；丟出後 lock_until 內不追）=====
        if now < self.lock_until:
            return

        if (now - self.last_cmd_t) > self.follow_dt:
            err_x = x - self.x_set
            err_y = y - self.y_set

            # 左右
            if err_x < -self.dead_x:
                self.exceed_x_cnt += 1
                if self.exceed_x_cnt >= self.need_cnt_x:
                    dur = self.burst_from_err(err_x)
                    self.send("L", dur)
                    self.last_cmd_t = now
                    self.exceed_x_cnt = 0
                return
            elif err_x > self.dead_x:
                self.exceed_x_cnt += 1
                if self.exceed_x_cnt >= self.need_cnt_x:
                    dur = self.burst_from_err(err_x)
                    self.send("R", dur)
                    self.last_cmd_t = now
                    self.exceed_x_cnt = 0
                return
            else:
                self.exceed_x_cnt = 0

            # 前後（左右在 deadzone 內才做）
            if err_y < -self.dead_y:
                self.exceed_y_cnt += 1
                if self.exceed_y_cnt >= self.need_cnt_y:
                    dur = self.burst_from_err(err_y)
                    self.send("F", dur)
                    self.last_cmd_t = now
                    self.exceed_y_cnt = 0
                return
            elif err_y > self.dead_y:
                self.exceed_y_cnt += 1
                if self.exceed_y_cnt >= self.need_cnt_y:
                    dur = self.burst_from_err(err_y)
                    self.send("B", dur)
                    self.last_cmd_t = now
                    self.exceed_y_cnt = 0
                return
            else:
                self.exceed_y_cnt = 0

        # ===== 備援 ARMED（給 watchdog 用）=====
        # 這裡只把 armed 設起來，讓 “突然消失” 也能觸發一次（但 early 已經會先觸發）
        if speed > self.throw_speed:
            self.armed_until = now + self.armed_window
            self.intercept_sent = False
            self.throw_vx = self.vx_f
            self.throw_vy = self.vy_f

    def watchdog(self):
        # 備援：armed 後突然消失 -> 觸發一次
        now = self.now_s()
        if self.is_armed(now) and (not self.intercept_sent) and ((now - self.last_seen_t) > self.miss_trigger):
            self.intercept_sent = True
            direction = self.decide_direction(self.throw_vx, self.throw_vy)
            self.send(direction, self.intercept_duration)
            self.lock_until = now + self.intercept_duration + 0.15
            self.get_logger().warn(f"THROW (MISSING) -> {direction},{self.intercept_duration:.2f}")


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
