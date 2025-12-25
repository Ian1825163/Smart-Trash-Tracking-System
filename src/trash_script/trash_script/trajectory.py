#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import String
import math


class TrajectoryNode(Node):
    def __init__(self):
        super().__init__('trajectory_node')

        self.sub = self.create_subscription(Point, '/target_coord', self.cb, 10)
        self.pub = self.create_publisher(String, '/burst_cmd', 10)

        # --- tunable ---
        self.x_set = 0.0
        self.y_set = 0.25          # 你鏡頭位置特殊，這個一定要現場調
        self.dead_x = 0.15
        self.dead_y = 0.12
        self.follow_dt = 0.20      # 每 0.2s 最多下1次指令（避免 spam）
        self.follow_burst = 0.12

        # throw detection (用速度)
        self.throw_speed = 1.2
        self.alpha = 0.35          # 速度低通
        self.armed_window = 0.6
        self.miss_trigger = 0.15   # armed後 X 秒看不到就觸發

        # intercept behavior
        self.intercept_duration = 0.80
        self.vx_lr_th = 0.4        # vx 大於這個才算要左右

        # --- state ---
        self.last_pos = None
        self.last_t = None
        self.vx_f = 0.0
        self.vy_f = 0.0

        self.last_seen_t = 0.0
        self.last_cmd_t = 0.0

        self.armed_until = 0.0
        self.intercept_sent = False

        self.throw_vx = 0.0
        self.throw_vy = 0.0

        #asjust follow sensitivity
        self.exceed_x_cnt = 0
        self.exceed_y_cnt = 0
        self.need_cnt = 5   # 3 consecutive frames (can be tuned wrt FPS)

        self.timer = self.create_timer(0.05, self.watchdog)  # 20Hz watchdog
        self.get_logger().info("Trajectory node started.")

    def cb(self, msg: Point):
        t = self.get_clock().now().nanoseconds / 1e9
        self.last_seen_t = t

        x = float(msg.x)
        y = float(msg.y)

        # --- velocity ---
        vx = 0.0
        vy = 0.0
        if self.last_pos is not None and self.last_t is not None:
            dt = t - self.last_t
            if dt > 1e-3:
                vx = (x - self.last_pos[0]) / dt
                vy = (y - self.last_pos[1]) / dt

        self.vx_f = (1 - self.alpha) * self.vx_f + self.alpha * vx
        self.vy_f = (1 - self.alpha) * self.vy_f + self.alpha * vy

        self.last_pos = (x, y)
        self.last_t = t

        # --- FOLLOW ---
        if (t - self.last_cmd_t) > self.follow_dt and (not self.is_armed(t)):
            err_x = x - self.x_set
            err_y = y - self.y_set
        
        dur = self.burst_from_err(err_x)


        # follow in left-right direction
        if err_x < -self.dead_x:
            self.exceed_x_cnt += 1
            if self.exceed_x_cnt >= self.need_cnt:
                self.send("L", dur)
                self.last_cmd_t = t
                self.exceed_x_cnt = 0
            return
        
        elif err_x > self.dead_x:
            self.exceed_x_cnt += 1
            if self.exceed_x_cnt >= self.need_cnt:
                self.send("R", dur)
                self.last_cmd_t = t
                self.exceed_x_cnt = 0
            return
        else:
            self.exceed_x_cnt = 0

        # follow in forward-backward direction
        if err_y < -self.dead_y:
            self.exerr_xceed_y_cnt += 1
            if self.exceed_y_cnt >= self.need_cnt:
                self.send("L", dur)
                self.last_cmd_t = t
                self.exceed_y_cnt = 0
            return
        
        elif err_y > self.dead_y:
            self.exceed_y_cnt += 1
            if self.exceed_y_cnt >= self.need_cnt:
                self.send("R", dur)
                self.last_cmd_t = t
                self.exceed_y_cnt = 0
            return
        else:
            self.exceed_y_cnt = 0

        # --- ARMED (丟出偵測) ---
        speed = math.hypot(self.vx_f, self.vy_f)
        if speed > self.throw_speed:
            self.armed_until = t + self.armed_window
            self.intercept_sent = False
            self.throw_vx = self.vx_f
            self.throw_vy = self.vy_f
            self.get_logger().warn(f"ARMED speed={speed:.2f} vx={self.throw_vx:.2f} vy={self.throw_vy:.2f}")

    def is_armed(self, t: float) -> bool:
        return t < self.armed_until

    def burst_from_err(self, e, k=0.10, tmin=0.06, tmax=0.18):
        return max(tmin, min(tmax, k*abs(e)))


    def watchdog(self):
        t = self.get_clock().now().nanoseconds / 1e9

        # armed後，短時間「突然不見」 => 視為丟出離開視野，觸發一次衝刺
        if self.is_armed(t) and (not self.intercept_sent) and ((t - self.last_seen_t) > self.miss_trigger):
            self.intercept_sent = True

            # 用 throw_vx/vy 決定方向（最穩的 demo：可斜向）
            fb = "B" if self.throw_vy > 0 else "F"

            lr = ""
            if self.throw_vx > self.vx_lr_th:
                lr = "R"
            elif self.throw_vx < -self.vx_lr_th:
                lr = "L"

            direction = fb + lr if lr else fb   # "BR" / "BL" / "B" / "F"
            self.send(direction, self.intercept_duration)
            self.get_logger().warn(f"INTERCEPT {direction},{self.intercept_duration:.2f}")

    def send(self, direction: str, duration: float):
        cmd = String()
        cmd.data = f"{direction},{duration:.2f}"
        self.pub.publish(cmd)


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
