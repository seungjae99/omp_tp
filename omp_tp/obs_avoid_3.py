#!/usr/bin/env python3

# 2차 refactoring code(조금 더 정리된 FSM 구조)

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from enum import Enum
import cv2


class State(Enum):
    NAVIGATING = 0
    AVOIDING = 1


class ObstacleAvoidNode(Node):
    # 상수 정의
    GRAY_THRESHOLD = 50
    PIXEL_THRESHOLD = 10_000

    FWD_SPEED = 0.8
    TURN_SPEED = 1.5

    def __init__(self):
        super().__init__('csj_obstacle_avoid')

        self.create_subscription(Image, '/image', self._on_image, 10)
        self.pub_th1 = self.create_publisher(Float64, '/my_boat/thrust1', 10)
        self.pub_th2 = self.create_publisher(Float64, '/my_boat/thrust2', 10)
        self.pub_gray = self.create_publisher(Image, '/gray_image', 10)

        # 내부 상태 변수 선언 및 초기화
        self.bridge = CvBridge()
        self.state  = State.NAVIGATING
        self.obstacle = False
        self.state_start = self.get_clock().now()

        # 회피 시퀀스: (action, duration)
        self.avoid_sequence = [
            ("left", 3.0),
            ("forward", 7.0),
            ("right", 3.0),
            ("forward", 4.0),
            ("right", 3.0),
            ("forward", 7.0),
            ("left", 3.0),
        ]
        
        # step 초기화
        self.avoid_step = 0
        self.last_avoid_step = -1  # 로그 중복 방지용

        self.create_timer(0.1, self._control_loop)

    def _on_image(self, msg: Image):
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
        _, bw = cv2.threshold(gray, self.GRAY_THRESHOLD, 255, cv2.THRESH_BINARY_INV)
        count = cv2.countNonZero(bw)
        self.obstacle = (count > self.PIXEL_THRESHOLD)

        # 그레이 이미지 퍼블리시
        gray_img = self.bridge.cv2_to_imgmsg(gray, encoding='mono8')
        gray_img.header = msg.header
        self.pub_gray.publish(gray_img)

    # 시작
    def _control_loop(self):
        now     = self.get_clock().now()
        elapsed = (now - self.state_start).nanoseconds / 1e9

        # navigation 상태
        if self.state == State.NAVIGATING:
            if self.obstacle:
                self.get_logger().info("Obstacle detected → start avoiding")
                self.state = State.AVOIDING
                self.avoid_step = 0
                self.last_avoid_step = -1
                self.state_start = now
            else:
                self._drive(self.FWD_SPEED, self.FWD_SPEED)

        else:  # AVOIDING
            action, duration = self.avoid_sequence[self.avoid_step]

            # 새 스텝 진입 시 로그
            if self.avoid_step != self.last_avoid_step:
                step_no = self.avoid_step + 1                             # 다음 step 으로 넘어감
                self.get_logger().info(f"Current step: {step_no}, Current action: {action}")
                self.last_avoid_step = self.avoid_step

            # action 수행(3가지: 직진, 좌회전, 우회전)
            if action == "forward":
                self._drive(self.FWD_SPEED, self.FWD_SPEED)
            elif action == "left":
                self._drive(-self.TURN_SPEED, self.TURN_SPEED)
            elif action == "right":
                self._drive(self.TURN_SPEED, -self.TURN_SPEED)

            # 스텝 완료 시 다음으로 넘어감
            if elapsed >= duration:
                self.avoid_step += 1
                self.state_start = now           # elapsed 초기화
                # 모든 스텝 끝나면 NAVIGATING 복귀
                if self.avoid_step >= len(self.avoid_sequence):
                    self.get_logger().info("Avoidance complete → resuming navigation")
                    self.state = State.NAVIGATING

    def _drive(self, t1: float, t2: float):
        self.pub_th1.publish(Float64(data=t1))
        self.pub_th2.publish(Float64(data=t2))


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()