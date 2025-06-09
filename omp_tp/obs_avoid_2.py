#!/usr/bin/env python3

# 1차 refactoring code(매우 기본적인 FSM 구조)

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from enum import Enum
import cv2


class State(Enum):
    NAVIGATING    = 0
    AVOID_LEFT1   = 1
    AVOID_FWD1    = 2
    AVOID_RIGHT1  = 3
    AVOID_FWD2    = 4
    AVOID_RIGHT2  = 5
    AVOID_FWD3    = 6
    AVOID_LEFT2   = 7


class ObstacleAvoidNode(Node):
    # 상수 정의
    GRAY_THRESHOLD = 50      # 그레이스케일 임계치
    PIXEL_THRESHOLD = 10000   # 픽셀 개수 임계치

    FWD_SPEED = 0.8
    TURN_SPEED = 1.5

    DUR_LEFT1   = 3.0  # 좌회전 시간
    DUR_FWD1    = 7.0  # 첫 전진 시간
    DUR_RIGHT1  = 3.0  # 첫 우회전 시간
    DUR_FWD2    = 4.0  # 두 번째 전진 시간
    DUR_RIGHT2  = 3.0  # 두 번째 우회전 시간
    DUR_FWD3    = 7.0  # 세 번째 전진 시간
    DUR_LEFT2   = 3.0  # 최종 좌회전 시간

    def __init__(self):
        super().__init__('csj_obstacle_avoid')

        self.create_subscription(Image, '/image', self._on_image, 10)
        self.pub_th1 = self.create_publisher(Float64, '/my_boat/thrust1', 10)
        self.pub_th2 = self.create_publisher(Float64, '/my_boat/thrust2', 10)
        self.pub_gray_img = self.create_publisher(Image, '/gray_image', 10)

        # 내부 변수 초기화
        self.bridge = CvBridge()
        self.state = State.NAVIGATING
        self.state_start = self.get_clock().now()
        self.obstacle_detected = False

        self.create_timer(0.1, self._control_loop)

    # 이미지 처리 및 탐지
    # 흑백 변환, 50보다 높은 픽셀 수 카운트, 10000개보다 많으면 장애물이라고 인식
    def _on_image(self, msg: Image):
        cv_img = self.bridge.imgmsg_to_cv2(msg)
        gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
        _, bw = cv2.threshold(gray, self.GRAY_THRESHOLD, 255, cv2.THRESH_BINARY_INV)
        count = cv2.countNonZero(bw)

        self.obstacle_detected = (count > self.PIXEL_THRESHOLD)
        # self.get_logger().debug(f"dark_pixels={count}, detected={self.obstacle_detected}")
        
        # 흑백 이미지 퍼블리시
        gray_msg = self.bridge.cv2_to_imgmsg(gray, encoding='mono8')
        gray_msg.header = msg.header
        self.pub_gray_img.publish(gray_msg)

    # Control
    def _control_loop(self):
        
        now = self.get_clock().now()
        elapsed = (now - self.state_start).nanoseconds / 1e9

        if self.state == State.NAVIGATING:
            if self.obstacle_detected:
                self._transition(State.AVOID_LEFT1, "장애물 감지 → 좌회전 시작")
            else:
                self._drive_forward()

        elif self.state == State.AVOID_LEFT1:
            self._turn_left()
            if elapsed >= self.DUR_LEFT1:
                self._transition(State.AVOID_FWD1, "좌회전 완료 → 전진")

        elif self.state == State.AVOID_FWD1:
            self._drive_forward()
            if elapsed >= self.DUR_FWD1:
                self._transition(State.AVOID_RIGHT1, "전진 완료 → 우회전")

        elif self.state == State.AVOID_RIGHT1:
            self._turn_right()
            if elapsed >= self.DUR_RIGHT1:
                self._transition(State.AVOID_FWD2, "우회전 완료 → 전진")

        elif self.state == State.AVOID_FWD2:
            self._drive_forward()
            if elapsed >= self.DUR_FWD2:
                self._transition(State.AVOID_RIGHT2, "전진 완료 → 우회전")

        elif self.state == State.AVOID_RIGHT2:
            self._turn_right()
            if elapsed >= self.DUR_RIGHT2:
                self._transition(State.AVOID_FWD3, "우회전 완료 → 전진")

        elif self.state == State.AVOID_FWD3:
            self._drive_forward()
            if elapsed >= self.DUR_FWD3:
                self._transition(State.AVOID_LEFT2, "전진 완료 → 좌회전")

        elif self.state == State.AVOID_LEFT2:
            self._turn_left()
            if elapsed >= self.DUR_LEFT2:
                self._transition(State.NAVIGATING, "회피 완료 → 직진 재개")

    # 상태 퍼블리시
    # 직진
    def _drive_forward(self):
        self._publish(self.FWD_SPEED, self.FWD_SPEED)
    # 좌회전
    def _turn_left(self):
        self._publish(-self.TURN_SPEED, self.TURN_SPEED)
    # 우회전
    def _turn_right(self):
        self._publish(self.TURN_SPEED, -self.TURN_SPEED)
    # 속도 명령 publish
    def _publish(self, v1, v2):
        self.pub_th1.publish(Float64(data=v1))
        self.pub_th2.publish(Float64(data=v2))
    # 상태 전환 메서드
    def _transition(self, next_state: State, log_msg: str):
        self.get_logger().info(log_msg)
        self.state = next_state
        self.state_start = self.get_clock().now()


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
