#!/usr/bin/env python3

# my original code

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from enum import Enum
import cv2


# 간단한 state machine 사용
class State(Enum):
    NAVIGATING    = 0   # 평소 직진
    AVOID_LEFT1   = 1   # 3초 좌회전
    AVOID_FWD1    = 2
    AVOID_RIGHT1  = 3   # 3초 우회전
    AVOID_FWD2    = 4   # 2초 전진
    AVOID_RIGHT2  = 5   # 3초 우회전
    AVOID_FWD3    = 6
    AVOID_LEFT2   = 7   # 3초 좌회전

class ObstacleAvoidNode(Node):
    def __init__(self):
        super().__init__('csj_obstacle_avoid')

        self.image_sub = self.create_subscription(Image, '/image', self.image_callback, 10)
        self.th1_pub = self.create_publisher(Float64, '/my_boat/thrust1', 10)
        self.th2_pub = self.create_publisher(Float64, '/my_boat/thrust2', 10)

        # 초기화
        self.state = State.NAVIGATING
        self.detected = False
        self.state_start = self.get_clock().now()


        self.THRESH_VALUE = 50         # 어둡다고 판단할 그레이스케일 임계치
        self.COUNT_LIMIT = 8000        # 이 개수 이상이면 장애물로 간주
        self.FWD_SPEED = 0.8
        self.TURN_SPEED = 1.5
        self.DUR_LEFT = 3.0
        self.DUR_FWD = 7.0
        self.DUR_FWD1 = 3.0
        self.DUR_RIGHT = 3.0

        self.control_timer = self.create_timer(0.1, self.control_loop)

        self.bridge = CvBridge()

    def image_callback(self, msg: Image):
        # 이미지 불러오고 흑백으로 변환
        cv_img = self.bridge.imgmsg_to_cv2(msg)
        gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)

        # 이진화 → 어두운 픽셀 카운트
        _, bw = cv2.threshold(gray, self.THRESH_VALUE, 255, cv2.THRESH_BINARY_INV)
        count = cv2.countNonZero(bw)
        self.detected = (count > self.COUNT_LIMIT)
        self.get_logger().debug(f"dark_px={count}, detected={self.detected}")

        ## 전처리된 이미지 퍼블리시
        # img_msg = self.bridge.cv2_to_imgmsg(gray, encoding='mono8')
        # self.image_pub.publish(img_msg)


    def control_loop(self):
        now = self.get_clock().now()
        elapsed = (now - self.state_start).nanoseconds / 1e9

        if self.state == State.NAVIGATING:
            # 정상 항로 주행 → 장애물 감지 시 왼쪽으로 회피
            if self.detected:
                self.get_logger().info("Obstacle detected! start avoidance")
                self._set_state(State.AVOID_LEFT1)
            else:
                self.get_logger().info("[NAVIGATING] Navigating... No Obstacle detected... ")
                self._publish_thrust(self.FWD_SPEED, self.FWD_SPEED)
        
        elif self.state == State.AVOID_LEFT1:
            self.get_logger().info("[AVOID_LEFT1] Begin turning left for 3s")
            # 좌회전: thruster1 뒤로, thruster2 앞으로
            self._publish_thrust(-self.TURN_SPEED, self.TURN_SPEED)
            # 3초 지나면 우회전 회피 모드로 변경
            if elapsed >= self.DUR_LEFT:
                self._set_state(State.AVOID_FWD1)
    
        elif self.state == State.AVOID_FWD1:
            # 8초동안 직진 회피 수행
            self.get_logger().info("[AVOID_FWD1] GO Forward for 5s")
            self._publish_thrust(self.FWD_SPEED, self.FWD_SPEED)
            # 8초 지나면 우회전 회피 모드로 변경
            if elapsed >= self.DUR_FWD:
                self.get_logger().info("Forward done -> Turn right for 3sec")
                self._set_state(State.AVOID_RIGHT1)
                
        elif self.state == State.AVOID_RIGHT1:
            # 3초동안 우회전 회피 수행
            self.get_logger().info("[AVOID_RIGHT1] Turn Right for 3s")
            self._publish_thrust(self.TURN_SPEED, -self.TURN_SPEED)
            # 3초 지나면 직진 회피 수행
            if elapsed >= self.DUR_RIGHT:
                self.get_logger().info("Turn right done -> Go Forward for 2s")
                self._set_state(State.AVOID_FWD2)

        elif self.state == State.AVOID_FWD2:
            # 8초동안 직진 회피 수행
            self.get_logger().info("[AVOID_FWD2] GO Forward for 5s")
            self._publish_thrust(self.FWD_SPEED, self.FWD_SPEED)
            # 8초 지나면 우회전 회피 모드로 변경
            if elapsed >= self.DUR_FWD1:
                self.get_logger().info("Forward done -> Turn right for 3sec")
                self._set_state(State.AVOID_RIGHT2)
                
        elif self.state == State.AVOID_RIGHT2:
            # 3초동안 우회전 회피 수행
            self.get_logger().info("[AVOID_RIGHT2] Turn Right for 3s")
            self._publish_thrust(self.TURN_SPEED, -self.TURN_SPEED)
            # 3초 지나면 다시 좌회전 회피 수행
            if elapsed >= self.DUR_RIGHT:
                self.get_logger().info("Turn right done -> Turn left again for 3sec")
                self._set_state(State.AVOID_FWD3)

        elif self.state == State.AVOID_FWD3:
            # 8초동안 직진 회피 수행
            self.get_logger().info("[AVOID_FWD3] GO Forward for 5s")
            self._publish_thrust(self.FWD_SPEED, self.FWD_SPEED)
            # 8초 지나면 우회전 회피 모드로 변경
            if elapsed >= self.DUR_FWD:
                self.get_logger().info("Forward done -> Turn right for 3sec")
                self._set_state(State.AVOID_LEFT2)
        
        elif self.state == State.AVOID_LEFT2:
            self.get_logger().info("[AVOID_LEFT2] Turn Left for 3s")
            self._publish_thrust(-self.TURN_SPEED, self.TURN_SPEED)
            if elapsed >= self.DUR_LEFT:
                self.get_logger().info("Obstacle Avoidance DONE! Go FORWARD!")
                self._set_state(State.NAVIGATING)


    def _publish_thrust(self, v1: float, v2: float):
        msg1 = Float64(data=v1)
        msg2 = Float64(data=v2)
        self.th1_pub.publish(msg1)
        self.th2_pub.publish(msg2)

    def _set_state(self, new_state: State):
        self.state = new_state
        self.state_start = self.get_clock().now()


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
