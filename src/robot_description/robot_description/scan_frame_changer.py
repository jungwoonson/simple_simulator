import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class ScanFrameChanger(Node):
    """
    LiDAR 스캔 메시지의 frame_id를 변경하는 노드

    역할:
    1. /scan_raw 토픽 구독 (Gazebo에서 오는 원본)
    2. frame_id를 'lidar_link'로 변경
    3. /scan 토픽으로 재발행 (SLAM으로 전달)
    """
    def __init__(self):
        super().__init__('scan_frame_changer')

        # /scan_raw 토픽 구독 (Gazebo 브릿지에서 받음)
        self.subscription = self.create_subscription(
            LaserScan,  # 메시지 타입
            '/scan_raw',  # 구독할 토픽
            self.scan_callback,  # 콜백 함수
            10)  # QoS (메시지 큐 크기)

        # /scan 토픽 발행 (SLAM으로 보냄)
        self.publisher = self.create_publisher(
            LaserScan,  # 메시지 타입
            '/scan',  # 발행할 토픽
            10)  # QoS

    def scan_callback(self, msg):
        """
        메시지를 받을 때마다 실행되는 콜백 함수

        Parameters:
            msg: LaserScan 메시지 (Gazebo에서 온 원본)
        """
        # frame_id를 변경 (예: 'simple_robot/base_link/lidar_link' → 'lidar_link')
        msg.header.frame_id = 'lidar_link'

        # 변경된 메시지를 /scan 토픽으로 발행
        self.publisher.publish(msg)

def main(args=None):
    """노드 실행 메인 함수"""
    rclpy.init(args=args)  # ROS2 초기화
    node = ScanFrameChanger()  # 노드 생성
    rclpy.spin(node)  # 노드 실행 (종료될 때까지 계속 실행)
    node.destroy_node()  # 노드 정리
    rclpy.shutdown()  # ROS2 종료

if __name__ == '__main__':
    main()