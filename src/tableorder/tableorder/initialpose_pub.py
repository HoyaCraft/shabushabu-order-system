import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.timer = self.create_timer(1.0, self.publish_initial_pose)  # 1초마다 실행
        self.initial_pose_confirmed = False  # 초기 위치 설정 여부 확인용 플래그

        # AMCL 노드로부터 피드백을 받기 위해 /amcl_pose 구독
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_pose_callback,
            10
        )

    def publish_initial_pose(self):
        if self.initial_pose_confirmed:
            self.get_logger().info('초기 위치 설정이 확인되었습니다. 퍼블리싱 중지!')
            return

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = 0.0 #-1.651675525387458  # 초기 X 위치
        msg.pose.pose.position.y = 0.0 #-0.5314712223684817  # 초기 Y 위치
        msg.pose.pose.orientation.z = 0.0  # 방향 (yaw)
        msg.pose.pose.orientation.w = 1.0  # 방향 (yaw)

        self.publisher.publish(msg)
        self.get_logger().info('초기 위치 메시지가 퍼블리시되었습니다.')

    def amcl_pose_callback(self, msg):
        # AMCL이 초기 위치를 설정했는지 확인
        if not self.initial_pose_confirmed:
            self.initial_pose_confirmed = True
            self.get_logger().info('AMCL로부터 초기 위치 설정이 확인되었습니다!')

def main():
    rclpy.init()
    node = InitialPosePublisher()
    try:
        rclpy.spin(node)  # 노드를 지속적으로 실행
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
