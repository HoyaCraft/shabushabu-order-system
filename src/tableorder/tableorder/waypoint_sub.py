from std_msgs.msg import Int32, String
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
import math
import time
from collections import deque


class CommandSubscriber(Node):
    def __init__(self):
        super().__init__('command_subscriber')

        # Subscribe to button_topic
        self.subscription = self.create_subscription(
            Int32,
            'button_topic',
            self.command_callback,
            10
        )

        # Subscribe to home_command topic
        self.home_subscription = self.create_subscription(
            String,
            'home_command',  # HOME 명령을 받을 ROS2 토픽
            self.home_callback,
            10
        )

        # Publisher for robot status
        self.status_publisher = self.create_publisher(String, 'robot_status', 10)

        # Action client for navigation
        self.action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # Predefined positions with (x, y, w, z)
        self.positions = {
            1: (2.7237, 1.5732, 0.0, -1.0),
            2: (2.6830, 0.5184, 0.0, -1.0),
            3: (2.7919, -0.5693, 0.0, -1.0),
            4: (1.7157, 1.6043, 0.0, -1.0),
            5: (1.7098, 0.5177, 0.0, -1.0),
            6: (1.7098, -0.5797, 0.0, -1.0),
            7: (0.5586, 1.6310, 0.0, -1.0),
            8: (0.5933, 0.5395, 0.0, -1.0),
            9: (0.5712, -0.6038, 0.0, -1.0)
        }

        self.home_position = (0.0, 0.0, 0.0, 1.0)  # Home position
        self.command_queue = deque()  # Queue to store commands
        self.waiting = True
        self.current_goal = None  # 현재 목표 저장
        self.publish_status("waiting")  # Initial waiting log

    def publish_status(self, status):
        msg = String()
        msg.data = status
        self.status_publisher.publish(msg)

    def command_callback(self, msg):
        command = msg.data
        
        if command in self.positions:
            self.command_queue.append(command)  # 명령을 큐에 추가
            if self.waiting:
                self.execute_command(self.command_queue.popleft())
        else:
            self.get_logger().info(f"Received invalid command: {command}")

    def home_callback(self, msg):
        """HOME 명령을 ROS2 퍼블리시를 통해 받을 때 실행"""
        if msg.data.lower() == "home":
            if self.command_queue:
                self.publish_status("done")
                time.sleep(2)  # 2초간 "done" 상태 유지
                next_command = self.command_queue.popleft()
                self.execute_command(next_command)
            else:
                self.publish_status("running")
                self.waiting = False  # 홈 이동 중이므로 waiting 상태 해제
                self.execute_command("home")  # 홈 이동 실행

    def execute_command(self, command):
        self.waiting = False
        self.current_goal = command
        if command == "home":
            x, y, w, z = self.home_position
        else:
            x, y, w, z = self.positions.get(command, self.home_position)
        self.publish_status("running")  # Publish running status
        self.send_goal(x, y, w, z)

    def send_goal(self, x, y, w, z):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=z, w=w)

        self.action_client.wait_for_server()
        
        # Send goal and attach feedback and result callbacks
        send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        pass  # No additional feedback logging

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.publish_status("Goal rejected")  # Publish goal rejection status
            self.get_logger().info("Goal rejected")
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result()
        if result:
            if self.current_goal == "home":
                self.publish_status("waiting")  # 홈에 도착하면 waiting 상태로 변경
                self.get_logger().info("Arrived at home, switching to waiting state")
                self.waiting = True
            else:
                self.publish_status("done0")  # 목표 지점 도착 시 "done0" 상태 전송
                self.get_logger().info("Done0")
                self.waiting = True
        else:
            self.publish_status("Failed")  # Publish failure status
            self.get_logger().info("Failed to reach the goal")


def main(args=None):
    rclpy.init(args=args)
    node = CommandSubscriber()

    try:
        rclpy.spin(node)  # Keep the node alive to listen for commands
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
