import sys
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QGridLayout
from rclpy.node import Node
import rclpy
from std_msgs.msg import Int32  # 정수를 퍼블리시하기 위한 메시지 타입

class PublisherNode(Node):
    def __init__(self):
        super().__init__('button_publisher')
        # 퍼블리셔 생성
        self.publisher = self.create_publisher(Int32, 'button_topic', 10)

    def publish_data(self, data):
        msg = Int32()
        msg.data = data
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: {data}')


class MyWindow(QWidget):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node  # ROS 2 퍼블리셔 노드
        self.initUI()

    def initUI(self):
        self.setWindowTitle('테이블 번호')  # 창 제목 설정
        layout = QGridLayout()  # 그리드 레이아웃 생성

        # 버튼을 생성하고 그리드에 추가
        for i in range(3):
            for j in range(3):
                button = QPushButton(str(i * 3 + j + 1), self)  # 버튼 생성 및 텍스트 설정
                button.clicked.connect(
                    lambda checked, idx=i * 3 + j + 1: self.buttonClicked(idx)
                )  # 버튼 클릭 시 이벤트 핸들러 연결
                layout.addWidget(button, i, j)  # 그리드에 버튼 추가
        self.setLayout(layout)  # 레이아웃 설정

    def buttonClicked(self, index):
        print(f'Button {index} Clicked!')
        self.ros_node.publish_data(index)  # 버튼 번호를 ROS 2 퍼블리셔로 전달


def main():
    # ROS 2 초기화
    rclpy.init()

    # ROS 2 노드 생성
    ros_node = PublisherNode()

    # PyQt5 애플리케이션 생성
    app = QApplication(sys.argv)
    window = MyWindow(ros_node)
    window.show()

    # PyQt5와 ROS 2 동시 실행
    try:
        while rclpy.ok():
            rclpy.spin_once(ros_node, timeout_sec=0.1)  # ROS 이벤트 처리
            app.processEvents()  # PyQt5 이벤트 처리
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        rclpy.shutdown()
        sys.exit(app.exec_())


if __name__ == '__main__':
    main()
