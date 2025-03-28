import sys
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont, QMovie, QPixmap
from PyQt5.QtWidgets import *
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pygame
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy


# ROS2 노드 클래스 (퍼블리시 및 서브스크라이브 포함)
class Turtlebot3SubscriberNode(Node):
    def __init__(self, update_log_callback):
        super().__init__('turtlebot3_subscriber_node')

        self.qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
            )

        # Subscriber to receive robot status logs
        self.subscription = self.create_subscription(
            String,
            'robot_status',  # Topic name for status logs
            self.status_callback,
            qos_profile=self.qos
        )
        self.update_log_callback = update_log_callback

        # Publisher to send 'HOME' command
        self.publisher = self.create_publisher(String, 'home_command', qos_profile=self.qos)

    def status_callback(self, msg):
        # Update the log in the PyQt GUI
        self.update_log_callback(msg.data)

    def send_home(self):
        """Publish 'HOME' command to bring the robot home."""
        msg = String()
        msg.data = 'HOME'
        self.publisher.publish(msg)
        self.get_logger().info("Published: HOME")


# MainWindow for PyQt5
class Mainwindow(QMainWindow):
    def __init__(self, ros_node, parent=None):
        super(Mainwindow, self).__init__(parent)
        self.ros_node = ros_node  # ROS2 노드 참조
        self.initUi()

        # Widgets added as the central widget
        self.mywidget = Widgets(self.ros_node, self)
        self.setCentralWidget(self.mywidget)

    def initUi(self):
        self.setWindowTitle("복귀 실행창")
        self.setGeometry(300, 300, 500, 500)  # Adjusted size for log display


# Widgets for the main window
class Widgets(QWidget):
    def __init__(self, ros_node, parent):
        super(Widgets, self).__init__(parent)
        self.ros_node = ros_node  # ROS2 노드 참조
        self.initWidget()
        pygame.mixer.init()

    def initWidget(self):
        # Label for instructions
        label = QLabel("사용 완료 시 눌러주세요.", self)
        label.setAlignment(Qt.AlignCenter)

        # Button to send 'HOME' command
        self.button1 = QPushButton('ComeBack', self)
        self.button1.setFixedSize(250, 40)
        self.button1.setEnabled(False)  # Default disabled
        
        # Log display label (for showing robot logs)
        self.log_label = QLabel("waiting", self)
        self.log_label.setAlignment(Qt.AlignCenter)
        self.log_label.setStyleSheet("color: green; font-size: 14px;")

        # Image/GIF Label
        self.image_label = QLabel(self)
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setFixedSize(300, 300)

        # Load images and GIFs
        self.waiting_img = QPixmap("/home/shin/table_ws/image/ko.png")  # Set the correct path
        self.done_img = QPixmap("/home/shin/table_ws/image/ko.png")  # Same image for done0 and done
        self.running_gif = QMovie("/home/shin/table_ws/image/pin.gif")  # Set the correct path

        # Layout setup
        layout = QVBoxLayout(self)
        layout.addWidget(label)
        layout.addWidget(self.image_label, alignment=Qt.AlignCenter)
        layout.addWidget(self.button1, alignment=Qt.AlignCenter)
        layout.addWidget(self.log_label, alignment=Qt.AlignCenter)
        layout.addStretch()
        self.setLayout(layout)

        # Button click signal
        self.button1.clicked.connect(self.on_button1_clicked)

    def update_log(self, message):
        """Update the log label with the latest status and enable button for 'done0'."""
        self.log_label.setText(f"Status: {message}")

        if message.lower() == "done0":
            self.button1.setEnabled(True)
        elif message.lower() in ["waiting", "running", "done"]:
            self.button1.setEnabled(False)
        
        # Update UI based on status
        if message.lower() in ["waiting", "done0", "done"]:
            self.image_label.setPixmap(self.done_img)
            pygame.mixer.music.stop()
        elif message.lower() == "running":
            self.image_label.setMovie(self.running_gif)
            self.running_gif.start()
            pygame.mixer.music.load("/home/shin/table_ws/sound/ble.mp3")  # Set the correct path
            pygame.mixer.music.play(-1)  # Loop indefinitely
        else:
            self.running_gif.stop()
            pygame.mixer.music.stop()

        # Force UI to update
        QApplication.processEvents()

    def on_button1_clicked(self):
        """Send 'HOME' command when the button is clicked."""
        self.ros_node.send_home()


# Main function
def main(args=None):
    # ROS2 initialization
    rclpy.init(args=args)

    # Define a callback for log updates
    def update_log_callback(message):
        gui.mywidget.update_log(message)

    # ROS2 node
    ros_node = Turtlebot3SubscriberNode(update_log_callback)

    # PyQt5 application
    app = QApplication(sys.argv)
    gui = Mainwindow(ros_node)
    gui.show()

    # Timer to process ROS2 events
    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0.1))
    timer.start(100)  # 100 ms interval

    try:
        app.exec_()
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
