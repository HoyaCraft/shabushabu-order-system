import sys
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont, QMovie
from PyQt5.QtWidgets import *
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pygame
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

# ROS2 노드 클래스 (퍼블리시 및 서브스크라이브 포함)
class Turtlebot3SubscriberNode(Node):
    def __init__(self, update_log_callback):
        super().__init__('turtlebot3_subscriber_node')

        self.qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,  # 신뢰성 높은 데이터 전송
            durability=DurabilityPolicy.VOLATILE,   # 비휘발성 (실시간 데이터만 수신)
            history=HistoryPolicy.KEEP_LAST,        # 마지막 몇 개의 데이터만 유지
            depth=10                                # 버퍼 크기
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
        self.publisher = self.create_publisher(String, 'HOME', qos_profile=self.qos)

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
        self.running_sound_playing = False  # Track whether the sound is already playing

    def initWidget(self):
        # Label for instructions
        label = QLabel("사용 완료 시 눌러주세요.", self)
        label.setAlignment(Qt.AlignCenter)

        # Button to send 'HOME' command
        self.button1 = QPushButton('ComeBack', self)
        self.button1.setFixedSize(250, 40)
        self.button1.setStyleSheet("""
            QPushButton {
                border: 2px solid #000000;
                border-radius: 5px;
                background-color: #F0F0F0;
                color: #000000;
                font-weight: bold;
                font-size: 17px;
                padding: 10px;
            }
            QPushButton:hover {
                background-color: #D3D3D3;
            }
            QPushButton:pressed {
                background-color: #A9A9A9;
                color: #FFFFFF;
            }
        """)

        # Log display label (for showing robot logs)
        self.log_label = QLabel("waiting", self)
        self.log_label.setAlignment(Qt.AlignCenter)
        self.log_label.setStyleSheet("color: green; font-size: 14px;")

        # **GIF display label**
        self.gif_label = QLabel(self)  # QLabel for GIF
        self.gif_label.setAlignment(Qt.AlignCenter)
        self.gif_label.setScaledContents(True)  # Enable scaling for GIFs to fit QLabel size
        self.gif_label.setFixedSize(200, 200)  # Set fixed size for QLabel

        # **Load GIFs with QMovie**
        self.running_gif = QMovie("/home/shin/table_ws/image/pin.gif")  # Replace with your 'running' GIF file path
        self.waiting_gif = QMovie("/home/lee/img/waiting.gif")  # Replace with your 'waiting' GIF file path
        self.done_gif = QMovie("/home/lee/img/bab.gif")        # Replace with your 'done' GIF file path

        # Connect frameChanged signal for forced repaint
        self.running_gif.frameChanged.connect(self.force_update)
        self.waiting_gif.frameChanged.connect(self.force_update)
        self.done_gif.frameChanged.connect(self.force_update)

        # **Set default GIF**
        self.gif_label.setMovie(self.waiting_gif)
        self.waiting_gif.start()

        # Font for the main label
        font = QFont()
        font.setPointSize(16)
        label.setFont(font)

        # Button click signal
        self.button1.clicked.connect(self.on_button1_clicked)

        # Initialize pygame mixer for sound
        pygame.mixer.init()

        # Layout setup
        layout = QVBoxLayout(self)
        layout.addWidget(label)
        layout.addWidget(self.gif_label, alignment=Qt.AlignCenter)
        layout.addWidget(self.button1, alignment=Qt.AlignCenter)
        layout.addWidget(self.log_label, alignment=Qt.AlignCenter)
        layout.addStretch()
        self.setLayout(layout)

    def force_update(self):
        """Force the QLabel to repaint."""
        self.gif_label.repaint()  # Force QLabel to repaint

    def play_running_sound(self):
        """Play running sound in a loop."""
        if not self.running_sound_playing:
            pygame.mixer.music.load("/home/shin/table_ws/sound/ble.mp3")
            pygame.mixer.music.play(-1)  # -1 means loop indefinitely
            self.running_sound_playing = True

    def stop_running_sound(self):
        """Stop running sound."""
        if self.running_sound_playing:
            pygame.mixer.music.stop()
            self.running_sound_playing = False

    def update_log(self, message):
        """Update the log label with the latest status and adjust color and GIF."""
        self.log_label.setText(f"Status: {message}")

        # Change text color and GIF based on status
        if message.lower() == "running":
            self.log_label.setStyleSheet("color: red; font-size: 14px;")
            self.running_gif = QMovie("/home/lee/img/pin.gif")  # Recreate the QMovie object
            self.running_gif.frameChanged.connect(self.force_update)
            self.gif_label.setMovie(self.running_gif)  # Set the new GIF
            self.running_gif.start()  # Restart the GIF
            print(f"Running GIF State: {self.running_gif.state()}")  # Debugging

            # Play running sound in loop
            self.play_running_sound()
            self.button1.setEnabled(False)
        elif message.lower() == "waiting":
            self.log_label.setStyleSheet("color: green; font-size: 14px;")
            self.waiting_gif.stop()
            self.gif_label.setMovie(self.waiting_gif)
            self.waiting_gif.start()

            # Stop running sound
            self.stop_running_sound()
            self.button1.setEnabled(True)
        elif message.lower() == "done":
            self.log_label.setStyleSheet("color: blue; font-size: 14px;")
            self.done_gif.stop()
            self.gif_label.setMovie(self.done_gif)
            self.done_gif.start()

            # Stop running sound
            self.stop_running_sound()
            self.button1.setEnabled(True)
        else:
            self.log_label.setStyleSheet("color: black; font-size: 14px;")
            self.gif_label.clear()  # Clear GIF if status is unknown

            # Stop running sound
            self.stop_running_sound()
            self.button1.setEnabled(True)

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
