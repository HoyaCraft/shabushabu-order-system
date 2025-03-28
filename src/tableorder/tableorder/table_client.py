import sys
import rclpy
from rclpy.node import Node
from service_interface.srv import MenuOrder
from PySide2.QtCore import *
from PySide2.QtGui import *
from PySide2.QtWidgets import *
from functools import partial
from std_msgs.msg import Int32

class TableClient(Node):
    def __init__(self, ros_node):
        """
        TableClient 클래스 초기화
        """
        super().__init__('table_client')
        self.ros_node = ros_node  # ROS 2 노드 참조
        self.preorder_dic = {}  # 주문 데이터를 저장할 딕셔너리

        self.publisher_waypoint = self.create_publisher(
            Int32,
            '/button_topic',
            10  # 큐 크기
        )

        self.subscription_refresh = self.create_subscription(
            Int32,
            '/table_refresh',
            self.table_refresh,
            10
        )

    def send_order(self, preorder_dic, ui_main_window):
        """
        ROS2 서비스 요청 전송
        """
        client = self.ros_node.create_client(MenuOrder, 'order_service')

        if not client.wait_for_service(timeout_sec=1.0):
            self.ros_node.get_logger().info("Service server is not ready.")
            QMessageBox.critical(None, "Error", "Service server is not ready.")
            return
        
        request = MenuOrder.Request()
        request.menu = str(preorder_dic)

        self.ros_node.get_logger().info(f"[Client] Sending order: {request.menu}")

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.ros_node, future)

        if future.done():
            try:
                response = future.result()
                self.ros_node.get_logger().info(f"[Client] Received response: {response.message}")

                if response.message == "ok":
                    # 주문 성공 -> 주문 내역을 ordered_list로 이동
                    order_copy = preorder_dic.copy()  # preorder_dic 복사본 생성
                    ui_main_window.move_to_ordered_list(order_copy)
                    QMessageBox.information(None, "Response", "주문이 완료되었습니다.")
                elif response.message == "sorry":
                    # 주문 실패 -> 주문 전 메뉴 초기화
                    ui_main_window.clear_preorder_list()
                    QMessageBox.warning(None, "Response", "주문이 취소되었습니다.")
                else:
                    QMessageBox.information(None, "Response", f"주문 응답: {response.message}")

            except Exception as e:
                self.ros_node.get_logger().info(f"[Client] Service call failed: {e}")
                QMessageBox.critical(None, "Error", f"서비스 요청 실패: {e}")

    def publish_waypoint(self, window_id):
        """
        버튼 클릭 이벤트를 퍼블리시
        """
        msg = Int32()
        msg.data = int(window_id.replace("t", ""))  # 't1' -> 1 변환
        self.publisher_waypoint.publish(msg)  # 메시지 퍼블리시
        self.ros_node.get_logger().info(f"Published button event with ID: {window_id}")
    
    def table_refresh(self, window_id: int):
        self.refresh_order_window(window_id)


class Ui_MainWindow(QMainWindow):
    """Qt 기반 메인 윈도우 클래스"""
    def __init__(self, table_client, ros_node, window_id):
        super().__init__()
        self.table_client = table_client
        self.ros_node = ros_node
        self.window_id = window_id
        self.order_list = {"Adult": 0, "Child": 0, "Soup": 0, "Meat": 0, "Soju": 0, "Beer": 0, "Baijiu": 0, "Mushroom": 0, "Seafood": 0}
        self.preorder_dic = {"Id": self.window_id}  # 초기 주문 데이터
        self.ordered_list = {}  # 🔹 주문된 메뉴 초기화
        self.setupUi()

    def send_order(self):
        """각 창에서 개별적으로 주문을 처리하도록 변경"""
        order_copy = self.preorder_dic.copy()  # preorder_dic의 복사본 생성
        self.table_client.send_order(order_copy, self)  # 복사본을 전달하여 원본 유지

    def setupUi(self):
        self.setWindowTitle("MainWindow")
        self.resize(800, 600)

        # Central Widget
        self.centralwidget = QWidget(self)
        self.setCentralWidget(self.centralwidget)

        # Text Browser (Header)
        self.textBrowser = QTextBrowser(self.centralwidget)
        self.textBrowser.setGeometry(QRect(10, 10, 781, 51))
        self.textBrowser.setHtml("<p align='center' style='font-size:24pt;'>Shabu-Shabu</p>")

        # 그릇 수거 버튼**
        self.collect_dish_button = QPushButton("Dish Retrieval", self.centralwidget)
        self.collect_dish_button.setGeometry(QRect(650, 20, 130, 30))  # 위치 및 크기 설정
        self.collect_dish_button.clicked.connect(self.collect_dishes)  # 클릭 이벤트 연결

        # Tab Widget
        self.Person = QTabWidget(self.centralwidget)
        self.Person.setGeometry(QRect(250, 70, 541, 441))

        # Tabs
        for tab_name in ["Person", "Soup", "Meat", "Alcohol", "Extra Menu"]:
            tab = QWidget()
            main_layout = QHBoxLayout(tab)

            if tab_name == "Person":
                # Left Layout
                left_layout = QVBoxLayout()
                main_layout.addLayout(left_layout)

                # Right Layout
                right_layout = QVBoxLayout()
                main_layout.addLayout(right_layout)

                # Left: Image
                image_label = QLabel(tab)
                image_label.setPixmap(QPixmap("/home/shin/table_ws/image/adult.jpeg").scaled(200, 150, Qt.KeepAspectRatio))
                image_label.setAlignment(Qt.AlignCenter)
                left_layout.addWidget(image_label)

                # Left: Text
                price_label = QLabel(f"Adult\n Price: 12,900 원/인", tab)
                price_label.setAlignment(Qt.AlignCenter)
                price_label.setStyleSheet("font-size: 18px; font-weight: bold;")
                right_layout.addWidget(price_label)

                # Left: Buttons
                button_layout_adult = QHBoxLayout()
                right_layout.addLayout(button_layout_adult)

                minus_button_adult = QPushButton("-", tab)
                plus_button_adult = QPushButton("+", tab)

                # **수정된 부분**: Adult 버튼 클릭 이벤트 연결
                minus_button_adult.clicked.connect(partial(self.handle_count_change, "Adult", -1))
                plus_button_adult.clicked.connect(partial(self.handle_count_change, "Adult", 1))

                button_layout_adult.addWidget(plus_button_adult)
                button_layout_adult.addWidget(minus_button_adult)

                # Right: Image
                image_label = QLabel(tab)
                image_label.setPixmap(QPixmap("/home/shin/table_ws/image/child.jpeg").scaled(450, 150, Qt.KeepAspectRatio))
                image_label.setAlignment(Qt.AlignCenter)
                left_layout.addWidget(image_label)

                # Right: Text
                price_label = QLabel(f"Child\n Price: 5,900 원/인", tab)
                price_label.setAlignment(Qt.AlignCenter)
                price_label.setStyleSheet("font-size: 18px; font-weight: bold;")
                right_layout.addWidget(price_label)

                # Right: Buttons
                button_layout_child = QHBoxLayout()
                right_layout.addLayout(button_layout_child)

                minus_button_child = QPushButton("-", tab)
                plus_button_child = QPushButton("+", tab)

                # Child 버튼 클릭 이벤트 연결
                minus_button_child.clicked.connect(partial(self.handle_count_change, "Child", -1))
                plus_button_child.clicked.connect(partial(self.handle_count_change, "Child", 1))

                button_layout_child.addWidget(plus_button_child)
                button_layout_child.addWidget(minus_button_child)

            elif tab_name == "Soup":
                # Left Layout
                left_layout = QVBoxLayout()
                main_layout.addLayout(left_layout)

                # Right Layout
                right_layout = QVBoxLayout()
                main_layout.addLayout(right_layout)

                # QButtonGroup 추가 (라디오 버튼 상호 배제 설정)
                button_group = QButtonGroup(tab)  # 라디오 버튼 그룹 생성
                button_group.setExclusive(True)  # 상호 배제를 활성화

                # Basic Soup Section
                # Left: Image for Basic Soup
                image_label_basic = QLabel(tab)
                image_label_basic.setPixmap(QPixmap("/home/shin/table_ws/image/basic.jpeg").scaled(200, 150, Qt.KeepAspectRatio))
                image_label_basic.setAlignment(Qt.AlignCenter)
                left_layout.addWidget(image_label_basic)

                # Right: Text for Basic Soup
                price_label_basic = QLabel(f"Basic Soup", tab)
                price_label_basic.setAlignment(Qt.AlignCenter)
                price_label_basic.setStyleSheet("font-size: 18px; font-weight: bold;")
                right_layout.addWidget(price_label_basic)

                # Right: Radio Button for Basic Soup
                basic_Soup_radio = QRadioButton("Basic", tab)  # 라디오 버튼 생성
                button_group.addButton(basic_Soup_radio, id=1)  # 그룹에 추가
                right_layout.addWidget(basic_Soup_radio, alignment=Qt.AlignCenter)

                # Spicy Soup Section
                # Left: Image for Spicy Soup
                image_label_spicy = QLabel(tab)
                image_label_spicy.setPixmap(QPixmap("/home/shin/table_ws/image/spicy.jpeg").scaled(200, 150, Qt.KeepAspectRatio))
                image_label_spicy.setAlignment(Qt.AlignCenter)
                left_layout.addWidget(image_label_spicy)

                # Right: Text for Spicy Soup
                price_label_spicy = QLabel(f"Spicy Soup", tab)
                price_label_spicy.setAlignment(Qt.AlignCenter)
                price_label_spicy.setStyleSheet("font-size: 18px; font-weight: bold;")
                right_layout.addWidget(price_label_spicy)

                # Right: Radio Button for Spicy Soup
                spicy_Soup_radio = QRadioButton("Spicy", tab)  # 라디오 버튼 생성
                button_group.addButton(spicy_Soup_radio, id=2)  # 그룹에 추가
                right_layout.addWidget(spicy_Soup_radio, alignment=Qt.AlignCenter)

                # 버튼 그룹 클릭 이벤트 연결
                button_group.buttonClicked[int].connect(self.handle_soup_selection)

            elif tab_name == "Meat":
                # Left Layout
                left_layout = QVBoxLayout()
                main_layout.addLayout(left_layout)

                # Right Layout
                right_layout = QVBoxLayout()
                main_layout.addLayout(right_layout)

                # Left: Image
                image_label = QLabel(tab)
                image_label.setPixmap(QPixmap("/home/shin/table_ws/image/meat.jpeg").scaled(200, 150, Qt.KeepAspectRatio))
                image_label.setAlignment(Qt.AlignCenter)
                left_layout.addWidget(image_label)

                # Left: Text
                price_label = QLabel(f"Meat : 200g\n Price: 4000 원", tab)
                price_label.setAlignment(Qt.AlignCenter)
                price_label.setStyleSheet("font-size: 18px; font-weight: bold;")
                right_layout.addWidget(price_label)

                # Left: Buttons
                button_layout_Meat = QHBoxLayout()
                right_layout.addLayout(button_layout_Meat)

                minus_button_Meat = QPushButton("-", tab)
                plus_button_Meat = QPushButton("+", tab)

                # Adult 버튼 클릭 이벤트 연결
                minus_button_Meat.clicked.connect(partial(self.handle_count_change, "Meat", -1))
                plus_button_Meat.clicked.connect(partial(self.handle_count_change, "Meat", 1))

                button_layout_Meat.addWidget(plus_button_Meat)
                button_layout_Meat.addWidget(minus_button_Meat)

            elif tab_name == "Alcohol":
                # Left Layout
                left_layout = QVBoxLayout()
                main_layout.addLayout(left_layout)

                # Right Layout
                right_layout = QVBoxLayout()
                main_layout.addLayout(right_layout)

                # 1: Image
                image_label = QLabel(tab)
                image_label.setPixmap(QPixmap("/home/shin/table_ws/image/soju.jpeg").scaled(200, 150, Qt.KeepAspectRatio))
                image_label.setAlignment(Qt.AlignCenter)
                left_layout.addWidget(image_label)

                # 1: Text
                price_label = QLabel(f"Soju\n Price : 4000원", tab)
                price_label.setAlignment(Qt.AlignCenter)
                price_label.setStyleSheet("font-size: 18px; font-weight: bold;")
                right_layout.addWidget(price_label)

                # Left: Buttons
                button_layout_Soju = QHBoxLayout()
                right_layout.addLayout(button_layout_Soju)

                minus_button_Soju = QPushButton("-", tab)
                plus_button_Soju = QPushButton("+", tab)

                # **수정된 부분**: Adult 버튼 클릭 이벤트 연결
                minus_button_Soju.clicked.connect(partial(self.handle_count_change, "Soju", -1))
                plus_button_Soju.clicked.connect(partial(self.handle_count_change, "Soju", 1))

                button_layout_Soju.addWidget(plus_button_Soju)
                button_layout_Soju.addWidget(minus_button_Soju)

                # 2: Image
                image_label = QLabel(tab)
                image_label.setPixmap(QPixmap("/home/shin/table_ws/image/cass.jpeg").scaled(450, 150, Qt.KeepAspectRatio))
                image_label.setAlignment(Qt.AlignCenter)
                left_layout.addWidget(image_label)

                # 2: Text
                price_label = QLabel(f"Beer\n Price : 4000원", tab)
                price_label.setAlignment(Qt.AlignCenter)
                price_label.setStyleSheet("font-size: 18px; font-weight: bold;")
                right_layout.addWidget(price_label)

                # Left: Buttons
                button_layout_Beer = QHBoxLayout()
                right_layout.addLayout(button_layout_Beer)

                minus_button_Beer = QPushButton("-", tab)
                plus_button_Beer = QPushButton("+", tab)

                # **수정된 부분**: Adult 버튼 클릭 이벤트 연결
                minus_button_Beer.clicked.connect(partial(self.handle_count_change, "Beer", -1))
                plus_button_Beer.clicked.connect(partial(self.handle_count_change, "Beer", 1))

                button_layout_Beer.addWidget(plus_button_Beer)
                button_layout_Beer.addWidget(minus_button_Beer)

                # 3: Image
                image_label = QLabel(tab)
                image_label.setPixmap(QPixmap("/home/shin/table_ws/image/go.jpeg").scaled(450, 150, Qt.KeepAspectRatio))
                image_label.setAlignment(Qt.AlignCenter)
                left_layout.addWidget(image_label)

                # 3: Text
                price_label = QLabel(f"Baijiu\n Price : 12000원", tab)
                price_label.setAlignment(Qt.AlignCenter)
                price_label.setStyleSheet("font-size: 18px; font-weight: bold;")
                right_layout.addWidget(price_label)

                # Left: Buttons
                button_layout_Baijiu = QHBoxLayout()
                right_layout.addLayout(button_layout_Baijiu)

                minus_button_Baijiu = QPushButton("-", tab)
                plus_button_Baijiu = QPushButton("+", tab)

                # Adult 버튼 클릭 이벤트 연결
                minus_button_Baijiu.clicked.connect(partial(self.handle_count_change, "Baijiu", -1))
                plus_button_Baijiu.clicked.connect(partial(self.handle_count_change, "Baijiu", 1))

                button_layout_Baijiu.addWidget(plus_button_Baijiu)
                button_layout_Baijiu.addWidget(minus_button_Baijiu)


            elif tab_name == "Extra Menu":
                # Left Layout
                left_layout = QVBoxLayout()
                main_layout.addLayout(left_layout)

                # Right Layout
                right_layout = QVBoxLayout()
                main_layout.addLayout(right_layout)

                # 1: Image
                image_label = QLabel(tab)
                image_label.setPixmap(QPixmap("/home/shin/table_ws/image/Assorted Mushroom.jpeg").scaled(200, 150, Qt.KeepAspectRatio))
                image_label.setAlignment(Qt.AlignCenter)
                left_layout.addWidget(image_label)

                # 1: Text
                price_label = QLabel(f"Mushroom\n Price : 6000원", tab)
                price_label.setAlignment(Qt.AlignCenter)
                price_label.setStyleSheet("font-size: 18px; font-weight: bold;")
                right_layout.addWidget(price_label)

                # Left: Buttons
                button_layout_Mushroom = QHBoxLayout()
                right_layout.addLayout(button_layout_Mushroom)

                minus_button_Mushroom = QPushButton("-", tab)
                plus_button_Mushroom = QPushButton("+", tab)

                # Adult 버튼 클릭 이벤트 연결
                minus_button_Mushroom.clicked.connect(partial(self.handle_count_change, "Mushroom", -1))
                plus_button_Mushroom.clicked.connect(partial(self.handle_count_change, "Mushroom", 1))

                button_layout_Mushroom.addWidget(plus_button_Mushroom)
                button_layout_Mushroom.addWidget(minus_button_Mushroom)

                # 2: Image
                image_label = QLabel(tab)
                image_label.setPixmap(QPixmap("/home/shin/table_ws/image/Assorted Seafood.jpeg").scaled(450, 150, Qt.KeepAspectRatio))
                image_label.setAlignment(Qt.AlignCenter)
                left_layout.addWidget(image_label)

                # 2: Text
                price_label = QLabel(f"Seafood\n Price : 8000원", tab)
                price_label.setAlignment(Qt.AlignCenter)
                price_label.setStyleSheet("font-size: 18px; font-weight: bold;")
                right_layout.addWidget(price_label)

                # Left: Buttons
                button_layout_Seafood = QHBoxLayout()
                right_layout.addLayout(button_layout_Seafood)

                minus_button_Seafood = QPushButton("-", tab)
                plus_button_Seafood = QPushButton("+", tab)

                # **수정된 부분**: Adult 버튼 클릭 이벤트 연결
                minus_button_Seafood.clicked.connect(partial(self.handle_count_change, "Seafood", -1))
                plus_button_Seafood.clicked.connect(partial(self.handle_count_change, "Seafood", 1))

                button_layout_Seafood.addWidget(plus_button_Seafood)
                button_layout_Seafood.addWidget(minus_button_Seafood)

            # Add tab with the complete layout
            self.Person.addTab(tab, tab_name)

         # Text Edit (Instructions)
        self.textEdit = QTextBrowser(self.centralwidget)
        self.textEdit.setGeometry(QRect(10, 70, 221, 371))
        self.textEdit.setHtml(
    """
    <p align="center" style="margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;">
        <span style="font-size:16pt; font-weight:600;">테이블 오더</span>
    </p>
    <p align="center" style="margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;">
        <span style="font-size:16pt; font-weight:600;">사용 방법</span>
    </p>
    <p style="margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;">
        <span style="font-size:16pt; font-weight:600;">필수</span>
    </p>
    <p style="margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;">
        <span style="font-size:14pt;">1. 인원을 선택해주세요. </span>
    </p>
    <p style="margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;">
        <span style="font-size:12pt;">(소인 기준 : 6세 이하)</span>
    </p>
    <p style="margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;">
        <span style="font-size:14pt;">2. 육수를 선택해주세요.</span>
    </p>
    <p style="margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;">
        <span style="font-size:12pt;">(기본/매운)</span>
    </p>
    <p style="margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;">
        <span style="font-size:16pt; font-weight:600;">선택</span>
    </p>
    <p style="margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;">
        <span style="font-size:14pt;">- 고기를 선택해주세요. </span>
    </p>
    <p style="margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;">
        <span style="font-size:14pt;">- 술을 선택해주세요. </span>
    </p>
    <p style="margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;">
        <span style="font-size:12pt;">(소주/맥주/고량주)</span>
    </p>
    <p style="margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;">
            <span style="font-size:14pt;">- 추가 메뉴</span>
    </p>
    """
)

        # Vertical Layout for Order Information
        self.verticalLayoutWidget = QWidget(self.centralwidget)
        self.verticalLayoutWidget.setGeometry(QRect(1, 450, 238, 140))
        self.verticalLayout = QVBoxLayout(self.verticalLayoutWidget)

        # Order Details
        self.textBrowser_2 = QTextBrowser(self.verticalLayoutWidget)
        self.textBrowser_2.setHtml("<p align='center' style='font-size:14pt; font-weight:600;'>아래 버튼을 통해 주문 정보를 확인이 가능합니다.</p>")
        self.textBrowser_2.setGeometry(QRect(10, 450, 221, 131))
        self.verticalLayout.addWidget(self.textBrowser_2)

        # Order Button
        self.My_Order = QPushButton("My Order", self.verticalLayoutWidget)
        self.My_Order.clicked.connect(self.show_order_window)  # 버튼 클릭 이벤트 연결
        self.verticalLayout.addWidget(self.My_Order)

        # Submit Button
        self.pushButton = QPushButton("Order", self.centralwidget)
        self.pushButton.setGeometry(QRect(250, 520, 541, 61))
        self.pushButton.clicked.connect(self.send_order)  # Order 버튼에 send_order 메서드 연결

    def collect_dishes(self):
        """그릇 수거 버튼 클릭 시 호출되는 메서드"""
        QMessageBox.information(self, "수거 요청", "그릇 수거를 요청했습니다.")  # 알림 창 표시
        print("그릇 수거 요청.")  # 콘솔 출력

        self.table_client.publish_waypoint(self.window_id)

    def open_multiple_windows(self):
        for i in range(1, 10):
            dialog = QDialog(self)
            dialog.setWindowTitle(f"Window {i}")
            dialog.setFixedSize(300, 200)
            layout = QVBoxLayout(dialog)
            label = QLabel(f"This is window {i}", dialog)
            label.setAlignment(Qt.AlignCenter)
            layout.addWidget(label)
            dialog.show()

    def handle_count_change(self, count_type, change):
        if count_type not in self.order_list:
            return  # 잘못된 타입이 전달되면 종료

        # 수량 변경
        self.order_list[count_type] += change
        if self.order_list[count_type] < 0:  # 음수 방지
            self.order_list[count_type] = 0

        # table_client.preorder_dic 업데이트
        self.preorder_dic[count_type] = self.order_list[count_type]
        self.preorder_dic["Id"] = self.window_id

        # 저장된 preorder_dic 출력
        print("Updated preorder_dic:", self.preorder_dic)

        # UI 업데이트
        self.update_order_display()

    def handle_soup_selection(self, button_id):
        """Soup 선택 처리"""
        if button_id == 1:
            self.order_list["Soup"] = 0
        elif button_id == 2:
            self.order_list["Soup"] = 1

        # preorder_dic 업데이트
        self.preorder_dic["Soup"] = self.order_list["Soup"]

        # 저장된 preorder_dic 출력
        print("Updated preorder_dic:", self.preorder_dic)

        # UI 업데이트
        self.update_order_display()

    def update_order_display(self):
        adult_count = self.order_list["Adult"]
        child_count = self.order_list["Child"]
        soup_count = "Basic" if self.order_list["Soup"] == 0 else "Spicy"
        meat_count = self.order_list["Meat"]
        soju_count = self.order_list["Soju"]
        beer_count = self.order_list["Beer"]
        baijiu_count = self.order_list["Baijiu"]
        mush_count = self.order_list["Mushroom"]
        seafood_count = self.order_list["Seafood"]
        
    def refresh_order_window(self):
        order_window = QDialog(self)
        ordered_menu_browser = QTextBrowser(order_window)
        ordered_html = "<ul>"
        ordered_html += "<li>주문 내역 없음</li>"
        ordered_html += "</ul>"
        ordered_menu_browser.setHtml(ordered_html)
        layout = QVBoxLayout(order_window)
        layout.addWidget(ordered_menu_browser)
    
    
    def show_order_window(self):
        """My_Order 버튼을 눌렀을 때 새 창을 띄우고 주문 내역을 표시"""
        order_window = QDialog(self)
        order_window.setWindowTitle("Order Details")
        order_window.setFixedSize(500, 510)

        layout = QVBoxLayout(order_window)

        # 🔹 주문 전 선택한 메뉴 표시
        selected_menu_label = QLabel("주문 전 선택한 메뉴", order_window)
        selected_menu_label.setAlignment(Qt.AlignCenter)
        selected_menu_label.setStyleSheet("font-size: 14pt; font-weight: bold;")
        layout.addWidget(selected_menu_label)

        selected_menu_browser = QTextBrowser(order_window)
        selected_html = "<ul>"
        if len(self.preorder_dic) > 1:  # ID 제외하고 데이터가 있으면
            for key, value in self.preorder_dic.items():
                if key == "Soup":  
                    value = "Basic" if value == 0 else "Spicy"  # Soup 값 변환
                selected_html += f"<li>{key}: {value}</li>"
        else:
            selected_html += "<li>주문 내역 없음</li>"
        selected_html += "</ul>"
        selected_menu_browser.setHtml(selected_html)

        layout.addWidget(selected_menu_browser)

        # 🔹 주문된 메뉴 표시 (서비스 OK 응답 후)
        ordered_menu_label = QLabel("주문된 메뉴", order_window)
        ordered_menu_label.setAlignment(Qt.AlignCenter)
        ordered_menu_label.setStyleSheet("font-size: 14pt; font-weight: bold;")
        layout.addWidget(ordered_menu_label)

        ordered_menu_browser = QTextBrowser(order_window)
        ordered_html = "<ul>"
        if self.ordered_list:  # ✅ 주문된 내역이 있을 경우만 표시
            for key, value in self.ordered_list.items():
                if key == "Soup":
                    value = "Basic" if value == 0 else "Spicy"  # Soup 값 변환
                if key != "Id":
                    ordered_html += f"<li>{key}: {value}</li>"
        else:
            ordered_html += "<li>주문 내역 없음</li>"
        ordered_html += "</ul>"
        ordered_menu_browser.setHtml(ordered_html)
        layout.addWidget(ordered_menu_browser)

        # 닫기 버튼
        close_button = QPushButton("Close", order_window)
        close_button.clicked.connect(order_window.close)
        layout.addWidget(close_button, alignment=Qt.AlignCenter)

        order_window.exec_()  # 새 창 실행


    def handle_response(self, future):
        """서비스 응답 처리"""
        try:
            response = future.result()
            self.ros_node.get_logger().info(f"Received response: {response.message}")
            QMessageBox.information(self, "Response", f"응답: {response.message}")
        except Exception as e:
            self.ros_node.get_logger().info(f"Service call failed: {e}")
            QMessageBox.critical(self, "Error", f"서비스 요청 실패: {e}")
    

    def move_to_ordered_list(self, order_copy):
        """주문 완료 후 주문 전 선택한 메뉴를 주문된 메뉴로 이동하고 초기화"""
        if order_copy:  # order_copy가 비어있지 않은 경우만 처리
            for key, value in order_copy.items():
                if key != "Id":  # ID는 이동하지 않음
                    # 🔹 주문된 메뉴로 데이터 이동
                    if key in self.ordered_list:
                        self.ordered_list[key] += value  # 기존 값에 추가
                    else:
                        self.ordered_list[key] = value  # 새 항목 추가

            # 🔹 디버깅 로그 출력
            print(f"Ordered list updated: {self.ordered_list}")  # 주문된 메뉴 확인 로그

            # 🔹 주문 전 선택한 메뉴 초기화
            self.preorder_dic.clear()  # 전체 초기화
            print(f"Preorder list cleared: {self.preorder_dic}")  # 초기화 로그

            # 🔹 UI 업데이트
            self.update_order_display()  # 주문 내역 업데이트

        else:
            # 주문 내역이 비어 있을 때 처리
            self.ros_node.get_logger().warning("Preorder list is empty. Nothing to move.")
            print("⚠️ Preorder list is empty. Nothing to move.")


    def clear_preorder_list(self):
        """주문이 'sorry'로 실패한 경우 주문 전 메뉴 초기화"""
        self.preorder_dic.clear()  # 주문 전 데이터 초기화
        self.preorder_dic["Id"] = self.window_id  # ID는 유지
        self.update_order_display()  # UI 업데이트
        self.ros_node.get_logger().info(f"Preorder list cleared for {self.window_id}")  # 로그 출력


def main():
    rclpy.init()

    # **QApplication 생성은 가장 먼저 실행**
    app = QApplication(sys.argv)

    # ROS2 클라이언트 노드 생성
    ros_node = Node("client_node")  # 클라이언트 노드 생성
    table_client = TableClient(ros_node)

    # 창 9개 생성
    windows = []  # 창들을 저장할 리스트
    for i in range(9):
        window_id = f"t{i + 1}"  # 창 ID 생성 (t1, t2, ..., t9)
        
        # Ui_MainWindow 인스턴스 생성 시 window_id 전달
        mainWindow = Ui_MainWindow(table_client, ros_node, window_id)
        
        # 창 제목 설정
        mainWindow.setWindowTitle(f"Main Window {window_id}")  # 창 제목에 ID 포함

        # 창 위치 설정
        x = 100 + (i % 3) * 300  # 3열 배치: x 좌표
        y = 100 + (i // 3) * 250  # 3행 배치: y 좌표
        mainWindow.move(x, y)  # 창 위치 이동

        # ID를 표시할 QLabel 추가
        id_label = QLabel(mainWindow.centralwidget)
        id_label.setText(window_id)  # ID 텍스트 설정
        id_label.setGeometry(QRect(25, 25, 50, 20))  # 위치와 크기 설정
        id_label.setStyleSheet("font-size: 18pt; font-weight: bold; color: black;")  # 스타일 설정

        mainWindow.show()  # 창 표시
        windows.append(mainWindow)  # 창을 리스트에 추가

    # Qt 실행 루프
    exit_code = app.exec_()

    # ROS2 종료
    ros_node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)



if __name__ == "__main__":
    main()
