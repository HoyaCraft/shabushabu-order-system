import sys
import re
import time
import json
import ast
from PySide2 import QtCore, QtGui, QtWidgets
from PySide2.QtCore import Signal
from tableorder.ui import ui_counter_dp
import locale
import os

###############################################################################
# (1) ROS2 rclpy 및 서비스 관련 임포트
###############################################################################
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Int32
from service_interface.srv import MenuOrder  # 서비스 정의에 맞게 변경
import threading
import queue
import pymysql


UI_FILE = os.path.join(os.path.dirname(__file__),'ui','counter_dp.ui')
ui_dir = os.path.join(os.path.dirname(__file__), 'ui')
sys.path.append(ui_dir)
locale.setlocale(locale.LC_TIME, 'ko_KR.UTF-8')

###############################################################################
# (2) ROS Node 클래스 정의
###############################################################################
class CounterRosNode(Node):
    def __init__(self, window, order_queue, response_queue):
        super().__init__('counter_node')
        self.window = window
        self.order_queue = order_queue
        self.response_queue = response_queue

        # ROS 노드를 GUI에 할당
        self.window.ros_node = self
        print("ROS node assigned to window.ros_node")


        self.subscription_done = self.create_subscription(
            String,
            '/robot_status',
            self.waypointCallback,
            10
        )
        self.publisher_waypoint = self.create_publisher(
            Int32,
            '/button_topic',
            10
        )

        # --- 서비스 서버 ---
        self.service = self.create_service(MenuOrder, 'order_service', self.handle_order_service)

        # 응답을 저장할 변수와 이벤트 초기화
        self.response_event = threading.Event()
        self.current_response = None

        self.get_logger().info("Service 'order_service' is ready.")

    def handle_order_service(self, request, response):
        order = request.menu
        self.get_logger().info(f"Received order: {order}")

        # GUI에 주문 추가
        self.order_queue.put(order)
        
        # 이벤트 리셋 및 대기
        self.response_event.clear()
        self.current_response = None

        self.get_logger().info("Waiting for user response (ok/sorry)...")

        try:
            # 응답을 대기 (타임아웃 30초)
            user_response = self.response_queue.get(timeout=30)
            response.message = user_response
            self.get_logger().info(f"Responding with: {response.message}")
        except queue.Empty:
            response.message = "sorry"  # 타임아웃 시 기본 응답
            self.get_logger().info("No response received. Responding with default: sorry")

        #response.message = "ok"  # 기본 응답 예시
        #self.get_logger().info(f"Responding with: {response.message}")
        return response


    def set_service_response(self, resp):
        """GUI에서 호출하여 서비스 응답을 설정"""
        self.current_response = resp
        self.response_event.set()

    def waypointCallback(self, msg: String):
        """amrqt_node에서 'done' 수신 → 조리 중 목록 첫 항목 제거"""
        if msg.data == "done":
            self.window.onAMRNodeCallback(True)
            self.window.setAMRState("done")

        """waypoint_node에서 'waiting' 수신 → AMR 상태 charging"""
        if msg.data == "waiting":
            self.window.setAMRState("waiting")
        
        if msg.data == "running":
            self.window.setAMRState("running")

    def publishWaypointGoal(self, table_num: int):
        msg = Int32()
        msg.data = table_num
        self.publisher_waypoint.publish(msg)
        self.get_logger().info(f"[publishWaypointGoal] 테이블 번호 = {table_num}")

        

def ros_thread_main(window, order_queue, response_queue):
    rclpy.init()
    ros_node = CounterRosNode(window=window, order_queue=order_queue, response_queue=response_queue)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(ros_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    ros_node.destroy_node()
    rclpy.shutdown()

###############################################################################
# (3) 배터리 위젯
###############################################################################
class BatteryWidget(QtWidgets.QFrame):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._batteryLevel = 100
        self._batteryState = "waiting"
        self.setMinimumSize(100, 100)

    def setBatteryLevel(self, level: int):
        self._batteryLevel = max(0, min(100, level))
        self.update()

    def setBatteryState(self, state: str):
        self._batteryState = state
        self.update()

    def paintEvent(self, event: QtGui.QPaintEvent):
        super().paintEvent(event)
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing, True)

        rect = self.rect()
        size = min(rect.width(), rect.height())
        radius = size // 2
        center = rect.center()

        # 배터리 색상
        if self._batteryLevel <= 50:
            color = QtGui.QColor("red")
        elif self._batteryLevel <= 80:
            color = QtGui.QColor("yellow")
        else:
            color = QtGui.QColor("green")

        # 원
        painter.setBrush(color)
        painter.setPen(QtCore.Qt.NoPen)
        painter.drawEllipse(center, radius, radius)

        # 퍼센트
        painter.setPen(QtCore.Qt.black)
        font = painter.font()
        font.setPointSize(radius // 2)
        painter.setFont(font)
        textRect = QtCore.QRect(center.x() - radius, center.y() - radius, size, size)
        painter.drawText(textRect, QtCore.Qt.AlignCenter, f"{self._batteryLevel}%")

        # 상태
        painter.setPen(QtCore.Qt.white)
        smallerFont = QtGui.QFont(font)
        smallerFont.setPointSize(radius // 4)
        painter.setFont(smallerFont)
        stateRect = QtCore.QRect(center.x() - radius, center.y() - (radius // 3), size, size)
        painter.drawText(stateRect, QtCore.Qt.AlignHCenter | QtCore.Qt.AlignBottom, self._batteryState)

###############################################################################
# (4) "조리 중" 목록 항목 위젯 (라벨 + Go 버튼)
###############################################################################
class CookingItemWidget(QtWidgets.QWidget):
    goClicked = Signal(str)  # 주문 텍스트를 보내는 시그널

    def __init__(self, orderText: str, parent=None):
        super().__init__(parent)
        self._orderText = orderText
        self.initUI()

    def initUI(self):
        layout = QtWidgets.QHBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)

        # 주문 내용 라벨
        self.labelOrder = QtWidgets.QLabel(self._orderText, self)
        self.labelOrder.setWordWrap(True)
        self.labelOrder.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Preferred)

        # 라벨 폰트 20pt
        font_label = self.labelOrder.font()
        font_label.setPointSize(20)
        self.labelOrder.setFont(font_label)

        layout.addWidget(self.labelOrder, stretch=3)

        # Go 버튼
        self.buttonGo = QtWidgets.QPushButton("Go", self)
        font_btn = self.buttonGo.font()
        font_btn.setPointSize(20)  # 글자 크기 20
        self.buttonGo.setFont(font_btn)

        # 버튼이 너무 작게 나오지 않도록 고정 크기(예: 가로 100, 세로 60)
        self.buttonGo.setFixedSize(100, 60)

        layout.addWidget(self.buttonGo, stretch=1)
        self.setLayout(layout)

        self.buttonGo.clicked.connect(self.onGoButtonClicked)

    def onGoButtonClicked(self):
        self.goClicked.emit(self._orderText)

    def sizeHint(self):
        """
        라벨(글자 크기 20) + Go(글자 크기 20, 100x60)가 안 잘리도록
        충분한 크기를 힌트로 줍니다.
        """
        return QtCore.QSize(400, 100)

###############################################################################
# (5) 전체 메인 GUI 로직 (CounterDP)
###############################################################################
class CounterDP(QtWidgets.QMainWindow, ui_counter_dp.Ui_MainWindow):
    def __init__(self, order_queue, response_queue):

        super().__init__()
        #uic.loadUi(UI_FILE, self)
        self.setupUi(self)
        # ROS와 통신할 큐 저장
        self.order_queue = order_queue
        self.response_queue = response_queue

        # UI 요소 연결
        self.pushButtonOk.clicked.connect(self.onOkClicked)
        self.pushButtonSorry.clicked.connect(self.onSorryClicked)

        # 대기 주문을 처리할 타이머 설정
        self.order_timer = QtCore.QTimer()
        self.order_timer.timeout.connect(self.check_new_order)
        self.order_timer.start(100)  # 100ms 간격으로 주문 확인




        # 탭
        self.tabWidget = self.findChild(QtWidgets.QTabWidget, "tabWidget")
        self.tabMain = self.findChild(QtWidgets.QWidget, "tabMain")
        self.tabEdit = self.findChild(QtWidgets.QWidget, "tabEdit")

        # 메인 탭
        self.batteryFrame = self.findChild(QtWidgets.QFrame, "batteryFrame")
        self.listWidgetWaiting = self.findChild(QtWidgets.QListWidget, "listWidgetWaiting")
        self.listWidgetCooking = self.findChild(QtWidgets.QListWidget, "listWidgetCooking")
        self.listWidgetStock = self.findChild(QtWidgets.QListWidget, "listWidgetStock")

        self.pushButtonCheckWaiting = self.findChild(QtWidgets.QPushButton, "pushButtonCheckWaiting")
        self.pushButtonOk = self.findChild(QtWidgets.QPushButton, "pushButtonOk")
        self.pushButtonSorry = self.findChild(QtWidgets.QPushButton, "pushButtonSorry")

        # 에디트 탭
        self.tableWidgetMenu = self.findChild(QtWidgets.QTableWidget, "tableWidgetMenu")
        self.pushButtonSave = self.findChild(QtWidgets.QPushButton, "pushButtonSave")
        self.pushButtonReset = self.findChild(QtWidgets.QPushButton, "pushButtonReset")

        # 배터리 위젯 삽입
        self._batteryWidget = BatteryWidget(self.batteryFrame)
        vlay = QtWidgets.QVBoxLayout(self.batteryFrame)
        vlay.setContentsMargins(0, 0, 0, 0)
        vlay.addWidget(self._batteryWidget)

        # AMR 상태
        self._batteryLevel = 100
        self._amrState = "waiting"
        self._amrRunning = False
        self._chargingStartTime = 0

        # 메뉴 DB (예시)
        self._originalMenuData = [
            {"name": "Adult", "price": 12900, "stock": None},
            {"name": "Child", "price": 5900, "stock": None},
            {"name": "Soup", "price": 0, "stock": None},
            {"name": "Meat", "price": 4000, "stock": 100},
            {"name": "Soju", "price": 4000, "stock": 100},
            {"name": "Beer", "price": 4000, "stock": 100},
            {"name": "Baijiu", "price": 10000, "stock": 100},
            {"name": "Mushroom", "price": 6000, "stock": 100},
            {"name": "Seafood", "price": 8000, "stock": 100},
        ]
        self._editedMenuData = []

        self.initUI()
        self.initSignals()

        # 1초 타이머
        self._timer = QtCore.QTimer(self)
        self._timer.setInterval(1000)
        self._timer.timeout.connect(self.onTimer)
        self._timer.start()


        # Lock for thread-safe operations
        self.lock = threading.Lock()

    def setRosNode(self, ros_node):
        """메인 GUI에서 ROS 노드 객체를 저장해둠."""
        self.ros_node = ros_node

    def initUI(self):
        self.setMinimumSize(640, 360)
        self.rebuildMainTabLayout()
        self.loadMenuData()
        self.updateStockList()

        # OK / SORRY는 초기 숨김
        self.pushButtonOk.setVisible(False)
        self.pushButtonSorry.setVisible(False)

        # 대기 목록 항목 스타일: 테두리, 글자 크기 등
        self.listWidgetWaiting.setStyleSheet("""
            QListWidget::item {
                border: 1px solid #CCCCCC;
                padding: 5px;
                margin: 3px;
            }
            QListWidget::item:selected {
                border: 2px solid #0078D7;
                background-color: #C0E4FF;
            }
        """)

        # 조리 목록 스타일: 테두리만
        self.listWidgetCooking.setStyleSheet("""
            QListWidget::item {
                border: 1px solid #CCCCCC;
                padding: 5px;
                margin: 3px;
            }
        """)

    def rebuildMainTabLayout(self):
        mainLayout = QtWidgets.QHBoxLayout(self.tabMain)
        mainLayout.setContentsMargins(10, 10, 10, 10)

        leftLayout = QtWidgets.QVBoxLayout()
        hbox = QtWidgets.QHBoxLayout()
        hbox.addWidget(self.pushButtonCheckWaiting)
        hbox.addWidget(self.pushButtonOk)
        hbox.addWidget(self.pushButtonSorry)
        leftLayout.addLayout(hbox)

        leftLayout.addWidget(QtWidgets.QLabel("결제 대기 중 목록"))
        leftLayout.addWidget(self.listWidgetWaiting)

        leftLayout.addWidget(QtWidgets.QLabel("조리 중 목록"))
        leftLayout.addWidget(self.listWidgetCooking)

        rightLayout = QtWidgets.QVBoxLayout()

        # 날짜/시간 표시용 라벨
        self.labelDateTime = QtWidgets.QLabel("", alignment=QtCore.Qt.AlignRight)
        font_dt = self.labelDateTime.font()
        font_dt.setPointSize(12)
        self.labelDateTime.setFont(font_dt)
        rightLayout.addWidget(self.labelDateTime)

        # 배터리
        rightLayout.addWidget(self.batteryFrame)

        # 로봇 상태 라벨
        self.labelRobotState = QtWidgets.QLabel("(로봇 상태)", alignment=QtCore.Qt.AlignCenter)
        font = self.labelRobotState.font()
        font.setPointSize(14)
        self.labelRobotState.setFont(font)
        rightLayout.addWidget(self.labelRobotState)

        rightLayout.addWidget(QtWidgets.QLabel("현재 재고 현황", alignment=QtCore.Qt.AlignCenter))
        rightLayout.addWidget(self.listWidgetStock)

        mainLayout.addLayout(leftLayout, stretch=3)
        mainLayout.addLayout(rightLayout, stretch=1)

    def initSignals(self):
        # 메인 탭
        self.pushButtonCheckWaiting.clicked.connect(self.onCheckWaiting)
        self.pushButtonOk.clicked.connect(self.onOkClicked)
        self.pushButtonSorry.clicked.connect(self.onSorryClicked)

        # 에디트 탭
        self.pushButtonSave.clicked.connect(self.onSaveMenuData)
        self.pushButtonReset.clicked.connect(self.onResetMenuData)

    #####################################################################
    # 에디트 탭: 메뉴 로딩, +/- , Save, Reset
    #####################################################################
    def loadMenuData(self):
        self._editedMenuData = [dict(d) for d in self._originalMenuData]
        self.tableWidgetMenu.setRowCount(len(self._editedMenuData))
        self.tableWidgetMenu.setColumnCount(4)
        self.tableWidgetMenu.setHorizontalHeaderLabels(["메뉴명", "가격", "재고", "수정"])

        for i, menu in enumerate(self._editedMenuData):
            nameItem = QtWidgets.QTableWidgetItem(menu["name"])
            priceItem = QtWidgets.QTableWidgetItem(str(menu["price"]))
            stockItem = QtWidgets.QTableWidgetItem(str(menu["stock"]) if menu["stock"] is not None else "N/A")

            self.tableWidgetMenu.setItem(i, 0, nameItem)
            self.tableWidgetMenu.setItem(i, 1, priceItem)
            self.tableWidgetMenu.setItem(i, 2, stockItem)

            # +/-
            w = QtWidgets.QWidget()
            hl = QtWidgets.QHBoxLayout(w)
            hl.setContentsMargins(0, 0, 0, 0)

            btnPlus = QtWidgets.QPushButton("+")
            btnMinus = QtWidgets.QPushButton("-")

            btnPlus.clicked.connect(lambda chk, r=i: self.onChangeStock(r, +1))
            btnMinus.clicked.connect(lambda chk, r=i: self.onChangeStock(r, -1))

            hl.addWidget(btnPlus)
            hl.addWidget(btnMinus)
            w.setLayout(hl)

            self.tableWidgetMenu.setCellWidget(i, 3, w)

        self.tableWidgetMenu.resizeColumnsToContents()

    def onChangeStock(self, row, diff):
        stockItem = self.tableWidgetMenu.item(row, 2)
        if stockItem and self._editedMenuData[row]["stock"] is not None:
            currentStock = int(stockItem.text())
            newStock = max(0, currentStock + diff)
            stockItem.setText(str(newStock))

    def onSaveMenuData(self):
        for i in range(self.tableWidgetMenu.rowCount()):
            name = self.tableWidgetMenu.item(i, 0).text()
            price = int(self.tableWidgetMenu.item(i, 1).text())
            stock_text = self.tableWidgetMenu.item(i, 2).text()
            stock = int(stock_text) if stock_text.isdigit() else None
            self._editedMenuData[i]["name"] = name
            self._editedMenuData[i]["price"] = price
            self._editedMenuData[i]["stock"] = stock

        self._originalMenuData = [dict(d) for d in self._editedMenuData]
        self.updateStockList()

        QtWidgets.QMessageBox.information(self, "저장", "메뉴 수정 사항을 저장했습니다.")

    def onResetMenuData(self):
        self.loadMenuData()

    def updateStockList(self):
        self.listWidgetStock.clear()
        for menu in self._originalMenuData:
            if menu["stock"] is not None:
                self.listWidgetStock.addItem(f"{menu['name']}: {menu['stock']}개")
            else:
                self.listWidgetStock.addItem(f"{menu['name']}: 재고 관리 안함")

    #####################################################################
    # 결제 대기 목록 관련
    #####################################################################
    def check_new_order(self):
        """ROS 노드로부터 새로운 주문이 있는지 확인하고 처리"""
        while not self.order_queue.empty():
            order_str = self.order_queue.get()
            self.addWaitingOrderFromService(order_str)    
    
    
    def onCheckWaiting(self):
        if self.listWidgetWaiting.count() > 0:
            self.pushButtonOk.setVisible(True)
            self.pushButtonSorry.setVisible(True)
        else:
            QtWidgets.QMessageBox.information(self, "알림", "결제 대기 중 주문이 없습니다.")

    def addWaitingOrder(self, table_num, menu_orders):
        """
        대기 목록에 (HH:MM - 테이블 n - 메뉴1 x 개수1, 메뉴2 x 개수2...) 형식으로 표시
        + 폰트 20pt로 표시
        """
        now = time.strftime("%H:%M", time.localtime())

        # 메뉴와 수량을 문자열로 병합
        orders_summary = ", ".join([f"{menu_name} : {quantity}" for menu_name, quantity in menu_orders.items()])
        text = f"{now} - 테이블 {table_num} - {orders_summary}"

        # Qt 위젯 생성
        listItem = QtWidgets.QListWidgetItem(text)
        font = listItem.font()
        font.setPointSize(20)  # 글자 크기 20
        listItem.setFont(font)

        self.listWidgetWaiting.addItem(listItem)

    def addWaitingOrderFromService(self, msg: String):
        """
        서비스로부터 받은 주문을 대기 목록에 추가
        주문 문자열 형식에 맞게 파싱하여 추가
        예: "T3: 메뉴1 x 2, 메뉴2 x 1"
        """
        try:
            # 문자열 파싱
            self.data= ast.literal_eval(msg)

            # 테이블 번호와 주문 정보 분리
            id_value = self.data.get("Id", "")
            if id_value and isinstance(id_value, str):  # Id가 문자열인지 확인
                match = re.search(r"t(\d+)", id_value)
                if match:  # 정규식 결과가 None이 아닌 경우
                    table_num = int(match.group(1))  # 'T3'에서 숫자 3 추출
                else:
                    raise ValueError(f"Invalid Id format: {id_value}")  # Id 형식이 잘못된 경우
            else:
                raise ValueError("Id 값이 유효하지 않습니다.")

            orders = {k: v  for k, v in self.data.items() if v is not None and k != "Id"}

            # PyQt 메서드로 UI 업데이트
            self.addWaitingOrder(table_num, orders)


        except Exception as e:
            raise ValueError(f"문자열 파싱 오류: {e}")

    def onOkClicked(self):
        """
        OK 버튼 클릭 시 현재 대기 목록의 첫 번째 주문에 'ok' 응답
        """
        if self.listWidgetWaiting.count() > 0:
            item = self.listWidgetWaiting.item(0)
            orderText = item.text()

            self.response_queue.put("ok")

            # 재고 차감
            # 주문 텍스트에서 각 메뉴와 수량을 파싱
            matches = re.findall(r"([\w가-힣]+)\s*:\s*(\d+)", orderText)
            table_match = re.search(r"테이블\s+(\d+)", orderText)
            for menu_name, quantity in matches:
                quantity = int(quantity)
                for menu in self._originalMenuData:
                    if menu["name"] == menu_name:
                        if menu["stock"] is not None:  # None이 아닌 경우에만 계산
                            menu["stock"] = max(0, menu["stock"] - quantity)
                        break
            if table_match:
                table_num = int(table_match.group(1))
                table_id = f"t{table_num}"
            else:
                table_id = "Unknown"

            # Convert matches to dictionary format
            current_order = {menu_name: int(quantity) for menu_name, quantity in matches}

            # Initialize self.temp if not already present
            if not hasattr(self, "temp"):
                self.temp = {}
                self.empty = []

            # Update self.temp with the new order
            if table_id not in self.temp:
                self.temp[table_id] = current_order
            else:
                for menu_name, quantity in current_order.items():
                    if menu_name in self.temp[table_id]:
                        self.temp[table_id][menu_name] += quantity
                    else:
                        self.temp[table_id][menu_name] = quantity

            # UI 업데이트
            self.updateStockList()
            self.loadMenuData()
            self.listWidgetWaiting.takeItem(0)
            if self.listWidgetWaiting.count() == 0:
                self.pushButtonOk.setVisible(False)
                self.pushButtonSorry.setVisible(False)

            # "조리 중" 목록: 커스텀 위젯으로 표시
            cookingWidget = CookingItemWidget(orderText)
            cookingWidget.goClicked.connect(self.onGoClicked)

            # QListWidgetItem에는 텍스트를 넣지 않아야 겹치지 않음
            listItem = QtWidgets.QListWidgetItem()
            self.listWidgetCooking.addItem(listItem)
            self.listWidgetCooking.setItemWidget(listItem, cookingWidget)

            # 위젯의 sizeHint로 아이템 높이 지정
            listItem.setSizeHint(cookingWidget.sizeHint())

            # 대기 목록에서 제거
            self.listWidgetWaiting.takeItem(0)

            try:
                conn = pymysql.connect(
                    host='localhost',
                    port=3306,    
                    user='shin',     
                    passwd='newpassword',     
                    db='kichen',     
                    charset='utf8'
                )
                cur = conn.cursor()

                temp_order = self.temp[table_id]

                data = [
                        table_id,
                        temp_order.get("Child", 0),
                        temp_order.get("Adult", 0),
                        temp_order.get("Soup", 0),
                        temp_order.get("Meat", 0.),
                        temp_order.get("Soju", 0),
                        temp_order.get("Beer", 0),
                        temp_order.get("Baijiu", 0),
                        temp_order.get("Mushroom", 0),
                        temp_order.get("Seafood", 0)
                    ]

                if table_id in self.empty : 
                            print("yes")
                            query = f"""
                                    UPDATE kichenTable
                                    SET 
                                        Child = %s,
                                        Adult = %s,
                                        Soup = %s,
                                        Meat = %s,
                                        Soju = %s,
                                        Beer = %s,
                                        Baijiu = %s,
                                        Mushroom = %s,
                                        Seafood = %s
                                    WHERE Id = %s AND DATE(Day) = DATE(NOW())
                                    """
                            values = data[1:]+[data[0]]
                # Prepare data for MySQL
                else :
                    query = "INSERT INTO kichenTable VALUES( NOW(), %s, %s, %s, %s, %s, %s, %s, %s, %s, %s)"
                    """INSERT INTO kichenTable (Id, Child, Adult, Soup, Meat, Soju, Beer, Baijiu, Mushroom, Seafood)
                                VALUES (NOW(),%s, %s, %s, %s, %s, %s, %s, %s, %s, %s)
                                ON DUPLICATE KEY UPDATE 
                                Child=VALUES(Child), Adult=VALUES(Adult), Soup=VALUES(Soup),
                                Meat=VALUES(Meat), Soju=VALUES(Soju), Beer=VALUES(Beer),
                                Baijiu=VALUES(Baijiu), Mushroom=VALUES(Mushroom), Seafood=VALUES(Seafood);"""
                    values = data
                    self.empty.append(table_id)



                cur.execute(query, values)
                conn.commit()
                cur.close()
                print(self.temp)

                
                QtWidgets.QMessageBox.information(self, "Success", "Order uploaded to MySQL.")
            except Exception as e:
                QtWidgets.QMessageBox.critical(self, "MySQL Error", f"Failed to upload order: {e}")

            # 서비스 응답 설정
            if self.ros_node:
                self.ros_node.set_service_response("ok")

            # 버튼 숨기기
            if self.listWidgetWaiting.count() == 0:
                self.pushButtonOk.setVisible(False)
                self.pushButtonSorry.setVisible(False)

    def onSorryClicked(self):
        """Sorry 버튼 클릭 시 현재 대기 목록의 첫 번째 주문에 'sorry' 응답"""
        if self.listWidgetWaiting.count() > 0:
            self.listWidgetWaiting.takeItem(0)

            self.response_queue.put("sorry")

            # 버튼 숨기기
            if self.listWidgetWaiting.count() == 0:
                self.pushButtonOk.setVisible(False)
                self.pushButtonSorry.setVisible(False)

    #####################################################################
    # 조리 중 - Go 버튼
    #####################################################################
    def onGoClicked(self, orderText: str):
        if self._batteryLevel <= 50 or self._amrState in ["running", "error"]:
            QtWidgets.QMessageBox.warning(self, "AMR 상태", "배터리가 50% 이하이거나 (running/error) 상태이므로 이동 불가")
            return

        # 이동 시작
        self._amrState = "running"
        self._batteryWidget.setBatteryState(self._amrState)

        # ROS → waypoint_node(테이블 번호) 전송
        if self.ros_node:
            m = re.search(r"테이블\s+(\d+)", orderText)
            table_num = -1
            if m:
                table_num = int(m.group(1))
            self.ros_node.publishWaypointGoal(table_num)



    #####################################################################
    # AMR 노드 콜백 (done=true 들어왔을 때)
    #####################################################################
    def onAMRNodeCallback(self, received_bool: bool):
        if received_bool:
            if self.listWidgetCooking.count() > 0:
                self.listWidgetCooking.takeItem(0)
            self._amrState = "done"
            self._amrRunning = False

            self._batteryWidget.setBatteryState(self._amrState)

    #####################################################################
    # waypoint_node에서 waiting(arrived) 들어왔을 때 → AMR charging
    #####################################################################
    def setAMRState(self, state: str):
        self._amrState = state
        self._batteryWidget.setBatteryState(state)

    #####################################################################
    # 배터리 소모/충전 타이머
    #####################################################################
    def onTimer(self):
        # 날짜/시간 업데이트
        now = QtCore.QDateTime.currentDateTime()
        date_str = now.toString("yyyy-MM-dd (ddd) HH:mm:ss")
        self.labelDateTime.setText(date_str)

        # 배터리 소모/충전
        if self._amrState == "running":
            self._batteryLevel = max(0, self._batteryLevel - 0.5)
            self._chargingStartTime = 0
            if self._batteryLevel == 0:
                self._amrState = "error"
        elif self._amrState == "done":
            self._batteryLevel = max(0, self._batteryLevel - 0.5)
            if self._batteryLevel == 0:
                self._amrState = "error"
        elif self._amrState == "waiting":
            if self._chargingStartTime == 0:
                self._chargingStartTime = time.time()
            elapsed = time.time() - self._chargingStartTime
            # 3초 후 1%씩 충전
            if elapsed > 3:
                self._batteryLevel = min(100, self._batteryLevel + 1)

        self._batteryWidget.setBatteryLevel(self._batteryLevel)
        self._batteryWidget.setBatteryState(self._amrState)
        if hasattr(self, 'labelRobotState'):
            self.labelRobotState.setText(self._amrState)

    #####################################################################
    # 16:9 비율 유지 (선택사항)
    #####################################################################
    def resizeEvent(self, event):
        desired_ratio = 16 / 9
        w = self.width()
        h = self.height()
        current_ratio = w / h
        if abs(current_ratio - desired_ratio) > 0.01:
            if current_ratio > desired_ratio:
                new_w = int(h * desired_ratio)
                self.resize(new_w, h)
            else:
                new_h = int(w / desired_ratio)
                self.resize(w, new_h)
        super().resizeEvent(event)

###############################################################################
# (6) 메인 실행 함수
###############################################################################
def main():

    order_queue = queue.Queue()
    response_queue = queue.Queue()

    # Qt 애플리케이션 및 GUI 생성
    app = QtWidgets.QApplication(sys.argv)
    window = CounterDP(order_queue, response_queue)
    window.show()

    # ROS 노드를 별도 스레드에서 실행
    ros_thread = threading.Thread(target=ros_thread_main, args=(window, order_queue, response_queue), daemon=True)
    ros_thread.start()



    # Qt 실행 루프
    exit_code = app.exec_()

    # ROS 스레드 정리 (daemon=True 이므로 앱 종료 시 자동 종료)
    sys.exit(exit_code)

    '''
    # ROS 초기화
    rclpy.init()

    app = QtWidgets.QApplication(sys.argv)
    window = CounterDP()
    window.show()

    # ROS 노드
    ros_node = CounterRosNode(window)
    window.setRosNode(ros_node)

    # spin_once 주기적 호출
    spin_timer = QtCore.QTimer()
    spin_timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0.01))
    spin_timer.start(50)

    exit_code = app.exec_()

    ros_node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)
    '''

if __name__ == "__main__":
    main()
