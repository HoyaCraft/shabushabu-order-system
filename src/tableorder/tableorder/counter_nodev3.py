import sys
import re
import time
import ast
from collections import defaultdict

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
from std_msgs.msg import String, Int32
from service_interface.srv import MenuOrder
import threading
import queue
import pymysql  # 주석 그대로 유지

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
        self.publisher_refresh = self.create_publisher(
            Int32,
            '/table_refresh',
            10
        )


        # 서비스
        self.service = self.create_service(MenuOrder, 'order_service', self.handle_order_service)

        self.response_event = threading.Event()
        self.current_response = None

        self.get_logger().info("Service 'order_service' is ready.")

    def handle_order_service(self, request, response):
        order = request.menu
        self.get_logger().info(f"Received order: {order}")
        self.order_queue.put(order)

        self.response_event.clear()
        self.current_response = None

        self.get_logger().info("Waiting for user response (ok/sorry)...")

        try:
            user_response = self.response_queue.get(timeout=30)
            response.message = user_response
            self.get_logger().info(f"Responding with: {response.message}")
        except queue.Empty:
            response.message = "sorry"
            self.get_logger().info("No response received. Responding with default: sorry")

        return response

    def set_service_response(self, resp):
        self.current_response = resp
        self.response_event.set()

    def waypointCallback(self, msg: String):
        if msg.data == "done0":
            # QTimer.singleShot(0, ...) 대신 QMetaObject.invokeMethod 사용
            from PySide2.QtCore import QMetaObject, Qt
            
            # 메인 윈도우의 'handleDoneInMainThread' 메서드(슬롯)를 
            # 메인 스레드(QueuedConnection)에서 호출해달라고 요청
            QMetaObject.invokeMethod(
                self.window,
                "handleDoneInMainThread",
                Qt.QueuedConnection
            )
            
        elif msg.data == "waiting":
            from PySide2.QtCore import QMetaObject, Qt
            QMetaObject.invokeMethod(
                self.window,
                "handleWaitingInMainThread",
                Qt.QueuedConnection
            )
            
        elif msg.data == "running":
            from PySide2.QtCore import QMetaObject, Qt
            QMetaObject.invokeMethod(
                self.window,
                "handleRunningInMainThread",
                Qt.QueuedConnection
            )


    def publishWaypointGoal(self, table_int: int):
        msg = Int32()
        msg.data = table_int
        self.publisher_waypoint.publish(msg)
        self.get_logger().info(f"[publishWaypointGoal] 테이블(들) = {table_int}")

    def publishRefresh(self, table_int: int):
        msg = Int32()
        msg.data = table_int
        self.publisher_refresh.publish(msg)
        self.get_logger().info(f"[publishRefresh] 테이블(들) = {table_int}")


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

        if self._batteryLevel <= 50:
            color = QtGui.QColor("red")
        elif self._batteryLevel <= 80:
            color = QtGui.QColor("yellow")
        else:
            color = QtGui.QColor("green")

        painter.setBrush(color)
        painter.setPen(QtCore.Qt.NoPen)
        painter.drawEllipse(center, radius, radius)

        painter.setPen(QtCore.Qt.black)
        font = painter.font()
        font.setPointSize(radius // 2)
        painter.setFont(font)
        textRect = QtCore.QRect(center.x() - radius, center.y() - radius, size, size)
        painter.drawText(textRect, QtCore.Qt.AlignCenter, f"{self._batteryLevel}%")

        painter.setPen(QtCore.Qt.white)
        smallerFont = QtGui.QFont(font)
        smallerFont.setPointSize(radius // 4)
        painter.setFont(smallerFont)
        stateRect = QtCore.QRect(center.x() - radius, center.y() - (radius // 3), size, size)
        painter.drawText(stateRect, QtCore.Qt.AlignHCenter | QtCore.Qt.AlignBottom, self._batteryState)

###############################################################################
# (4) "조리 중" 목록 아이템 위젯 (+ 버튼 - 토글)
###############################################################################
class CookingItemWidget(QtWidgets.QWidget):
    # 시그널: (table_num, new_state)
    plusClicked = Signal(object)

    def __init__(self, orderText: str, parent=None):
        super().__init__(parent)
        self._orderText = orderText
        self._table_num = self._parse_table_num(orderText)

        self._plusSelected = False
        self.initUI()

    def _parse_table_num(self, text: str) -> int:
        m = re.search(r"테이블\s+(\d+)", text)
        if m:
            return int(m.group(1))
        return -1

    def initUI(self):
        layout = QtWidgets.QHBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)

        self.buttonPlus = QtWidgets.QPushButton("+", self)
        self.buttonPlus.setFixedSize(60, 60)
        font_btn = self.buttonPlus.font()
        font_btn.setPointSize(20)
        self.buttonPlus.setFont(font_btn)

        # 초기(비선택) 스타일
        self._updatePlusButtonStyle(self._plusSelected)

        self.buttonPlus.clicked.connect(self.onPlusButtonClicked)
        layout.addWidget(self.buttonPlus, stretch=0)

        self.labelOrder = QtWidgets.QLabel(self._orderText, self)
        self.labelOrder.setWordWrap(True)
        self.labelOrder.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Preferred)

        font_label = self.labelOrder.font()
        font_label.setPointSize(20)
        self.labelOrder.setFont(font_label)
        layout.addWidget(self.labelOrder, stretch=1)

        self.setLayout(layout)

    def onPlusButtonClicked(self):
        new_state = not self._plusSelected
        self._plusSelected = new_state
        self._updatePlusButtonStyle(new_state)

        if self._table_num != -1:
            self.plusClicked.emit((self._table_num, self._plusSelected))

    def _updatePlusButtonStyle(self, selected: bool):
        if selected:
            style = """
                QPushButton {
                    color: black;
                    border: 2px solid #555;
                    border-radius: 30px;
                    background-color: #CCFF99; /* 선택 상태 */
                }
            """
        else:
            style = """
                QPushButton {
                    color: black;
                    border: 2px solid #555;
                    border-radius: 30px;
                    background-color: #EEEEEE; 
                }
            """
        self.buttonPlus.setStyleSheet(style)

    def forceKeepSelectedStyle(self):
        style = """
            QPushButton {
                color: black;
                border: 2px solid #555;
                border-radius: 30px;
                background-color: #CCFF99; /* 선택 상태(비활성) */
            }
        """
        self.buttonPlus.setStyleSheet(style)
        self.buttonPlus.setEnabled(False)

    def forceKeepUnselectedStyle(self):
        style = """
            QPushButton {
                color: black;
                border: 2px solid #555;
                border-radius: 30px;
                background-color: #EEEEEE; 
            }
        """
        self.buttonPlus.setStyleSheet(style)
        self.buttonPlus.setEnabled(False)

    def sizeHint(self):
        return QtCore.QSize(400, 100)

###############################################################################
# (팝업) 테이블 정보 다이얼로그
###############################################################################
class TableInfoDialog(QtWidgets.QDialog):
    def __init__(self, parent, table_id, earliest_time, total_orders, total_price, all_orders_list):
        super().__init__(parent)
        self.setWindowTitle(f"테이블 {table_id} 정보")
        self.resize(500, 400)

        self.parentWindow = parent
        self.table_id = table_id
        self.earliest_time = earliest_time
        self.total_orders = total_orders
        self.total_price = total_price
        self.all_orders_list = all_orders_list

        self.setupUI()

    def setupUI(self):
        layout = QtWidgets.QVBoxLayout(self)

        orders_breakdown = ", ".join(f"{menu}:{qty}" for menu, qty in self.total_orders.items())
        info_label = QtWidgets.QLabel(
            f"최초 주문시간: {self.earliest_time}\n"
            f"총 가격: {self.total_price}원\n"
            f"주문 총합: {orders_breakdown} "
        )
        font_info = info_label.font()
        font_info.setPointSize(12)
        info_label.setFont(font_info)
        layout.addWidget(info_label)

        scrollArea = QtWidgets.QScrollArea()
        scrollArea.setWidgetResizable(True)
        areaWidget = QtWidgets.QWidget()
        vlay = QtWidgets.QVBoxLayout(areaWidget)

        for (timeStr, orderDict) in self.all_orders_list:
            order_summary = ", ".join(f"{k}:{v}" for k, v in orderDict.items())
            lbl = QtWidgets.QLabel(f"{timeStr} → {order_summary}")
            lbl.setWordWrap(True)
            vlay.addWidget(lbl)

        scrollArea.setWidget(areaWidget)
        layout.addWidget(scrollArea)

        btnRefresh = QtWidgets.QPushButton("Refresh")
        btnRefresh.clicked.connect(self.onRefreshClicked)
        layout.addWidget(btnRefresh)

    def onRefreshClicked(self):
        self.parentWindow.clearTableData(self.table_id)
        self.close()

        if self.ros_node:
            self.ros_node.publishRefresh(self.table_id)


###############################################################################
# (5) 전체 메인 GUI 로직
###############################################################################
class CounterDP(QtWidgets.QMainWindow, ui_counter_dp.Ui_MainWindow):
    def __init__(self, order_queue, response_queue):
        super().__init__()
        self.setupUi(self)

        self.order_queue = order_queue
        self.response_queue = response_queue

        self.order_timer = QtCore.QTimer()
        self.order_timer.timeout.connect(self.check_new_order)
        self.order_timer.start(100)


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

        # 배터리 위젯
        self._batteryWidget = BatteryWidget(self.batteryFrame)
        vlay = QtWidgets.QVBoxLayout(self.batteryFrame)
        vlay.setContentsMargins(0, 0, 0, 0)
        vlay.addWidget(self._batteryWidget)

        # AMR 상태
        self._batteryLevel = 100
        self._amrState = "waiting"
        self._chargingStartTime = 0

        # 메뉴 DB
        self._originalMenuData = [
            {"name": "Adult",    "price": 12900, "stock": None},
            {"name": "Child",    "price": 5900,  "stock": None},
            {"name": "Soup",     "price": 0,     "stock": None},
            {"name": "Meat",     "price": 4000,  "stock": 100},
            {"name": "Soju",     "price": 4000,  "stock": 100},
            {"name": "Beer",     "price": 4000,  "stock": 100},
            {"name": "Baijiu",   "price": 10000, "stock": 100},
            {"name": "Mushroom", "price": 6000,  "stock": 100},
            {"name": "Seafood",  "price": 8000,  "stock": 100},
        ]
        self._editedMenuData = []

        # 주문 누적
        self.temps = {}
        self.allOrders = {}
        self.earliestTimeMap = {}

        # +버튼 선택된 테이블 순서와 카운트
        self.selected_tables_order = []
        self.selected_counts = defaultdict(int)

        self.initUI()
        self.initSignals()

        # 1초 주기 타이머
        self._timer = QtCore.QTimer(self)
        self._timer.setInterval(1000)
        self._timer.timeout.connect(self.onTimer)
        self._timer.start()

    @QtCore.Slot()
    def handleDoneInMainThread(self):
        # done 처리
        self.onAMRNodeCallback(True)
        self.setAMRState("done")

    @QtCore.Slot()
    def handleWaitingInMainThread(self):
        self.setAMRState("waiting")

    @QtCore.Slot()
    def handleRunningInMainThread(self):
        self.setAMRState("running")


    def setRosNode(self, ros_node):
        self.ros_node = ros_node

    def initUI(self):
        self.setMinimumSize(640, 360)
        self.rebuildMainTabLayout()
        self.loadMenuData()
        self.updateStockList()

        # OK / SORRY 초기 숨김
        self.pushButtonOk.setVisible(False)
        self.pushButtonSorry.setVisible(False)

        # 스타일
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

        self.labelDateTime = QtWidgets.QLabel("", alignment=QtCore.Qt.AlignRight)
        font_dt = self.labelDateTime.font()
        font_dt.setPointSize(12)
        self.labelDateTime.setFont(font_dt)
        rightLayout.addWidget(self.labelDateTime)

        goBatteryLayout = QtWidgets.QHBoxLayout()
        goBatteryLayout.setSpacing(10)

        self.buttonGlobalGo = QtWidgets.QPushButton("Go")
        self.buttonGlobalGo.setFixedSize(100, 100)
        # 활성/비활성은 배경색으로만 구분, 글자는 항상 검은색
        self.buttonGlobalGo.setStyleSheet("""
            QPushButton {
                color: black;  
                border: 2px solid #555;
                border-radius: 50px;
                font-size: 20px;
                background-color: #AAAAAA; /* 초기엔 비활성(회색) */
            }
            QPushButton:disabled {
                background-color: #CCCCCC; 
            }
            QPushButton:enabled {
                background-color: #99FF99; /* 활성 시 연두색 */
            }
        """)
        self.buttonGlobalGo.setEnabled(False)
        self.buttonGlobalGo.clicked.connect(self.onGlobalGoClicked)

        goBatteryLayout.addWidget(self.buttonGlobalGo, alignment=QtCore.Qt.AlignCenter)
        goBatteryLayout.addWidget(self.batteryFrame, alignment=QtCore.Qt.AlignCenter)

        rightLayout.addLayout(goBatteryLayout)

        self.labelRobotState = QtWidgets.QLabel("(로봇 상태)", alignment=QtCore.Qt.AlignCenter)
        font = self.labelRobotState.font()
        font.setPointSize(14)
        self.labelRobotState.setFont(font)
        rightLayout.addWidget(self.labelRobotState)

        self.tableButtonsFrame = QtWidgets.QFrame(self.tabMain)
        self.tableButtonsFrame.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.tableButtonsLayout = QtWidgets.QGridLayout(self.tableButtonsFrame)
        self.tableButtonsLayout.setContentsMargins(5, 5, 5, 5)

        self.tableButtons = {}
        positions = [(i, j) for i in range(3) for j in range(3)]
        num = 1
        for (row, col) in positions:
            btn = QtWidgets.QPushButton(str(num))
            btn.setFixedSize(60, 60)
            btn.setEnabled(False)
            btn.setStyleSheet("""
                QPushButton {
                    border: 2px solid #555;
                    border-radius: 30px;
                    font-size: 16px;
                    background-color: white;
                }
                QPushButton:enabled {
                    background-color: #CCFFCC;
                }
            """)
            btn.clicked.connect(lambda _=False, t=num: self.onTableButtonClicked(t))
            self.tableButtonsLayout.addWidget(btn, row, col)
            self.tableButtons[num] = btn
            num += 1

        rightLayout.addWidget(self.tableButtonsFrame, stretch=1)

        stockContainer = QtWidgets.QWidget()
        stockLayout = QtWidgets.QVBoxLayout(stockContainer)
        lblStock = QtWidgets.QLabel("현재 재고 현황", alignment=QtCore.Qt.AlignCenter)
        stockLayout.addWidget(lblStock)
        stockLayout.addWidget(self.listWidgetStock)
        rightLayout.addWidget(stockContainer, stretch=2)

        mainLayout.addLayout(leftLayout, stretch=3)
        mainLayout.addLayout(rightLayout, stretch=1)

    def initSignals(self):
        self.pushButtonCheckWaiting.clicked.connect(self.onCheckWaiting)
        self.pushButtonOk.clicked.connect(self.onOkClicked)
        self.pushButtonSorry.clicked.connect(self.onSorryClicked)
        self.pushButtonSave.clicked.connect(self.onSaveMenuData)
        self.pushButtonReset.clicked.connect(self.onResetMenuData)

    #####################################################################
    # 에디트 탭
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

    #####################################################################
    # 결제 대기 목록
    #####################################################################
    def check_new_order(self):
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
        now = time.strftime("%H:%M", time.localtime())
        orders_summary = ", ".join([f"{menu_name} : {quantity}" for menu_name, quantity in menu_orders.items()])
        text = f"테이블 {table_num} - {orders_summary}"

        listItem = QtWidgets.QListWidgetItem(text)
        font = listItem.font()
        font.setPointSize(20)
        listItem.setFont(font)
        self.listWidgetWaiting.addItem(listItem)

    def addWaitingOrderFromService(self, msg: str):
        try:
            data = ast.literal_eval(msg)
            id_value = data.get("Id", "")
            if id_value and isinstance(id_value, str):
                match = re.search(r"t(\d+)", id_value)
                if match:
                    table_num = int(match.group(1))
                else:
                    raise ValueError(f"Invalid Id format: {id_value}")
            else:
                raise ValueError("Id 값이 유효하지 않습니다.")

            orders = {k: v for k, v in data.items() if v is not None and k != "Id"}
            self.addWaitingOrder(table_num, orders)
        except Exception as e:
            raise ValueError(f"문자열 파싱 오류: {e}")

    def onOkClicked(self):
        if self.listWidgetWaiting.count() > 0:
            item = self.listWidgetWaiting.item(0)
            btn_orderText = item.text()

            now = time.strftime("%H:%M", time.localtime())
            orderText = f"{now} - {btn_orderText}"

            self.response_queue.put("ok")

            # 재고 차감
            matches = re.findall(r"([\w가-힣]+)\s*:\s*(\d+)", orderText)
            table_match = re.search(r"테이블\s+(\d+)", orderText)
            if table_match:
                table_num = int(table_match.group(1))
                table_id = f"t{table_num}"
            else:
                table_num = -1
                table_id = "Unknown"

            for menu_name, quantity in matches:
                quantity = int(quantity)
                for menu in self._originalMenuData:
                    if menu["name"] == menu_name and menu["stock"] is not None:
                        menu["stock"] = max(0, menu["stock"] - quantity)
                        break

            # 주문 누적
            current_order = {}
            btn_match = re.findall(r"([\w가-힣]+)\s*:\s*(\d+)", btn_orderText)
            for menu_name, qty in btn_match:
                current_order[menu_name] = int(qty)

            now_time_str = time.strftime("%H:%M", time.localtime())
            if table_id not in self.temps:
                self.temps[table_id] = {}
            if table_id not in self.allOrders:
                self.allOrders[table_id] = []
            if table_id not in self.earliestTimeMap:
                self.earliestTimeMap[table_id] = now_time_str

            for menu_name, qty in current_order.items():
                self.temps[table_id][menu_name] = self.temps[table_id].get(menu_name, 0) + qty

            self.allOrders[table_id].append((now_time_str, current_order))

            self.updateStockList()
            self.loadMenuData()

            self.listWidgetWaiting.takeItem(0)
            if self.listWidgetWaiting.count() == 0:
                self.pushButtonOk.setVisible(False)
                self.pushButtonSorry.setVisible(False)
            
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

            # 조리 중 아이템 추가
            cookingWidget = CookingItemWidget(orderText)
            cookingWidget.plusClicked.connect(self.onPlusClicked)
            listItem2 = QtWidgets.QListWidgetItem()
            self.listWidgetCooking.addItem(listItem2)
            self.listWidgetCooking.setItemWidget(listItem2, cookingWidget)
            listItem2.setSizeHint(cookingWidget.sizeHint())

            # 테이블 버튼 활성화
            if table_num in self.tableButtons:
                self.tableButtons[table_num].setEnabled(True)

            if self.ros_node:
                self.ros_node.set_service_response("ok")

            # (MySQL 주석)
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

                temp_order = self.temps[table_id]

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


    def onSorryClicked(self):
        if self.listWidgetWaiting.count() > 0:
            self.listWidgetWaiting.takeItem(0)
            self.response_queue.put("sorry")
            if self.listWidgetWaiting.count() == 0:
                self.pushButtonOk.setVisible(False)
                self.pushButtonSorry.setVisible(False)

    #####################################################################
    # + 버튼 (토글) 신호 => selected_tables_order[] / selected_counts[] 갱신
    #####################################################################
    def onPlusClicked(self, data):
        table_num, new_state = data

        if self._amrState != "waiting":
            return

        if new_state:
            # 선택 On
            self.selected_counts[table_num] += 1
            if self.selected_counts[table_num] == 1:
                self.selected_tables_order.append(table_num)
        else:
            # 해제 Off
            if self.selected_counts[table_num] > 0:
                self.selected_counts[table_num] -= 1
                if self.selected_counts[table_num] == 0:
                    if table_num in self.selected_tables_order:
                        self.selected_tables_order.remove(table_num)
                    del self.selected_counts[table_num]

        if self.selected_tables_order:
            self.buttonGlobalGo.setEnabled(True)
        else:
            self.buttonGlobalGo.setEnabled(False)

    #####################################################################
    # Go 버튼 => selected_tables_order를 이어붙여 int 변환 => ROS 발행
    #####################################################################
    def onGlobalGoClicked(self):
        if not self.selected_tables_order:
            return
        table_str = "".join(str(t) for t in self.selected_tables_order)

        try:
            route_int = int(table_str)
        except ValueError:
            QtWidgets.QMessageBox.warning(self, "오류", f"테이블 경로 변환 실패: {table_str}")
            return

        if self.ros_node:
            self.ros_node.publishWaypointGoal(route_int)

        self.setAMRState("running")
        self.buttonGlobalGo.setEnabled(False)

    #####################################################################
    # done => selected_tables_order[0] 테이블 +버튼눌린 아이템 모두 삭제
    #####################################################################
    def onAMRNodeCallback(self, received_bool: bool):
        if received_bool:
            if self.selected_tables_order:
                table_done = self.selected_tables_order[0]
                self.removeAllCookingItems(table_done)
                self.selected_tables_order.pop(0)
                if table_done in self.selected_counts:
                    del self.selected_counts[table_done]

    def removeAllCookingItems(self, table_num: int):
        """
        +가 눌린(._plusSelected=True) 아이템만 완전히 제거.
        +버튼 Off 상태 아이템은 남겨둠.
        """
        for i in reversed(range(self.listWidgetCooking.count())):
            list_item = self.listWidgetCooking.item(i)
            widget = self.listWidgetCooking.itemWidget(list_item)
            if not widget:
                continue

            if widget._table_num == table_num and widget._plusSelected:
                # 1) takeItem() -> QListWidgetItem를 리스트에서 빼냄
                removed_item = self.listWidgetCooking.takeItem(i)
                # 2) widget.deleteLater()로 위젯 메모리 해제
                if removed_item and widget:
                    widget.deleteLater()

    #####################################################################
    # AMR 상태 바뀔 때 => waiting 아니면 버튼 클릭 막기
    # 이미 +누른 항목은 "선택 색" 그대로 유지
    #####################################################################
    def setAMRState(self, state: str):
        self._amrState = state
        self._batteryWidget.setBatteryState(state)
        if hasattr(self, 'labelRobotState'):
            self.labelRobotState.setText(state)

        self.updatePlusButtonsEnabled()

    def updatePlusButtonsEnabled(self):
        enable_plus = (self._amrState == "waiting")
        for i in range(self.listWidgetCooking.count()):
            item = self.listWidgetCooking.item(i)
            widget = self.listWidgetCooking.itemWidget(item)
            if widget:
                if enable_plus:
                    widget.buttonPlus.setEnabled(True)
                    widget._updatePlusButtonStyle(widget._plusSelected)
                else:
                    if widget._plusSelected:
                        widget.forceKeepSelectedStyle()
                    else:
                        widget.forceKeepUnselectedStyle()

    #####################################################################
    # 배터리 소모/충전
    #####################################################################
    def onTimer(self):
        now = QtCore.QDateTime.currentDateTime()
        date_str = now.toString("yyyy-MM-dd (ddd) HH:mm:ss")
        self.labelDateTime.setText(date_str)

        if self._amrState == "running":
            self._batteryLevel = max(0, self._batteryLevel - 0.5)
            self._chargingStartTime = 0
            if self._batteryLevel == 0:
                self._amrState = "error"
            elif self._batteryLevel <= 30:
                self._amrState = "warning"
        elif self._amrState == "done":
            self._batteryLevel = max(0, self._batteryLevel - 0.5)
            if self._batteryLevel == 0:
                self._amrState = "error"
        elif self._amrState == "waiting":
            if self._chargingStartTime == 0:
                self._chargingStartTime = time.time()
            elapsed = time.time() - self._chargingStartTime
            if elapsed > 3:
                self._batteryLevel = min(100, self._batteryLevel + 1)

        self._batteryWidget.setBatteryLevel(self._batteryLevel)

    #####################################################################
    # 테이블 버튼 클릭 => 팝업
    #####################################################################
    def onTableButtonClicked(self, table_num):
        table_id = f"t{table_num}"
        if table_id not in self.temps or table_id not in self.allOrders:
            return

        earliest_time = self.earliestTimeMap.get(table_id, "-")
        total_orders = self.temps[table_id]

        total_price = 0
        for menu_name, qty in total_orders.items():
            for m in self._originalMenuData:
                if m["name"] == menu_name:
                    total_price += m["price"] * qty

        all_orders_list = self.allOrders[table_id]

        dialog = TableInfoDialog(
            parent=self,
            table_id=table_id,
            earliest_time=earliest_time,
            total_orders=total_orders,
            total_price=total_price,
            all_orders_list=all_orders_list
        )
        dialog.exec_()

    def clearTableData(self, table_id):
        if table_id in self.temps:
            del self.temps[table_id]
        if table_id in self.allOrders:
            del self.allOrders[table_id]
        if table_id in self.earliestTimeMap:
            del self.earliestTimeMap[table_id]

        m = re.search(r"t(\d+)", table_id)
        if m:
            num = int(m.group(1))
            if num in self.tableButtons:
                self.tableButtons[num].setEnabled(False)

    #####################################################################
    # 16:9 비율 유지(선택)
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
# (6) 메인 실행
###############################################################################
def main():
    order_queue = queue.Queue()
    response_queue = queue.Queue()

    app = QtWidgets.QApplication(sys.argv)
    window = CounterDP(order_queue, response_queue)
    window.show()

    ros_thread = threading.Thread(target=ros_thread_main, args=(window, order_queue, response_queue), daemon=True)
    ros_thread.start()

    exit_code = app.exec_()
    sys.exit(exit_code)

if __name__ == "__main__":
    main()
