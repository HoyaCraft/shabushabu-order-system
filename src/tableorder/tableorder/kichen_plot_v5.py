import sys
import numpy as np
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import Qt
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import pymysql
from collections import defaultdict

conn = pymysql.connect(
    host='localhost',
    port=3306,
    user='shin', 
    passwd='newpassword', 
    db='kichen', 
    charset='utf8',
    autocommit=True
    )

class MyWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setupUI()
        self.row = None
        self.cur = conn.cursor()
        self.temp_list = []
        self.soju_list = []
        self.beer_list = []
        self.baijiu_list = []
        self.meat_list = []
        self.seafood_list = []
        self.mushroom_list=[]
        self.people_check = False
        self.meat_check = False
        self.soju_check = False
        self.beer_check = False
        self.baijiu_check = False
        self.mushroom_check = False
        self.seafood_check = False
        self.plot_flag = False
        self.group_by = "month"

    def setupUI(self):
        self.setGeometry(600, 200, 1200, 600)  # 창 크기 유지
        self.setWindowTitle("매출 분석 그래프")
        self.setWindowIcon(QIcon('icon.png'))

        # 그래프를 표시할 FigureCanvas 추가
        self.fig = plt.Figure()
        self.canvas = FigureCanvas(self.fig)
        self.canvas.setParent(self)
        self.canvas.setGeometry(50, 50, 800, 500)  # 그래프 창 위치와 크기 유지

        # 버튼 추가 및 위치 조정
        self.money_graph = QPushButton("매출", self)
        self.money_graph.move(875, 350)  # 버튼 위치 설정
        self.money_graph.setFixedSize(130, 200)  # 버튼 크기 설정
        self.money_graph.clicked.connect(self.pushButtonClicked1)

        self.much_graph = QPushButton("판매 수량", self)
        self.much_graph.move(1045, 350)  # 버튼 위치 설정
        self.much_graph.setFixedSize(130, 200)  # 버튼 크기 설정
        self.much_graph.clicked.connect(self.pushButtonClicked2)

        ####################    meat    #######################
        self.meat = QCheckBox("meat", self)
        self.meat.move(875, 100)
        self.meat.setChecked(False)

        font = self.meat.font()
        font.setPointSize(15)
        self.meat.setFont(font)

        self.meat.setStyleSheet("QCheckBox::indicator { width: 30px; height: 30px; }")
        self.meat.stateChanged.connect(self.MeatCheck)

        ####################    mushroom    #######################
        self.mushroom = QCheckBox("mushroom", self)
        self.mushroom.move(875, 150)
        self.mushroom.setChecked(False)

        font = self.mushroom.font()
        font.setPointSize(15)
        self.mushroom.setFont(font)

        self.mushroom.setStyleSheet("QCheckBox::indicator { width: 30px; height: 30px; }")
        self.mushroom.stateChanged.connect(self.MushroomCheck)

        ####################    seafood    #######################
        self.seafood = QCheckBox("seafood", self)
        self.seafood.move(875, 200)
        self.seafood.setChecked(False)

        font = self.seafood.font()
        font.setPointSize(15)
        self.seafood.setFont(font)

        self.seafood.setStyleSheet("QCheckBox::indicator { width: 30px; height: 30px; }")
        self.seafood.stateChanged.connect(self.SeafoodCheck)


        ####################    soju    #######################
        self.soju = QCheckBox("soju", self)
        self.soju.move(875, 100)
        self.soju.setChecked(False)

        font = self.soju.font()
        font.setPointSize(15)
        self.soju.setFont(font)

        self.soju.setStyleSheet("QCheckBox::indicator { width: 30px; height: 30px; }")
        self.soju.stateChanged.connect(self.SojuCheck)

        ####################    beer    #######################
        self.beer = QCheckBox("beer", self)
        self.beer.move(875, 150)
        self.beer.setChecked(False)

        font = self.beer.font()
        font.setPointSize(15)
        self.beer.setFont(font)

        self.beer.setStyleSheet("QCheckBox::indicator { width: 30px; height: 30px; }")
        self.beer.stateChanged.connect(self.BeerCheck)

        ####################    baijiu    #######################
        self.baijiu = QCheckBox("baijiu", self)
        self.baijiu.move(875, 200)
        self.baijiu.setChecked(False)

        font = self.baijiu.font()
        font.setPointSize(15)
        self.baijiu.setFont(font)

        self.baijiu.setStyleSheet("QCheckBox::indicator { width: 30px; height: 30px; }")
        self.baijiu.stateChanged.connect(self.BaijiuCheck)

        ####################    ma_combobox    #########################
        self.ma_com = QComboBox(self)
        self.ma_com.addItem('Main menu')
        self.ma_com.addItem('Alcohol')
        self.ma_com.move(875, 50)
        self.ma_com.currentIndexChanged.connect(self.onComboBoxChanged)
        self.onComboBoxChanged(0)

        ####################    time_combobox    #########################
        self.day_com = QComboBox(self)
        self.day_com.addItem('Month')
        self.day_com.addItem('Day')
        self.day_com.addItem('Hour')
        self.day_com.move(1045, 50)
        self.day_com.currentIndexChanged.connect(self.DaySelect)
        self.onComboBoxChanged(0)

    def onComboBoxChanged(self, index):
        # 콤보박스에서 선택된 인덱스를 확인하여 체크박스를 보이거나 숨김
        if self.ma_com.currentIndex() == 1:
            self.beer.setVisible(True)
            self.soju.setVisible(True)   # 체크박스 보임
            self.baijiu.setVisible(True)
            self.meat.setVisible(False)  # 체크박스 숨김
            self.mushroom.setVisible(False)
            self.seafood.setVisible(False)
            self.plot_flag = False

        else:
            self.soju.setVisible(False)  # 체크박스 숨김
            self.beer.setVisible(False)
            self.baijiu.setVisible(False)
            self.meat.setVisible(True)
            self.mushroom.setVisible(True)   # 체크박스 보임
            self.seafood.setVisible(True)
            self.plot_flag = True

    def DaySelect(self, index):
        # 콤보박스에서 선택된 인덱스를 확인하여 체크박스를 보이거나 숨김
        self.group_by = "month"  # 기본값 설정
        if self.day_com.currentIndex() == 1:
            self.group_by = "day"  # 월 기준 그룹화
        elif self.day_com.currentIndex() == 2:
            self.group_by = "hour" 

    def MeatCheck(self, state) :
        self.meat_check = state == Qt.Checked
        #print("meat change : ",self.meat_check)

    def MushroomCheck(self, state) :
        self.mushroom_check = state == Qt.Checked
        #print("mushroom change : ",self.mushroom_check)

    def SeafoodCheck(self, state) :
        self.seafood_check = state == Qt.Checked
        #print("seafood change : ",self.seafood_check)

    def SojuCheck(self, state) :
        self.soju_check = state == Qt.Checked
        #print("soju change : ",self.soju_check)

    def BeerCheck(self, state) :
        self.beer_check = state == Qt.Checked
        #print("beer change : ",self.beer_check)

    def BaijiuCheck(self, state) :
        self.baijiu_check = state == Qt.Checked
        #print("baijiu change : ",self.baijiu_check)
        ###########################################################

    from collections import defaultdict

    def RefreshCursor(self) :
        self.cur = conn.cursor()

    def pushButtonClicked2(self):
        self.RefreshCursor()
        self.fig.clear()
        ax = self.fig.add_subplot(111)  # 새로운 subplot 추가
        self.PlotListReset()

        # 날짜별 데이터를 저장할 딕셔너리
        plotdic = defaultdict(lambda: defaultdict(int))  # {day: {category: sum}}
        categories = ['meat', 'seafood', 'mushroom', 'soju', 'beer', 'baijiu']

        self.cur.execute("SELECT day, meat, soju, beer, baijiu, mushroom, seafood FROM kichenTable")
        while True:
            self.row = self.cur.fetchone()
            if self.row is None:
                break

            month, day, hour = self.DaySplit(self.row[0])
            if self.group_by == "day":
                key = day
            elif self.group_by == "hour":
                key = hour
            else: 
                key = month

            if self.meat_check:
                plotdic[key]['meat'] += self.row[1]
            if self.seafood_check:
                plotdic[key]['seafood'] += self.row[6]
            if self.mushroom_check:
                plotdic[key]['mushroom'] += self.row[5]
            if self.soju_check:
                plotdic[key]['soju'] += self.row[2]
            if self.beer_check:
                plotdic[key]['beer'] += self.row[3]
            if self.baijiu_check:
                plotdic[key]['baijiu'] += self.row[4]

        x = sorted(plotdic.keys())
        x_pos = range(len(x))
        namelist = [key for key in categories if any(plotdic[day][key] > 0 for day in x)]  # 선택된 카테고리 확인

        if not namelist:
            print("No data to plot.")
            return

        bar_width = min(0.8 / len(namelist), 0.2)

        for i, name in enumerate(namelist):
            y_values = [plotdic[day][name] for day in x]
            ax.bar(
                [pos + (i - len(namelist) / 2) * bar_width for pos in x_pos],
                y_values,
                bar_width,
                alpha=0.5,
                label=name
            )

        ax.set_xticks(x_pos)
        ax.set_xticklabels(x)  # x축 레이블을 날짜로 설정
        ax.set_xlabel("Day")
        ax.set_ylabel("Count")
        ax.set_title("Sales Analysis by Day")
        ax.legend()
        self.canvas.draw()

    def DaySplit(self, data):
        month = data.strftime("%m")
        day = data.strftime("%d")
        hour = data.strftime("%H")
        return month, day, hour


    def PlotListReset(self) :
        self.id_list = []
        self.soju_list = []
        self.beer_list = []
        self.baijiu_list = []
        self.meat_list = []
        self.seafood_list = []
        self.mushroom_list=[]
        
    def pushButtonClicked1(self):
        self.RefreshCursor()
        self.fig.clear()
        ax = self.fig.add_subplot(111)  # 단일 subplot 생성
        self.PlotListReset()

        # 날짜별 데이터를 저장할 딕셔너리
        plotdic = defaultdict(lambda: defaultdict(int))  # {day: {category: sum}}
        total_sales = defaultdict(int)  # {key: 총 매출}
        prices = {
            'meat': 10000,
            'seafood': 8000,
            'mushroom': 6000,
            'soju': 4000,
            'beer': 4000,
            'baijiu': 10000,
        }
        categories = ['meat', 'seafood', 'mushroom', 'soju', 'beer', 'baijiu']

        # SQL 쿼리 실행
        self.cur.execute("SELECT day, meat, soju, beer, baijiu, mushroom, seafood FROM kichenTable")
        
        while True:
            self.row = self.cur.fetchone()
            if self.row is None:
                break

            # 날짜 추출
            month, day, hour = self.DaySplit(self.row[0])
            if self.group_by == "day":
                key = day
            elif self.group_by == "hour":
                key = hour
            else:
                key = month

            # 각 카테고리에 대해 선택된 항목만 합산
            for category in categories:
                if getattr(self, f"{category}_check"):
                    plotdic[key][category] += self.row[categories.index(category) + 1] * prices[category]
                    total_sales[key] += self.row[categories.index(category) + 1] * prices[category]

        # x축 (날짜/시간/월) 설정
        x = sorted(plotdic.keys())  # 날짜 정렬
        x_pos = range(len(x))
        namelist = [key for key in categories if any(plotdic[day][key] > 0 for day in x)]  # 선택된 카테고리 확인

        if not namelist:
            print("No data to plot.")
            return

        bar_width = min(0.8 / len(namelist), 0.2)  # 막대 폭 동적 계산

        # 막대그래프: 카테고리별 매출
        for i, name in enumerate(namelist):
            y_values = [plotdic[day][name] for day in x]
            ax.bar(
                [pos + (i - len(namelist) / 2) * bar_width for pos in x_pos],
                y_values,
                bar_width,
                alpha=0.7,
                label=name
            )

        # 꺾은선그래프: 총 매출
        line_values = [total_sales[day] for day in x]
        ax.plot(x_pos, line_values, marker="o", color="red", label="Total Sales", linewidth=2)

        # 그래프 설정
        ax.set_xticks(x_pos)
        ax.set_xticklabels(x)
        ax.set_xlabel(self.group_by.capitalize())
        ax.set_ylabel("Sales (₩)")
        ax.set_title("Sales Analysis")
        ax.legend()

        # 그래프 업데이트
        self.canvas.draw()



    def closeEvent(self, event):
        conn.close()
        print("closing")
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MyWindow()
    window.show()
    app.exec_()
