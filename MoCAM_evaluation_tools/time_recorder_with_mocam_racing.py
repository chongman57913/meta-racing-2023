from PyQt5.QtWidgets import QApplication, QWidget, QHBoxLayout, QVBoxLayout, QLabel, QPushButton
from PyQt5.QtCore import QTimer, QProcess
import sys
# from mpc_Town04_1 import mpc_path_tracking

class Timer(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()
        # self.mpc_path_tracking=mpc_path_tracking()
        # self.mpc_path_tracking.controller()

    def initUI(self):
        # 初始化
        self.time = 0
        self.isCounting = False

        # 显示时间的标签
        self.timeLabel = QLabel("00:00:00")
        self.timeLabel.setStyleSheet("font-size: 60px")

        # 开始计时按钮
        self.startButton = QPushButton("开始计时")
        self.startButton.setStyleSheet("font-size: 30px")
        self.startButton.clicked.connect(self.startTimer)

        # 停止计时按钮
        self.stopButton = QPushButton("停止计时")
        self.stopButton.setStyleSheet("font-size: 30px")
        self.stopButton.clicked.connect(self.stopTimer)

        # 保存时间按钮
        self.saveButton = QPushButton("保存时间")
        self.saveButton.setStyleSheet("font-size: 30px")
        self.saveButton.clicked.connect(self.saveTime)

        # 水平布局
        hbox = QHBoxLayout()
        hbox.addWidget(self.startButton)
        hbox.addWidget(self.stopButton)
        hbox.addWidget(self.saveButton)

        # 垂直布局
        vbox = QVBoxLayout()
        vbox.addWidget(self.timeLabel)
        vbox.addLayout(hbox)

        self.setLayout(vbox)

        # 定时器
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.updateTime)
        self.timer.start(1000) 

        # 监听特定脚本启动
        self.process = QProcess(self)
        self.process.started.connect(self.startTimer)
        self.process.finished.connect(self.processFinished) 
        # self.process.start()
        self.process.start("python", ["mpc_Town04_launch.py"])  # 替换成你的特定脚本路径

        # 监听特定脚本启动
        self.process = QProcess(self)
        self.process.started.connect(self.startTimer)
        self.process.finished.connect(self.processFinished) 
        #轮询定时器
        # self.pollTimer = QTimer(self)
        # self.pollTimer.timeout.connect(self.checkVariable)
        # self.pollTimer.start(1000)  # 每秒轮询一次

    # 开始计时
    def startTimer(self):
        self.isCounting = True
        self.timer.start(1000)

    # 停止计时
    def stopTimer(self):
        self.isCounting = False
        self.timer.stop()

    # def checkVariable(self):
    #     # 检查变量是否为True
    #     if self.mpc_path_tracking.flag:
    #         self.stopTimer()
    #         self.saveTime()

    # 更新时间
    def updateTime(self):
        try:
            self.time += 1
            self.updateTimeLabel()
        except KeyboardInterrupt:
            self.stopTimer()
            sys.exit(0)

    # 更新时间标签
    def updateTimeLabel(self):
        hours = int(self.time / 3600)
        minutes = int((self.time % 3600) / 60)
        seconds = self.time % 60
        self.timeLabel.setText(f"{hours:02d}:{minutes:02d}:{seconds:02d}")

    # 保存时间
    def saveTime(self):
        if self.isCounting:
            self.stopTimer()
        savedTime = self.time
        self.time = 0
        self.updateTimeLabel()
        print(f"Saved time: {savedTime} seconds")

    def processFinished(self):
        if self.isCounting:
            self.stopTimer()
        sys.exit(0)

if __name__ == '__main__':
    try:
        app = QApplication([])
        timer = Timer()
        timer.show()
        app.exec_()
    except KeyboardInterrupt:
        sys.exit(0)
