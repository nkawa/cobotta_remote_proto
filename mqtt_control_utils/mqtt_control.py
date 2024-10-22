"""
制御の共通インターフェース化検討例となるモジュール。
マルチプロセスの並列処理には対応していないなどの理由で、
現状、以下のロボットのみ対応している。
- Jaka Zu 5s
- Doosan A0509s
マルチプロセスの並列処理を考慮すると、インターフェースの設計から
見直しが必要な可能性あり。
"""
import tkinter as tk
from tkinter import *
from threading import Thread

from .mqtt_feeder.mqtt_feeder import MQTTFeeder


class RobotControl:
    """
    ロボット制御のインターフェース。
    ros2_controlを参考にした。
    受信した目標値から制御値を計算する処理と、ロボットに制御値を指令する処理が
    同一スレッドになるため、Pythonで実現するのは難しいかもしれない。
    """
    def on_activate(self):
        raise NotImplementedError
    def on_deactivate(self):
        raise NotImplementedError
    def on_target(self):
        raise NotImplementedError
    def on_timer(self):
        pass

class MQTTControl:
    """MQTTによる制御のエントリーポイント。"""
    def __init__(
        self,
        robot_control: RobotControl,
        mqtt_feeder: MQTTFeeder,
    ):
        self.robot_control = robot_control
        self.mqtt_feeder = mqtt_feeder

    def __del__(self):
        self.mqtt_feeder.stop()
        self.robot_control.on_deactivate()

    def robot_connect(self):
        self.robot_control.on_activate()

    def mqtt_connect(self):
        # 目標軌道生成ループ（別制御ループがない場合はここで制御も行う）
        self.mqtt_feeder.set_on_message(self.robot_control.on_target)
        self.mqtt_feeder.start()
        # 別制御ループ
        self.timer = Thread(target=self.robot_control.on_timer, daemon=True)
        self.timer.start()

    def mainloop(self):
        self.robot_connect()
        self.mqtt_connect()
        self.mqtt_feeder.join()

class MQTTWin:
    def __init__(self, root, control: MQTTControl):
        """
        MQTTによる制御のGUIクラス例。

        NOTE: 基本的な制御ロジックはControlクラスが担うが、
        GUIの要素はロボットによって変わる可能性があるため、
        ロボットによって異なるGUIクラスのサブクラス化が必要になる可能性あり。
        """
        self.control = control
        self.root = root
        self.root.title(f"MQTT-{self.control.robot.name} Controller")
        self.root.geometry("600x800")

        self.mqbutton = Button(self.root,
                               text=f"Connect {self.control.robot.name}",
                               padx=5, command=self.robot_connect)
        self.mqbutton.grid(row=0,column=0,padx=2,pady=10)

        self.mqbutton = Button(self.root, text="Connect MQTT", padx=5,
                               command=self.target_connect)
        self.mqbutton.grid(row=0,column=1,padx=2,pady=10)

    def robot_connect(self):
        self.control.robot_connect()

    def target_connect(self):
        self.control.mqtt_connect()

def run_mqtt_win_control(control: MQTTControl) -> None:
    """共通インタフェースによる制御のGUI実行例。"""
    root = tk.Tk()
    mqwin = MQTTWin(root, control)
    mqwin.root.lift()
    root.mainloop()
