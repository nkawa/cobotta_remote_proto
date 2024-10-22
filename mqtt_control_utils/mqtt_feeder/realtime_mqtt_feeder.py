import datetime
import json
import logging
import os
import time
from typing import Any, Callable, Optional

from paho.mqtt import client as mqtt

from .mqtt_feeder import MQTTFeeder


logger = logging.getLogger(__name__)
logger.addHandler(logging.NullHandler())

class RealtimeMQTTFeeder(MQTTFeeder):
    def __init__(
        self,
        host: str = "192.168.207.22",
        port: int = 1883,
        keepalive: int = 60,
        save: bool = False,
        save_path: Optional[str] = None,
    ) -> None:
        """
        MQTTクライアントのラッパークラス

        on_connect, on_disconnect, on_messageには追加のコールバックを指定可能

        引数はそれぞれ
            on_connect: (client, userdata, flag, rc)
            on_disconnect: (client, userdata, rc)
            on_message: (js, client, userdata, msg)
        """
        self.save = save
        self.save_path = save_path
        # MQTTの接続設定
        self.client = mqtt.Client()
        # MQTTの接続
        self.client.connect(host, port, keepalive=keepalive)
        self.set_on_connect()
        self.set_on_disconnect()
        self.set_on_message()

    def __del__(self):
        # 強制終了の時にファイル保存を完了する
        # 別スレッドで実行する場合必ずしも呼ばれるとは限らないので
        # 場合によっては強制終了したいときは明示的にloop_stopを呼ぶ
        self._stop_save()

    def _start_save(self):
        self.save_fd = None
        if self.save:
            if self.save_path is None:
                self.save_path = f'mqtt_{datetime.datetime.now().strftime("%Y%m%d%H%M%S")}.jsonl'
            dirname = os.path.dirname(self.save_path)
            if len(dirname) != 0:
                os.makedirs(dirname, exist_ok=True)
            self.save_fd = open(self.save_path, "w")
            logger.info(f"Start saving MQTT to {self.save_path}")

    def _stop_save(self):
        if self.save_fd is not None:
            self.save_fd.close()
            self.save_fd = None
            logger.info(f"Stop saving MQTT to {self.save_path}")

    def set_on_connect(
        self,
        on_connect: Optional[Callable[[Any], None]] = None,
    ) -> Callable[[Any], None]:
        def on_connect_base(client, userdata, flag, rc):
            # connected -> subscribe
            logger.info("Connected with result code " + str(rc))
            self.client.subscribe("webxr/pose")

        if on_connect is None:
            self.client.on_connect = on_connect_base
            return

        def on_connect_add(client, userdata, flag, rc):
            on_connect_base(client, userdata, flag, rc)
            on_connect(client, userdata, flag, rc)

        self.client.on_connect = on_connect_add

    def set_on_disconnect(
        self,
        on_disconnect: Optional[Callable[[Any], None]] = None,
    ) -> Callable[[Any], None]:
        def on_disconnect_base(client, userdata, rc):
            if rc != 0:
                logger.warning("Unexpected disconnection.")

        if on_disconnect is None:
            self.client.on_disconnect = on_disconnect_base
            return

        def on_disconnect_add(client, userdata, rc):
            on_disconnect_base(client, userdata, rc)
            on_disconnect(client, userdata, rc)

        self.client.on_disconnect = on_disconnect_add

    def set_on_message(
        self,
        on_message: Optional[Callable[[Any], None]] = None,
    ) -> Callable[[Any], None]:
        def on_message_base(client, userdata, msg):
            js = json.loads(msg.payload)
            logger.debug("Message: " + json.dumps(js))
            return js

        def on_message_save(client, userdata, msg):
            js = on_message_base(client, userdata, msg)
            # https://stackoverflow.com/questions/1938048/high-precision-clock-in-python
            # time.timeの時間自体の精度はLinuxとMacで+- 1 us or 0.001 ms、Windowsで+- 16 ms
            # その値を表現する精度は、Python float (= C double)の有効桁数は15桁、
            # UNIX時間は2024年現在小数点以上10桁なので、残り小数点以下5桁の精度は0.1msはあるはず
            t = time.time()
            js["time"] = t
            if self.save_fd is not None:
                s = json.dumps(js)
                self.save_fd.write(s + "\n")
                logger.debug("JSON: " + s)
            return js

        if on_message is None:
            if not self.save:
                self.client.on_message = on_message_base
            else:
                self.client.on_message = on_message_save
            return

        def on_message_add(client, userdata, msg):
            js = on_message_base(client, userdata, msg)
            on_message(js, client, userdata, msg)

        def on_message_save_add(client, userdata, msg):
            js = on_message_save(client, userdata, msg)
            on_message(js, client, userdata, msg)

        if not self.save:
            self.client.on_message = on_message_add
        else:
            self.client.on_message = on_message_save_add

    def start(self):
        """
        通信処理開始(ノンブロッキング)
        """
        self._start_save()
        self.client.loop_start()

    def forever(self):
        """
        通信処理開始(ブロッキング)
        """
        self._start_save()
        self.client.loop_forever()

    def join(self):
        while True:
            try:
                time.sleep(1)
            except KeyboardInterrupt:
                break
        self.client.loop_stop()
        self._stop_save()

    def stop(self):
        self.client.loop_stop()
        self._stop_save()
