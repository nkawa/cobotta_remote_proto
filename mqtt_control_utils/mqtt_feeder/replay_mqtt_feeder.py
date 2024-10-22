from threading import Event, Thread
import time
from typing import Any, Callable, Dict, List

from .mqtt_feeder import MQTTFeeder


class ReplayMQTTFeeder(MQTTFeeder):
    def __init__(
        self,
        jss: List[Dict[str, Any]],
    ) -> None:
        """
        指定した時刻を狙って、実際の時刻と目標位置を生成し、
        コールバックへ渡す。コールバックの処理時間は指定した時刻の間隔よりも
        短いことを想定している。
        """
        self.jss = jss
        self.callback = lambda js: None
        self.event = Event()

    def set_on_message(
        self,
        callback: Callable[[Dict[str, Any]], None]
    ) -> None:
        self.callback = callback

    def start(self):
        self.thread = Thread(target=self._generate, daemon=True)
        self.thread.start()

    def join(self):
        self.thread.join() 

    def stop(self):
        self.event.set()
        self.thread.join()

    def _generate(self):
        self._generate_by_diff()

    def _generate_by_diff(self):
        is_first = True
        for js in self.jss:
            if is_first:
                j_prev = js["time"]
                t_prev = time.time()
                is_first = False
            else:
                j = js["time"]
                while True:
                    t = time.time()
                    if t - t_prev >= j - j_prev:
                        break
                j_prev = j
                t_prev = t
            self.callback(js, client=None, userdata=None, msg=None)
            if self.event.is_set():
                break

    def _generate_from_start(self):
        t_start = time.time()
        j_start = None
        for js in self.jss:
            if j_start is None:
                j_start = js["time"]
            while True:
                t = time.time()
                if t - t_start >= js["time"] - j_start:
                    break
            self.callback(js, client=None, userdata=None, msg=None)
            if self.event.is_set():
                break
