from typing import Any, Callable, Optional
from abc import ABC, abstractmethod

class MQTTFeeder(ABC):
    """目標値データのフィードを行うクラスのひな型。"""
    @abstractmethod
    def set_on_message(
        self,
        on_message: Optional[Callable[[Any], None]] = None,
    ) -> Callable[[Any], None]:
        pass

    @abstractmethod
    def start(self):
        pass

    @abstractmethod
    def join(self):
        pass

    @abstractmethod
    def stop(self):
        pass
