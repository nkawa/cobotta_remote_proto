"""ログ用モジュール。"""
import logging
import logging.handlers
from multiprocessing import Process
import multiprocessing
import sys
from typing import Any, Dict, List, Union


def show_log(
    modules: Union[str, List[str]],
    loglevels: Union[str, List[str]],
    handlers: Dict[str, Dict[str, Any]] = {"StreamHandler": {}},
    format: str = '[%(asctime)s][%(name)s][%(levelname)s] - %(message)s',
) -> None:
    """
    モジュール単位でのログ表示管理を行う。
    ハンドラー、フォーマットはすべて共通。
    """
    if modules in ["all", ["all"]] :
        if isinstance(loglevels, list):
            loglevels = loglevels[0]
        logging.basicConfig(
            format=format,
            level=loglevels,
            stream=sys.stdout,
        )
        return
    if isinstance(modules, str):
        modules = [modules]

    if isinstance(loglevels, str):
        loglevels = [loglevels] * len(modules)
    else:
        if len(loglevels) == 1:
            loglevels = [loglevels[0]] * len(modules)

    handlers = [handlers] * len(modules)

    for module, loglevel, handler_dict in zip(modules, loglevels, handlers):
        logger = logging.getLogger(module)
        for name, args in handler_dict.items():
            handler = getattr(logging, name)(**args)
            formatter = logging.Formatter(format)
            handler.setFormatter(formatter)
            logger.addHandler(handler)
        logger.setLevel(loglevel)

def show_log_mp(
    log_queue: multiprocessing.Queue,
    modules: Union[str, List[str]],
    loglevels: Union[str, List[str]],
    handlers: Dict[str, Dict[str, Any]] = {"StreamHandler": {}},
    format: str = '[%(asctime)s][%(name)s][%(levelname)s] - %(message)s',
) -> logging.handlers.QueueListener:
    """
    モジュール単位でのログ表示管理を行う（並列プロセス用）。
    使い方はtest_show_log_mpを参照。
    ハンドラー、フォーマットはすべて共通。
    """
    if modules in ["all", ["all"]] :
        if isinstance(loglevels, list):
            loglevels = loglevels[0]
        logging.basicConfig(
            format=format,
            level=loglevels,
            stream=sys.stdout,
        )
        return
    if isinstance(modules, str):
        modules = [modules]

    if isinstance(loglevels, str):
        loglevels = [loglevels] * len(modules)
    else:
        if len(loglevels) == 1:
            loglevels = [loglevels[0]] * len(modules)

    handler_instances = []
    for handler_name in handlers:
        handler_args = handlers[handler_name]
        handler = getattr(logging, handler_name)(**handler_args)
        formatter = logging.Formatter(format)
        handler.setFormatter(formatter)
        handler_instances.append(handler)
    listener = logging.handlers.QueueListener(log_queue, *handler_instances)

    for module, loglevel in zip(modules, loglevels):
        logger = logging.getLogger(module)
        handler = logging.handlers.QueueHandler(log_queue)
        logger.addHandler(handler)
        logger.setLevel(loglevel)

    return listener

def example_show_log_mp():
    """
    show_log_mpの使用例

    参考:

    - [logging.handlers --- ログ記録ハンドラー](https://docs.python.org/ja/3/library/logging.handlers.html)
    - [マルチプロセス環境でのファイルへのロギングについて](https://qiita.com/JuvenileTalk9/items/cf65ccb0e12ff14dc7c1)
    - [【Python】マルチプロセス処理のログを1ファイルにまとめる方法](https://shun-studio.com/python/multiprocess-single-log/)

    lib.py:

    ```python
    import logging

    logger = logging.getLogger(__name__)

    def log():
        logger.info("log_info")
        logger.warning("log_warning")
    ```

    上記の同階層のlib.pyとして保存することでテスト可能

    $ python log.py 
    [2024-09-10 15:46:05,643][lib][INFO] - log_info
    [2024-09-10 15:46:05,643][lib][WARNING] - log_warning
    [2024-09-10 15:46:05,645][lib][INFO] - log_info
    [2024-09-10 15:46:05,645][lib][WARNING] - log_warning

    繰り返し実行するとたまにフリーズしてしまうため使用には注意
    """
    def f():
        logger = logging.getLogger(__name__)
        logger.info("f_info")
        logger.info("f_warning")
    queue = multiprocessing.Queue()

    listener = show_log_mp(
        log_queue=queue,
        modules=["__main__", "lib"],
        loglevels=["INFO", "DEBUG"],
        handlers={"StreamHandler": {}, "FileHandler": {"filename": "log.txt", "mode": "w"}},
    )

    try:
        from lib import log
    except ImportError:
        print("Please follow the documentation instruction.")
        return
    listener.start()
    log()
    f()
    p = Process(target=log)
    p.start()
    p.join()
    p = Process(target=f)
    p.start()
    p.join()
    listener.stop()

class Recorder:
    """ファイルに姿勢情報を記録するためのクラス"""
    def __init__(self, path: str) -> None:
        self.f = open(path, "w")

    def log(self, log: Dict[str, Any]) -> None:
        self.f.write(str(log).replace("'", '"') + "\n")

    def __del__(self) -> None:
        self.f.close()

if __name__ == "__main__":
    example_show_log_mp()
