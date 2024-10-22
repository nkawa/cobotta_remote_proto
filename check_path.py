"""軌跡の評価。軌跡は制御間隔で生成されていることを想定。"""
from argparse import ArgumentDefaultsHelpFormatter, ArgumentParser
import json
import logging
import time

import numpy as np
import pandas as pd

from .denso_robot import DensoRobot
from mqtt_control_utils.log import show_log
from mqtt_control_utils.trajectory import load_mqtt, mqtt_to_traj
from robot.dummy_robot import DummyRobot


logger = logging.getLogger(__name__)


def load_control_dataframe(path: str) -> pd.DataFrame:
    """軌跡の評価結果を読み込むための関数。"""
    jss = load_mqtt(path)
    data = []
    for i, js in enumerate(jss):
        datum = {}
        datum["i"] = i
        datum["kind"] = "t_cmd_start"
        datum["t"] = js["t_cmd_start"]
        x, y, z, xd, yd, zd = js["pose_cmd"]
        datum["x"] = x
        datum["y"] = y
        datum["z"] = z
        datum["xd"] = xd
        datum["yd"] = yd
        datum["zd"] = zd
        data.append(datum)

        datum = {}
        datum["i"] = i
        datum["kind"] = "t_cmd_end"
        datum["t"] = js["t_cmd_end"]
        x, y, z, xd, yd, zd = js["pose_cmd"]
        datum["x"] = x
        datum["y"] = y
        datum["z"] = z
        datum["xd"] = xd
        datum["yd"] = yd
        datum["zd"] = zd
        data.append(datum)

        datum = {}
        datum["i"] = i
        datum["kind"] = "pose_state_from_cmd_at_t_cmd_end"
        datum["t"] = js["t_cmd_end"]
        x, y, z, xd, yd, zd = js["pose_state_from_cmd"]
        datum["x"] = x
        datum["y"] = y
        datum["z"] = z
        datum["xd"] = xd
        datum["yd"] = yd
        datum["zd"] = zd
        data.append(datum)

        datum = {}
        datum["i"] = i
        datum["kind"] = "t_state_start"
        datum["t"] = js["t_state_start"]
        x, y, z, xd, yd, zd = js["pose_state_from_state"]
        datum["x"] = x
        datum["y"] = y
        datum["z"] = z
        datum["xd"] = xd
        datum["yd"] = yd
        datum["zd"] = zd
        data.append(datum)

        datum = {}
        datum["i"] = i
        datum["kind"] = "t_state_end"
        datum["t"] = js["t_state_end"]
        x, y, z, xd, yd, zd = js["pose_state_from_state"]
        datum["x"] = x
        datum["y"] = y
        datum["z"] = z
        datum["xd"] = xd
        datum["yd"] = yd
        datum["zd"] = zd
        data.append(datum)

    df = pd.DataFrame(data)
    df["t"] -= df["t"].iloc[0]
    return df

if __name__ == "__main__":
    parser = ArgumentParser(
        description="軌跡入力に対する評価用プログラム",
        formatter_class=ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--target-path", default="target.jsonl", help="軌跡ファイルへのパス")
    parser.add_argument("--save-path", default="control.jsonl", help="保存ファイルへのパス")
    parser.add_argument("--default-fig", default=-2, type=int, choices=[-2, -3], help="Denso Cobotta Pro 900の形態 (ロボットアームの関節が取る腕・ひじ・手首の形態) の自動設定モード。-2は形態保持重視。-3はエラー回避重視")
    parser.add_argument("--default-servo-mode", default="0x001", choices=["0x001", "0x101", "0x201"], help="Denso Cobotta Pro 900のスレーブモードのモード。0: 0x001、1: 0x101、2: 0x201を指定する")
    parser.add_argument("--use-state-from-state", action="store_true", help="状態取得専用コマンドを実行する。制御への影響は特になし")
    parser.add_argument("--read-spd-acc", action="store_true", help="速度、加速度設定を読み取るか（この速度、加速度は設定値かつスレーブモードには関係がない）")
    parser.add_argument("--log-modules", nargs="*", default=["__main__"], help="ログをとるモジュール")
    parser.add_argument("--loglevels", nargs="*", default="INFO", help="モジュールのログレベル")
    parser.add_argument("--use-dummy", action="store_true", help="デバッグ用にダミーのロボットを使う")
    args = parser.parse_args()

    show_log(args.log_modules, args.loglevels)
    target_poses = mqtt_to_traj(load_mqtt(args.target_path))[1]
    default_servo_mode = int(args.default_servo_mode, base=16)
    try:
        if args.use_dummy:
            robot = DummyRobot(
                servo_control_mode="abs",
                command_time=0.008,
            )
        else:
            robot = DensoRobot(
                default_servo_mode=default_servo_mode,
                default_fig=args.default_fig,
            )

        robot.start()
        robot.enable()
        robot.move_default_pose_until_completion()

        default_pose = robot.get_default_pose()
        default_pose = np.asarray(default_pose)

        robot.enter_servo_mode()
        logger.info("Loop begin")

        f = open(args.save_path, "w")
        for target_pose in target_poses:
            control_pose = (default_pose + np.asarray(target_pose)).tolist()
            logger.debug(("control_pose", control_pose))
            t_cmd_start = time.time()
            t_from_cmd, state_pose_from_cmd = robot.move_pose_servo(control_pose)
            t_cmd_end = time.time()
            if args.use_state_from_state:
                t_state_start = time.time()
                state_pose_from_state = robot.get_current_pose()
                t_state_end = time.time()
            if args.read_spd_acc:
                print(robot.cur_spd())
                print(robot.cur_acc())
                print(robot.cur_ext_spd())
                print(robot.cur_ext_acc())
            logger.debug(("t_datum_start", time.time()))
            datum = dict(
                t_cmd_start=t_cmd_start,
                t_cmd_end=t_cmd_end,
                t_from_cmd=t_from_cmd,
                pose_cmd=control_pose,
                pose_state_from_cmd=state_pose_from_cmd,
            )
            if args.use_state_from_state:
                datum.update(
                    t_state_start=t_state_start,
                    pose_state_from_state=state_pose_from_state,
                    t_state_end=t_state_end,
                )
            logger.debug(("t_datum_end", time.time()))
            logger.debug(("t_dump_start", time.time()))
            js = json.dumps(datum)
            logger.debug(("t_dump_end", time.time()))
            logger.debug(("t_write_start", time.time()))
            f.write(js + "\n")
            logger.debug(("t_write_end", time.time()))
            if default_servo_mode == 0x101:
                time.sleep(args.wait)
        logger.info("Loop end")
        robot.stop_move_pose_servo(control_pose)
        robot.leave_servo_mode()
        robot.move_default_pose_until_completion()
    except Exception as e:
        import traceback
        traceback.print_exc()
        robot.log_error(e)
    finally:
        del robot
