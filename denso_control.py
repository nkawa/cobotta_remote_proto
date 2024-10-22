"""MQTT、軌道を入力とした制御。マルチプロセス、目標位置と制御位置のSoftplus補間の制御。"""
from argparse import ArgumentDefaultsHelpFormatter, ArgumentParser
import logging
import multiprocessing
from multiprocessing import Array, Process, Queue, Value
import queue
import time
from typing import Tuple

import numpy as np
import pandas as pd

from denso_robot import DensoRobot
from mqtt_control_utils.angle import norm_angle_180, norm_angle_360, norm_pose_180, norm_pose_360
from mqtt_control_utils.log import show_log_mp, Recorder
from mqtt_control_utils.mqtt_feeder.realtime_mqtt_feeder import RealtimeMQTTFeeder
from mqtt_control_utils.mqtt_feeder.replay_mqtt_feeder import ReplayMQTTFeeder
from mqtt_control_utils.trajectory import load_mqtt
from mqtt_control_utils.transform import Transform
from robot.dummy_robot import DummyRobot


logger = logging.getLogger(__name__)


def load_control_dataframe(
    control_path: str, run_robot_path: str
) -> pd.DataFrame:
    """軌道の評価結果を読み込むための関数。"""

    jss = load_mqtt(control_path)
    data = []
    for js in jss:
        try:
            datum = {}
            if js["kind"] == "control":
                datum["kind"] = "control_planned"
            else:
                datum["kind"] = js["kind"]
            datum["t"] = js["time"]
            if datum["kind"] in ["target", "base", "diff_control"]:
                x, y, z, xd, yd, zd = js["pos"]
                datum["x"] = x
                datum["y"] = y
                datum["z"] = z
                datum["xd"] = xd
                datum["yd"] = yd
                datum["zd"] = zd
                data.append(datum)
            elif datum["kind"] in ["control_planned"]:
                for pos in js["pos"]:
                    x, y, z, xd, yd, zd = pos
                    datum = datum.copy()
                    datum["x"] = x
                    datum["y"] = y
                    datum["z"] = z
                    datum["xd"] = xd
                    datum["yd"] = yd
                    datum["zd"] = zd
                    data.append(datum)
            else:
                print(f'control: kind: {datum["kind"]} exists and is not loaded to dataframe')
        except:
            import traceback
            traceback.print_exc()
            print(f"{js=}")
    df_c = pd.DataFrame(data)

    jss = load_mqtt(run_robot_path)
    data = []
    for js in jss:
        try:
            if js["kind"] in ["control", "state"]:
                datum = {}
                datum["kind"] = js["kind"]
                datum["t"] = js["time"]
                x, y, z, xd, yd, zd = js["pos"]
                datum["x"] = x
                datum["y"] = y
                datum["z"] = z
                datum["xd"] = xd
                datum["yd"] = yd
                datum["zd"] = zd
                data.append(datum)
            else:
                print(f'run_robot: kind: {js["kind"]} exists and is not loaded to dataframe')
        except:
            import traceback
            traceback.print_exc()
            print(f"{js=}")
    df_r = pd.DataFrame(data)

    df = pd.concat([df_c, df_r], axis=0)
    df["t"] -= df["t"].min()
    # 原点が異なるので合わせる
    kinds = ["target", "control", "state", "base", "control_planned"]
    for kind in kinds:
        for coord in ["x", "y", "z"]:
            df.loc[df["kind"] == kind, coord] = df.loc[df["kind"] == kind, coord] - df.loc[df["kind"] == kind, coord].iloc[0]
        for coord in ["xd", "yd", "zd"]:
            df.loc[df["kind"] == kind, coord] = norm_angle_360(df.loc[df["kind"] == kind, coord] - df.loc[df["kind"] == kind, coord].iloc[0])
    return df

def softplus(x):
    """
    ReLU関数に近いがx = 0でxと傾きが滑らかな関数
    """
    return np.log(1 + np.exp(x))

def get_diff_factors(
    target_interval: float, control_interval: float
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Softplus関数ベースの関数で目標値を補間して、制御値の時間間隔で
    サンプリングする

    target_interval: 平均的な目標値の時間間隔
    control_interval: 制御値の時間間隔

    制御系列の時間と位置を返す
    """
    ti = target_interval 
    ci = control_interval
    # 点数
    n = 2 * ti // ci
    ts = -ti + np.arange(1, n + 1) * ci
    # softplusの[-4, 4]のスケールでの推移を利用
    # 1 - (softplus(0) - softplus(-4)) / (softplus(4) - softplus(-4)) = 0.8312506868394661
    # すなわち制御をはじめてからt = target_intervalでは目標差分の83%にまで
    # 到達するようにしている
    xlim = 4
    ylim = softplus(xlim)
    xs = ts / ti * xlim
    # [0, 1]にスケール
    ys = 1 - softplus(-xs) / ylim
    # [0, 2 * ts]にスケール
    ts += ti
    # 停止信号を追加
    ts_add = -ti + np.arange(n + 1, n + 1 + int(n // 2)) * ci + ti
    ys_add = np.ones(int(n // 2))
    # 制御系列の時間と位置
    ts = np.append(ts, ts_add)
    ys = np.append(ys, ys_add)
    return ts, ys

class DensoControl:
    def __init__(
        self,
        q_control_poses: Queue,
        arr_base_pose,
        v_base_pose,
        scale_mqtt_vs_real=1,
        transform=None,
        use_all_target=False,
        robot_interval=0.008,
        control_interval=0.05,
        feeder="replay",
        v_lim_pos=50,
        v_lim_rot=50,
        input_angle_unit="deg",
        save_feed=False,
        save_control=False,
    ):
        if feeder == "replay":
            jss = load_mqtt("target.jsonl")
            self.mqtt_feeder = ReplayMQTTFeeder(jss)
            self.wait_for_robot = True
        elif feeder == "mqtt":
            self.mqtt_feeder = RealtimeMQTTFeeder(
                save=save_feed,
                save_path="mqtt.jsonl",
            )
            self.wait_for_robot = False
        else:
            raise ValueError
        self.mqtt_feeder.set_on_message(self.on_target)
        self.q_control_poses = q_control_poses
        self.arr_base_pose = arr_base_pose
        self.v_base_pose = v_base_pose
        self.scale_mqtt_vs_real = scale_mqtt_vs_real
        self.use_all_target = use_all_target
        self.robot_interval = robot_interval
        self.control_interval = control_interval
        self.v_lim_pos = v_lim_pos
        self.v_lim_rot = v_lim_rot
        self.input_angle_unit = input_angle_unit
        self.recorder = None
        if save_control:
            self.recorder = Recorder("control.jsonl")
        if transform is None:
            self.transform = Transform()
        else:
            self.transform = transform
        self.lt = time.time()
        _, ys = get_diff_factors(
            self.control_interval,
            self.robot_interval,
        )
        self.diff_factors = np.asarray(ys)
        self.reset_pose()

    def start(self):
        self.mqtt_feeder.start()

    def stop(self):
        self.mqtt_feeder.stop()

    def on_target(self, js, client, userdata, msg):
        # データ取得
        if 'pos' in js:
            x = -js['pos']['x']
            y = js['pos']['z']
            z = js['pos']['y']
            xd = 0
            yd = 0
            zd = 0
#                        x="-x", y="z", z="y", xd="-xd", yd="zd", zd="yd",

#            xd = js['ori']['x']
#            yd = js['ori']['y']
#            zd = js['ori']['z']
            t = time.time()
        else:
            logger.warning("Unexpected JSON", js)
            return
        # 座標変換
#        x, y, z, xd, yd, zd = self.transform(x, y, z, xd, yd, zd)
        x *= self.scale_mqtt_vs_real
        y *= self.scale_mqtt_vs_real
        z *= self.scale_mqtt_vs_real
        if self.input_angle_unit == "rad":
            xd *= 180
            yd *= 180
            zd *= 180
        xd = norm_angle_360(xd)
        yd = norm_angle_360(yd)
        zd = norm_angle_360(zd)

        target_dict = {'kind': 'target', 'pos': [x, y, z, xd, yd, zd], 'time': t}
        if self.recorder is not None:
            self.recorder.log(target_dict)
        logger.debug(target_dict)

        # 更新
        # 位置制御
        if self.is_reset:
            flag = False
            # リセットから1回目
            # 目標姿勢を最初からすべてロボットに送りたいとき（評価用）
            if self.wait_for_robot:
                while True:
                    # ロボットが認識できるまで待つ
                    if self.v_base_pose.value == 1:
                        flag = True
                        break
            # ロボットが認識できた時点からの目標値を使いたいとき（実用）
            else:
                if self.v_base_pose.value == 1:
                    flag = True
            # ロボットが認識できたら
            if flag:
                # 遠隔コントローラーとロボットそれぞれの原点
                self.base_robot = np.asarray(self.arr_base_pose[:])
                self.base_mqtt = np.asarray([x, y, z, xd, yd, zd])
                self.lx = x
                self.ly = y
                self.lz = z
                self.lxd = xd
                self.lyd = yd
                self.lzd = zd
                self.is_reset = False
        else:
            # リセットから2回目以降
            if (
                # 目標姿勢の更新有無によらずに制御姿勢を更新
                self.use_all_target or
                # 目標姿勢の更新があるときだけに制御姿勢を更新
                (
                    self.lx != x or
                    self.ly != y or
                    self.lz != z or
                    self.lxd != xd or
                    self.lyd != yd or
                    self.lzd != zd
                )
            ):
                # 現状、目標位置の差分の値自体は、制御位置の生成には使用せず、
                # 制御位置の生成を行うかどうかの判定にのみ使用している
                dx = x - self.lx
                dy = y - self.ly
                dz = z - self.lz
                dxd = norm_angle_180(xd - self.lxd)
                dyd = norm_angle_180(yd - self.lyd)
                dzd = norm_angle_180(zd - self.lzd)
                self.lx = x
                self.ly = y
                self.lz = z
                self.lxd = xd
                self.lyd = yd
                self.lzd = zd
                if 'pad' in js:
                    pd = js['pad']
                    if pd['b0'] != 1:
                        # 制御姿勢の更新
                        # 目標姿勢のその原点からの相対姿勢
                        target_pose_rel = norm_pose_360(np.asarray([x, y, z, xd, yd, zd]) - self.base_mqtt)
                        # ロボットの絶対姿勢
                        base_pose = self.arr_base_pose[:]
                        # ロボットのその原点からの相対姿勢
                        base_pose_rel = norm_pose_360(np.asarray(base_pose) - self.base_robot)
                        base_dict = {'kind': 'base', 'pos': (norm_pose_360(self.base_robot + base_pose_rel)).tolist(), 'time': time.time()}
                        if self.recorder is not None:
                            self.recorder.log(base_dict)
                        # ロボット姿勢からの目標姿勢の相対姿勢
                        diff_control_pose = norm_pose_180(target_pose_rel - base_pose_rel)
                        # 位置、角度ごとに平均制限速度で移動した場合の目標姿勢までにかかる時間
                        # 制限速度は、現状、実験した限りで最大約200 mm/s。
                        # 現状のSoftplusベースの補間では、最大速度の1/4程度が平均制限速度となる
                        # 角度は十分検証していない、仮に50 deg/sとしている
                        v_lim_pos = self.v_lim_pos
                        v_lim_rot = self.v_lim_rot
                        t_lim_pos = np.max(np.abs(diff_control_pose[:3])) / v_lim_pos
                        t_lim_rot = np.max(np.abs(diff_control_pose[3:])) / v_lim_rot
                        t_lim = max(t_lim_pos, t_lim_rot)
                        # 目標姿勢までのサンプル点系列 (0~1の間)
                        # 時間が目標姿勢の更新間隔以下なら、目標姿勢は大きく
                        # 更新されていないので、もともとの目標姿勢の更新間隔で
                        # ロボットの移動がほぼ完了するようにする
                        if t_lim <= self.control_interval:
                            diff_factors = self.diff_factors
                        # 時間が更新間隔より大きいなら、目標姿勢は大きく更新されて
                        # いるので、その時間でロボットの移動がほぼ完了する
                        # ようにする
                        else:
                            _, diff_factors = get_diff_factors(t_lim, self.robot_interval)
                        # 目標姿勢のサンプル点系列 (ロボットに指令する制御姿勢)
                        diff_control_poses = \
                            diff_control_pose[None, :] * diff_factors[:, None]
                        diff_control_dict = {'kind': 'diff_control', 'pos': diff_control_pose.tolist(), 'time': time.time()}
                        if self.recorder is not None:
                            self.recorder.log(diff_control_dict)
                        control_poses = (
                            norm_pose_360((self.base_robot[None, :] + base_pose_rel[None, :] + diff_control_poses).T).T
                        ).tolist()
                        control_dict = {'kind': 'control', 'pos': control_poses, 'time': time.time()}
                        if self.recorder is not None:
                            self.recorder.log(control_dict)
                        # 制御姿勢の系列を更新
                        self.q_control_poses.put(control_poses)
                    if pd['bA']:
                        self.reset_pose()

    def reset_pose(self):
        self.lx = 0
        self.ly = 0
        self.lz = 0
        self.lxd = 0
        self.lyd = 0
        self.lzd = 0
        self.is_reset = True

def run_robot(
    q_control_poses: Queue,
    arr_base_pose,
    v_base_pose, 
    use_dummy: bool = False,
    default_fig: int = -2,
    default_servo_mode: int = 0x201,
    save_control: bool = False,
    host: str = "192.168.5.45",
    port: int = 5007,
):
    """
    ロボットに制御値を送るためのプロセスで実行する関数。
    """
    try:
        # この関数は並列プロセス内部で実行するので
        # この関数内で各モジュールを初期化する必要がある
        recorder = None
        if save_control:
            recorder = Recorder("run_robot.jsonl")
        # デバッグ用
        if use_dummy:
            robot = DummyRobot(
                servo_control_mode="abs",
                command_time=0.008,
            )
        else:
            robot = DensoRobot(
                default_fig=default_fig,
                default_servo_mode=default_servo_mode,
                host=host,
                port=port,
            )
        robot.start()
        robot.enable()
        robot.move_default_pose_until_completion()
        robot.enter_servo_mode()
        i_pose = 0
        n_poses = 0
        # 制御ループ
        # 各ループは平均8 ms以内で完了しないと
        # ロボットへの制御値が途絶えてエラーになる
        while True:
            # 制御姿勢の系列に更新があるかどうか確認
            try:
                new_control_poses = q_control_poses.get(timeout=1e-6)
                # ある場合新しい制御姿勢の系列を使う
                control_poses = new_control_poses
                n_poses = len(control_poses)
                i_pose = 0
            except queue.Empty:
                # ない場合現在の制御姿勢の系列を使う
                pass
            # 制御姿勢の系列を使う
            if i_pose <= n_poses - 1:
                control_pose = control_poses[i_pose]
                control_dict = {'kind': 'control', 'pos': control_pose, 'time': time.time()}
                logger.debug(control_dict)
                if recorder is not None:
                    recorder.log(control_dict)
                # 制御姿勢を指令
                try:
                    robot.move_pose_servo(control_pose)
                    # if use_dummy:
                    #     raise ValueError("Mock error in dummy robot")
                except Exception as e:
                    # エラー時はロボットからエラーを見やすくして表示
                    robot.log_error(e)
                    # モータ、スレーブモードを自動再起動可能な
                    # エラーなら再起動
                    # 速度、加速度過大などはロボットが静止後、
                    # 再起動するだけでよいので自動でできるが、
                    # 特異姿勢異常などはユーザーの手動操作で
                    # 特異姿勢から脱してから再起動する必要があり、
                    # また角度制御するなら問題にならないので
                    # 自動再起動せずに停止させている
                    restarted = robot.try_restart(e)
                    if restarted:
                        logger.info("Restarted from move_pose_servo error")
                        # 制御はしないが、状態姿勢を保存する
                        state_pose = robot.get_current_pose()
                        state_dict = {'kind': 'state', 'pos': state_pose, 'time': time.time()}
                        logger.debug(state_dict)
                        if recorder is not None:
                            recorder.log(state_dict)
                        for i in range(len(state_pose)):
                            arr_base_pose[i] = state_pose[i]
                        # ロボット姿勢を保存したことを示す
                        v_base_pose.value = 1
                        i_pose = 0
                        n_poses = 0
                    else:
                        logger.info("Stopped by move_pose_servo error")
                        break
                else:
                    # 状態姿勢を取得
                    state_pose = robot.get_current_pose()
                    state_dict = {'kind': 'state', 'pos': state_pose, 'time': time.time()}
                    logger.debug(state_dict)
                    if recorder is not None:
                        recorder.log(state_dict)
                    # 基本は実行した制御姿勢を保存する
                    for i in range(len(control_pose)):
                        arr_base_pose[i] = control_pose[i]
                    # ロボット姿勢を保存したことを示す
                    v_base_pose.value = 1
                    i_pose += 1
            # 制御姿勢の系列がない場合または使い終わったら
            else:
                # 制御はしないが、状態姿勢を保存する
                state_pose = robot.get_current_pose()
                state_dict = {'kind': 'state', 'pos': state_pose, 'time': time.time()}
                logger.debug(state_dict)
                if recorder is not None:
                    recorder.log(state_dict)
                for i in range(len(state_pose)):
                    arr_base_pose[i] = state_pose[i]
                # ロボット姿勢を保存したことを示す
                v_base_pose.value = 1

        # 実行しなくてもデストラクタ内で実行される
        robot.leave_servo_mode()
        robot.disable()
    # Ctrl + cで止めた場合にエラーメッセージを表示しない
    except KeyboardInterrupt:
        logger.info("KeyboardInterrupt detected in run_robot")
        pass

if __name__ == "__main__":
    parser = ArgumentParser(
        description="MQTT/軌道入力に対する制御プログラム",
        formatter_class=ArgumentDefaultsHelpFormatter,
    )
    # 全般
    parser.add_argument("--log", action="store_true", help="ログを記録")
    parser.add_argument("--save-control", action="store_true", help="目標値、制御値、状態値の保存。")
    # 目標値受信、制御値計算
    parser.add_argument("--feeder", choices=["replay", "mqtt"], default="mqtt", help="入力データ。replay: 軌道ファイル、mqtt: MQTT。")
    # feeder = "replay"の場合のみ
    parser.add_argument("--target-path", default="target.jsonl", help="軌道ファイルへのパス。feederがreplayの場合のみ有効")
    # feeder = "mqtt"の場合のみ
    parser.add_argument("--save-feed", action="store_true", help="MQTTで受信した生データの保存。feederがmqttの場合のみ有効")
    # feeder共通の場合
    parser.add_argument("--disable-transform", action="store_true", help="VRコントローラと開発環境でのロボットの座標系を揃えないようにする")
    parser.add_argument("--scale-mqtt-vs-real", default=1, type=float, help="姿勢の入力に対する実ロボットの姿勢のスケール。VRコントローラからの入力の場合は例えば110くらいがよい。入力データのスケールをよく確認したうえで設定すること")
    parser.add_argument("--input-angle-unit", default="rad", choices=["rad", "deg"], help="入力の姿勢の角度の単位。入力データのスケールをよく確認したうえで設定すること")
    parser.add_argument("--use-all-target", action="store_true", help="目標値 (入力データ) が更新されるとき以外も制御値を生成する。基本的に不要")
    # ロボット制御
    parser.add_argument("--default-fig", default=-2, type=int, choices=[-2, -3], help="Denso Cobotta Pro 900の形態 (ロボットアームの関節が取る腕・ひじ・手首の形態) の自動設定モード。-2は形態保持重視。-3はエラー回避重視")
    parser.add_argument("--default-servo-mode", default="0x201", choices=["0x001", "0x101", "0x201"], help="Denso Cobotta Pro 900のスレーブモードのモード。0: 0x001、1: 0x101、2: 0x201を指定する")
    parser.add_argument("--v-lim-pos", default=200, type=float, help="TCPの位置の速度制限の目安 (mm/s)。速度、加速度超過エラーで止まる場合はこの値を下げると止まりにくくなるが遅延する")
    parser.add_argument("--v-lim-rot", default=200, type=float, help="TCPの角度の速度制限の目安 (deg/s)。速度、加速度超過エラーで止まる場合はこの値を下げると止まりにくくなるが遅延する")
    parser.add_argument("--host", default="192.168.5.45", help="ロボットのIPアドレス")
    parser.add_argument("--port", default=5007, type=int, help="ロボットのポート")
    # デバッグ用
    parser.add_argument("--use-dummy", action="store_true", help="デバッグ用にダミーのロボットを使う")
    args = parser.parse_args()

    # ログレベルは、目標値、制御値、状態値の表示と、ロボットのエラーの表示のために固定している
    if args.log:
        if args.use_dummy:
            log_queue = multiprocessing.Queue()
            loq_queue_listener = show_log_mp(
                log_queue=log_queue,
                modules=["__main__", "dummy_robot.dummy_robot"],
                loglevels=["DEBUG", "ERROR"],
                handlers={"StreamHandler": {}, "FileHandler": {"filename": "log.txt", "mode": "w"}},
            )
            loq_queue_listener.start()
        else:
            log_queue = multiprocessing.Queue()
            loq_queue_listener = show_log_mp(
                log_queue=log_queue,
                # 本スクリプトのログレベルとロボットクラスのエラーを取得する
                # ためのログレベルを設定
                modules=["__main__", "denso_robot.denso_robot"],
                loglevels=["DEBUG", "ERROR"],
                handlers={"StreamHandler": {}, "FileHandler": {"filename": "log.txt", "mode": "w"}},
            )
            loq_queue_listener.start()

    if args.disable_transform:
        transform = Transform()
    else:
        # VRコントローラと開発環境でのロボットの座標系を揃える
        transform = Transform(
            x="-x", y="z", z="y", xd="-xd", yd="zd", zd="yd",
        )

    # プロセス間共有変数
    q_control_poses = Queue()
    arr_base_pose = Array("d", [0, 0, 0, 0, 0, 0])
    # arr_base_poseの値だけでは、arr_base_poseに有効な値が格納されているか
    # かわからないので、以下の変数で判定する
    v_base_pose = Value("i", 0)

    # ロボット制御用プロセス
    robot_process = Process(
        target=run_robot,
        args=(
            q_control_poses,
            arr_base_pose,
            v_base_pose,
            args.use_dummy,
            args.default_fig,
            int(args.default_servo_mode, base=16),
            args.save_control,
            args.host,
            args.port,
        )
    )
    logger.info("robot_process.start")
    robot_process.start()

    # 目標位置受信、制御位置生成スレッド
    denso_control = DensoControl(
        q_control_poses,
        arr_base_pose,
        v_base_pose,
        scale_mqtt_vs_real=args.scale_mqtt_vs_real,
        transform=transform,
        feeder=args.feeder,
        use_all_target=args.use_all_target,
        input_angle_unit=args.input_angle_unit,
        save_feed=args.save_feed,
        v_lim_pos=args.v_lim_pos,
        v_lim_rot=args.v_lim_rot,
        save_control=args.save_control,
    )
    logger.info("control.start")
    denso_control.start()

    # メインプロセスは処理終了まで待機
    while True:
        try:
            time.sleep(1)
        # Ctrl + cを送信しても、直ちには終了しないことがある。
        # また、各プロセス、スレッドは中断するが、
        # 終了せずに止まることもある（出力ファイルや
        # ログが更新されなくなることで確認できる）。
        # その場合、もう一度Ctrl + cを送信すると、
        # 終了できる（何らかの理由でmultiprocessingが
        # デッドロックを起こしていると考えられるエラーメッセージが出る）
        except KeyboardInterrupt:
            logger.info("KeyboardInterrupt detected in main")
            robot_process.join()
            denso_control.stop()
            loq_queue_listener.stop()
            break
