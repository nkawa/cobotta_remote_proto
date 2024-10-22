import copy
import logging
import time
import traceback
from typing import Tuple

from bcap_python.bcapclient import BCAPClient
from bcap_python.orinexception import ORiNException
from robot import Robot

logger = logging.getLogger(__name__)

# エラーコード

# ベースライン
E = 0x100000000

def python_error_to_original_error_str(hr: int) -> str:
    # Reference: https://github.com/ShoheiKobata/orin_bcap_python_samples/blob/master/SimpleSamples/06_error_handling.py
    # 正負で変換式が異なる
    if hr < 0:
        return hex(E + hr)
    else:
        return hex(hr)

def original_error_to_python_error(e: int) -> int:
    return e - E

E_BUF_FULL = original_error_to_python_error(0x83201483)  # バッファオーバーフロー
E_ORDER_DELAY = original_error_to_python_error(0x84201482)  # 指令値生成遅延
E_MOTOR_ON_WHILE_OFF_TRANSITION = original_error_to_python_error(0x8350106e)
E_ACCEL_LARGE_JOINT_1 = original_error_to_python_error(0x84204041)  # 指令値加速度過大
E_ACCEL_LARGE_JOINT_2 = original_error_to_python_error(0x84204042)
E_ACCEL_LARGE_JOINT_3 = original_error_to_python_error(0x84204043)
E_ACCEL_LARGE_JOINT_4 = original_error_to_python_error(0x84204044)
E_ACCEL_LARGE_JOINT_5 = original_error_to_python_error(0x84204045)
E_ACCEL_LARGE_JOINT_6 = original_error_to_python_error(0x84204046)
E_ACCEL_LARGE_JOINT_7 = original_error_to_python_error(0x84204047)
E_ACCEL_LARGE_JOINT_8 = original_error_to_python_error(0x84204048)
E_ACCEL_LARGE_JOINTS = [
    E_ACCEL_LARGE_JOINT_1,
    E_ACCEL_LARGE_JOINT_2,
    E_ACCEL_LARGE_JOINT_3,
    E_ACCEL_LARGE_JOINT_4,
    E_ACCEL_LARGE_JOINT_5,
    E_ACCEL_LARGE_JOINT_6,
    E_ACCEL_LARGE_JOINT_7,
    E_ACCEL_LARGE_JOINT_8,
]
E_VEL_LARGE_JOINT_1 = original_error_to_python_error(0x84204051)  # 指令値指令速度過大
E_VEL_LARGE_JOINT_2 = original_error_to_python_error(0x84204052)
E_VEL_LARGE_JOINT_3 = original_error_to_python_error(0x84204053)
E_VEL_LARGE_JOINT_4 = original_error_to_python_error(0x84204054)
E_VEL_LARGE_JOINT_5 = original_error_to_python_error(0x84204055)
E_VEL_LARGE_JOINT_6 = original_error_to_python_error(0x84204056)
E_VEL_LARGE_JOINT_7 = original_error_to_python_error(0x84204057)
E_VEL_LARGE_JOINT_8 = original_error_to_python_error(0x84204058)
E_VEL_LARGE_JOINTS = [
    E_VEL_LARGE_JOINT_1,
    E_VEL_LARGE_JOINT_2,
    E_VEL_LARGE_JOINT_3,
    E_VEL_LARGE_JOINT_4,
    E_VEL_LARGE_JOINT_5,
    E_VEL_LARGE_JOINT_6,
    E_VEL_LARGE_JOINT_7,
    E_VEL_LARGE_JOINT_8,
]
E_NOT_IN_SLAVE_MODE = original_error_to_python_error(0x83500121)
E_MOTOR_OFF = original_error_to_python_error(0x81501003)

class DensoRobot(Robot):
    """Denso Cobotta Pro 900の制御クラス。"""
    def __init__(
        self,
        name: str = "denso_cobotta_pro_900",
        default_servo_mode: int = 0x001,
        default_fig: int = -2,
        host: str = "192.168.5.45",
        port: int = 5007,
        timeout: float = 5,
    ):
        super().__init__(name)
        self._bcap = None
        self._hRob = 0
        self._hCtrl = 0
        assert default_servo_mode in [0x001, 0x101, 0x201, 0x002, 0x102, 0x202]
        self._default_servo_mode = default_servo_mode
        # 形態自動設定
        # -2: 形態が大きく変化するのを抑制する
        # -3: ソフトリミットエラーや可動範囲外エラーにならない
        self._default_fig = default_fig
        logger.info(("default_servo_mode", self._default_servo_mode))
        logger.info(("default_fig", self._default_fig))
        # パラメータ
        self.host = host
        self.port = port
        self.timeout = timeout

    def start(self):
        logger.info("Robot start")
        self._bcap = BCAPClient(self.host, self.port, self.timeout)
        # 400 ms以上通信がないとエラー
        self._bcap.service_start(",WDT=400")

        # 第2引数にはコントローラー名を指定する。指定しない場合は自動で割り当てられる。
        self._hCtrl = self._bcap.controller_connect("", "CaoProv.DENSO.VRC9", "localhost", "")
        self._hRob = self._bcap.controller_getrobot(self._hCtrl, "Robot")
        self._default_pose = [560, 150, 460, 180, 0, 90]
        self._interval = 0.008

    def enable(self):
        logger.info("enable")
        # STO状態（セーフティ状態）を解除する
        self._bcap.controller_execute(self._hCtrl, "ManualReset")
        # ティーチングペンダントのエラーをクリアする
        self._bcap.controller_execute(self._hCtrl, "ClearError")
        # ロボットの軸の制御権
        # 第3引数のリストの2番目が0のとき内部速度を100%にリセットする
        self._bcap.robot_execute(self._hRob, "Takearm", [0, 0])
        # 外部速度(%)の設定
        # スレーブモードでは外部速度は反映されない
        self._bcap.robot_execute(self._hRob, "ExtSpeed", [20])
        self._bcap.robot_execute(self._hRob, "Motor", 1)

    def set_default_pose(self, pose):
        self._default_pose = pose

    def get_default_pose(self):
        return self._default_pose

    def move_default_pose(self):
        logger.info("move_default_pose")
        self.move_pose(self._default_pose)

    def move_pose(self, pose):
        pose = self._add_fig_if_necessary(pose)
        prev_servo_mode = self.is_in_servo_mode()
        if prev_servo_mode:
            self.leave_servo_mode()
        x, y, z, rx, ry, rz, fig = pose
        # 参考：https://www.fa-manuals.denso-wave.com/jp/COBOTTA%20PRO/016664/
        # 第2引数
        # 補間指定
        # 1: MOVE P（PTP動作（最短時間経路））
        # 2: MOVE L (直線補間動作（最短距離経路)）
        # 3: MOVE C（円弧補間動作）
        # 4: MOVE S
        # 第3引数
        # @Eは、目標位置にエンコーダ値（測定位置）が到達するまで待ち停止する。このときの位置は関節角度。
        # @Pは、目標位置の近く（自動設定）にエンコーダ値が到達したら次のコマンドに移行する。
        # @<数字>は、@Pを、目標位置の近くを数字（mm）に設定した上で実行。
        self._bcap.robot_move(self._hRob, 1, f"@E P({x}, {y}, {z}, {rx}, {ry}, {rz}, {fig})")
        if prev_servo_mode:
            self.enter_servo_mode()

    def get_current_pose(self):
        cur_pos = self._bcap.robot_execute(self._hRob, "CurPos")
        # x, y, z, rx, ry, rz, fig = cur_pos

        # ロボットコントローラからタイムスタンプも追加で返すことができる
        # cur_pos_ex = bcap.robot_execute(hRob, "CurPosEx")
        # t, x, y, z, rx, ry, rz, fig = cur_pos_ex
        # t: コントローラ電源ONからの時間（msec）
        # 他の処理と比較するには同じプログラムで時間を計算したほうが便利なので
        # 使用しない
        return cur_pos[:6]

    def enter_servo_mode(self):
        logger.info("enter_servo_mode")
        self.enter_servo_mode_by_mode(self._default_servo_mode)

    def leave_servo_mode(self):
        logger.info("leave_servo_mode")
        self.slvChangeMode = 0x000
        self._bcap.robot_execute(self._hRob, "slvChangeMode", self.slvChangeMode)

    def disable(self):
        logger.info("disable")
        # NOTE: 異常終了した場合に
        # エラーコード: -2147023170 (0x800706be、マニュアルにのっていない)が
        # 出ることあり
        try:
            self._bcap.robot_execute(self._hRob, "Motor", 0)
        except ORiNException as e:
            # とりあえず無視しても現状問題ない
            if e.hresult == -2147023170:
                pass
            # この関数は__del__の中で呼ばれるので例外で中断させてはいけない
            # ので他のエラーが見つかったら表示だけする
            else:
                self.log_error(e)
        self._bcap.robot_execute(self._hRob, "Givearm")

    def stop(self):
        logger.info("stop")
        if self._hRob != 0:
            self._bcap.robot_release(self._hRob)
            self._hRob = 0
        if self._hCtrl != 0:
            self._bcap.controller_disconnect(self._hCtrl)
            self._hCtrl = 0
        if self._bcap is not None:
            self._bcap.service_stop()
            self._bcap = None

    def get_suggested_servo_interval(self):
        base = 0.008
        if self._default_servo_mode == 0x001:
            return base
        elif self._default_servo_mode == 0x101:
            return base
        elif self._default_servo_mode == 0x201:
            return 0

    def is_in_servo_mode(self) -> bool:
        return self._bcap.robot_execute(self._hRob, "slvGetMode") != 0x000

    def switch_servo_mode(self, mode: int = 0x001):
        self.leave_servo_mode()
        self.enter_servo_mode(mode)

    def enter_servo_mode_by_mode(self, mode: int = 0x001):
        # スレーブモードで実行可
        # STO状態（セーフティ状態）を解除する
        self._bcap.controller_execute(self._hCtrl, "ManualReset")
        # return以降ははスレーブモードでなければ実行不可
        if self.is_in_servo_mode():
            return
        # スレーブモードの出力設定
        self.set_servo_format(output_mode=0x0011, timestamp_unit=1)
        # スレーブモードでは実行不可
        self._bcap.controller_execute(self._hCtrl, "ClearError")
        # 0x000: モード解除、0x001: P型モード0設定、0x101: P型モード1設定、0x201: P型モード2設定
        # エラー時には自動的に0に戻る
        # スレーブモードでは実行不可
        self.slvChangeMode = mode
        self._bcap.robot_execute(self._hRob, "slvChangeMode", self.slvChangeMode)

    def move_pose_servo(self, pose) -> Tuple[float, Tuple[float, float, float, float, float, float]]:
        """
        スレーブモードでの位置制御。

        時間と現在の状態位置を返す。

        この時間はコントローラ内部の時間で、ユーザースクリプトの
        時間と少しずれているので使わないほうがよい。
        """
        logger.debug("move_pose_servo")
        pose = self._add_fig_if_necessary(pose)

        if self.slvChangeMode == 0x001:
            ret = self._move_pose_servo_mode_0(pose)
        elif self.slvChangeMode == 0x101:
            ret = self._move_pose_servo_mode_1(pose)
        elif self.slvChangeMode == 0x201:
            ret = self._move_pose_servo_mode_2(pose)
        else:
            raise ValueError

        t, ret_pose = ret
        ret_pose = ret_pose[:6]
        return t, ret_pose

    def try_restart(self, e: Exception) -> bool:
        if type(e) is ORiNException:
            hr = e.hresult
            if hr < 0:
                if hr in (
                    E_VEL_LARGE_JOINTS +
                    E_ACCEL_LARGE_JOINTS +
                    [E_NOT_IN_SLAVE_MODE] +
                    [E_MOTOR_OFF] +
                    [E_ORDER_DELAY]
                ):
                    self.recover_automatic_servo()
                    time.sleep(1)
                    return True
        return False

    def _move_pose_servo_mode_0(self, target_pose):
        while True:
            try:
                ret = self._bcap.robot_execute(
                    self._hRob, "slvMove", target_pose
                )
                return ret
            except ORiNException as e:
                hr = e.hresult
                # モード0はバッファオーバーフロー時の挙動を
                # ユーザーが選択できるがここではモード2と同じく
                # 何もせずに同じコマンドを送るようにしている
                # 通信の分だけこちらの方が間隔にばらつきがでるかもしれない
                if hr == E_BUF_FULL:
                    continue
                raise e
            except Exception as e:
                raise e

    def _move_pose_servo_mode_1(self, target_pose):
        ret = self._bcap.robot_execute(self._hRob, "slvMove", target_pose)
        return ret

    def _move_pose_servo_mode_2(self, target_pose):
        ret = self._bcap.robot_execute(self._hRob, "slvMove", target_pose)
        return ret

    def set_servo_format(self, output_mode: int = 0x0001, timestamp_unit: int = 0):
        """
        output_mode:
          0x0001: 位置のみ
          0x0011: タイムスタンプと位置
        timestamp_unit:
          0: ms
          1: us
        """
        self._bcap.robot_execute(self._hRob, "slvRecvFormat", [output_mode, timestamp_unit])

    def stop_move_pose_servo(self, last_target_pose):
        self.move_pose_servo(last_target_pose)
        if not self.is_in_servo_mode():
            self.recover_automatic_servo()

    def StoState(self) -> bool:
        """STO 状態（セーフティ状態）を返します"""
        return self._bcap.controller_execute(self._hCtrl, "StoState")

    def GetCurErrorInfoAll(self) -> bool:
        """現在発生しているエラーの情報を返します"""
        # エラーの情報: エラーコード、エラーメッセージ、サブコード、ファイルID＋行番号、プログラム名、行番号、ファイルID
        n_errors = self._bcap.controller_execute(self._hCtrl, "GetCurErrorCount")
        for i in range(n_errors):
            # i = 0が最新のエラー
            logger.error(self._bcap.controller_execute(self._hCtrl, "GetCurErrorInfo", i))

    def GetErrorLogAll(self) -> bool:
        """エラーログの情報を取得します"""
        # エラーログ: エラーコード、時間、プログラム名、行番号、エラーメッセージ、オリジナルエラーコード、呼び出し元、IPアドレス
        n_errors = self._bcap.controller_execute(self._hCtrl, "GetCurErrorCount")
        for i in range(n_errors):
            # i = 0が最新のエラー
            logger.error(self._bcap.controller_execute(self._hCtrl, "GetErrorLog", i))

    def SceneInfo(self):
        cur_scene = self._bcap.robot_execute(self._hRob, "CurScene")
        cur_sub_scene = self._bcap.robot_execute(self._hRob, "CurSubScene")
        scene_info = self._bcap.robot_execute(self._hRob, "SceneInfo", [cur_scene, cur_sub_scene])
        logger.info(f"{scene_info=}")

    def motion_skip(self):
        self._bcap.robot_execute(self._hRob, "MotionSkip", [-1, 0])

    def log_error(self, e: Exception):
        logger.error("Error trace:", exc_info=True)
        if type(e) is ORiNException:
            logger.error("ORiN exception in controller")
            if self._hCtrl == 0:
                logger.error("Controller handler is dead")
            else:
                logger.error("Controller handler is alive")
            hr = e.hresult
            logger.error(f"Error code: {python_error_to_original_error_str(hr)}")
            desc = self._bcap.controller_execute(self._hCtrl, "GetErrorDescription", hr)
            logger.error(f"Error description: {desc}")

    def format_error(self, e: Exception):
        s = ""
        s = s + "Error trace:" + traceback.format_exc() + "\n"
        if type(e) is ORiNException:
            s += "ORiN exception in controller\n"
            if self._hCtrl == 0:
                s += "Controller handler is dead\n"
            else:
                s += "Controller handler is alive\n"
            hr = e.hresult
            s += f"Error code: {python_error_to_original_error_str(hr)}\n"
            desc = self._bcap.controller_execute(self._hCtrl, "GetErrorDescription", hr)
            s += f"Error description: {desc}\n"
        return s

    def is_in_range(self, target_pose) -> bool:
        # 返り値は、
        # 0: 可動範囲内、1~63: ソフトリミットである軸のビット、
        # -1: 軸構成上計算不可能な位置、-2: 特異点
        ret = self._bcap.robot_execute(self._hRob, "OutRange", target_pose)
        return ret == 0

    def recover_automatic_servo(self):
        # 以下の方法で復帰できるエラーに対してのみ呼ぶこと
        # 頻繁に呼ばれうるので無駄な処理は入れないこと

        # 位置監視以外のエラーは自動で復帰できる
        # STO状態（セーフティ状態）を解除する
        self._bcap.controller_execute(self._hCtrl, "ManualReset")
        # ティーチングペンダントのエラーをクリアする
        self._bcap.controller_execute(self._hCtrl, "ClearError")
        # 不要。制御権は解除されない
        # self._bcap.robot_execute(self._hRob, "Takearm", [0, 0])
        # 不要。スレーブモードに外部速度は関係ない
        # self._bcap.robot_execute(self._hRob, "ExtSpeed", [20])

        # モーターをできるだけONにしようとする
        # モーター損傷回避のためmax_trials以上はエラーとしておく
        max_trials = 3
        i_trials = 0
        while True:
            while True:
                i_trials += 1
                try:
                    # 第3引数のリストの2番目はデフォルトで0で完了待ち（ブロッキング）
                    self._bcap.robot_execute(self._hRob, "Motor", [1, 0])
                    break
                # E_MOTOR_ON_WHILE_OFF_TRANSITIONの場合にやり直す
                # 他の場合はエラー処理しない
                except ORiNException as e:
                    if i_trials == max_trials:
                        raise e
                    if e.hresult == E_MOTOR_ON_WHILE_OFF_TRANSITION:
                        # エラークリアが必要
                        self._bcap.controller_execute(self._hCtrl, "ClearError")
                        # 0.008だと待ちすぎなので下げる
                        time.sleep(0.001)
                    else:
                        raise e
                except Exception as e:
                    raise e

            # モーターがONになったかどうかはslvChangeModeが成功してはじめてわかる
            try:
                # スレーブモードがOFFになっているのでONにする
                # 元のslvChangeModeを使う
                self._bcap.robot_execute(self._hRob, "slvChangeMode", self.slvChangeMode)
                break
            # E_MOTOR_OFFの場合にやり直す
            # 他の場合はエラー処理しない
            except ORiNException as e:
                if i_trials == max_trials:
                    raise e
                if e.hresult == E_MOTOR_OFF:
                    # エラークリアが必要
                    self._bcap.controller_execute(self._hCtrl, "ClearError")
                    time.sleep(0.001)
                else:
                    raise e
            except Exception as e:
                raise e

    def _add_fig_if_necessary(self, pose):
        assert len(pose) in [6, 7]
        if len(pose) == 6:
            pose = copy.deepcopy(pose)
            pose.append(self._default_fig)
        return pose

    def __del__(self):
        if self._hRob != 0:
            self.leave_servo_mode()
            self.disable()
        self.stop()
        logger.info("Robot deleted")

    def get_current_joint(self):
        # コントローラ内部で一定周期（8ms）に更新された現在位置をJ 型で取得する
        # 8関節分出るがCobotta Pro 900は6関節分のみ有効
        cur_jnt = self._bcap.robot_execute(self._hRob, "CurJnt")
        return cur_jnt[:6]

    def cur_spd(self):
        # 内部速度の設定値を返す
        return self._bcap.robot_execute(self._hRob, "CurSpd")

    def cur_acc(self):
        # 内部加速度の設定値を返す
        return self._bcap.robot_execute(self._hRob, "CurAcc")

    def cur_ext_spd(self):
        # 外部速度の設定値を返す
        return self._bcap.robot_execute(self._hRob, "CurExtSpd")

    def cur_ext_acc(self):
        # 外部加速度の設定値を返す
        return self._bcap.robot_execute(self._hRob, "CurExtAcc")

    def speed(self, value: float):
        # 内部速度を設定する(100%まで)
        # -1: 手先速度を表す
        # 通常TakeArmで100%に初期化される
        self._bcap.robot_speed(self._hRob, -1, value)

    def ext_speed(
        self,
        speed: float = 20,
        accel: float = -2, 
        decel: float = -2,
    ):
        # 外部速度、加速度、減速度を設定する(100%まで。-1はそのまま、-2は
        # 外部速度の二乗を100で割った値)
        # 実測度は外部速度と内部速度の掛け算で決定される
        return self._bcap.robot_execute(
            self._hRob,
            "ExtSpeed",
            [speed, accel, decel],
        )

    def accelerate(self, accel: float = -1, decel: float = -1):
        # 内部加速度、減速度を設定する(100%まで。-1はそのまま)
        # -1: 手先加速度を表す
        # 通常TakeArmで100%に初期化される
        self._bcap.robot_accelerate(self._hRob, -1, accel, decel)
