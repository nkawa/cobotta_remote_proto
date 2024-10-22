"""軌跡、軌道 (タイムスタンプ付きの軌跡) を扱うためのモジュール。"""
import copy
import json
import math
from typing import Any, Dict, List, Optional, Tuple, Union

from matplotlib.axes import Axes
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def generate_traj(
    axes: List[int],
    amplitudes: List[float],
    n_waypoints: int = 1000,
    duration: int = 8,
) -> Tuple[List[List[float]], List[float]]:
    """
    軌道として時刻のリストと姿勢のリストを生成する。

    姿勢は[x, y, z, rx, ry, rz]を仮定する。

    姿勢の各要素 (軸) は、サインカーブに近いが、最初と最後の速度が0になるような曲線から
    サンプリングするか、すべて0の系列となるかのどちらかになる。
    どの軸をサインカーブに近い曲線からサンプリングするかはaxesで指定する。
    amplitudesは対応するaxesの曲線の振幅を指定する。

    n_waypointsは曲線のサンプリング数を指定する。

    durationは軌道の最初から最後までの時間を指定する。

    返り値は、timestamps (時刻のリスト) とtarget_poses (姿勢のリスト)の
    タプルである。
    """
    assert len(axes) == len(amplitudes)
    target_poses = []
    timestamps = []
    original_pose = [0, 0, 0, 0, 0, 0]
    for i in range(n_waypoints):
        target_pose = copy.copy(original_pose)
        for ax, amp in zip(axes, amplitudes):
            # サインカーブに近いが、最初と最後の速度が0になるような曲線
            target_pose[ax] = original_pose[0] + \
                amp / 2 * \
                (1 -  math.cos(4 * math.pi * i / (n_waypoints - 1))) * \
                (2 * (math.sin(2 * math.pi * i / (n_waypoints - 1)) >= 0) - 1)
            # サインカーブ
            # target_pose[ax] = original_pose[0] + \
            #     amp * math.sin(2 * math.pi * i / n_waypoints)
        target_poses.append(target_pose)
        timestamps.append(duration / (n_waypoints - 1) * i)
    return timestamps, target_poses

def diff_poses(target_poses: List[List[float]]) -> List[List[float]]:
    """
    姿勢のリストの差分のリストを作成する。
    """
    poses = np.array(target_poses)
    diffs = np.concatenate(
        [
            np.zeros((1, poses.shape[1])),
            np.diff(poses, axis=0)
        ],
        axis=0,
    )
    return diffs

def traj_to_mqtt(
    timestamps: List[float], target_poses: List[List[float]],
) -> List[Dict[str, Any]]:
    """
    軌道を表すtimestamps (時刻のリスト) とtarget_poses (姿勢のリスト)から
    mqttメッセージのjsonのリストを作成する。
    """
    jss = []
    for t, p in zip(timestamps, target_poses):
        x, y, z, xd, yd, zd = p
        js = {}
        js["pos"] = {}
        js['pos']['x'] = x
        js['pos']['y'] = y
        js['pos']['z'] = z
        js["ori"] = {}
        js['ori']['x'] = xd
        js['ori']['y'] = yd
        js['ori']['z'] = zd
        js['pad'] = {}
        js['pad']['bA'] = False
        js['pad']['b0'] = 0
        js["time"] = t
        jss.append(js)
    return jss

def save_mqtt(
    jss: List[Dict[str, Any]],
    save_path: str = "target.jsonl"
) -> None:
    """
    mqttメッセージのjsonのリストをjsonlとして保存する。
    """
    with open(save_path, "w") as f:
        for js in jss:
            f.write(json.dumps(js) + "\n")

def load_mqtt(path: str) -> List[Dict[str, Any]]:
    """
    jsonlからmqttメッセージのjsonのリストを読み込む。
    """
    with open(path, "r") as f:
        jss = [json.loads(line) for line in f]
    return jss

def mqtt_to_traj(jss: List[Dict[str, Any]]) -> Tuple[List[float], List[List[float]]]:
    """
    mqttメッセージのjsonのリストから
    軌道を表すtimestamps (時刻のリスト) とtarget_poses (姿勢のリスト)を
    抽出する。
    """
    timestamps, target_poses = [], []
    for js in jss:
        if ('pos' in js) and ('time' in js):
            x = js['pos']['x']
            y = js['pos']['y']
            z = js['pos']['z']
            xd = js['ori']['x']
            yd = js['ori']['y']
            zd = js['ori']['z']
            t = js['time']
            timestamps.append(t)
            target_poses.append([x, y, z, xd, yd, zd])
    return timestamps, target_poses

def mqtt_to_dataframe(jss: List[Dict[str, Any]]) -> None:
    """
    mqttメッセージのjsonのリストをデータフレームに変換。
    """
    data = []
    for js in jss:
        if ('pos' in js) and ('time' in js):
            data.append(dict(
                x=js['pos']['x'],
                y=js['pos']['y'],
                z=js['pos']['z'],
                xd=js['ori']['x'],
                yd=js['ori']['y'],
                zd=js['ori']['z'],
                t=js['time'],
            ))
    df = pd.DataFrame(data)
    return df

def plot_dataframe(
    df: pd.DataFrame,
    kinds: Union[str, List[str]] = "all",
    coords: Union[str, List[str]] = "all",
    params: Union[str, List[str]] = "all",
    kwargs_by_kinds: Optional[Dict[str, Any]] = None,
    show: bool = False,
    **kwargs,
) -> Axes:
    """
    mqttメッセージのjsonのリストから変換したデータフレームを
    可視化する。

    引数:
        - df: データフレーム
        - kinds: 可視化するデータの種類 (例: "target", "control", "status")。"all"はすべて。
        - coords: 可視化する姿勢 ("x", "y", "z", "xd", "yd", "zd")。"all"はすべて。
        - params: 可視化するkinematics (位置: "pos", 速度: "vel", 加速度: "acc")。"all"はすべて。
        - kwargs_by_kinds: データの種類ごとのプロット用変数。
        - show: Trueの場合、plt.show()を実行し、axを返さない。Falseの場合、axを返すだけ。
        - kwargs: プロット用変数。
    """
    if "kind" in df.columns:
        if kinds == "all":
            kinds = df["kind"].unique().tolist()
        dfs = [df[df["kind"] == kind] for kind in kinds]
        kind_exists = True
        n_kinds = len(kinds)
        if kwargs_by_kinds is None:
            kwargs_by_kinds = {kind: kwargs for kind in kinds}
    else:
        dfs = [df]
        kind_exists = False
    coord_choices = ["x", "y", "z", "xd", "yd", "zd"]
    if isinstance(coords, str):
        if coords == "all":
            coords = coord_choices
        else:
            assert coords in coord_choices
            coords = [coords]
    param_choices = ["pos", "vel", "acc"]
    if isinstance(params, str):
        if params == "all":
            params = param_choices
        else:
            assert params in param_choices
            params = [params]
    n_coords = len(coords)
    n_params = len(params)
    _, axs = plt.subplots(
        nrows=n_coords * n_params,
        ncols=1,
        figsize=(18, 3 * n_coords * n_params),
        layout="constrained",
        sharex=True,
    )
    if n_coords * n_params == 1:
        axs = [axs]
    for ik, df in enumerate(dfs):
        for ic, coord in enumerate(coords):
            ts = df["t"]
            vs = df[coord].diff() / ts.diff()
            accs = vs.diff() / ts.diff()
            for ip, param in enumerate(params):
                ax = axs[n_params * ic + ip]
                if param == "pos":
                    line = ax.plot(ts, df[coord], kwargs.get("fmt", "-"))[0]
                    ax.set_ylabel(f"${coord}$")
                elif param == "vel":
                    line = ax.plot(ts, vs, kwargs.get("fmt", "-"))[0]
                    ax.set_ylabel(f"$v_{{{coord}}}$")
                else:
                    line = ax.plot(ts, accs, kwargs.get("fmt", "-"))[0]
                    ax.set_ylabel(f"$a_{{{coord}}}$")
                ax.set_xlabel("time (s)")
                ax.xaxis.set_tick_params(labelbottom=True)
                if kind_exists:
                    line.set_label(kinds[ik])
                    line.set_color(f"C{ik}")
                    color = kwargs_by_kinds[kinds[ik]].get("color", None)
                    if color is not None:
                        line.set_color(color)
                    else:
                        color = kwargs.get("color", None)
                        if color is not None:
                            line.set_color(color)
                    linestyle = kwargs_by_kinds[kinds[ik]].get("linestyle", None)
                    if linestyle is not None:
                        line.set_linestyle(linestyle)
                    marker = kwargs_by_kinds[kinds[ik]].get("marker", None)
                    if marker is not None:
                        line.set_marker(marker)
                if kind_exists and ik == n_kinds - 1:
                    ax.legend()
    if n_coords * n_params == 1:
        axs = axs[0]
    if show:
        plt.show()
    else:
        return axs

if __name__ == "__main__":
    # 軌道の作成例
    timestamps, target_poses = generate_traj(
        [0],
        [25],
        n_waypoints=1000,
        duration=8,
    )
    jss = traj_to_mqtt(timestamps, target_poses)
    save_mqtt(jss, "target.jsonl")
