"""座標変換を行うモジュール。"""
import numpy as np


class Transform:
    def __init__(
        self,
        x: str = "x",
        y: str = "y",
        z: str = "z",
        xd: str = "xd",
        yd: str = "yd",
        zd: str = "zd",
    ) -> None:
        """
        座標系の変換を行う。
        引数は、<変換元の座標系の軸> = "±<変換先の座標系の軸>"
        となっている。
        """
        transform = {"x": x, "y": y, "z": z, "xd": xd, "yd": yd, "zd": zd}
        pos_strs = ["x", "y", "z"]
        rot_strs = ["xd", "yd", "zd"]
        # [x, y, z]内の置換と正負の反転のみであること
        assert all(x == y for x, y in zip(sorted([transform[s].strip("-") for s in pos_strs]), pos_strs))
        # [xd, yd, zd]内の置換と正負の反転のみであること
        assert all(x == y for x, y in zip(sorted([transform[s].strip("-") for s in rot_strs]), rot_strs))
        offsets = [0, 3]
        self.permute = np.arange(6)
        self.sign = np.ones(6)
        for offset, strs in zip(offsets, [pos_strs, rot_strs]):
            for i, s in enumerate(strs):
                t = transform[s]
                if t.startswith("-"):
                    self.sign[i + offset] = -1
                    t = t.strip("-")
                self.permute[i + offset] = strs.index(t) + offset

    def __call__(
        self, x: float, y: float, z: float, xd: float, yd: float, zd: float
    ) -> np.ndarray:
        return (np.array([x, y, z, xd, yd, zd])[self.permute] * self.sign).tolist()
