#!/usr/bin/env python3
"""Forward kinematics computer — pure Python, no rendering dependency.

Parses the URDF kinematic tree and computes world-space end-effector
positions for a given set of joint values.
"""
import xml.etree.ElementTree as ET
import math
import os
from typing import Dict, Tuple, List, Optional

import numpy as np

# URDF 文件位于 aider_terminal/URDF/aider/ 目录
# 通过 kinematic 包的 _PROJ_ROOT 定位项目根，避免数 dirname 层数
from .. import _PROJ_ROOT
URDF_DIR = os.path.join(_PROJ_ROOT, "URDF", "aider")
URDF_PATH = os.path.join(URDF_DIR, "urdf", "aider_pro.SLDASM.urdf")


# ---------------------------------------------------------------------------
# 4×4 变换矩阵工具
# ---------------------------------------------------------------------------

def _translate(x: float, y: float, z: float) -> np.ndarray:
    """构造 4×4 平移变换矩阵。"""
    m = np.eye(4)
    m[:3, 3] = [x, y, z]
    return m


def _rotate_axis(axis: Tuple[float, float, float], angle: float) -> np.ndarray:
    """绕任意单位轴旋转的 4×4 矩阵 (Rodrigues)."""
    ux, uy, uz = axis
    c = math.cos(angle)
    s = math.sin(angle)
    t = 1 - c
    return np.array([
        [c + ux*ux*t,     ux*uy*t - uz*s,  ux*uz*t + uy*s, 0],
        [uy*ux*t + uz*s,  c + uy*uy*t,     uy*uz*t - ux*s, 0],
        [uz*ux*t - uy*s,  uz*uy*t + ux*s,  c + uz*uz*t,    0],
        [0,               0,               0,               1],
    ])


def _rpy(rpy_str: str) -> np.ndarray:
    """Convert 'r p y' (radians) space-separated string → rotation matrix."""
    parts = [float(x) for x in rpy_str.split()]
    r, p, y = (parts + [0, 0, 0])[:3]
    Rx = _rotate_axis((1, 0, 0), r)
    Ry = _rotate_axis((0, 1, 0), p)
    Rz = _rotate_axis((0, 0, 1), y)
    return Rx @ Ry @ Rz


# ---------------------------------------------------------------------------
# URDF 解析 & FK
# ---------------------------------------------------------------------------

class FKComputer:
    """Loads a URDF and computes forward kinematics on demand."""

    def __init__(self, urdf_path: str = URDF_PATH):
        """加载 URDF 文件并解析运动学树。"""
        self.tree = ET.parse(urdf_path)
        self.root = self.tree.getroot()
        self._links: Dict[str, dict] = {}
        self._joints: Dict[str, dict] = {}
        self._parents: Dict[str, str] = {}
        self._parse()

    # -- 解析 ---------------------------------------------------------------

    def _parse(self) -> None:
        """解析 URDF XML：提取 links、joints、origin 变换、旋转轴、关节限位。"""
        for el in self.root.findall("link"):
            name = el.attrib["name"]
            self._links[name] = {"name": name}

        for el in self.root.findall("joint"):
            name = el.attrib["name"]
            jtype = el.attrib["type"]
            parent = el.find("parent").attrib["link"]
            child = el.find("child").attrib["link"]

            org = el.find("origin")
            xyz = tuple(float(v) for v in (org.attrib.get("xyz", "0 0 0").split()))
            rpy_str = org.attrib.get("rpy", "0 0 0")
            rpy_mat = _rpy(rpy_str)
            T_origin = _translate(*xyz) @ rpy_mat

            axis_el = el.find("axis")
            axis = tuple(float(v) for v in (axis_el.attrib.get("xyz", "0 0 1").split())) if axis_el is not None else (0, 0, 1)

            lim_el = el.find("limit")
            lower = float(lim_el.attrib.get("lower", 0)) if lim_el is not None else 0
            upper = float(lim_el.attrib.get("upper", 0)) if lim_el is not None else 0

            self._joints[name] = {
                "name": name,
                "type": jtype,
                "parent": parent,
                "child": child,
                "origin": T_origin,
                "axis": axis,
                "lower": lower,
                "upper": upper,
            }
            self._parents[child] = parent

    # -- 单关节变换 ---------------------------------------------------------

    def _joint_transform(self, jname: str, value: float) -> np.ndarray:
        """4×4 变换矩阵: 父连杆坐标系 → 子连杆坐标系。"""
        j = self._joints.get(jname)
        if j is None:
            return np.eye(4)

        if j["type"] == "prismatic":
            ax = np.array(j["axis"])
            T_var = _translate(*(ax * value))
        elif j["type"] in ("revolute", "continuous"):
            T_var = _rotate_axis(j["axis"], value)
        else:
            T_var = np.eye(4)

        return j["origin"] @ T_var

    # -- 正运动学 ----------------------------------------------------------

    def _forward_chain(self, leaf_link: str, joint_values: Dict[str, float]) -> np.ndarray:
        """给定叶子连杆名和关节值，返回该连杆原点在世界系的 4×4 位姿。"""
        path = [leaf_link]
        while path[-1] in self._parents:
            path.append(self._parents[path[-1]])
        path.reverse()

        T = np.eye(4)
        for i in range(len(path) - 1):
            parent = path[i]
            child = path[i + 1]
            jname = self._joint_for(parent, child)
            val = joint_values.get(jname, 0.0)
            T_joint = self._joint_transform(jname, val)
            T = T @ T_joint
        return T

    def _joint_for(self, parent: str, child: str) -> Optional[str]:
        """根据父子连杆名查找关节名。"""
        for name, j in self._joints.items():
            if j["parent"] == parent and j["child"] == child:
                return name
        return None

    def pos(self, leaf_link: str, joint_values: Dict[str, float]) -> Tuple[float, float, float]:
        """返回 leaf_link 原点在 base_link 坐标系下的 (x, y, z)."""
        T = self._forward_chain(leaf_link, joint_values)
        return tuple(T[:3, 3].tolist())

    # -- 末端执行器便捷接口 -------------------------------------------------

    def end_effectors(
        self, joint_values: Dict[str, float]
    ) -> Dict[str, Tuple[float, float, float]]:
        """返回左右臂末端在 base_link 下的世界坐标 (URDF Z-up)。

        返回: {
            "left":  (x, y, z),
            "right": (x, y, z),
        }
        """
        result = {}
        for side, link in [("left", "left_arm8"), ("right", "right_arm8")]:
            if link in self._links:
                result[side] = self.pos(link, joint_values)
        return result

    # -- 任意连杆位置 -------------------------------------------------------

    def link_pos(self, link_name: str, joint_values: Dict[str, float]) -> Tuple[float, float, float]:
        """返回任意连杆原点在 base_link 下的 (x, y, z)."""
        if link_name not in self._links:
            return (0.0, 0.0, 0.0)
        return self.pos(link_name, joint_values)

    # -- 关节元数据 ---------------------------------------------------------

    def joint_info(self) -> Dict[str, dict]:
        """返回所有关节的元数据（类型、限位、父子关系）。"""
        return {
            name: {
                "type": j["type"],
                "parent": j["parent"],
                "child": j["child"],
                "axis": list(j["axis"]),
                "lower": j["lower"],
                "upper": j["upper"],
            }
            for name, j in self._joints.items()
        }

    def joint_names(self) -> List[str]:
        """返回所有关节名称列表。"""
        return list(self._joints.keys())


# ---------------------------------------------------------------------------
# 自测
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    eng = FKComputer()
    print("Joints:", list(eng._joints.keys()))

    zero = eng.end_effectors({})
    print("Zero pose:")
    print(f"  left  = ({zero['left'][0]:.4f}, {zero['left'][1]:.4f}, {zero['left'][2]:.4f})")
    print(f"  right = ({zero['right'][0]:.4f}, {zero['right'][1]:.4f}, {zero['right'][2]:.4f})")

    test_vals = {
        "left_arm2": 0.3, "left_arm3": -0.5, "left_arm4": 0.4,
        "right_arm2": 0.3, "right_arm3": -0.5, "right_arm4": 0.4,
        "lift_Link": 0.1, "waist_Link": 0.2,
    }
    pose = eng.end_effectors(test_vals)
    print("Test pose:")
    print(f"  left  = ({pose['left'][0]:.4f}, {pose['left'][1]:.4f}, {pose['left'][2]:.4f})")
    print(f"  right = ({pose['right'][0]:.4f}, {pose['right'][1]:.4f}, {pose['right'][2]:.4f})")
