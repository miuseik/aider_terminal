"""VR 动作录制 / 回放。

录制发生在 Terminal 端（control_loop 每帧解算后），存到本地 recordings/ 目录。
前端只发指令（start/stop/play/rename）并通过 status 拿到动作列表。

录制数据每帧包含（左右臂都记）：
- position: 手柄原始位置 (VR 房间系, WebXR local-floor)
- target_position: TCP 位置 [x,y,z]
- target_orientation: TCP 姿态四元数 [x,y,z,w]
- joints: 8 个 arm 关节角 [deg]
- gripper: 夹爪角度 [deg]
- trigger / joystick: 控制激活状态

rec_type 决定回放通道：
- "target": 走 AIInputProvider.send_tcp（绝对 TCP 位姿），用于纯 VR
- "joint":  走关节角直发 adapter，用于 VR+外骨骼混合
两种数据录制时都存，回放按 rec_type 选通道。
"""
import os
import json
import time
import asyncio
import threading

REC_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__))))), "recordings")

_lock = threading.Lock()


def _ensure_dir():
    os.makedirs(REC_DIR, exist_ok=True)


def list_recordings():
    """返回动作名列表（带元信息）。"""
    _ensure_dir()
    out = []
    for fn in sorted(os.listdir(REC_DIR)):
        if fn.endswith(".json"):
            try:
                with open(os.path.join(REC_DIR, fn), "r", encoding="utf-8") as f:
                    d = json.load(f)
                out.append({
                    "name": d.get("name", fn[:-5]),
                    "rec_type": d.get("rec_type", "target"),
                    "frames": len(d.get("frames", [])),
                    "duration": d.get("duration", 0.0),
                })
            except Exception:
                pass
    return out


def rename_recording(old_name, new_name):
    _ensure_dir()
    old_path = os.path.join(REC_DIR, old_name + ".json")
    new_path = os.path.join(REC_DIR, new_name + ".json")
    if not os.path.exists(old_path):
        return False
    if os.path.exists(new_path):
        return False
    os.rename(old_path, new_path)
    try:
        with open(new_path, "r", encoding="utf-8") as f:
            d = json.load(f)
        d["name"] = new_name
        with open(new_path, "w", encoding="utf-8") as f:
            json.dump(d, f, ensure_ascii=False, indent=2)
    except Exception:
        pass
    return True


class Recording:
    """一次录制会话。"""

    def __init__(self, name, rec_type):
        self.name = name
        self.rec_type = rec_type  # "target" | "joint"
        self.frames = []
        self.start_time = time.time()
        self.frame_interval = 0.02  # 50Hz 采样
        self._last_sample = 0.0

    def sample(self, left_state, right_state, robot_interface):
        """采集一帧。left_state/right_state 为 ArmState，已解算完。

        按 frame_interval 节流，避免录制帧率过高。
        """
        now = time.time()
        if now - self._last_sample < self.frame_interval:
            return
        self._last_sample = now

        def grab(arm_name, arm_state):
            joints = []
            gripper = None
            try:
                angles = robot_interface.get_arm_angles(arm_name)
                joints = [float(a) for a in angles[:8]]
                from aiderminal.robots.aloha.settings import GRIPPER_INDEX
                gripper = float(angles[GRIPPER_INDEX]) if len(angles) > GRIPPER_INDEX else None
            except Exception:
                pass
            tp = arm_state.target_position
            to = arm_state.target_orientation
            trigger_key = f"{arm_name}Controller"
            vr_raw = getattr(robot_interface, "vr_raw_data", {}) or {}
            trigger = vr_raw.get(trigger_key, {}).get("trigger", None)
            joystick = vr_raw.get(trigger_key, {}).get("joystick", None)
            vr_pos = vr_raw.get(trigger_key, {}).get("position", None)
            return {
                "position": [float(vr_pos[k]) for k in ('x', 'y', 'z')] if vr_pos else None,
                "target_position": [float(v) for v in tp] if tp is not None else None,
                "target_orientation": [float(v) for v in to] if to is not None else None,
                "joints": joints,
                "gripper": gripper,
                "trigger": trigger,
                "joystick": joystick,
            }

        frame = {
            "t": round(now - self.start_time, 3),
            "left": grab("left", left_state),
            "right": grab("right", right_state),
        }
        self.frames.append(frame)

    def save(self):
        _ensure_dir()
        duration = round(time.time() - self.start_time, 3)
        data = {
            "name": self.name,
            "rec_type": self.rec_type,
            "frame_rate": round(1.0 / self.frame_interval, 1),
            "duration": duration,
            "frames": self.frames,
        }
        path = os.path.join(REC_DIR, self.name + ".json")
        with open(path, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
        return path


class PlaybackProvider:
    """回放录制动作。基于 AIInputProvider 的 TCP 通道 + 关节角直发。"""

    def __init__(self, control_loop):
        self.cl = control_loop
        self._task = None
        self._stop = False

    async def play(self, name, rec_type):
        """逐帧回放。rec_type 决定通道。"""
        _ensure_dir()
        path = os.path.join(REC_DIR, name + ".json")
        if not os.path.exists(path):
            print(f"❌ [Playback] 找不到录制: {name}")
            return
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        frames = data.get("frames", [])
        if not frames:
            print(f"❌ [Playback] 录制 {name} 无帧")
            return

        ri = self.cl.robot_interface
        if not ri:
            print("❌ [Playback] 无 robot_interface")
            return

        from aiderminal.inputs.ai_handler import AIInputProvider
        ai = AIInputProvider(self.cl.command_queue)
        await ai.start()

        # 激活双臂位置控制（类似握把按下）
        await ai.enable("left")
        await ai.enable("right")

        self._stop = False
        interval = 1.0 / data.get("frame_rate", 50.0)
        print(f"▶️ [Playback] 开始回放 {name} ({rec_type}), {len(frames)} 帧, {interval:.3f}s/帧")

        for fr in frames:
            if self._stop:
                break
            for arm_name in ("left", "right"):
                arm_fr = fr.get(arm_name, {})
                if rec_type == "target":
                    tp = arm_fr.get("target_position")
                    to = arm_fr.get("target_orientation")
                    if tp:
                        await ai.send_tcp(arm_name, tp, to)
                else:  # joint
                    joints = arm_fr.get("joints")
                    gripper = arm_fr.get("gripper")
                    if joints:
                        angles = [float(a) for a in joints[:8]]
                        while len(angles) < 8:
                            angles.append(0.0)
                        if gripper is not None:
                            from aiderminal.robots.aloha.settings import GRIPPER_INDEX
                            if len(angles) > GRIPPER_INDEX:
                                angles[GRIPPER_INDEX] = float(gripper)
                        try:
                            ri.update_arm_angles(
                                arm_name,
                                angles,
                                0.0, 0.0,
                                gripper if gripper is not None else 0.0,
                                0.0,
                                override_wrist=True,
                            )
                        except Exception as e:
                            print(f"⚠️ [Playback] update_arm_angles 失败: {e}")
            await asyncio.sleep(interval)

        await ai.disable("left")
        await ai.disable("right")
        print(f"⏹️ [Playback] 回放结束: {name}")

    def stop(self):
        self._stop = True
