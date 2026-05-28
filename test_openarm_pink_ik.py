import os
import sys
import time

# 禁止 MeshCat 自动打开浏览器
os.environ["BROWSER"] = ""

import meshcat_shapes
import numpy as np
import qpsolvers
from loop_rate_limiters import RateLimiter
import pink
from pink import solve_ik
from pink.tasks import FrameTask, PostureTask
from pink.utils import custom_configuration_vector
from pink.visualization import start_meshcat_visualizer
import pinocchio as pin


class OpenArmXSolver():
    def __init__(
        self,
        # 使用 OpenArmX 的双臂 URDF 路径
        urdf_path: str = "C:/www/codeing/open_origin/openArmX/openarmx_mujoco/openarmx_v10_bimanual.urdf"
    ):
        print(f"Loading URDF from: {urdf_path}")
        
        # 提取 URDF 文件所在目录以定位资源文件 (meshes)
        urdf_package_path = os.path.dirname(urdf_path)
        
        # 关键：将 openarmx_description 路径映射到 package://openarmx_description
        # Pinocchio 会在这些目录下寻找与 package 名称匹配的文件夹
        package_dirs = [
            "C:/www/codeing/open_origin/openArmX",
            urdf_package_path
        ]
        
        # 加载机器人模型
        self.robot = pin.RobotWrapper.BuildFromURDF(
            urdf_path,
            package_dirs=package_dirs,
            root_joint=pin.JointModelFreeFlyer() # 增加浮动基座，方便仿真观察
        )

        self.init_base_position = np.array([0.0, 0.0, 0.5]) # 抬高一点，避免撞地
        self.init_base_orientation = np.array([1.0, 0.0, 0.0, 0.0])

        # MeshCat 可视化器
        print("Starting MeshCat visualizer...")
        self.visualizer = start_meshcat_visualizer(self.robot)
        self.viewer = self.visualizer.viewer
        print("Please open the visualizer manually at: http://127.0.0.1:7000")

        # --- 关键修改：根据 OpenArmX URDF 定义末端执行器 ---
        # 根据刚才运行输出的 Frame 列表，确认末端名称为 openarmx_left_hand_tcp 和 openarmx_right_hand_tcp
        self.end_effectors: list = ["openarmx_right_hand_tcp", "openarmx_left_hand_tcp"]
        
        # 检查名称是否存在
        for name in self.end_effectors:
            if self.robot.model.getFrameId(name) == len(self.robot.model.frames):
                print(f"Warning: Frame '{name}' not found in model. Check your URDF.")
        
        self.end_effector_ids = [self.robot.model.getFrameId(name) for name in self.end_effectors]

        # 为末端执行器创建可视化坐标系
        for name in self.end_effectors:
            meshcat_shapes.frame(self.viewer[name + "_target"], opacity=0.5)
            meshcat_shapes.frame(self.viewer[name], opacity=1.0)

        self.end_effector_tasks = [FrameTask(
            name,
            position_cost=1.0,  # [代价] / [米]
            orientation_cost=0.0, # 纯位置控制
            lm_damping=1.0,
        ) for name in self.end_effectors]

        # 基座任务：保持机器人稳定
        self.base_tasks = [FrameTask(
            "root_joint", # 对应 FreeFlyer 的基座
            position_cost=100.0,
            orientation_cost=100.0,
            lm_damping=1.0,
        )]

        # 选择 QP 求解器
        self.solver = qpsolvers.available_solvers[0]
        if "quadprog" in qpsolvers.available_solvers:
            self.solver = "quadprog"

        self.last_update_time = time.perf_counter()

        # --- 关键修改：使用命名参数针对性调整每个关节 ---
        
        # 1. 打印所有关节名，方便我们“点名”
        print("\n--- OpenArmX Joint Names (Copy these for custom_configuration_vector) ---")
        # njoints 包含了 universe (索引0)，所以我们从 1 开始遍历
        for i in range(1, self.robot.model.njoints): 
            print(f"{self.robot.model.names[i]},")
        print("---------------------------------------------------------------\n")

        # 2. 使用命名参数初始化姿态偏好 (就像 Berkeley 那样)
        # 注意：这里使用的是上面打印出来的真实关节名
        try:
            self.q = pink.utils.custom_configuration_vector(
                self.robot,
                # --- 左臂姿态偏好 (向外、向后) ---
                openarmx_left_joint1=-0.1,
                openarmx_left_joint2=-0.3,    # 【向外】肩 roll (外展) -> 增大到 0.6
                openarmx_left_joint3=0.5,   # 【向后】肩 pitch (后伸) -> 设为负值
                openarmx_left_joint4=0.5,    # 肘 pitch (弯曲)
                openarmx_left_joint5=0.0,    
                openarmx_left_joint6=0.0,    
                openarmx_left_joint7=0.0,    
                
                # --- 右臂姿态偏好 (对称：向外、向后) ---
                openarmx_right_joint1=-0.1,
                openarmx_right_joint2=0.3,  # 【向外】肩 roll (外展) -> 负值代表向右撇
                openarmx_right_joint3=-0.5,  # 【向后】肩 pitch (后伸)
                openarmx_right_joint4=0.5,   # 肘 pitch (弯曲)
                openarmx_right_joint5=0.0,
                openarmx_right_joint6=0.0,
                openarmx_right_joint7=0.0,
            )
        except Exception as e:
            print(f"Named init error: {e}. Falling back to zero config.")
            self.q = np.zeros(self.robot.model.nq)
            self.q[2] = 0.5 # z-axis height
            self.q[6] = 1.0 # quaternion w

        # 3. 创建 PostureTask 并设置目标
        # 降低权重到 0.05，提高响应速度，减少滞后感
        self.posture_task = PostureTask(cost=0.05) 
        self.posture_task.set_target(self.q)

        print("Solver initialized successfully!")

    def update(self):

        
        t = time.perf_counter()

        # --- 增加 10 秒延时启动逻辑 ---
        if not hasattr(self, 'start_time'):
            self.start_time = t
            print("Waiting 10 seconds before starting motion...")
            # 初始化随机目标点
            self.current_target_left = np.array([0.3, 0.2, 0.5])
            self.current_target_right = np.array([0.3, -0.2, 0.5])
        
        if t - self.start_time < 10.0:
            # 在前 10 秒内，只更新显示，不改变目标点位置
            configuration = pink.Configuration(self.robot.model, self.robot.data, self.q)
            for i, name in enumerate(self.end_effectors):
                current_frame = configuration.get_transform_frame_to_world(self.end_effector_tasks[i].frame)
                self.viewer[name].set_transform(current_frame.np)
            self.visualizer.display(configuration.q)
            return configuration.q
        # ---------------------------------------

        # --- 恢复平滑的正弦波轨迹 ---
        delta_poses = [pin.SE3.Identity() for _ in range(len(self.end_effectors))]

        # 右手轨迹：平滑运动
        delta_poses[0].translation[0] = 0.35 + 0.10 * np.cos(1.0 * t)  
        delta_poses[0].translation[1] = -0.20 + 0.10 * np.sin(1.0 * t) 
        delta_poses[0].translation[2] = 0.50 + 0.10 * np.cos(1.5 * t)  

        # 左手轨迹：对称平滑运动
        delta_poses[1].translation[0] = 0.35 + 0.10 * np.sin(1.0 * t)
        delta_poses[1].translation[1] = 0.20 + 0.10 * np.cos(1.0 * t)  
        delta_poses[1].translation[2] = 0.50 + 0.10 * np.sin(1.5 * t) 

        desired_poses = delta_poses

        configuration = pink.Configuration(self.robot.model, self.robot.data, self.q)

        # 更新可视化
        for i, name in enumerate(self.end_effectors):
            self.viewer[name + "_target"].set_transform(desired_poses[i].np)
            current_frame = configuration.get_transform_frame_to_world(self.end_effector_tasks[i].frame)
            self.viewer[name].set_transform(current_frame.np)

        # 设置任务目标
        for i in range(len(self.end_effector_tasks)):
            self.end_effector_tasks[i].transform_target_to_world = desired_poses[i]
        
        # 保持基座不动
        self.base_tasks[0].transform_target_to_world = pin.SE3(
            pin.Quaternion.Identity(), 
            self.init_base_position
        )

        tasks = self.end_effector_tasks + self.base_tasks + [self.posture_task]

        dt = time.perf_counter() - self.last_update_time
        self.last_update_time = time.perf_counter()
        
        # 限制 dt 防止积分爆炸
        dt = min(dt, 0.05)

        # 计算速度并积分
        try:
            velocity = solve_ik(configuration, tasks, dt, solver=self.solver, safety_break=False)
            configuration.integrate_inplace(velocity, dt)
        except Exception as e:
            print(f"IK Solve Error: {e}")

        # 打印当前关节角度 (前几个)
        # print(configuration.q[7:17])

        # 显示
        self.visualizer.display(configuration.q)
        self.q = configuration.q

        return configuration.q


if __name__ == "__main__":
    solver = OpenArmXSolver()
    rate = RateLimiter(frequency=100.0, warn=False)

    print("Press Ctrl+C to stop.")
    try:
        while True:
            solver.update()
            rate.sleep()
    except KeyboardInterrupt:
        print("\nSimulation stopped.")
