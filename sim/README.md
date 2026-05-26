# Sim2Real - MuJoCo & PyBullet Simulation

这是一个包含 MuJoCo 和 PyBullet 两个仿真环境的项目，用于显示世界坐标系。

## 项目结构

```
sim/
├── mujoco_world.xml      # MuJoCo 世界模型文件
├── mujoco_sim.py         # MuJoCo 仿真脚本
├── pybullet_sim.py       # PyBullet 仿真脚本
├── requirements.txt      # Python 依赖包
└── README.md            # 说明文档
```

## 功能特性

- ✅ **MuJoCo 仿真环境**：使用 XML 定义的世界坐标系
- ✅ **PyBullet 仿真环境**：使用 Python API 创建的世界坐标系
- ✅ **世界坐标系显示**：
  - X轴（红色）：右方向
  - Y轴（绿色）：前方向
  - Z轴（蓝色）：上方向
- ✅ **交互式相机控制**：旋转、平移、缩放

## 安装依赖

### 方法1：使用 pip 安装

```bash
pip install -r requirements.txt
```

### 方法2：手动安装

```bash
# MuJoCo
pip install mujoco numpy

# PyBullet
pip install pybullet
```

## 运行仿真

### 运行 MuJoCo 仿真

```bash
cd C:\www\codeing\aider\aider_terminal\sim
python mujoco_sim.py
```

### 运行 PyBullet 仿真

```bash
cd C:\www\codeing\aider\aider_terminal\sim
python pybullet_sim.py
```

## 使用说明

### 相机控制
- **左键拖动**：旋转相机视角
- **右键拖动**：平移相机
- **滚轮**：缩放视图

### 退出
- 关闭窗口或按 ESC 键退出

## 坐标系说明

两个仿真环境都使用右手坐标系：
- **X轴（红色）**：向右
- **Y轴（绿色）**：向前
- **Z轴（蓝色）**：向上

原点处有一个白色球体标记，三个坐标轴各长 1 米，末端有箭头指示方向。

## 注意事项

1. **MuJoCo** 需要许可证（个人使用免费），首次运行时会自动配置
2. **PyBullet** 完全免费开源，无需额外配置
3. 确保你的系统支持 OpenGL 渲染
4. 如果遇到显示问题，尝试更新显卡驱动

## 开发信息

- Python 版本：3.13+
- 操作系统：Windows / Linux / macOS
- 图形要求：支持 OpenGL 的显卡
