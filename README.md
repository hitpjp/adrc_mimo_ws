# MIMO ADRC Omni-Directional UAV Simulation
**基于自抗扰控制 (ADRC) 的全向无人机 ROS 2 仿真系统**

![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue) ![C++](https://img.shields.io/badge/Language-C++17-orange) ![Platform](https://img.shields.io/badge/Platform-Linux%20%7C%20WSL2-green) ![License](https://img.shields.io/badge/License-MIT-lightgrey)

这是一个基于 **ROS 2 Humble** 开发的全向无人机（类似 Voliro 构型）仿真项目。核心采用 **MIMO ADRC (多输入多输出自抗扰控制)** 算法，实现了对全向飞行器的解耦控制、高精度轨迹跟踪（8字飞行）以及抗扰动能力。

项目内置了完整的动力学仿真环境与 Rviz2 可视化接口，并专门针对 **WSL2 (Windows Subsystem for Linux)** 环境进行了图形渲染优化。

## ✨ 主要功能 (Key Features)

* **MIMO ADRC 控制器**: 实现了多通道的 LADRC (线性自抗扰控制) 算法，能够有效估计并补偿系统内部动态和外部扰动，实现姿态与位置的解耦控制。
* **全向飞行仿真**: 模拟了全驱动无人机的 6-DOF 动力学特性，支持任意姿态下的平移运动 (例如：30° 固定俯仰角下的 8 字绕飞)。
* **实时可视化**: 集成 Rviz2，提供实时的无人机姿态 Marker 显示及坐标系变换 (TF)。
* **WSL2 开箱即用**: Launch 文件内置 `LIBGL_ALWAYS_SOFTWARE=1` 补丁，完美解决 WSL 环境下 Rviz2 黑屏或崩溃问题。
* **一键启动**: 通过 Python Launch 脚本同时管理仿真节点与可视化界面。

## 🛠️ 依赖环境 (Prerequisites)

* **操作系统**: Ubuntu 22.04 LTS (或 Windows WSL2)
* **ROS 版本**: ROS 2 Humble Hawksbill
* **编译工具**: Colcon
* **核心依赖**:
    * `rclcpp`
    * `geometry_msgs`
    * `std_msgs`
    * `rviz2`
    * `tf2`, `tf2_ros`

## 🚀 安装与编译 (Installation)

1.  **创建工作空间**:
    ```bash
    mkdir -p ~/adrc_mimo_ws/src
    cd ~/adrc_mimo_ws/src
    ```

2.  **克隆项目**:
    ```bash
    git clone [https://github.com/hitpjp/adrc_mimo_ws.git](https://github.com/hitpjp/adrc_mimo_ws.git)
    ```

3.  **安装依赖**:
    ```bash
    cd ~/adrc_mimo_ws
    rosdep install --from-paths src --ignore-src -r -y
    ```

4.  **编译**:
    ```bash
    colcon build --symlink-install --packages-select mimo_adrc
    ```

5.  **配置环境**:
    ```bash
    source install/setup.bash
    ```

## 🎮 运行 (Usage)

本项目支持一键启动仿真核心与可视化界面。

**WSL2 用户特别提示:**
请确保您的 XServer (如 VcXsrv/XLaunch) 已在 Windows 端启动，并且防火墙已允许通过。

```bash
# 启动仿真节点和 Rviz2
ros2 launch mimo_adrc mimo_launch.py