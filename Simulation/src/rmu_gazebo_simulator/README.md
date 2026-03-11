# rmu_gazebo_simulator

## 1. Introduction

rmu_gazebo_simulator 是基于 Gazebo (Ignition 字母版本) 的仿真环境，为 RoboMaster University 中的机器人算法开发提供仿真环境，方便测试 AI 算法，加快开发效率。

目前 rmu_gazebo_simulator 提供以下功能：

- rmul_2024, rmuc_2024, rmul_2025, rmuc_2025 仿真世界模型

- 网页端局域网联机对战

- 机器人底盘、云台、射击控制

| rmul_2024 | rmuc_2024 |
|:-----------------:|:--------------:|
|![spin_nav.gif](https://raw.githubusercontent.com/LihanChen2004/picx-images-hosting/master/spin_nav.1ove3nw63o.gif)|![rmuc_fly.gif](https://raw.githubusercontent.com/LihanChen2004/picx-images-hosting/master/rmuc_fly_image.1aoyoashvj.gif)|

## 2. Quick Start

### 2.1 Option 1: Docker

#### 2.1.1 Setup Environment

- [Docker](https://docs.docker.com/engine/install/)
- [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)
- 允许本地的 Docker 容器访问主机的 X11 显示

    ```bash
    xhost +local:docker
    ```

#### 2.1.2 Create Container

```bash
docker run -it --rm --name rmu_gazebo_simulator \
  --network host \
  --runtime nvidia \
  --gpus all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e "DISPLAY=$DISPLAY" \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /dev:/dev \
  ghcr.io/smbu-polarbear-robotics-team/rmu_gazebo_simulator:1.0.0
```

### 2.2 Option 2: Build From Source

#### 2.2.1 Setup Environment

- Ubuntu 22.04
- ROS: [Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- Ignition: [Fortress](https://gazebosim.org/docs/fortress/install_ubuntu/)

#### 2.2.2 Create Workspace

```bash
sudo pip install vcstool2
pip install xmacro
```

```bash
mkdir -p ~/ros_ws
cd ~/ros_ws
```

```bash
git clone https://github.com/SMBU-PolarBear-Robotics-Team/rmu_gazebo_simulator.git src/rmu_gazebo_simulator
```

```bash
vcs import src < src/rmu_gazebo_simulator/dependencies.repos
```

#### 2.2.3 Build

```sh
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```

```sh
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=release
```

### 2.3 Running

启动仿真环境

```sh
ros2 launch rmu_gazebo_simulator bringup_sim.launch.py
```

> [!NOTE]
> **注意：需要点击 Gazebo 左下角橙红色的 `启动` 按钮**

#### 2.3.1 Test Commands

控制机器人移动

```sh
ros2 run rmoss_gz_base test_chassis_cmd.py --ros-args -r __ns:=/red_standard_robot1/robot_base -p v:=0.3 -p w:=0.3
#根据提示进行输入，支持平移与自旋
```

机器人云台

```sh
ros2 run rmoss_gz_base test_gimbal_cmd.py --ros-args -r __ns:=/red_standard_robot1/robot_base
#根据提示进行输入，支持绝对角度控制
```

机器人射击

```sh
ros2 run rmoss_gz_base test_shoot_cmd.py --ros-args -r __ns:=/red_standard_robot1/robot_base
#根据提示进行输入
```

#### 2.3.2 网页端控制

支持局域网内联机操作，只需要将 localhost 改为主机 ip 即可。

操作手端

<http://localhost:5000/>

```sh
python3 src/rmu_gazebo_simulator/rmu_gazebo_simulator/scripts/player_web/main_no_vision.py
```

裁判系统端

<http://localhost:2350/>

```sh
python3 src/rmu_gazebo_simulator/rmu_gazebo_simulator/scripts/referee_web/main.py
```

#### 2.3.3 切换仿真世界

修改 [gz_world.yaml](./rmu_gazebo_simulator/config/gz_world.yaml) 中的 `world`。当前可选: `rmul_2024`, `rmuc_2024`, `rmul_2025`, `rmuc_2025`

## 配套导航仿真仓库

- 2025 SMBU PolarBear Sentry Navigation

    [pb2025_sentry_nav](https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav.git)

    ![cmu_nav_v1_0](https://raw.githubusercontent.com/LihanChen2004/picx-images-hosting/master/spin_nav.1ove3nw63o.gif)

## 维护者及开源许可证

Maintainer: Lihan Chen, <lihanchen2004@163.com>

rmu_gazebo_simulator is provided under Apache License 2.0.
