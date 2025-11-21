# 依赖安装与环境配置

本工作区基于 ROS 2 Humble，以下列出在构建和运行过程中需要安装的系统库与 ROS 包，以及与 AstraPro SDK 相关的环境配置。

## 必备系统库

- libpcap-dev
- libuvc-dev
- nlohmann-json3-dev
- xtensor-dev（会同时安装 libxtensor-dev 与 xtl-dev）
- libasio-dev
- libgoogle-glog-dev

可选（在未使用 Astra SDK 时支持 OpenNI2 相机）：
- libopenni2-dev

## 必备 ROS 2 包（Humble）

- ros-humble-async-web-server-cpp
- ros-humble-image-publisher
- ros-humble-filters
- ros-humble-behaviortree-cpp-v3
- ros-humble-nav2-behavior-tree
- ros-humble-nav2-util
- ros-humble-test-msgs
- ros-humble-diagnostic-updater（ublox_gps）
- ros-humble-rtcm-msgs（ublox_gps）

## 一次性安装命令

```bash
sudo apt-get update && sudo apt-get install -y \
  libpcap-dev libuvc-dev nlohmann-json3-dev xtensor-dev libasio-dev libgoogle-glog-dev \
  ros-humble-async-web-server-cpp ros-humble-image-publisher ros-humble-filters \
  ros-humble-behaviortree-cpp-v3 ros-humble-nav2-behavior-tree ros-humble-nav2-util \
  ros-humble-test-msgs ros-humble-diagnostic-updater ros-humble-rtcm-msgs
```

如需 OpenNI2 相机支持（未使用 Astra SDK）：
```bash
sudo apt-get install -y libopenni2-dev
```

## AstraPro SDK 环境配置

若使用 AstraPro 的官方 SDK，请设置以下环境变量以供相机与体感组件链接：

```bash
export ASTRA_SDK_INCLUDE=/home/AstraSDK/install/include
export ASTRA_SDK_LIB=/home/AstraSDK/install/lib
```

要求 SDK 路径中存在：
- 体感库：`libastra.so`、`libastra_core.so`、`libastra_core_api.so`
- 相机库：`libOpenNI2_astra.so` 或 `libOpenNI2.so`（位于 `ASTRA_SDK_LIB` 或其子目录 `OpenNI2`）

注：供应商预编译库需与系统架构一致；若架构不匹配，体感包将被跳过构建以保证工作区可继续编译。

## YDLidar 驱动

- 位置：`/home/hutzzo/ydlidar_ros2_driver`
- 启动前确保加载其安装前缀：

```bash
source /home/hutzzo/ydlidar_ros2_driver/install/setup.bash
```

## 构建建议

- 加载 ROS 2 环境后执行选择性构建验证：

```bash
source /opt/ros/humble/setup.bash
colcon build --event-handlers console_cohesion+
```

- 如使用 Astra SDK，确保在同一终端导出环境变量后再执行构建。