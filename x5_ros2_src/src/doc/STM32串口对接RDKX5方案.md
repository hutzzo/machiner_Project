# STM32 串口对接 RDKX5 方案

## 现状评估

- 状态帧（24 字节，`0x7B ... XOR[22] 0x7D`）：已在 `turn_on_wheeltec_robot` 中实现接收与解析（里程计、IMU、电压），对应代码位于 `src/turn_on_wheeltec_robot/src/wheeltec_robot.cpp:377`、`494`。
- 车辆控制帧（11 字节 CAR）：已实现下发（`cmd_vel` → 串口），对应代码位于 `src/turn_on_wheeltec_robot/src/wheeltec_robot.cpp:55`，校验位生成与尾部符合文档描述。
- 机械臂角度控制帧（16 字节 MOVEIT，`0xAA ... mode chk 0xBB`）：未实现。

## 方案设计

- 保持现有状态帧与 CAR 控制链路不变。
- 新增 MOVEIT 帧桥接：订阅话题 `stm32_moveit_joint_cmd`，消息为 7 元浮点数组（`J1..J6` 弧度，末位为 `mode`），按协议转换为 16 字节帧并通过串口发送至 STM32。

## 帧映射与构造

- 输入：`std_msgs/Float32MultiArray`，长度≥6；若长度≥7，第 7 元为 `mode`，否则 `mode=1`。
- 量纲：弧度 ×1000 → `int16`；高字节在前（大端）。
- 帧布局：
  - `[0]=0xAA`
  - `[1..12]=J1..J6` 每关节 2 字节：高字节、低字节
  - `[13]=mode`
  - `[14]=XOR([0..13])`
  - `[15]=0xBB`

## 使用说明

- 话题：`stm32_moveit_joint_cmd`
- 示例：`ros2 topic pub /stm32_moveit_joint_cmd std_msgs/Float32MultiArray "{data: [0.5, 0, 0, 0, 0, 0, 1]}"`

## 变更点

- 新增订阅与串口下发函数至 `turn_on_wheeltec_robot`，不影响原有逻辑。