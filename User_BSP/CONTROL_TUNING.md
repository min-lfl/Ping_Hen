# 单加速度前馈控制调参说明

本控制链路用于底板上的 MPU6050 测量平移加速度，并通过连杆机构改变杆角度。它不使用球位置反馈。

控制流程：

```text
Ay/Az -> 零偏修正 -> 二级低通 -> atan2 理想杆角
      -> 连杆标定表 -> 临界阻尼轨迹 -> 速度/加速度/jerk 限制
      -> 快速绝对位置命令
```

首次调试必须取出球，并确认连杆在 `-250 ~ +250` 脉冲内不会撞击机械限位。

## 1. 参数位置

绝大多数参数位于 `User_BSP/Control_Config.h`：

- `CONTROL_AUTOMATIC_ENABLE`：`1` 自动补偿，`0` 关闭自动补偿并允许按键标定。
- `CONTROL_MOTOR_SPEED_RPM`：电机驱动器速度上限。
- `CONTROL_MOTOR_ACCEL_PARAM`：驱动器加速度参数。不要设置为 `0`，`0` 是直接启动。
- `CONTROL_MANUAL_TEST_PULSE`：关闭自动补偿后，左右按键发送的测试脉冲。
- `CONTROL_ACCEL_Y_BIAS_G`：Ay 静止零偏。
- `CONTROL_ACCEL_TO_ROD_SIGN`：加速度到杆方向的符号，只能设为 `+1.0f` 或 `-1.0f`。
- `CONTROL_ACCEL_LPF_POLE_HZ`：加速度低通。默认 `18 Hz`。
- `CONTROL_LEVEL_PULSE`：杆真正水平时的绝对脉冲位置。
- `CONTROL_MIN_RELATIVE_PULSE` / `MAX`：连杆安全范围。
- `CONTROL_PROFILE_NATURAL_HZ`：轨迹响应快慢的主参数。
- `CONTROL_PROFILE_DAMPING`：轨迹阻尼，默认临界阻尼 `1.0`。
- `CONTROL_MAX_PULSE_SPEED`：软件最大脉冲速度。
- `CONTROL_MAX_PULSE_ACCEL`：软件最大脉冲加速度。
- `CONTROL_MAX_PULSE_JERK`：软件最大脉冲加加速度。
- `CONTROL_SEND_HYSTERESIS_PULSE`：小于此值不重复发送命令。

连杆标定表位于 `User_BSP/User_Task.c` 顶部的 `linkage_cal_table[]`。

## 2. 标定连杆表

MakerWorld 页面没有给出所有铰点的精确坐标，而且打印装配间隙会改变实际传动关系，所以使用实测表比硬编码理论连杆长度可靠。

1. 取出球，将 `CONTROL_AUTOMATIC_ENABLE` 改为 `0`。
2. 将 `CONTROL_MANUAL_TEST_PULSE` 依次设为 `83`、`167`、`250`，每次重新编译下载。
3. 三个按键分别控制 `+测试值`、`CONTROL_LEVEL_PULSE`、`-测试值`。
4. 每个位置等待杆稳定，用数显水平仪测量真实杆角度。
5. 把七个结果写入 `linkage_cal_table[]`。

表必须按杆角度从小到大排列。例如实测结果如下：

```c
static const LinkageCalPoint_t linkage_cal_table[] = {
    {-7.10f, -250},
    {-4.80f, -167},
    {-2.35f,  -83},
    { 0.00f,    0},
    { 2.20f,   83},
    { 4.55f,  167},
    { 6.85f,  250},
};
```

正负角不需要对称。如果正脉冲导致负角，仍然按角度递增排列，但脉冲列可以从正数排到负数。

标定完成后恢复 `CONTROL_AUTOMATIC_ENABLE = 1`。

也可以在调试时通过 Keil Watch 临时把 `control_automatic_enabled` 改为 `1`，但重新上电后仍以宏定义为准。

## 3. 标定水平零点和加速度零偏

### 水平零点

底板静止且 Ay 已校准时，如果球仍持续向一侧滚，先调 `CONTROL_LEVEL_PULSE`，不要修改加速度偏置来代替机械水平零点。

每次只改 `1 ~ 3` 脉冲，直到静止时杆真正水平。绝对位置零点必须在每次上电后可重复；如果驱动器零点会变化，需要增加回零流程。

### Ay 零偏

在 Keil Watch 中观察：

```text
control_debug_ay_filtered_g
control_debug_az_filtered_g
```

底板静止时，前者应接近 `0.0000 g`，后者的绝对值应接近 `1.0 g`。

代码使用：

```c
ay_corrected = ay_raw - CONTROL_ACCEL_Y_BIAS_G;
```

因此 `CONTROL_ACCEL_Y_BIAS_G` 应填写静止时测得的原始 Ay 平均值。不要根据单个样本调整，至少平均 2 秒。

## 4. 确认方向

无球测试底板向前加速时，杆必须向能够产生反向重力分力的方向倾斜。如果方向相反，只修改：

```c
#define CONTROL_ACCEL_TO_ROD_SIGN (+1.0f)
```

或者改回 `-1.0f`。不要通过交换标定表顺序来临时修正方向。

## 5. 推荐调参顺序

1. 标定 `linkage_cal_table[]`。
2. 调 `CONTROL_LEVEL_PULSE`。
3. 调 `CONTROL_ACCEL_Y_BIAS_G`。
4. 确认 `CONTROL_ACCEL_TO_ROD_SIGN`。
5. 无球调滤波和轨迹，确保杆没有低频摆动和机械冲击。
6. 放球后，在恒定加速度阶段检查补偿角度比例。
7. 最后调启动、刹车阶段的动态响应。

不要同时修改三个以上参数，否则无法判断是哪一项生效。

## 6. 动态参数怎么调

### 杆动作过猛、球被抛起

按以下顺序处理：

1. `CONTROL_PROFILE_NATURAL_HZ` 从 `4.0` 降到 `3.0`。
2. `CONTROL_MAX_PULSE_ACCEL` 每次降低约 20%。
3. `CONTROL_MAX_PULSE_SPEED` 每次降低约 20%。
4. 驱动器加速度调得更柔和，但不要设为 `0`。
5. 必要时降低底板自身运动轨迹的 jerk。

降低 `CONTROL_MAX_PULSE_JERK` 会让动作更柔和，但过低会造成刹车滞后和过冲。如果降低 jerk 后过冲更明显，应恢复 jerk，并降低 `NATURAL_HZ` 或提高阻尼。

### 响应太慢，启动和刹车时球先移动

按以下顺序处理：

1. `CONTROL_PROFILE_NATURAL_HZ` 每次增加 `0.5 Hz`。
2. `CONTROL_MAX_PULSE_SPEED` 每次增加约 20%。
3. `CONTROL_MAX_PULSE_ACCEL` 每次增加约 20%。
4. `CONTROL_ACCEL_LPF_POLE_HZ` 从 `18` 增至 `22 ~ 25 Hz`。
5. 确认驱动器速度上限高于软件轨迹要求。

如果加快后重新出现低频摆动，应退回上一个稳定值。只有 IMU 事后测量时，不可能同时完全消除突变加速度的延迟和杆的冲击；底板使用 S 曲线运动会明显改善。

### 杆静止时高频小抖

1. `CONTROL_SEND_HYSTERESIS_PULSE` 从 `3` 增到 `4 ~ 6`。
2. `CONTROL_ACCEL_LPF_POLE_HZ` 从 `18` 降到 `12 ~ 15 Hz`。
3. 检查 `control_debug_target_pulse` 是否也在抖。
4. IMU 应刚性固定在底板，软泡棉会引入新的低频共振。

迟滞区内程序保持上次位置，不会命令电机回零。

### 杆出现固定频率的低频摆动

1. 取出球，底板静止测试。
2. 将 `CONTROL_PROFILE_NATURAL_HZ` 降到 `3.0`。
3. 将 `CONTROL_PROFILE_DAMPING` 增到 `1.2 ~ 1.5`。
4. 降低软件最大加速度。
5. 加固底板、轴承座和连杆间隙。

如果 `control_debug_target_pulse` 在摆动，说明电机反作用已经进入底板 IMU；先处理滤波、安装和结构。如果目标稳定而杆仍摆，问题位于驱动器或机械结构。若始终在同一频率共振，后续应针对实测频率增加陷波器。

### 恒定加速度阶段球仍向一侧滚

这主要是连杆表或水平零点问题，不是轨迹快慢问题：

- 补偿不足：同一目标杆角需要更大的脉冲绝对值。
- 补偿过度：同一目标杆角需要更小的脉冲绝对值。
- 加速和减速不对称：分别修改标定表的正角和负角部分。

先观察 `control_debug_target_angle_deg`，再核对该角度在标定表中插值得到的脉冲是否确实让杆达到相同实测角度。

### 经常顶到 `+250` 或 `-250`

观察 `control_debug_target_pulse`。如果经常达到限幅，说明当前机构安全角度不足或标定表端点太小。确认机械余量后才允许扩大 `CONTROL_MIN/MAX_RELATIVE_PULSE` 并重新标定端点；不要只扩大软件限幅。

## 7. 推荐观察变量

在 Keil Watch 中添加：

```text
control_debug_ay_filtered_g
control_debug_az_filtered_g
control_debug_target_angle_deg
control_debug_target_pulse
control_debug_command_pulse
control_debug_profile_speed
```

- `target_pulse` 抖而 `command_pulse` 平稳：滤波和轨迹正在正常抑制噪声。
- 两者都抖：滤波不足、结构耦合或轨迹频率过高。
- `target_pulse` 变化及时但 `command_pulse` 落后：轨迹限制过严。
- `command_pulse` 及时但实际杆落后：驱动器速度/加速度不足、间隙过大或命令未及时执行。
