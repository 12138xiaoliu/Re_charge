# motion_test

红外回充与底盘运动测试程序。当前主线回充入口是 `IR_recharge.c`，通过左右红外输入设备采样 `MSC_SCAN` 码值，按采样窗口统计有效信号；有信号时进入导引处理，无信号时执行分阶段搜索动作。

## 目录结构

```text
.
├── include/
│   └── motion.h          # 运动接口声明、动作标定值、socket/状态文件路径
├── src/
│   ├── motion.c          # 底盘/舵机动作封装，向电机控制 socket 发送单字符命令
│   ├── motion_cli.c      # motion 命令行测试入口
│   └── recive_irevent.c  # 旧版红外回充测试程序
├── IR_recharge.c         # 当前红外回充调试入口
├── Makefile              # 构建 motion、recive_irevent、IR_recharge
└── out/
    ├── bin/              # 构建输出的可执行文件
    └── obj/              # 构建中间产物
```

## 构建与运行

```sh
make                 # 构建全部目标
make IR_recharge     # 只构建当前回充调试程序
make motion          # 只构建运动命令行测试程序
make clean           # 清理 out/ 和根目录旧目标文件
```

构建输出位于 `out/bin/`：

- `out/bin/IR_recharge`：当前红外回充调试程序。
- `out/bin/motion`：运动命令行测试工具。
- `out/bin/recive_irevent`：旧版红外回充测试程序。

运行 `IR_recharge` 前需要确认：

- 左右红外输入设备路径与代码一致：`/dev/input/event3`、`/dev/input/event2`。
- 电机控制 socket 已启动：`/tmp/motor_ctl_socket`。
- 如需覆盖提示音命令，可设置环境变量 `IR_TEST_AUDIO_CMD`。

## 运动模块

`src/motion.c` 封装底盘和舵机动作，核心接口声明在 `include/motion.h`。

关键函数：

- `motion_send_raw()`：向 `/tmp/motor_ctl_socket` 发送底层单字符命令。
- `motion_set_speed_level()`：设置底盘速度档位，支持 `1/2/3`。
- `motion_forward_distance_mm()` / `motion_turn_right_distance_mm()`：按标定值换算执行时长，并在动作后发送停止命令。
- `motion_is_tof_estop_locked()`：前进前读取 `/tmp/tof_estop.lock`，急停锁开启时拒绝前进命令。
- `motion_action_*()`：基于 `MOTION_FIXED_*` 标定值封装固定动作。

当前固定动作标定值在 `include/motion.h` 中维护，例如：

- `MOTION_FIXED_FORWARD_20CM_VALUE`
- `MOTION_FIND_SINGLE_FORWARD_VALUE`
- `MOTION_FIXED_RIGHT_45_VALUE`
- `MOTION_FIXED_RIGHT_90_VALUE`

## IR_recharge 主流程

`IR_recharge.c` 的主循环在 `main()` 中：

1. `open_device()` 打开左右红外输入设备，并通过 `set_nonblock()` 设置非阻塞。
2. 使用 `epoll_create1()` / `epoll_ctl()` 监听左右输入设备。
3. 每轮采样前调用 `flush_all_pending_events()` 清掉旧事件。
4. 调用 `collect_signal_stats()` 在 `IR_SAMPLE_WINDOW_MS` 窗口内采样。
5. `read_device_events()` 读取 `EV_MSC / MSC_SCAN` 事件，按 `USE_BYTE_INDEX` 提取真实 8bit 码值。
6. `record_signal_sample()` 将码值计入左右侧统计，`finalize_sample_stats()` 计算每侧最高频码值。
7. `print_sample_summary()` 打印本轮频率统计。
8. `sample_has_valid_signal()` 判断是否存在达到 `IR_VALID_THRESHOLD` 的有效码值。
9. 有有效信号时进入 `handle_valid_signal()`；无有效信号时进入 `run_no_signal_search()`。

主要参数：

- `IR_SAMPLE_WINDOW_MS = 2000`：每轮采样窗口 2 秒。
- `IR_VALID_THRESHOLD = 3`：单个码值在窗口内达到 3 次才认为有效。
- `USE_BYTE_INDEX = 0`：从 32bit 扫描码的第 0 个字节取真实码值。
- `SEARCH_ROTATE_STEP_DEG = 18`：搜索状态累计角度步进。
- `SEARCH_FULL_ROTATE_DEG = 360`：一整圈搜索阈值。
- `SEARCH_MAX_XDISTANCE = 6`：横向展开搜索的最大 20cm 步数。

## 红外码值

右侧码值：

- `0x11`：右轮检测到偏左信号
- `0x12`：右轮检测到中间信号
- `0x13`：右轮检测到偏右信号
- `0x14`：右轮检测到远距离信号

左侧码值：

- `0x21`：左轮检测到偏左信号
- `0x22`：左轮检测到中间信号
- `0x23`：左轮检测到偏右信号
- `0x24`：左轮检测到远距离信号

码值归属由 `signal_side_from_raw()` 判断；如果设备侧别和码值归属不一致，`read_device_events()` 会按码值归属计入对应侧统计，便于排查接线或驱动映射问题。

## 当前回充策略

### 有信号处理

入口函数：`handle_valid_signal()`。

当前行为：

1. 调用 `reset_search_state()` 清空无信号搜索进度。
2. 如果上一轮未处于导引状态，调用 `play_test_audio()` 播放提示音。
3. 设置 `state->guidance_active = true`。
4. 调用 `enter_normal_guidance_flow()` 打印达到阈值的有效码值。
5. 调用 `run_guidance_action_for_valid_signal()` 分析当前有效信号并选择动作。
6. 当前动作选择内部通过 `find_forward_pair_hit()` 判断是否满足直线前进组合。
7. 命中直线前进组合时调用 `find_single_forward()` 前进一次。
8. 未命中直线前进组合但单侧目标码出现时，执行 1 度微调后退。

直线前进组合规则：

- 右侧 `0x12` 与左侧 `0x22` 必须同时达到 `IR_VALID_THRESHOLD`。
- 其他组合不再触发直线前进。

当前 `find_single_forward()` 使用低速档 `1`，并调用：

```c
motion_forward_distance_mm(MOTION_FIND_SINGLE_FORWARD_VALUE)
```

微调规则：

- 右侧 `0x12` 出现，左侧 `0x22` 未达到阈值：调用 `motion_action_left_1deg()`，再调用 `motion_action_backward_5cm()`。
- 左侧 `0x22` 出现，右侧 `0x12` 未达到阈值：调用 `motion_action_right_1deg()`，再调用 `motion_action_backward_5cm()`。
- `0x12` 和 `0x22` 都未出现：本轮不下发导引动作。

### 无信号搜索

入口函数：`run_no_signal_search()`。

搜索状态保存在 `ir_runtime_state_t`：

- `step`：搜索阶段。
- `angle`：当前阶段累计旋转角度。
- `xdistance`：`step=2` 横向展开时前进的 20cm 次数。
- `guidance_active`：上一轮是否处于导引状态。

当前三阶段策略：

1. `step=0`：原地慢速右转搜索。
   - 每轮调用 `advance_rotate_progress()`。
   - 实际动作通过 `search_rotate_right_once()` 执行。
   - 累计角度达到 `SEARCH_FULL_ROTATE_DEG` 后进入 `step=1`。

2. `step=1`：先前进 20cm，再原地慢速右转搜索一整圈。
   - 阶段开始时调用 `search_forward_20cm()`。
   - 之后继续调用 `advance_rotate_progress()`。
   - 累计角度达到一整圈后进入 `step=2`。

3. `step=2`：右转 90 度后横向展开搜索。
   - 阶段开始时调用 `search_turn_right_90deg()`。
   - 按 `xdistance` 次数调用 `search_forward_20cm()`。
   - 再原地慢速右转搜索一整圈。
   - 每完成一圈后 `xdistance++`。
   - 当 `xdistance > SEARCH_MAX_XDISTANCE` 时重置回 `step=0`。

所有搜索动作前都会通过 `set_search_speed(1)` 切到低速档。

## motion CLI

`motion` 用于手动测试底盘和舵机动作：

```sh
out/bin/motion raw <char>
out/bin/motion speed <1|2|3>
out/bin/motion left <distance>
out/bin/motion right <distance>
out/bin/motion forward <distance>
out/bin/motion backward <distance>
out/bin/motion action <left45|right45|forward20|backward20|left1|right1|forward5|backward5|circle_left|circle_right|search_right|left90|right90>
out/bin/motion servo <forward|backward|reset>
```
