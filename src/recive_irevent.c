#include <errno.h>
#include <fcntl.h>
#include <linux/input.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/epoll.h>
#include <time.h>
#include <unistd.h>

#include "../include/motion.h"

/*
 * 红外回充测试程序
 *
 * 设计思路：
 * 1. 分别监听左右轮红外接收器的 input 事件设备；
 * 2. 在固定采样窗口内统计各已知红外码值出现频次，以频次表示信号强度；
 * 3. 若一侧窗口内仅出现一种已知码值，则将该码值作为当前轮次有效结果；
 * 4. 若一侧窗口内出现多种已知码值，则判定该侧为多码竞争，本轮只记录日志不动作；
 * 5. 仅当本轮完全没有已知信号时，才执行分阶段搜索动作并在每步后重新采样。
 */
#define DEV_LEFT  "/dev/input/event3"
#define DEV_RIGHT "/dev/input/event2"

/* epoll 最多同时关注左右两个红外输入设备。 */
#define MAX_EVENTS               2
/* 一次批量读取 input_event 的缓存大小。 */
#define BUF_SIZE                 64
/* 从 input_event.value 的哪个字节提取真实红外码值。 */
#define USE_BYTE_INDEX           0
/* 每侧支持统计的已知红外码值个数。 */
#define IR_KNOWN_SIGNAL_COUNT    4
/* 每轮采样窗口时长，固定窗口内的出现次数即该窗口的信号强度。 */
#define IR_SAMPLE_WINDOW_MS      5000
/* epoll 的轮询超时，避免长时间阻塞。 */
#define IR_EPOLL_WAIT_TIMEOUT_MS 100

/**
 * @brief 红外传感器所在侧别
 */
typedef enum {
    IR_SENSOR_LEFT = 0,
    IR_SENSOR_RIGHT,
} ir_sensor_side_t;

/**
 * @brief 当前文件内部使用的动作枚举
 */
typedef enum {
    IR_ACTION_NONE = 0,
    IR_ACTION_LEFT45,
    IR_ACTION_RIGHT45,
    IR_ACTION_FORWARD20,
} ir_action_id_t;

/**
 * @brief 单侧红外在当前采样窗口内的统计归纳结果
 */
typedef enum {
    IR_SIGNAL_NONE = 0,
    IR_SIGNAL_CLEAR,
    IR_SIGNAL_AMBIGUOUS,
} ir_signal_resolution_t;

typedef struct {
    int counts[IR_KNOWN_SIGNAL_COUNT];
    int total_count;
    int unique_count;
    ir_signal_resolution_t resolution;
    uint32_t raw;
    uint32_t top1_raw;
    uint32_t top2_raw;
    int top1_count;
    int top2_count;
} ir_signal_state_t;

/**
 * @brief 左右两侧红外在当前采样轮次内的稳定结果
 */
typedef struct {
    ir_signal_state_t left;
    ir_signal_state_t right;
} ir_tracker_t;

/**
 * @brief 单个红外码值映射出的动作序列
 * @param source_side  产生该序列的信号来源侧
 * @param source_raw   产生该序列的原始码值
 * @param action_count 序列中的动作个数，当前支持 0/1/2
 * @param actions      动作数组，按顺序依次执行
 */
typedef struct {
    ir_sensor_side_t source_side;
    uint32_t source_raw;
    int action_count;
    ir_action_id_t actions[2];
} ir_action_sequence_t;

typedef int (*ir_search_motion_fn_t)(void);

typedef struct {
    const char *name;
    ir_search_motion_fn_t action;
} ir_search_step_t;

/**
 * @brief 获取单调时钟毫秒值
 * @param 无
 * @return 当前单调时钟时间，单位毫秒
 */
static long long monotonic_ms_now(void)
{
    struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long long)ts.tv_sec * 1000LL + ts.tv_nsec / 1000000LL;
}

/**
 * @brief 将文件描述符设置为非阻塞模式
 * @param fd 目标文件描述符
 * @return 0以上表示成功，-1表示失败
 */
static int set_nonblock(int fd)
{
    int flags = fcntl(fd, F_GETFL, 0);

    if (flags < 0) {
        return -1;
    }

    return fcntl(fd, F_SETFL, flags | O_NONBLOCK);
}

/**
 * @brief 打开红外输入设备并设置为非阻塞
 * @param dev 设备路径
 * @return 打开的文件描述符；失败时直接退出进程
 */
static int open_device(const char *dev)
{
    int fd = open(dev, O_RDONLY);

    if (fd < 0) {
        perror(dev);
        exit(1);
    }

    if (set_nonblock(fd) < 0) {
        perror("set_nonblock");
        close(fd);
        exit(1);
    }

    return fd;
}

/**
 * @brief 返回侧别对应的中文描述
 * @param side 传感器侧别
 * @return "左侧" 或 "右侧"
 */
static const char *side_desc(ir_sensor_side_t side)
{
    return side == IR_SENSOR_RIGHT ? "右侧" : "左侧";
}

/**
 * @brief 根据红外码值推断其所属侧别
 * @param raw 原始红外码值
 * @param side_out 返回推断出的侧别
 * @return true表示推断成功，false表示码值未知
 */
static bool signal_side_from_raw(uint32_t raw, ir_sensor_side_t *side_out)
{
    if (raw >= 0x11 && raw <= 0x14) {
        if (side_out != NULL) {
            *side_out = IR_SENSOR_RIGHT;
        }
        return true;
    }

    if (raw >= 0x21 && raw <= 0x24) {
        if (side_out != NULL) {
            *side_out = IR_SENSOR_LEFT;
        }
        return true;
    }

    return false;
}

/**
 * @brief 将原始红外码值翻译为人可读的中文描述
 * @param side 产生该码值的传感器侧别，码值可推断时仅作为未知场景回退
 * @param raw  原始码值
 * @return 对应的中文描述字符串
 */
static const char *sensor_raw_desc(ir_sensor_side_t side, uint32_t raw)
{
    ir_sensor_side_t inferred_side;

    if (signal_side_from_raw(raw, &inferred_side)) {
        side = inferred_side;
    }

    if (side == IR_SENSOR_LEFT) {
        switch (raw) {
        case 0x21:
            return "左轮检测到偏左信号";
        case 0x22:
            return "左轮检测到中间信号";
        case 0x23:
            return "左轮检测到偏右信号";
        case 0x24:
            return "左轮检测到远距离信号";
        default:
            return "左侧未知信号";
        }
    }

    switch (raw) {
    case 0x11:
        return "右轮检测到偏左信号";
    case 0x12:
        return "右轮检测到中间信号";
    case 0x13:
        return "右轮检测到偏右信号";
    case 0x14:
        return "右轮检测到远距离信号";
    default:
        return "右侧未知信号";
    }
}

/**
 * @brief 输出当前采样轮次内某一侧的信号描述
 * @param signal 当前侧的频率归纳结果
 * @param side   传感器侧别
 * @return 若无信号返回“无有效信号”，否则返回原始码值描述
 */
static const char *signal_resolution_desc(ir_signal_resolution_t resolution)
{
    switch (resolution) {
    case IR_SIGNAL_CLEAR:
        return "clear";
    case IR_SIGNAL_AMBIGUOUS:
        return "ambiguous";
    default:
        return "none";
    }
}

/**
 * @brief 输出当前采样轮次内某一侧的信号描述
 * @param signal 当前侧的频率统计结果
 * @param side   传感器侧别
 * @return 若无信号返回“无有效信号”，多码竞争返回“多码竞争”，否则返回原始码值描述
 */
static const char *tracker_signal_desc(const ir_signal_state_t *signal,
                                       ir_sensor_side_t side)
{
    if (signal->resolution == IR_SIGNAL_NONE) {
        return side == IR_SENSOR_LEFT ? "左侧无有效信号" : "右侧无有效信号";
    }

    if (signal->resolution == IR_SIGNAL_AMBIGUOUS) {
        return side == IR_SENSOR_LEFT ? "左侧多码竞争" : "右侧多码竞争";
    }

    return sensor_raw_desc(side, signal->raw);
}

/**
 * @brief 将内部动作枚举转换为中文动作名
 * @param action 内部动作枚举
 * @return 对应动作的中文名称
 */
static const char *action_desc(ir_action_id_t action)
{
    switch (action) {
    case IR_ACTION_LEFT45:
        return "左转45度";
    case IR_ACTION_RIGHT45:
        return "右转45度";
    case IR_ACTION_FORWARD20:
        return "前进20厘米";
    default:
        return "无动作";
    }
}

/**
 * @brief 清空当前采样轮次中的左右频率统计结果
 * @param tracker 采样状态缓存
 * @return 无
 */
static void reset_cycle_signals(ir_tracker_t *tracker)
{
    memset(&tracker->left, 0, sizeof(tracker->left));
    memset(&tracker->right, 0, sizeof(tracker->right));
}

/**
 * @brief 判断某个码值是否属于已知红外信号
 * @param raw  原始码值
 * @return true 表示为有效码值，false 表示无效或未知
 */
static bool is_known_signal(uint32_t raw)
{
    return signal_side_from_raw(raw, NULL);
}

/**
 * @brief 将指定侧别的红外码值映射到频率统计数组下标
 * @param side 传感器侧别
 * @param raw  原始码值
 * @return 0以上表示已知码值下标，-1表示未知码值
 */
static int signal_index_from_raw(ir_sensor_side_t side, uint32_t raw)
{
    if (side == IR_SENSOR_RIGHT) {
        switch (raw) {
        case 0x11:
            return 0;
        case 0x12:
            return 1;
        case 0x13:
            return 2;
        case 0x14:
            return 3;
        default:
            return -1;
        }
    }

    switch (raw) {
    case 0x21:
        return 0;
    case 0x22:
        return 1;
    case 0x23:
        return 2;
    case 0x24:
        return 3;
    default:
        return -1;
    }
}

/**
 * @brief 根据统计数组下标恢复指定侧别的红外码值
 * @param side 传感器侧别
 * @param index 统计数组下标
 * @return 对应原始码值；下标非法时返回0
 */
static uint32_t signal_raw_from_index(ir_sensor_side_t side, int index)
{
    static const uint32_t k_left_signals[IR_KNOWN_SIGNAL_COUNT] = {0x21, 0x22, 0x23, 0x24};
    static const uint32_t k_right_signals[IR_KNOWN_SIGNAL_COUNT] = {0x11, 0x12, 0x13, 0x14};

    if (index < 0 || index >= IR_KNOWN_SIGNAL_COUNT) {
        return 0;
    }

    return side == IR_SENSOR_RIGHT ? k_right_signals[index] : k_left_signals[index];
}

/**
 * @brief 判断某一侧当前窗口结果是否为可直接参与动作判定的 clear 状态
 * @param signal 当前侧统计结果
 * @return true表示 clear，false表示不是
 */
static bool signal_is_clear(const ir_signal_state_t *signal)
{
    return signal->resolution == IR_SIGNAL_CLEAR;
}

/**
 * @brief 判断某一侧当前窗口内是否收到过至少一个已知信号
 * @param signal 当前侧统计结果
 * @return true表示收到过已知信号，false表示没有
 */
static bool signal_has_any(const ir_signal_state_t *signal)
{
    return signal->total_count > 0;
}

/**
 * @brief 判断当前采样轮次是否收到过任意已知信号
 * @param tracker 当前轮次信号缓存
 * @return true表示收到过，false表示没有
 */
static bool tracker_has_any_signal(const ir_tracker_t *tracker)
{
    return signal_has_any(&tracker->left) || signal_has_any(&tracker->right);
}

/**
 * @brief 判断当前采样轮次是否存在多码竞争
 * @param tracker 当前轮次信号缓存
 * @return true表示存在多码竞争，false表示没有
 */
static bool tracker_has_ambiguous_signal(const ir_tracker_t *tracker)
{
    return tracker->left.resolution == IR_SIGNAL_AMBIGUOUS ||
           tracker->right.resolution == IR_SIGNAL_AMBIGUOUS;
}

/**
 * @brief 打印单条原始 input_event 解析结果
 * @param tag  当前设备标签
 * @param raw  原始 32 位 value
 * @param b0   value 的第 0 字节
 * @param b1   value 的第 1 字节
 * @param b2   value 的第 2 字节
 * @param b3   value 的第 3 字节
 * @param real 实际用于判定的码值
 * @return 无
 */
static void print_raw_debug(const char *tag, uint32_t raw, uint8_t real)
{
    ir_sensor_side_t device_side =
        strcmp(tag, "RIGHT") == 0 ? IR_SENSOR_RIGHT : IR_SENSOR_LEFT;
    ir_sensor_side_t signal_side = device_side;
    const char *signal_side_text;

    if (!signal_side_from_raw(real, &signal_side)) {
        signal_side = device_side;
    }

    signal_side_text = side_desc(signal_side);
    printf("[%s] 收到原始信号 raw=0x%02x | real=0x%02x | code_side=%s -> %s\n",
           tag, raw & 0xFF, real, signal_side_text,
           sensor_raw_desc(signal_side, real));
}

/**
 * @brief 将单侧采样窗口的频率统计结果归纳为 none/clear/ambiguous
 * @param signal 当前侧统计结果
 * @param side   传感器侧别
 * @return 无
 */
static void finalize_signal_state(ir_signal_state_t *signal, ir_sensor_side_t side)
{
    int top1_index = -1;
    int top2_index = -1;

    signal->unique_count = 0;
    signal->resolution = IR_SIGNAL_NONE;
    signal->raw = 0;
    signal->top1_raw = 0;
    signal->top2_raw = 0;
    signal->top1_count = 0;
    signal->top2_count = 0;

    for (int i = 0; i < IR_KNOWN_SIGNAL_COUNT; i++) {
        int count = signal->counts[i];

        if (count <= 0) {
            continue;
        }

        signal->unique_count++;
        if (top1_index < 0 || count > signal->counts[top1_index]) {
            top2_index = top1_index;
            top1_index = i;
        } else if (top2_index < 0 || count > signal->counts[top2_index]) {
            top2_index = i;
        }
    }

    if (top1_index >= 0) {
        signal->top1_raw = signal_raw_from_index(side, top1_index);
        signal->top1_count = signal->counts[top1_index];
    }

    if (top2_index >= 0) {
        signal->top2_raw = signal_raw_from_index(side, top2_index);
        signal->top2_count = signal->counts[top2_index];
    }

    if (signal->unique_count == 0) {
        signal->resolution = IR_SIGNAL_NONE;
    } else if (signal->unique_count == 1) {
        signal->resolution = IR_SIGNAL_CLEAR;
        signal->raw = signal->top1_raw;
    } else {
        signal->resolution = IR_SIGNAL_AMBIGUOUS;
    }
}

/**
 * @brief 完成当前采样窗口的左右频率归纳
 * @param tracker 当前轮次信号缓存
 * @return 无
 */
static void finalize_cycle_signals(ir_tracker_t *tracker)
{
    finalize_signal_state(&tracker->left, IR_SENSOR_LEFT);
    finalize_signal_state(&tracker->right, IR_SENSOR_RIGHT);
}

/**
 * @brief 将采样窗口内的出现次数换算为频率
 * @param count 当前窗口内的出现次数
 * @return 对应频率，单位 Hz
 */
static double signal_count_to_hz(int count)
{
    return (double)count * 1000.0 / (double)IR_SAMPLE_WINDOW_MS;
}

/**
 * @brief 打印单侧采样窗口的频率统计明细
 * @param signal 当前侧统计结果
 * @param side   传感器侧别
 * @return 无
 */
static void print_side_frequency_stats(const ir_signal_state_t *signal,
                                       ir_sensor_side_t side)
{
    printf("[IR][STAT] %s total=%d(%.2fHz) unique=%d state=%s",
           side_desc(side), signal->total_count,
           signal_count_to_hz(signal->total_count), signal->unique_count,
           signal_resolution_desc(signal->resolution));

    for (int i = 0; i < IR_KNOWN_SIGNAL_COUNT; i++) {
        uint32_t raw = signal_raw_from_index(side, i);

        if (signal->counts[i] <= 0) {
            continue;
        }

        printf(" 0x%02x=%d(%.2fHz)", raw & 0xFF, signal->counts[i],
               signal_count_to_hz(signal->counts[i]));
    }

    if (signal->top1_count > 0) {
        printf(" top1=0x%02x(%d,%.2fHz)",
               signal->top1_raw & 0xFF, signal->top1_count,
               signal_count_to_hz(signal->top1_count));
    }

    if (signal->top2_count > 0) {
        printf(" top2=0x%02x(%d,%.2fHz)",
               signal->top2_raw & 0xFF, signal->top2_count,
               signal_count_to_hz(signal->top2_count));
    }

    if (signal->resolution == IR_SIGNAL_CLEAR) {
        printf(" final=0x%02x", signal->raw & 0xFF);
    }

    printf("\n");
}

/**
 * @brief 打印一个采样窗口结束后的左右统计结果
 * @param tracker 当前轮次信号缓存
 * @return 无
 */
static void print_cycle_summary(const ir_tracker_t *tracker)
{
    printf("[IR] 本轮频率统计: 左=%s | 右=%s\n",
           tracker_signal_desc(&tracker->left, IR_SENSOR_LEFT),
           tracker_signal_desc(&tracker->right, IR_SENSOR_RIGHT));
    print_side_frequency_stats(&tracker->left, IR_SENSOR_LEFT);
    print_side_frequency_stats(&tracker->right, IR_SENSOR_RIGHT);
}

/**
 * @brief 构造一个空动作序列
 * @param 无
 * @return 初始化后的空动作序列
 */
static ir_action_sequence_t make_empty_sequence(void)
{
    ir_action_sequence_t sequence;

    memset(&sequence, 0, sizeof(sequence));
    return sequence;
}

/**
 * @brief 将单个红外码值转换为动作序列
 * @param side 码值来源侧
 * @param raw  原始红外码值
 * @return 对应的动作序列；未知码值则返回空序列
 */
static ir_action_sequence_t build_sequence_for_signal(ir_sensor_side_t side, uint32_t raw)
{
    ir_action_sequence_t sequence = make_empty_sequence();

    sequence.source_side = side;
    sequence.source_raw = raw;
    if (side == IR_SENSOR_RIGHT) {
        switch (raw) {
        case 0x11:
            sequence.action_count = 1;
            sequence.actions[0] = IR_ACTION_FORWARD20;
            break;
        case 0x12:
            sequence.action_count = 1;
            sequence.actions[0] = IR_ACTION_LEFT45;
            break;
        case 0x13:
            sequence.action_count = 2;
            sequence.actions[0] = IR_ACTION_LEFT45;
            sequence.actions[1] = IR_ACTION_FORWARD20;
            break;
        case 0x14:
            sequence.action_count = 1;
            sequence.actions[0] = IR_ACTION_FORWARD20;
            break;
        default:
            break;
        }
        return sequence;
    }

    switch (raw) {
    case 0x21:
        sequence.action_count = 2;
        sequence.actions[0] = IR_ACTION_RIGHT45;
        sequence.actions[1] = IR_ACTION_FORWARD20;
        break;
    case 0x22:
        sequence.action_count = 1;
        sequence.actions[0] = IR_ACTION_RIGHT45;
        break;
    case 0x23:
        sequence.action_count = 1;
        sequence.actions[0] = IR_ACTION_FORWARD20;
        break;
    case 0x24:
        sequence.action_count = 1;
        sequence.actions[0] = IR_ACTION_FORWARD20;
        break;
    default:
        break;
    }

    return sequence;
}

/**
 * @brief 将单侧 clear 信号转换为双侧融合判定分值
 * @param side 码值来源侧
 * @param raw  原始红外码值
 * @return 纠偏分值；未知码值返回 0
 */
static int score_signal_for_fusion(ir_sensor_side_t side, uint32_t raw)
{
    if (side == IR_SENSOR_LEFT) {
        switch (raw) {
        case 0x21:
            return -2;
        case 0x22:
            return -1;
        case 0x23:
            return 0;
        case 0x24:
            return 0;
        default:
            return 0;
        }
    }

    switch (raw) {
    case 0x11:
        return 0;
    case 0x12:
        return 1;
    case 0x13:
        return 2;
    case 0x14:
        return 0;
    default:
        return 0;
    }
}

/**
 * @brief 根据双侧融合总分生成动作序列
 * @param total 左右融合后的总分
 * @return 对应动作序列
 */
static ir_action_sequence_t build_sequence_for_total_score(int total)
{
    ir_action_sequence_t sequence = make_empty_sequence();
    //左负右正，数值绝对值越大表示偏离越严重，需要的纠偏动作越多
    if (total <= -2) {       
        sequence.action_count = 2;
        sequence.actions[0] = IR_ACTION_RIGHT45;
        sequence.actions[1] = IR_ACTION_FORWARD20;
    } else if (total == -1) {
        sequence.action_count = 1;
        sequence.actions[0] = IR_ACTION_RIGHT45;
    } else if (total == 0) {
        sequence.action_count = 1;
        sequence.actions[0] = IR_ACTION_FORWARD20;
    } else if (total == 1) {
        sequence.action_count = 1;
        sequence.actions[0] = IR_ACTION_LEFT45;
    } else {
        sequence.action_count = 2;
        sequence.actions[0] = IR_ACTION_LEFT45;
        sequence.actions[1] = IR_ACTION_FORWARD20;
    }

    return sequence;
}

/**
 * @brief 打印动作序列中的每一步动作
 * @param sequence 动作序列
 * @return 无
 */
static void print_action_sequence(const ir_action_sequence_t *sequence)
{
    for (int i = 0; i < sequence->action_count; i++) {
        if (i > 0) {
            printf(" + ");
        }
        printf("%s", action_desc(sequence->actions[i]));
    }
}

/**
 * @brief 打印“收到什么信号、由谁判定、准备执行什么动作”
 * @param tracker  当前采样轮次信号缓存
 * @param reason   当前判定原因
 * @param sequence 已生成的动作序列
 * @return 无
 */
static void print_action_log(const ir_tracker_t *tracker, const char *reason,
                             const ir_action_sequence_t *sequence)
{
    printf("[IR] 收到信号: 左=%s | 右=%s\n",
           tracker_signal_desc(&tracker->left, IR_SENSOR_LEFT),
           tracker_signal_desc(&tracker->right, IR_SENSOR_RIGHT));
    printf("[IR] 判定来源: %s raw=0x%02x -> %s\n",
           side_desc(sequence->source_side),
           sequence->source_raw & 0xFF,
           sensor_raw_desc(sequence->source_side, sequence->source_raw));
    printf("[IR] 判定动作: %s -> ", reason);
    if (sequence->action_count == 0) {
        printf("无动作");
    } else {
        print_action_sequence(sequence);
    }
    printf("\n");
}

/**
 * @brief 打印双侧融合判定的日志
 * @param tracker     当前采样轮次信号缓存
 * @param left_score  左侧融合分值
 * @param right_score 右侧融合分值
 * @param total       左右融合总分
 * @param reason      当前判定原因
 * @param sequence    已生成的动作序列
 * @return 无
 */
static void print_dual_action_log(const ir_tracker_t *tracker, int left_score,
                                  int right_score, int total, const char *reason,
                                  const ir_action_sequence_t *sequence)
{
    printf("[IR] 收到信号: 左=%s | 右=%s\n",
           tracker_signal_desc(&tracker->left, IR_SENSOR_LEFT),
           tracker_signal_desc(&tracker->right, IR_SENSOR_RIGHT));
    printf("[IR] 联合判定分值: left_score=%d right_score=%d total=%d\n",
           left_score, right_score, total);
    printf("[IR] 判定动作: %s -> ", reason);
    if (sequence->action_count == 0) {
        printf("无动作");
    } else {
        print_action_sequence(sequence);
    }
    printf("\n");
}

/**
 * @brief 执行动作序列中的单步动作
 * @param action 单步动作枚举
 * @return 0表示执行成功，非0表示执行失败
 */
static int execute_action(ir_action_id_t action)
{
    switch (action) {
    case IR_ACTION_LEFT45:
        return motion_action_left_45deg();
    case IR_ACTION_RIGHT45:
        return motion_action_right_45deg();
    case IR_ACTION_FORWARD20:
        return motion_action_forward_20cm();
    default:
        return 0;
    }
}

/**
 * @brief 执行完整动作序列
 * @param tracker  当前轮次信号缓存
 * @param reason   当前判定原因
 * @param sequence 将要执行的动作序列
 * @return 0表示执行成功，非0表示执行失败
 */
static int run_sequence(const ir_tracker_t *tracker, const char *reason,
                        const ir_action_sequence_t *sequence)
{
    int ret = 0;

    print_action_log(tracker, reason, sequence);
    for (int i = 0; i < sequence->action_count; i++) {
        ret = execute_action(sequence->actions[i]);
        if (ret != 0) {
            printf("[IR] 动作执行失败: %s ret=%d\n",
                   action_desc(sequence->actions[i]), ret);
            return ret;
        }
    }

    return ret;
}

/**
 * @brief 记录当前轮次某一侧收到的一个已知红外码值
 * @param tracker 采样状态缓存
 * @param side    传感器侧别
 * @param raw     当前读取到的有效码值
 * @return 无
 */
static void record_signal_sample(ir_tracker_t *tracker, ir_sensor_side_t side,
                                 uint32_t raw)
{
    ir_signal_state_t *target = (side == IR_SENSOR_LEFT) ? &tracker->left : &tracker->right;
    int index = signal_index_from_raw(side, raw);

    if (index < 0) {
        return;
    }

    target->counts[index]++;
    target->total_count++;
}

/**
 * @brief 根据当前轮次 clear 信号生成并执行动作
 * @param tracker 当前采样轮次信号缓存
 * @return 0表示执行成功，非0表示执行失败
 */
static int evaluate_and_execute_action(ir_tracker_t *tracker)
{
    ir_action_sequence_t sequence;
    bool right_clear = signal_is_clear(&tracker->right);
    bool left_clear = signal_is_clear(&tracker->left);

    if (right_clear) {
        if (left_clear) {
            int left_score = score_signal_for_fusion(IR_SENSOR_LEFT, tracker->left.raw);
            int right_score =
                score_signal_for_fusion(IR_SENSOR_RIGHT, tracker->right.raw);
            int total;
            int ret;

            if (left_score < 0 && right_score > 0 &&
                abs(left_score) == right_score) {
                printf("[IR] 收到信号: 左=%s | 右=%s\n",
                       tracker_signal_desc(&tracker->left, IR_SENSOR_LEFT),
                       tracker_signal_desc(&tracker->right, IR_SENSOR_RIGHT));
                printf("[IR] 联合判定分值: left_score=%d right_score=%d\n",
                       left_score, right_score);
                printf("[IR] 判定动作: 左右联合判定冲突 -> 右微调搜索\n");
                ret = motion_action_right_1deg();
                if (ret != 0) {
                    printf("[IR] 动作执行失败: 右微调搜索 ret=%d\n", ret);
                }
                return ret;
            }

            total = left_score + right_score;
            sequence = build_sequence_for_total_score(total);
            if (total <= -2) {
                print_dual_action_log(
                    tracker, left_score, right_score, total,
                    "左右联合判定 total<=-2，向右修正后前进", &sequence);
            } else if (total == -1) {
                print_dual_action_log(
                    tracker, left_score, right_score, total,
                    "左右联合判定 total=-1，向右修正", &sequence);
            } else if (total == 0) {
                print_dual_action_log(
                    tracker, left_score, right_score, total,
                    "左右联合判定 total=0，对齐后前进", &sequence);
            } else if (total == 1) {
                print_dual_action_log(
                    tracker, left_score, right_score, total,
                    "左右联合判定 total=1，向左修正", &sequence);
            } else {
                print_dual_action_log(
                    tracker, left_score, right_score, total,
                    "左右联合判定 total>=2，向左修正后前进", &sequence);
            }

            for (int i = 0; i < sequence.action_count; i++) {
                ret = execute_action(sequence.actions[i]);
                if (ret != 0) {
                    printf("[IR] 动作执行失败: %s ret=%d\n",
                           action_desc(sequence.actions[i]), ret);
                    return ret;
                }
            }

            return 0;
        }

        sequence = build_sequence_for_signal(IR_SENSOR_RIGHT, tracker->right.raw);
        return run_sequence(tracker, "只收到右侧有效信号", &sequence);
    }

    if (left_clear) {
        sequence = build_sequence_for_signal(IR_SENSOR_LEFT, tracker->left.raw);
        return run_sequence(tracker, "只收到左侧有效信号", &sequence);
    }

    return 0;
}

/**
 * @brief 清空某个设备文件描述符中已经积压的旧事件
 * @param fd 设备文件描述符
 * @return 无
 */
static void flush_device_events(int fd)
{
    struct input_event ev_buf[BUF_SIZE];

    while (1) {
        ssize_t len = read(fd, ev_buf, sizeof(ev_buf));

        if (len < 0) {
            if (errno == EAGAIN) {
                break;
            }

            perror("flush read");
            break;
        }

        if (len == 0) {
            break;
        }
    }
}

/**
 * @brief 清空左右红外设备中积压的旧事件
 * @param fd_left  左侧红外设备文件描述符
 * @param fd_right 右侧红外设备文件描述符
 * @return 无
 */
static void flush_all_pending_events(int fd_left, int fd_right)
{
    flush_device_events(fd_left);
    flush_device_events(fd_right);
}

/**
 * @brief 从指定设备中读取并解析本批次红外事件，并按码值累加频次
 * @param fd      设备文件描述符
 * @param side    当前设备对应的传感器侧别，仅用于日志
 * @param tag     打印日志使用的设备标签
 * @param tracker 当前轮次信号缓存
 * @return true表示本次读取到了有效信号，false表示没有
 */
static bool read_device_events(int fd, ir_sensor_side_t side, const char *tag,
                               ir_tracker_t *tracker)
{
    struct input_event ev_buf[BUF_SIZE];
    bool got_valid_signal = false;

    while (1) {
        ssize_t len = read(fd, ev_buf, sizeof(ev_buf));

        if (len < 0) {
            if (errno == EAGAIN) {
                break;
            }

            perror("read");
            break;
        }

        if (len == 0) {
            break;
        }

        int count = (int)(len / sizeof(struct input_event));

        for (int i = 0; i < count; i++) {
            uint32_t raw;
            ir_sensor_side_t signal_side;
            uint8_t b0;
            uint8_t b1;
            uint8_t b2;
            uint8_t b3;
            uint8_t real = 0;

            if (ev_buf[i].type != EV_MSC || ev_buf[i].code != MSC_SCAN) {
                continue;
            }

            raw = (uint32_t)ev_buf[i].value;
            if (raw == 0 || raw == 1) {
                continue;
            }

            b0 = (raw >> 0) & 0xFF;
            b1 = (raw >> 8) & 0xFF;
            b2 = (raw >> 16) & 0xFF;
            b3 = (raw >> 24) & 0xFF;

            switch (USE_BYTE_INDEX) {
            case 0:
                real = b0;
                break;
            case 1:
                real = b1;
                break;
            case 2:
                real = b2;
                break;
            case 3:
                real = b3;
                break;
            default:
                real = b0;
                break;
            }

            print_raw_debug(tag, raw, real);

            if (!is_known_signal(real)) {
                printf("[IR] ignore unknown %s raw=0x%02x\n", tag, real);
                continue;
            }

            signal_side = side;
            if (signal_side_from_raw(real, &signal_side) && signal_side != side) {
                printf("[IR] %s 设备收到 %s 码值 0x%02x，按码值归属计入%s统计\n",
                       side_desc(side), side_desc(signal_side), real & 0xFF,
                       side_desc(signal_side));
            }

            record_signal_sample(tracker, signal_side, real);
            got_valid_signal = true;
        }
    }

    return got_valid_signal;
}

/**
 * @brief 在一个采样窗口内收集左右两侧已知码值频率，并完成窗口归纳
 * @param epfd     epoll 文件描述符
 * @param fd_left  左侧红外设备文件描述符
 * @param fd_right 右侧红外设备文件描述符
 * @param tracker  当前轮次信号缓存
 * @return true表示本轮采样至少收到一个有效信号，false表示没有
 */
static bool collect_stable_signals(int epfd, int fd_left, int fd_right,
                                   ir_tracker_t *tracker)
{
    struct epoll_event events[MAX_EVENTS];
    long long start_ms = monotonic_ms_now();
    bool got_valid_signal = false;

    reset_cycle_signals(tracker);

    while ((monotonic_ms_now() - start_ms) < IR_SAMPLE_WINDOW_MS) {
        int n = epoll_wait(epfd, events, MAX_EVENTS, IR_EPOLL_WAIT_TIMEOUT_MS);

        if (n < 0) {
            if (errno == EINTR) {
                continue;
            }

            perror("epoll_wait");
            break;
        }

        for (int i = 0; i < n; i++) {
            if (events[i].data.fd == fd_left) {
                got_valid_signal |= read_device_events(fd_left, IR_SENSOR_LEFT,
                                                       "LEFT ", tracker);
            } else if (events[i].data.fd == fd_right) {
                got_valid_signal |= read_device_events(fd_right, IR_SENSOR_RIGHT,
                                                       "RIGHT", tracker);
            }
        }
    }

    finalize_cycle_signals(tracker);
    return got_valid_signal;
}

/**
 * @brief 执行单步搜索动作，并在动作后立即重新采样红外频率统计
 * @param step         当前搜索步骤
 * @param epfd         epoll 文件描述符
 * @param fd_left      左侧红外设备文件描述符
 * @param fd_right     右侧红外设备文件描述符
 * @param tracker      当前轮次信号缓存
 * @param found_signal 返回是否已找到有效信号
 * @return 0表示执行成功，非0表示执行失败
 */
static int run_search_step_and_probe(const ir_search_step_t *step, int epfd,
                                     int fd_left, int fd_right,
                                     ir_tracker_t *tracker, bool *found_signal)
{
    int ret;

    if (step == NULL || step->action == NULL || tracker == NULL ||
        found_signal == NULL) {
        return -1;
    }

    printf("[IR][SEARCH] 执行搜索步骤: %s\n", step->name);
    ret = step->action();
    if (ret != 0) {
        printf("[IR][SEARCH] 搜索动作执行失败: %s ret=%d\n", step->name, ret);
        return ret;
    }

    flush_all_pending_events(fd_left, fd_right);
    printf("[IR][SEARCH] 步骤完成，开始 %d ms 搜索采样: %s\n",
           IR_SAMPLE_WINDOW_MS, step->name);

    *found_signal = collect_stable_signals(epfd, fd_left, fd_right, tracker);
    if (!*found_signal) {
        printf("[IR][SEARCH] 当前步骤后仍未收到有效信号: %s\n", step->name);
        return 0;
    }

    print_cycle_summary(tracker);
    if (tracker_has_ambiguous_signal(tracker)) {
        printf("[IR][SEARCH] 搜索采样出现多码竞争 -> 本轮不动作，返回主循环重采样\n");
        return 0;
    }

    return evaluate_and_execute_action(tracker);
}

/**
 * @brief 当本轮采样没有任何已知信号时执行分阶段搜索动作
 * @param epfd     epoll 文件描述符
 * @param fd_left  左侧红外设备文件描述符
 * @param fd_right 右侧红外设备文件描述符
 * @param tracker  当前采样轮次信号缓存
 * @param now_ms   当前时间戳，当前实现中仅为接口保留
 * @return 0表示执行成功，非0表示执行失败
 */
static int maybe_run_search_action(int epfd, int fd_left, int fd_right,
                                   ir_tracker_t *tracker, long long now_ms)
{
    // static const ir_search_step_t k_initial_scan_steps[] = {
    //     {"右转45度搜索(第1次)", motion_action_right_45deg},
    //     {"右转45度搜索(第2次)", motion_action_right_45deg},
    //     {"右转45度搜索(第3次)", motion_action_right_45deg},
    // };
    // static const ir_search_step_t k_n_prefix_steps[] = {
    //     {"前进20厘米搜索", motion_action_forward_20cm},
    //     {"左转45度搜索", motion_action_left_45deg},
    //     {"前进20厘米搜索", motion_action_forward_20cm},
    // };
    // static const ir_search_step_t k_n_loop_steps[] = {
    //     {"右转90度搜索", motion_action_right_90deg},
    //     {"前进20厘米搜索", motion_action_forward_20cm},
    //     {"左转90度搜索", motion_action_left_90deg},
    //     {"前进20厘米搜索", motion_action_forward_20cm},
    // };
    // bool found_signal = false;
    // int ret;
    // size_t i;

    // (void)now_ms;
    // printf("[IR] 收到信号: 左=%s | 右=%s\n",
    //        tracker_signal_desc(&tracker->left, IR_SENSOR_LEFT),
    //        tracker_signal_desc(&tracker->right, IR_SENSOR_RIGHT));
    // printf("[IR] 判定动作: 本轮采样窗口内未收到有效信号 -> 启动分阶段搜索\n");

    // for (i = 0; i < sizeof(k_initial_scan_steps) / sizeof(k_initial_scan_steps[0]); i++) {
    //     ret = run_search_step_and_probe(&k_initial_scan_steps[i], epfd, fd_left,
    //                                     fd_right, tracker, &found_signal);
    //     if (ret != 0 || found_signal) {
    //         return ret;
    //     }
    // }

    // printf("[IR][SEARCH] 连续右转45度搜索3次仍未找到信号，进入N字搜索\n");

    // for (i = 0; i < sizeof(k_n_prefix_steps) / sizeof(k_n_prefix_steps[0]); i++) {
    //     ret = run_search_step_and_probe(&k_n_prefix_steps[i], epfd, fd_left,
    //                                     fd_right, tracker, &found_signal);
    //     if (ret != 0 || found_signal) {
    //         return ret;
    //     }
    // }

    // while (1) {
    //     for (i = 0; i < sizeof(k_n_loop_steps) / sizeof(k_n_loop_steps[0]); i++) {
    //         ret = run_search_step_and_probe(&k_n_loop_steps[i], epfd, fd_left,
    //                                         fd_right, tracker, &found_signal);
    //         if (ret != 0 || found_signal) {
    //             return ret;
    //         }
    //     }
    // }
    printf("[IR] 收到信号: 左=%s | 右=%s\n",
           tracker_signal_desc(&tracker->left, IR_SENSOR_LEFT),
           tracker_signal_desc(&tracker->right, IR_SENSOR_RIGHT));
    printf("[IR] 判定动作: 本轮采样窗口内未收到有效信号 -> 暂不执行搜索动作，直接重采样\n");
    return 0;
}

/**
 * @brief 红外回充测试程序入口
 * @param 无
 * @return 正常情况下不返回；初始化失败返回 -1
 */
int main(void)
{
    int fd_left = open_device(DEV_LEFT);
    int fd_right = open_device(DEV_RIGHT);
    int epfd;
    struct epoll_event ev;
    ir_tracker_t tracker;

    memset(&tracker, 0, sizeof(tracker));

    epfd = epoll_create1(0);
    if (epfd < 0) {
        perror("epoll_create");
        close(fd_left);
        close(fd_right);
        return -1;
    }

    ev.events = EPOLLIN | EPOLLET;
    ev.data.fd = fd_left;
    epoll_ctl(epfd, EPOLL_CTL_ADD, fd_left, &ev);

    ev.events = EPOLLIN | EPOLLET;
    ev.data.fd = fd_right;
    epoll_ctl(epfd, EPOLL_CTL_ADD, fd_right, &ev);

    printf("Start receiving infrared docking events...\n");

    while (1) {
        long long now_ms;

        flush_all_pending_events(fd_left, fd_right);
        printf("[IR] start %d ms sample window\n", IR_SAMPLE_WINDOW_MS);

        collect_stable_signals(epfd, fd_left, fd_right, &tracker);
        now_ms = monotonic_ms_now();
        print_cycle_summary(&tracker);

        if (tracker_has_ambiguous_signal(&tracker)) {
            printf("[IR] 判定动作: 采样窗口内出现多码竞争 -> 本轮不动作，直接重采样\n");
        } else if (tracker_has_any_signal(&tracker)) {
            evaluate_and_execute_action(&tracker);
        } else {
            maybe_run_search_action(epfd, fd_left, fd_right, &tracker, now_ms);
        }
    }

    close(fd_left);
    close(fd_right);
    close(epfd);
    return 0;
}
