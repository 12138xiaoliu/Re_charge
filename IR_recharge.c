/**
 * @file IR_recharge.c
 * @brief 红外充电信号采样与无信号搜索调试程序
 *
 * 周期性读取左右红外接收头上报的 MSC_SCAN 码值，统计采样窗口内的有效命中频次。
 * 当连续一个采样窗口内没有达到阈值的信号时，按 step/angle/xdistance 状态执行渐进式搜索动作。
 */

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

#include "motion.h"

#define DEV_LEFT "/dev/input/event3"
#define DEV_RIGHT "/dev/input/event2"

#define MAX_EVENTS 2 //!< 最大事件数量
#define BUF_SIZE 64 //!< 缓冲区大小
#define USE_BYTE_INDEX 0 //!< 使用字节索引

#define IR_SAMPLE_WINDOW_MS 4000 //!< 采样窗口时长，单位毫秒
#define IR_EPOLL_WAIT_TIMEOUT_MS 100 //!< epoll_wait 超时时长，单位毫秒
//有效信号的频率阈值，单位Hz，低于该频率则认为信号丢失
#define IR_VALID_THRESHOLD 3
#define IR_KNOWN_SIGNAL_COUNT 4

#define SEARCH_ROTATE_STEP_DEG 18
#define SEARCH_FULL_ROTATE_DEG 360
#define SEARCH_MAX_XDISTANCE 6

#define IR_TEST_AUDIO_CMD_ENV "IR_TEST_AUDIO_CMD"
#define IR_TEST_AUDIO_CMD_DEFAULT "echo \"startClock /data/resource/ai/sleep_response.wav\" > /tmp/main2voice"

/** @brief 红外信号码值归属的传感器侧别。*/
typedef enum {
    IR_SENSOR_LEFT = 0,//!< 左侧红外传感器
    IR_SENSOR_RIGHT,//!< 右侧红外传感器
} ir_sensor_side_t;

/** @brief 单侧红外传感器在一个采样窗口内的统计结果。 */
typedef struct {
    int counts[IR_KNOWN_SIGNAL_COUNT];//!< 已知码值的命中计数
    int total_count;//!< 该侧传感器在采样窗口内的总命中次数
    uint32_t top_raw;//!< 当前窗口内命中次数最多的原始码值
    int top_count;//!< 当前窗口内命中次数最多的码值的命中次数
} ir_side_stats_t;

/** @brief 左右两侧红外传感器的窗口统计汇总。*/
typedef struct {
    ir_side_stats_t left;//!< 左侧传感器统计结果
    ir_side_stats_t right;//!< 右侧传感器统计结果
} ir_sample_stats_t;

/** @brief 命中直线前进条件的左右红外码值组合。
 * @param right_raw 右侧传感器的原始码值
 * @param left_raw 左侧传感器的原始码值
 * @param right_count 右侧码值的命中次数
 * @param left_count 左侧码值的命中次数
 */
typedef struct {
    uint32_t right_raw;//!< 右侧码值
    uint32_t left_raw;//!< 左侧码值
    int right_count;//!< 右侧码值的命中次数
    int left_count;//!< 左侧码值的命中次数
} ir_forward_pair_hit_t;

/** @brief 无信号搜索流程的运行状态。*/
typedef struct {
    int step; //!< 搜索阶段：0 原地旋转，1 前进一步后旋转，2 横向展开搜索              
    int angle;      //!< 当前阶段已累计的旋转角度，单位度         
    int xdistance;      //!< step=2 时横向展开的 20cm 步数，范围 [1, SEARCH_MAX_XDISTANCE]     
    bool guidance_active;    //!< 上一轮是否已进入导引状态，true 表示已进入过导引，false 表示未进入过导引或已重置搜索状态
} ir_runtime_state_t;

/**
 * @brief 获取单调时钟的当前毫秒时间戳
 * @return 当前单调时钟毫秒值
 */
static long long monotonic_ms_now(void)
{
    struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long long)ts.tv_sec * 1000LL + ts.tv_nsec / 1000000LL;
}

/**
 * @brief 设置文件描述符为非阻塞模式
 * @param fd 要设置的文件描述符
 * @return 0 表示成功，-1 表示失败
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
 * @brief 打开红外输入设备并设置为非阻塞模式
 * @param dev 输入设备路径
 * @return 已打开的设备文件描述符；失败时打印错误并退出进程
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
 * @brief 返回红外传感器侧别的中文描述
 * @param side 传感器侧别
 * @return 侧别描述字符串
 */
static const char *side_desc(ir_sensor_side_t side)
{
    return side == IR_SENSOR_RIGHT ? "右侧" : "左侧";
}

/**
 * @brief 根据原始码值判断信号来自左侧还是右侧传感器
 * @param raw 输入事件里提取出的 8bit 有效码值
 * @param side_out 返回码值所属侧别，可为 NULL
 * @return true 表示 raw 属于已知红外码值，false 表示未知码值
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
 * @brief 将原始码值翻译为人类可读的信号描述
 * @param side 码值所属侧别
 * @param raw 输入事件里提取出的 8bit 有效码值
 * @return 信号描述字符串
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
            return "左侧没有有效信号";
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
        return "右侧没有有效信号";
    }
}


/**
 * @brief 将单侧传感器的原始码值转换为索引，便于统计和频率计算
 * @param side 码值所属侧别
 * @param raw 输入事件里提取出的 8bit 有效码值
 * @return 索引值，范围 [0, IR_KNOWN_SIGNAL_COUNT-1]；如果 raw 不属于已知码值则返回 -1
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
 * @brief 将单侧传感器的统计索引转换回原始码值
 * @param side 码值所属侧别
 * @param index 统计数组索引
 * @return 对应原始码值；索引非法时返回 0
 */
static uint32_t signal_raw_from_index(ir_sensor_side_t side, int index)
{
    static const uint32_t k_left_codes[IR_KNOWN_SIGNAL_COUNT] = {0x21, 0x22, 0x23, 0x24};
    static const uint32_t k_right_codes[IR_KNOWN_SIGNAL_COUNT] = {0x11, 0x12, 0x13, 0x14};

    if (index < 0 || index >= IR_KNOWN_SIGNAL_COUNT) {
        return 0;
    }

    return side == IR_SENSOR_RIGHT ? k_right_codes[index] : k_left_codes[index];
}


/**
 * @brief 将信号计数转换为频率（Hz）
 * @param count 采样窗口内的信号计数
 * @return 频率值，单位Hz
 */
static double signal_count_to_hz(int count)
{
    return (double)count * 1000.0 / (double)IR_SAMPLE_WINDOW_MS;
}

/**
 * @brief 清空一个采样窗口的统计结果
 * @param stats 采样统计结构
 */
static void reset_sample_stats(ir_sample_stats_t *stats)
{
    memset(stats, 0, sizeof(*stats));
}

/**
 * @brief 将一个已知红外码值累加到采样统计中
 * @param stats 采样统计结构
 * @param side 码值所属侧别
 * @param raw 输入事件里提取出的 8bit 有效码值
 */
static void record_signal_sample(ir_sample_stats_t *stats, ir_sensor_side_t side,
                                 uint32_t raw)
{
    ir_side_stats_t *target = side == IR_SENSOR_LEFT ? &stats->left : &stats->right;
    int index = signal_index_from_raw(side, raw);

    if (index < 0) {
        return;
    }

    target->counts[index]++;
    target->total_count++;
}

/**
 * @brief 计算单侧采样统计中的最高频码值
 * @param side_stats 单侧采样统计结构
 * @param side 码值所属侧别
 */
static void finalize_side_stats(ir_side_stats_t *side_stats, ir_sensor_side_t side)
{
    side_stats->top_raw = 0;
    side_stats->top_count = 0;

    for (int i = 0; i < IR_KNOWN_SIGNAL_COUNT; i++) {
        if (side_stats->counts[i] > side_stats->top_count) {
            side_stats->top_count = side_stats->counts[i];
            side_stats->top_raw = signal_raw_from_index(side, i);
        }
    }
}

/**
 * @brief 计算左右两侧采样统计中的最高频码值
 * @param stats 采样统计结构
 */
static void finalize_sample_stats(ir_sample_stats_t *stats)
{
    finalize_side_stats(&stats->left, IR_SENSOR_LEFT);
    finalize_side_stats(&stats->right, IR_SENSOR_RIGHT);
}

/**
 * @brief 判断单侧采样统计中是否存在达到阈值的码值
 * @param side_stats 单侧采样统计结构
 * @return true 表示有有效信号，false 表示没有
 */
static bool side_has_valid_signal(const ir_side_stats_t *side_stats)
{
    for (int i = 0; i < IR_KNOWN_SIGNAL_COUNT; i++) {
        if (side_stats->counts[i] >= IR_VALID_THRESHOLD) {
            return true;
        }
    }

    return false;
}

/**
 * @brief 判断采样窗口内左右任一侧是否有有效信号
 * @param stats 采样统计结构
 * @return true 表示有有效信号，false 表示没有
 */
static bool sample_has_valid_signal(const ir_sample_stats_t *stats)
{
    return side_has_valid_signal(&stats->left) || side_has_valid_signal(&stats->right);
}

/**
 * @brief 查询单侧采样统计中指定原始码值的命中次数
 * @param side_stats 单侧采样统计结构
 * @param side 码值所属侧别
 * @param raw 输入事件里提取出的 8bit 有效码值
 * @return 指定码值的命中次数；参数非法或未知码值时返回 0
 */
static int side_signal_count_for_raw(const ir_side_stats_t *side_stats,
                                     ir_sensor_side_t side, uint32_t raw)
{
    int index;

    if (side_stats == NULL) {
        return 0;
    }

    index = signal_index_from_raw(side, raw);
    if (index < 0) {
        return 0;
    }

    return side_stats->counts[index];
}

/**
 * @brief 查找是否存在满足直线前进条件的左右码值组合
 * @param stats 采样统计结构
 * @param hit 输出命中的最佳组合，可为 NULL
 * @return true 表示存在满足阈值的组合，false 表示不存在
 *
 * 当前只有右侧 0x12 与左侧 0x22 同时达到阈值时才触发直线前进。
 */
static bool find_forward_pair_hit(const ir_sample_stats_t *stats,
                                  ir_forward_pair_hit_t *hit)
{
    int right_count;
    int left_count;
    int right_deviation_count;
    int left_deviation_count;
    if (stats == NULL) {
        return false;
    }
    // if(&stats->left.top_raw!=0x22 || &stats->right.top_raw!=0x12){
    //     return false;
    // }
    right_count = side_signal_count_for_raw(&stats->right, IR_SENSOR_RIGHT, 0x12);
    left_count = side_signal_count_for_raw(&stats->left, IR_SENSOR_LEFT, 0x22);
    right_deviation_count = side_signal_count_for_raw(&stats->right,IR_SENSOR_RIGHT,0x13);
    left_deviation_count = side_signal_count_for_raw(&stats->left,IR_SENSOR_LEFT,0x21);
    //0x12和0x13里任意一个没有达到阈值直接退出调整
    if (right_count < IR_VALID_THRESHOLD || left_count < IR_VALID_THRESHOLD) {
        return false;
    }
    if(right_deviation_count-left_deviation_count>IR_VALID_THRESHOLD){
        printf("右边右信号过强，可能偏右了，建议微调向左\n");
        return false;
    }
    if(left_deviation_count-right_deviation_count>IR_VALID_THRESHOLD){
        printf("左边左信号过强，可能偏左了，建议微调向右\n");
        return false;
    }
    if (hit != NULL) {
        hit->right_raw = 0x12;
        hit->left_raw = 0x22;
        hit->right_count = right_count;
        hit->left_count = left_count;
    }
    return true;
}

/**
 * @brief 打印单侧采样统计中达到阈值的有效命中
 * @param side_stats 单侧采样统计结构
 * @param side 码值所属侧别
 */
static void print_valid_hits_for_side(const ir_side_stats_t *side_stats,
                                      ir_sensor_side_t side)
{
    for (int i = 0; i < IR_KNOWN_SIGNAL_COUNT; i++) {
        uint32_t raw = signal_raw_from_index(side, i);

        if (side_stats->counts[i] < IR_VALID_THRESHOLD) {
            continue;
        }

        printf("[IR][VALID] %s 码值 0x%02x count=%d freq=%.2fHz -> %s\n",
               side_desc(side), raw & 0xFF, side_stats->counts[i],
               signal_count_to_hz(side_stats->counts[i]), sensor_raw_desc(side, raw));
    }
}

/**
 * @brief 打印单侧采样窗口的频率统计
 * @param side_stats 单侧采样统计结构
 * @param side 码值所属侧别
 */
static void print_side_frequency_stats(const ir_side_stats_t *side_stats,
                                       ir_sensor_side_t side)
{
    printf("[IR][STAT] %s total=%d(%.2fHz)\n",
           side_desc(side), side_stats->total_count,
           signal_count_to_hz(side_stats->total_count));

    for (int i = 0; i < IR_KNOWN_SIGNAL_COUNT; i++) {
        uint32_t raw = signal_raw_from_index(side, i);

        printf(" 0x%02x=%d(%.2fHz)\n", raw & 0xFF, side_stats->counts[i],
               signal_count_to_hz(side_stats->counts[i]));
    }

    if (side_stats->top_count > 0) {
        printf(" top=0x%02x(%d,%.2fHz)\n",
               side_stats->top_raw & 0xFF, side_stats->top_count,
               signal_count_to_hz(side_stats->top_count));
    }

    printf("\n");
}

/**
 * @brief 打印整个采样窗口内的统计结果摘要
 * @param stats 采样窗口统计结果
 */
static void print_sample_summary(const ir_sample_stats_t *stats)
{
    print_side_frequency_stats(&stats->left, IR_SENSOR_LEFT);
    print_side_frequency_stats(&stats->right, IR_SENSOR_RIGHT);
}

/**
 * @brief 打印单个红外输入事件的原始码值解析结果
 * @param tag 调试输出标签
 * @param raw 输入事件上报的 32bit 扫描码
 * @param real 按 USE_BYTE_INDEX 提取出的 8bit 有效码值
 */
static void print_raw_debug(const char *tag, uint32_t raw, uint8_t real)
{
    ir_sensor_side_t device_side =
        strcmp(tag, "RIGHT") == 0 ? IR_SENSOR_RIGHT : IR_SENSOR_LEFT;
    ir_sensor_side_t signal_side = device_side;

    if (!signal_side_from_raw(real, &signal_side)) {
        signal_side = device_side;
    }

    printf("[%s] raw=0x%08x real=0x%02x device_side=%s code_side=%s -> %s\n",
           tag, raw, real, side_desc(device_side), side_desc(signal_side),
           sensor_raw_desc(signal_side, real));
}


/**
 * @brief 刷新输入设备上积压的事件，避免旧事件干扰后续采样
 * @param fd 输入设备 fd
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
 * @brief 清空左右输入设备上积压的事件
 * @param fd_left 左侧输入设备 fd
 * @param fd_right 右侧输入设备 fd
 */
static void flush_all_pending_events(int fd_left, int fd_right)
{
    flush_device_events(fd_left);
    flush_device_events(fd_right);
}

/**
 * @brief 打印并统计单个输入设备当前积压的红外事件
 * @param fd 输入设备 fd
 * @param device_side 该设备物理安装的侧别
 * @param tag 调试输出标签
 * @param stats 采样窗口统计结果
 * @return true 表示本轮读取到了可识别的红外码值
 *
 * 输入设备上报的是 32bit 扫描码，本函数按 USE_BYTE_INDEX 提取其中一个字节作为真实码值。
 * 如果设备侧别与码值归属不一致，统计时以后者为准，方便排查接线或驱动映射问题。
 */
static bool read_device_events(int fd, ir_sensor_side_t device_side, const char *tag,
                               ir_sample_stats_t *stats)
{
    struct input_event ev_buf[BUF_SIZE];
    bool got_known_signal = false;

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
            ir_sensor_side_t signal_side = device_side;
            uint8_t real = 0;

            if (ev_buf[i].type != EV_MSC || ev_buf[i].code != MSC_SCAN) {
                continue;
            }

            raw = (uint32_t)ev_buf[i].value;
            if (raw == 0 || raw == 1) {
                continue;
            }

            /* 不同驱动版本可能把有效码值放在不同字节，调试时可通过 USE_BYTE_INDEX 切换。 */
            switch (USE_BYTE_INDEX) {
            case 0:
                real = (raw >> 0) & 0xFF;
                break;
            case 1:
                real = (raw >> 8) & 0xFF;
                break;
            case 2:
                real = (raw >> 16) & 0xFF;
                break;
            case 3:
                real = (raw >> 24) & 0xFF;
                break;
            default:
                real = (raw >> 0) & 0xFF;
                break;
            }

            print_raw_debug(tag, raw, real);

            if (!signal_side_from_raw(real, &signal_side)) {
                printf("[IR] ignore unknown %s raw=0x%02x\n", tag, real);
                continue;
            }

            if (signal_side != device_side) {
                printf("[IR] %s 设备收到 %s 码值 0x%02x，按码值归属计入%s统计\n",
                       side_desc(device_side), side_desc(signal_side), real & 0xFF,
                       side_desc(signal_side));
            }

            record_signal_sample(stats, signal_side, real);
            got_known_signal = true;
        }
    }

    return got_known_signal;
}

/**
 * @brief 在固定采样窗口内汇总左右红外传感器的命中统计
 * @param epfd 监听左右输入设备的 epoll fd
 * @param fd_left 左侧输入设备 fd
 * @param fd_right 右侧输入设备 fd
 * @param stats 输出统计结果
 * @return true 表示采样窗口内至少读到过一次已知红外码值
 */
static bool collect_signal_stats(int epfd, int fd_left, int fd_right,
                                 ir_sample_stats_t *stats)
{
    struct epoll_event events[MAX_EVENTS];
    long long start_ms = monotonic_ms_now();
    bool got_known_signal = false;

    reset_sample_stats(stats);

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
                got_known_signal |=
                    read_device_events(fd_left, IR_SENSOR_LEFT, "LEFT ", stats);
            } else if (events[i].data.fd == fd_right) {
                got_known_signal |=
                    read_device_events(fd_right, IR_SENSOR_RIGHT, "RIGHT", stats);
            }
        }
    }

    finalize_sample_stats(stats);
    return got_known_signal;
}

/**
 * @brief 重置无信号搜索流程到初始状态
 * @param state 搜索状态机
 */
static void reset_search_state(ir_runtime_state_t *state)
{
    state->step = 0;
    state->angle = 0;
    state->xdistance = 1;
}

/**
 * @brief 设置搜索动作执行前使用的速度档位
 * @param speed_level 目标速度档位，支持 1/2/3
 * @return 0 表示设置成功，非 0 表示设置失败
 */
static int set_search_speed(int speed_level)
{
    int ret = motion_set_speed_level(speed_level);

    if (ret != 0) {
        printf("[IR][SEARCH] 设置速度档失败 ret=%d\n", ret);
    }

    return ret;
}

/**
 * @brief 以低速执行一次右转 45 度搜索动作
 * @return 0 表示动作执行成功，非 0 表示设置速度或动作执行失败
 */
static int search_rotate_right_once(void)
{
    int ret = set_search_speed(1);

    if (ret != 0) {
        return ret;
    }

    return motion_turn_right_distance_mm(MOTION_FIXED_RIGHT_45_VALUE);
}

/**
 * @brief 以低速执行一次右转 90 度搜索动作
 * @return 0 表示动作执行成功，非 0 表示设置速度或动作执行失败
 */
static int search_turn_right_90deg(void)
{
    int ret = set_search_speed(1);

    if (ret != 0) {
        return ret;
    }

    return motion_turn_right_distance_mm(MOTION_FIXED_RIGHT_90_VALUE);
}

/**
 * @brief 检测到有效对齐信号后执行一次直线前进动作
 * @return 0 表示动作执行成功，非 0 表示设置速度或动作执行失败
 */
static int find_single_forward()
{
    int ret = set_search_speed(1);
    if (ret != 0) {
        return ret;
    }
    return motion_forward_distance_mm(MOTION_FIND_SINGLE_FORWARD_VALUE);
}

/**
 * @brief 以低速执行一次直线前进 20cm 搜索动作
 * @return 0 表示动作执行成功，非 0 表示设置速度或动作执行失败
 */
static int search_forward_20cm(void)
{
    int ret = set_search_speed(1);

    if (ret != 0) {
        return ret;
    }

    return motion_forward_distance_mm(MOTION_FIXED_FORWARD_20CM_VALUE);
}

/**
 * @brief 播放进入导引流程时的提示音
 * @return system() 的执行结果，0 通常表示命令启动成功
 */
static int play_test_audio(void)
{
    const char *cmd = getenv(IR_TEST_AUDIO_CMD_ENV);

    if (cmd == NULL || cmd[0] == '\0') {
        cmd = IR_TEST_AUDIO_CMD_DEFAULT;
    }

    printf("[IR][AUDIO] 播放测试音频 cmd=%s\n", cmd);
    return system(cmd);
}

/**
 * @brief 进入正常导引流程的钩子函数，当前实现为打印有效信号详情
 * @param stats 当前采样窗口统计结果
 * @return 0 表示处理成功，非 0 表示导引动作钩子失败
 *
 * 在 handle_valid_signal() 中首次检测到有效信号时调用该函数，后续可在此基础上增加更复杂的导引动作。
 */
static int enter_normal_guidance_flow(const ir_sample_stats_t *stats)
{
    print_valid_hits_for_side(&stats->left, IR_SENSOR_LEFT);
    print_valid_hits_for_side(&stats->right, IR_SENSOR_RIGHT);
    printf("[IR][GUIDE] 调试模式：已进入正常导引流程钩子\n");
    return 0;
}

/**
 * @brief 执行一次角度微调后再后退
 * @param action_desc 微调动作描述，用于日志
 * @param adjust_fn 微调动作函数
 * @return 0 表示动作序列执行成功，非 0 表示微调或后退失败
 */
static int run_adjust_then_backward(const char *action_desc, int (*adjust_fn)(void))
{
    int ret;

    printf("[IR][GUIDE] 执行动作: %s -> 后退5cm\n", action_desc);
    ret = adjust_fn();
    if (ret != 0) {
        printf("[IR][GUIDE] 微调动作失败: %s ret=%d\n", action_desc, ret);
        return ret;
    }

    ret = motion_action_backward_5cm();
    if (ret != 0) {
        printf("[IR][GUIDE] 后退5cm失败 ret=%d\n", ret);
    }
    return ret;
}

/**
 * @brief 根据当前有效红外信号选择并执行导引动作
 * @param stats 当前采样窗口统计结果
 * @return 0 表示无需动作或动作成功，非 0 表示动作执行失败
 *
 * 当前策略先判断右侧 0x12 + 左侧 0x22 的直线前进组合；
 * 只有单侧目标码值存在且另一侧未达阈值时，按偏差方向微调后退。
 */
static int run_guidance_action_for_valid_signal(const ir_sample_stats_t *stats)
{
    ir_forward_pair_hit_t forward_hit;
    int right_center_count;
    int left_center_count;
    int right_deviation_count;
    int left_deviation_count;
    //未命中0x12 0x22组合或不是对多数码值命中，才考虑单侧微调
    if (!find_forward_pair_hit(stats, &forward_hit)) {
        right_center_count = side_signal_count_for_raw(&stats->right, IR_SENSOR_RIGHT, 0x12);
        left_center_count = side_signal_count_for_raw(&stats->left, IR_SENSOR_LEFT, 0x22);
        right_deviation_count = side_signal_count_for_raw(&stats->right, IR_SENSOR_RIGHT, 0x13);
        left_deviation_count = side_signal_count_for_raw(&stats->left, IR_SENSOR_LEFT, 0x21);

        if (right_center_count > 0 && left_center_count < 2) {
            printf("[IR][GUIDE] 右侧0x12存在 count=%d，左侧0x22 count=%d 未达阈值 -> 左微调后退\n",
                   right_center_count, left_center_count);
            printf("\n左侧中心信号未达到阈值,向左前方微调前进\n");
            int ret = motion_action_forward_5cm();
            ret = motion_action_left_1deg();
            return ret;
        }

        if (left_center_count > 0 && right_center_count < 2) {
            printf("[IR][GUIDE] 左侧0x22存在 count=%d，右侧0x12 count=%d 未达阈值 -> 右微调后退\n",
                   left_center_count, right_center_count);
            int ret= run_adjust_then_backward("右微调1度", motion_action_left_1deg);
            ret = motion_action_right_1deg();
            return ret;
        }
        if (right_deviation_count > IR_VALID_THRESHOLD || left_deviation_count < IR_VALID_THRESHOLD) {
            printf("\n右侧信号过强,可能偏右了,建议微调向左\n");
            int ret = motion_action_left_1deg();
            ret= motion_action_backward_5cm();
            ret= motion_action_right_1deg();
            return ret;
        }
        if (left_deviation_count > 8 || right_deviation_count < IR_VALID_THRESHOLD) {
            printf("\n左侧信号过强,可能偏左了,建议微调向右\n");
            int ret = motion_action_right_1deg();
            ret= motion_action_backward_5cm();
            ret= motion_action_left_1deg(); 
            return ret;
        }

        printf("[IR][GUIDE] 当前有效信号未命中目标码值0x12/0x22,本轮不下发导引动作\n");
        return 0;
    }

    printf("[IR][GUIDE] 命中直线前进组合: 右侧0x%02x count=%d + 左侧0x%02x count=%d -> 直线前进\n",
           forward_hit.right_raw & 0xFF, forward_hit.right_count,
           forward_hit.left_raw & 0xFF, forward_hit.left_count);
    return find_single_forward();
}

/**
 * @brief 在首次检测到有效信号时切换到导引模式
 * @param stats 当前采样窗口统计结果
 * @param state 搜索状态机
 * @return 0 表示处理成功，非 0 表示导引动作钩子失败
 *
 * 进入导引模式前会清空搜索进度；只有从“未导引”切到“导引”时才播放一次提示音。
 * 状态处理完成后再交由 run_guidance_action_for_valid_signal() 分析当前有效信号并选择动作。
 */
static int handle_valid_signal(const ir_sample_stats_t *stats,
                               ir_runtime_state_t *state)
{
    bool was_guiding = state->guidance_active;
    int audio_ret = 0;
    int ret;

    /* 有效信号重新接管动作序列，先清空无信号搜索状态。 */
    reset_search_state(state);
    if (!was_guiding) {
        audio_ret = play_test_audio();
        if (audio_ret != 0) {
            printf("[IR][AUDIO] 播放测试音频失败 ret=%d\n", audio_ret);
        }
    }

    state->guidance_active = true;
    printf("\n[IR] %d秒内检测到有效信号(阈值>=%d) -> step=%d angle=%d xdistance=%d\n",
           IR_SAMPLE_WINDOW_MS / 1000, IR_VALID_THRESHOLD, state->step, state->angle, state->xdistance);

    ret = enter_normal_guidance_flow(stats);
    if (ret != 0) {
        return ret;
    }

    return run_guidance_action_for_valid_signal(stats);
}

/**
 * @brief 执行“当前阶段继续旋转搜索”这一公共动作（向右旋转）
 * @param state 搜索状态机
 * @return 0 表示旋转成功，非 0 表示底盘动作失败
 */
static int advance_rotate_progress(ir_runtime_state_t *state)
{
    int ret = search_rotate_right_once();

    if (ret != 0) {
        return ret;
    }

    state->angle += SEARCH_ROTATE_STEP_DEG;
    printf("[IR][SEARCH] 当前旋转进度 angle=%d/%d\n",
           state->angle, SEARCH_FULL_ROTATE_DEG);
    return 0;
}

/**
 * @brief 按 step/angle/xdistance 状态执行无信号搜索流程
 * @param state 搜索状态机
 * @return 0 表示本轮搜索动作完成，非 0 表示底盘动作失败
 *
 * 搜索流程分三阶段：
 * - step=0：原地每次右转 45 度，完成一整圈后进入 step=1；
 * - step=1：先前进 20cm，再重复一整圈旋转；
 * - step=2：右转 90 度后按 xdistance 前进若干个 20cm，再做一整圈旋转，逐步扩大搜索范围。
 */
static int run_no_signal_search(ir_runtime_state_t *state)
{
    int ret;

    state->guidance_active = false;
    //如果 step/angle/xdistance 的值异常，重置搜索状态，避免执行不可预期的动作序列。
    if (state->step < 0 || state->step > 2) {
        reset_search_state(state);
    }

    printf("[IR] 2秒内无有效信号 -> 进入无信号搜索 step=%d angle=%d xdistance=%d\n",
           state->step, state->angle, state->xdistance);

    switch (state->step) {
    case 0:
        printf("[IR][SEARCH] step=0 原地慢速旋转搜索\n");
        //旋转动作执行失败时直接返回错误码，不继续后续动作，避免叠加过多失败动作导致底盘异常。
        ret = advance_rotate_progress(state);
        if (ret != 0) {
            return ret;
        }
        /* 给底盘动作留出完成时间，避免下一条命令过早叠加。 */
        sleep(1);
        //判断当前旋转的角度是否已经完成一整圈，进入下一阶段
        if (state->angle >= SEARCH_FULL_ROTATE_DEG) {
            state->angle = 0;
            state->step = 1;
            printf("[IR][SEARCH] step=0 完成一圈，切换到 step=1\n");
        }
        return 0;

    case 1:
        if (state->angle == 0) {
            printf("[IR][SEARCH] step=1 先前进20cm\n");
            ret = search_forward_20cm();
            if (ret != 0) {
                return ret;
            }
        }
        sleep(1);
        printf("[IR][SEARCH] step=1 原地慢速旋转搜索\n");
        ret = advance_rotate_progress(state);
        if (ret != 0) {
            return ret;
        }
        sleep(1);

        if (state->angle >= SEARCH_FULL_ROTATE_DEG) {
            state->angle = 0;
            state->step = 2;
            printf("[IR][SEARCH] step=1 完成一圈，切换到 step=2\n");
        }
        return 0;

    case 2:
        if (state->angle == 0) {
            printf("[IR][SEARCH] step=2 先右转90度，再前进 %d x 20cm\n",
                   state->xdistance);
            ret = search_turn_right_90deg();
            sleep(1);
            if (ret != 0) {
                return ret;
            }
            sleep(1);
            for (int i = 0; i < state->xdistance; i++) {
                ret = search_forward_20cm();
                if (ret != 0) {
                    return ret;
                }
            }
        }

        printf("[IR][SEARCH] step=2 原地慢速旋转搜索\n");
        ret = advance_rotate_progress(state);
        if (ret != 0) {
            return ret;
        }
        sleep(1);
        if (state->angle >= SEARCH_FULL_ROTATE_DEG) {
            state->angle = 0;
            state->xdistance++;
            if (state->xdistance > SEARCH_MAX_XDISTANCE) {
                reset_search_state(state);
                printf("[IR][SEARCH] step=2 xdistance 超过 %d，回到 step=0\n",
                       SEARCH_MAX_XDISTANCE);
            } else {
                printf("[IR][SEARCH] step=2 完成一圈，xdistance 增加到 %d\n",
                       state->xdistance);
            }
        }
        return 0;

    default:
        reset_search_state(state);
        return 0;
    }
}

/**
 * @brief 调试入口：循环采样红外信号并驱动导引/搜索流程
 * @return 0 表示程序正常结束，-1 表示初始化失败
 */
int main(void)
{
    //初始化
    int fd_left = open_device(DEV_LEFT);
    int fd_right = open_device(DEV_RIGHT);
    int epfd;
    struct epoll_event ev;
    ir_sample_stats_t stats;
    ir_runtime_state_t state;

    memset(&state, 0, sizeof(state));
    
    reset_search_state(&state);

    // 设置输入设备事件监听
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

    printf("[IR] IR_recharge debug start, 窗口采样时间=%dms 有效信号阈值：单位时间出现%d次\n",
           IR_SAMPLE_WINDOW_MS, IR_VALID_THRESHOLD);

    while (1) {
        int ret;
        //每轮采样前先清空输入设备上积压的事件，避免旧事件干扰统计和状态判断
        flush_all_pending_events(fd_left, fd_right);
        printf("[IR] 开始 %d ms 采样\n", IR_SAMPLE_WINDOW_MS);
        //在固定采样窗口内汇总左右红外传感器的命中统计
        collect_signal_stats(epfd, fd_left, fd_right, &stats);

        print_sample_summary(&stats);
        //处理收到的信号
        if (sample_has_valid_signal(&stats)) {
            ret = handle_valid_signal(&stats, &state);
        } else {
            ret = run_no_signal_search(&state);
        }

        if (ret != 0) {
            printf("[IR] 当前轮动作执行失败 ret=%d\n", ret);
        }
    }

    close(fd_left);
    close(fd_right);
    close(epfd);
    return 0;
}
