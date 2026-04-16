#include "../include/motion.h"
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <time.h>
#include <unistd.h>


// 轮询等待的最小时间片，避免长时间阻塞且便于后续插入更多状态检查。
#define MOTION_POLL_MS 20

//当前测试程序中使用的线速度配置。
static int g_linear_speed_mmps = MOTION_DEFAULT_LINEAR_SPEED_MMPS;
//旋转速度，单位mm/s，实际意义是底盘转动时每秒钟轮子边缘扫过的毫米数，数值越大转得越快
static int g_turn_speed_mmps = MOTION_DEFAULT_TURN_SPEED_MMPS;
//转圈动作持续时长，单位ms，实际意义是执行一次转圈动作的时间长度，数值越大转得越慢
static int g_circle_duration_ms = MOTION_DEFAULT_CIRCLE_DURATION_MS;
//当前速度档位，1/2/3分别对应低/中/高速，初始值为0表示未设置过档位
static int g_current_speed_level = 0;

// g_motion_lock 用于串行化动作
static pthread_mutex_t g_motion_lock = PTHREAD_MUTEX_INITIALIZER;
//g_speed_lock 用于保护速度档位状态。
static pthread_mutex_t g_speed_lock = PTHREAD_MUTEX_INITIALIZER;

/**
 * @brief 获取单调时钟毫秒值
 * @param 无
 * @return 当前单调时钟的毫秒时间戳
 */
static long long motion_monotonic_ms_now(void)
{
    struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long long)ts.tv_sec * 1000LL + ts.tv_nsec / 1000000LL;
}

/**
 * @brief 将字符串写入指定状态文件
 * @param path 目标文件路径
 * @param value 要写入的字符串内容
 * @return 0表示写入成功，-1表示写入失败
 */
static int motion_update_file(const char *path, const char *value)
{
    int fd;
    size_t value_len;

    if (path == NULL || value == NULL) {
        return -1;
    }

    fd = open(path, O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (fd < 0) {
        printf("[motion] open %s failed: %s\n", path, strerror(errno));
        return -1;
    }

    value_len = strlen(value);
    if (write(fd, value, value_len) < 0) {
        printf("[motion] write %s failed: %s\n", path, strerror(errno));
        close(fd);
        return -1;
    }

    close(fd);
    return 0;
}

/**
 * @brief 更新最近一次底盘tof运动时间戳文件
 * @param 无
 * @return 无
 */
static void motion_update_tof_timestamp_file(void)
{
    char val[32];
    snprintf(val, sizeof(val), "%lld\n", motion_monotonic_ms_now());
    motion_update_file(MOTION_TOF_MOTION_TS_FILE, val);
}

/**
 * @brief 更新当前速度档位状态文件
 * @param speed_level 当前速度档位
 * @return 无
 */
static void motion_update_motor_speed_file(int speed_level)
{
    char val[8];

    snprintf(val, sizeof(val), "%d\n", speed_level);
    motion_update_file(MOTION_MOTOR_SPEED_FILE, val);
}

/**
 * @brief 判断命令是否为底盘移动类命令
 * @param cmd 单字符运动命令
 * @return true表示是底盘运动命令，false表示不是
 */
static bool motion_is_drive_cmd(char cmd)
{
    return cmd == 'W' || cmd == 'S' || cmd == 'A' || cmd == 'D';
}

/**
 * @brief 将距离值换算为执行时长
 * @param distance_mm 目标距离/标定值
 * @param speed_mmps 当前速度，单位mm/s
 * @return 换算后的时长，单位ms；失败返回-1
 */
static int motion_distance_to_duration_ms(int distance_mm, int speed_mmps)
{
    long long duration_ms;

    if (distance_mm <= 0 || speed_mmps <= 0) {
        return -1;
    }

    duration_ms = ((long long)distance_mm * 1000LL + speed_mmps - 1) / speed_mmps;
    if (duration_ms <= 0 || duration_ms > INT_MAX) {
        return -1;
    }
    printf("\n换算后的时长: distance_mm=%d speed_mmps=%d -> duration_ms=%lld\n",
           distance_mm, speed_mmps, duration_ms);
    return (int)duration_ms;
}

/**
 * @brief 执行按距离换算的底盘动作
 * @param cmd 单字符运动命令，支持 'W'/'S'/'A'/'D'
 * @param distance_mm 距离或标定值
 * @param speed_mmps 当前动作对应速度
 * @return 0表示执行成功，非0表示执行失败
 */
static int motion_run_distance_cmd(char cmd, int distance_mm, int speed_mmps)
{
    int duration_ms;
    int ret;
    int stop_ret;
    // 先将距离换算为时长，如果换算失败则直接返回错误。
    duration_ms = motion_distance_to_duration_ms(distance_mm, speed_mmps);
    if (duration_ms < 0) {
        return -1;
    }
    // 这里加锁是为了保证从发送命令到等待再到发送停止命令的整个过程不被其他动作打断，避免距离误差过大。
    pthread_mutex_lock(&g_motion_lock);

    //逻辑是先计算出需要执行的时长，再发送命令并等待这个时长，最后发送停止命令。这样可以避免因为底盘响应延迟导致的距离误差过大。
    ret = motion_send_raw(cmd);
    if (ret == 0) {
        motion_wait_ms(duration_ms);
        stop_ret = motion_stop();
        if (stop_ret != 0) {
            ret = stop_ret;
        }
    }

    pthread_mutex_unlock(&g_motion_lock);
    return ret;
}

/**
 * @brief 执行按固定时长运行的底盘动作
 * @param cmd 单字符运动命令
 * @param duration_ms 动作持续时长
 * @return 0表示执行成功，非0表示执行失败
 */
static int motion_run_timed_cmd(char cmd, int duration_ms)
{
    int ret;
    int stop_ret;

    if (duration_ms <= 0) {
        return -1;
    }

    pthread_mutex_lock(&g_motion_lock);

    ret = motion_send_raw(cmd);
    if (ret == 0) {
        motion_wait_ms(duration_ms);
        stop_ret = motion_stop();
        if (stop_ret != 0) {
            ret = stop_ret;
        }
    }

    pthread_mutex_unlock(&g_motion_lock);
    return ret;
}

/**
 * @brief 低速执行带有预设校准值的固定动作
 * @param cmd 运动命令字符，'W'/'S'/'A'/'D'
 * @param calibrated_value 预设的距离或时间校准值
 * @return 0表示执行成功，非0表示执行失败
 */
static int motion_run_low_speed_calibrated_action(char cmd, int calibrated_value)
{
    int ret;

    /* 固定动作统一在最低档下执行，减少速度变化对标定结果的影响。 */
    ret = motion_set_speed_level(MOTION_DEFAULT_SPEED_LEVEL);
    if (ret != 0) {
        return ret;
    }
    switch (cmd) {
    case 'W':
        return motion_forward_distance_mm(calibrated_value);
    case 'S':
        return motion_backward_distance_mm(calibrated_value);
    case 'A':
        return motion_turn_left_distance_mm(calibrated_value);
    case 'D':
        return motion_turn_right_distance_mm(calibrated_value);
    default:
        return -1;
    }
}

/**
 * @brief 读取 TOF 急停锁状态
 * @param 无
 * @return 1表示急停锁开启，0表示未开启或读取失败
 */
int motion_is_tof_estop_locked(void)
{
    int fd;
    char lock_flag = '0';
    ssize_t nread;

    fd = open(MOTION_TOF_ESTOP_LOCK_FILE, O_RDONLY);
    if (fd < 0) {
        return 0;
    }

    nread = read(fd, &lock_flag, 1);
    close(fd);

    return (nread > 0 && lock_flag == '1') ? 1 : 0;
}

/**
 * @brief 直接向底盘控制服务发送原始命令
 * @param cmd 单字符命令，如 'W'/'S'/'A'/'D'/'F'/'R'
 * @return 0表示发送成功，-1表示通信失败，-2表示被急停逻辑拦截
 */
int motion_send_raw(char cmd)
{
    int sockfd;
    struct sockaddr_un serv_addr;

    if (cmd == '\0') {
        return 0;
    }

    if (cmd == 'W' && motion_is_tof_estop_locked()) {
        printf("[motion] TOF estop lock active, reject forward command\n");
        return -2;
    }

    if (motion_is_drive_cmd(cmd)) {
        motion_update_tof_timestamp_file();
    }

    sockfd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (sockfd < 0) {
        printf("[motion] 创建 socket 失败: %s\n", strerror(errno));
        return -1;
    }

    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sun_family = AF_UNIX;
    strncpy(serv_addr.sun_path, MOTION_SOCKET_PATH, sizeof(serv_addr.sun_path) - 1);

    if (connect(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
        printf("[motion] connect failed path=%s err=%s\n",
               MOTION_SOCKET_PATH, strerror(errno));
        close(sockfd);
        return -1;
    }

    if (send(sockfd, &cmd, 1, 0) != 1) {
        printf("[motion] send cmd=%d('%c') failed: %s\n",
               (int)cmd, cmd == ' ' ? '_' : cmd, strerror(errno));
        close(sockfd);
        return -1;
    }

    printf("[motion] send cmd=%d('%c') ok\n", (int)cmd, cmd == ' ' ? '_' : cmd);
    close(sockfd);
    return 0;
}

/**
 * @brief 等待指定毫秒数
 * @param duration_ms 等待时长，单位ms
 * @return 0表示等待完成，-1表示参数非法
 */
int motion_wait_ms(int duration_ms)
{
    int waited_ms = 0;

    if (duration_ms < 0) {
        return -1;
    }

    while (waited_ms < duration_ms) {
        usleep(MOTION_POLL_MS * 1000);
        waited_ms += MOTION_POLL_MS;
    }

    return 0;
}

/**
 * @brief 停止底盘当前运动
 * @param 无
 * @return 0表示执行成功，非0表示执行失败
 */
int motion_stop(void)
{
    return motion_send_raw(' ');
}

/**
 * @brief 持续前进
 * @param 无
 * @return 0表示执行成功，非0表示执行失败
 */
int motion_forward(void)
{
    return motion_send_raw('W');
}

/**
 * @brief 持续后退
 * @param 无
 * @return 0表示执行成功，非0表示执行失败
 */
int motion_backward(void)
{
    return motion_send_raw('S');
}

/**
 * @brief 持续左转
 * @param 无
 * @return 0表示执行成功，非0表示执行失败
 */
int motion_turn_left(void)
{
    return motion_send_raw('A');
}

/**
 * @brief 持续右转
 * @param 无
 * @return 0表示执行成功，非0表示执行失败
 */
int motion_turn_right(void)
{
    return motion_send_raw('D');
}

/**
 * @brief 按设定距离/标定值前进
 * @param distance_mm 目标距离或标定值
 * @return 0表示执行成功，非0表示执行失败
 */
int motion_forward_distance_mm(int distance_mm)
{
    return motion_run_distance_cmd('W', distance_mm, motion_get_linear_speed_mmps());
}

/**
 * @brief 按设定距离/标定值后退
 * @param distance_mm 目标距离或标定值
 * @return 0表示执行成功，非0表示执行失败
 */
int motion_backward_distance_mm(int distance_mm)
{
    return motion_run_distance_cmd('S', distance_mm, motion_get_linear_speed_mmps());
}

/**
 * @brief 按设定距离/标定值左转
 * @param distance_mm 目标距离或标定值
 * @return 0表示执行成功，非0表示执行失败
 */
int motion_turn_left_distance_mm(int distance_mm)
{
    return motion_run_distance_cmd('A', distance_mm, motion_get_turn_speed_mmps());
}

/**
 * @brief 按设定距离/标定值右转
 * @param distance_mm 目标距离或标定值
 * @return 0表示执行成功，非0表示执行失败
 */
int motion_turn_right_distance_mm(int distance_mm)
{
    return motion_run_distance_cmd('D', distance_mm, motion_get_turn_speed_mmps());
}

/**
 * @brief 按当前转圈时长执行一次左转圈动作
 * @param 无
 * @return 0表示执行成功，非0表示执行失败
 */
int motion_circle_left_once(void)
{
    return motion_run_timed_cmd('A', motion_get_circle_duration_ms());
}

/**
 * @brief 按当前转圈时长执行一次右转圈动作
 * @param 无
 * @return 0表示执行成功，非0表示执行失败
 */
int motion_circle_right_once(void)
{
    return motion_run_timed_cmd('D', motion_get_circle_duration_ms());
}

/**
 * @brief 舵机向前抬起
 * @param 无
 * @return 0表示执行成功，非0表示执行失败
 */
int motion_servo_forward(void)
{
    return motion_send_raw('F');
}

/**
 * @brief 舵机向后回摆
 * @param 无
 * @return 0表示执行成功，非0表示执行失败
 */
int motion_servo_backward(void)
{
    return motion_send_raw('B');
}

/**
 * @brief 舵机复位
 * @param 无
 * @return 0表示执行成功，非0表示执行失败
 */
int motion_servo_reset(void)
{
    return motion_send_raw('R');
}

/**
 * @brief 设置底盘速度档位
 * @param speed_level 目标档位，支持 1/2/3
 * @return 0表示执行成功，非0表示执行失败
 */
int motion_set_speed_level(int speed_level)
{
    char cmd;
    int changed = 0;

    switch (speed_level) {
    case 1:
        cmd = 'L';
        break;
    case 2:
        cmd = 'M';
        break;
    case 3:
        cmd = 'H';
        break;
    default:
        return -1;
    }

    pthread_mutex_lock(&g_speed_lock);
    if (g_current_speed_level != speed_level) {
        g_current_speed_level = speed_level;
        changed = 1;
    }
    pthread_mutex_unlock(&g_speed_lock);

    if (changed) {
        motion_update_motor_speed_file(speed_level);
    }

    return motion_send_raw(cmd);
}

/**
 * @brief 获取当前缓存的速度档位
 * @param 无
 * @return 当前速度档位，未设置时返回0
 */
int motion_get_speed_level(void)
{
    int speed_level;

    pthread_mutex_lock(&g_speed_lock);
    speed_level = g_current_speed_level;
    pthread_mutex_unlock(&g_speed_lock);

    return speed_level;
}

/**
 * @brief 设置线速度参数
 * @param speed_mmps 线速度，单位mm/s
 * @return 无
 */
void motion_set_linear_speed_mmps(int speed_mmps)
{
    if (speed_mmps > 0) {
        g_linear_speed_mmps = speed_mmps;
    }
}

/**
 * @brief 设置转向速度参数
 * @param speed_mmps 转向速度，单位mm/s
 * @return 无
 */
void motion_set_turn_speed_mmps(int speed_mmps)
{
    if (speed_mmps > 0) {
        g_turn_speed_mmps = speed_mmps;
    }
}

/**
 * @brief 设置转圈动作持续时长
 * @param duration_ms 转圈时长，单位ms
 * @return 无
 */
void motion_set_circle_duration_ms(int duration_ms)
{
    if (duration_ms > 0) {
        g_circle_duration_ms = duration_ms;
    }
}

/**
 * @brief 获取当前线速度参数
 * @param 无
 * @return 当前线速度，单位mm/s
 */
int motion_get_linear_speed_mmps(void)
{
    return g_linear_speed_mmps;
}

/**
 * @brief 获取当前转向速度参数
 * @param 无
 * @return 当前转向速度，单位mm/s
 */
int motion_get_turn_speed_mmps(void)
{
    return g_turn_speed_mmps;
}

/**
 * @brief 获取当前转圈时长参数
 * @param 无
 * @return 当前转圈时长，单位ms
 */
int motion_get_circle_duration_ms(void)
{
    return g_circle_duration_ms;
}

/**
 * @brief 将协议命令翻译为底层单字符动作码
 * @param command 协议主命令
 * @param action 协议动作参数
 * @param buf 输出缓冲区
 * @param buf_size 输出缓冲区大小
 * @return 1表示翻译成功，0表示翻译失败
 */
int motion_translate_protocol_cmd(int command, uint8_t action, char *buf, size_t buf_size)
{
    char raw_cmd = '\0';

    if (buf == NULL || buf_size < 2) {
        return 0;
    }

    switch (command) {
    case MOTION_PROTOCOL_MOTOR_COMMAND:
        switch (action) {
        case MOTION_MOTOR_ACTION_FORWARD:
            raw_cmd = 'W';
            break;
        case MOTION_MOTOR_ACTION_BACKWARD:
            raw_cmd = 'S';
            break;
        case MOTION_MOTOR_ACTION_LEFT:
            raw_cmd = 'A';
            break;
        case MOTION_MOTOR_ACTION_RIGHT:
            raw_cmd = 'D';
            break;
        case MOTION_MOTOR_ACTION_STOP:
            raw_cmd = ' ';
            break;
        case MOTION_MOTOR_ACTION_LOW:
            raw_cmd = 'L';
            break;
        case MOTION_MOTOR_ACTION_MEDIUM:
            raw_cmd = 'M';
            break;
        case MOTION_MOTOR_ACTION_HIGH:
            raw_cmd = 'H';
            break;
        default:
            return 0;
        }
        break;
    case MOTION_PROTOCOL_SERVO_COMMAND:
        switch (action) {
        case MOTION_SERVO_ACTION_FORWARD:
            raw_cmd = 'F';
            break;
        case MOTION_SERVO_ACTION_BACKWARD:
            raw_cmd = 'B';
            break;
        case MOTION_SERVO_ACTION_RESET:
            raw_cmd = 'R';
            break;
        default:
            return 0;
        }
        break;
    case MOTION_PROTOCOL_ACTION_COMMAND:
        switch (action) {
        case MOTION_ACTION_CIRCLE_L:
            raw_cmd = 'Q';
            break;
        case MOTION_ACTION_EIGHT:
            raw_cmd = 'E';
            break;
        case MOTION_ACTION_Z:
            raw_cmd = 'Z';
            break;
        case MOTION_ACTION_GREET:
            raw_cmd = 'G';
            break;
        case MOTION_ACTION_CIRCLE_R:
            raw_cmd = 'P';
            break;
        case MOTION_ACTION_STOP:
            raw_cmd = ' ';
            break;
        default:
            return 0;
        }
        break;
    default:
        return 0;
    }

    buf[0] = raw_cmd;
    buf[1] = '\0';
    return 1;
}

/**
 * @brief 执行启动时的预设动作序列
 * @param 无
 * @return 0表示执行成功，非0表示执行失败
 */
int motion_run_startup_sequence(void)
{
    int ret;

    pthread_mutex_lock(&g_motion_lock);

    ret = motion_turn_left();
    if (ret != 0) {
        pthread_mutex_unlock(&g_motion_lock);
        printf("[motion] 启动序列: 左转失败 ret=%d\n", ret);
        return ret;
    }

    motion_wait_ms(110);
    ret = motion_turn_right();
    if (ret != 0) {
        pthread_mutex_unlock(&g_motion_lock);
        printf("[motion] 启动序列: 右转失败 ret=%d\n", ret);
        return ret;
    }

    motion_wait_ms(110);
    ret = motion_stop();
    if (ret != 0) {
        pthread_mutex_unlock(&g_motion_lock);
        printf("[motion] 启动序列: 停止失败 ret=%d\n", ret);
        return ret;
    }

    ret = motion_servo_forward();
    if (ret != 0) {
        pthread_mutex_unlock(&g_motion_lock);
        printf("[motion] 启动序列: 舵机前进失败 ret=%d\n", ret);
        return ret;
    }

    motion_wait_ms(220);
    ret = motion_servo_forward();
    if (ret != 0) {
        pthread_mutex_unlock(&g_motion_lock);
        printf("[motion] 启动序列: 舵机前进2失败 ret=%d\n", ret);
        return ret;
    }

    motion_wait_ms(220);
    ret = motion_servo_forward();
    if (ret != 0) {
        pthread_mutex_unlock(&g_motion_lock);
        printf("[motion] 启动序列: 伺服前进3失败 ret=%d\n", ret);
        return ret;
    }
    motion_wait_ms(220);
    // 最后一次前进动作结束后不需要再发停止命令，因为伺服动作是单次执行的，不会持续移动。
    pthread_mutex_unlock(&g_motion_lock);
    return ret;
}

   
   

/**
 * @brief 左转约45度固定动作
 * @param 无
 * @return 0表示执行成功，非0表示执行失败
 */
int motion_action_left_45deg(void)
{
    return motion_run_low_speed_calibrated_action('A', MOTION_FIXED_LEFT_45_VALUE);
}

/**
 * @brief 右转约45度固定动作
 * @param 无
 * @return 0表示执行成功，非0表示执行失败
 */
int motion_action_right_45deg(void)
{
    return motion_run_low_speed_calibrated_action('D', MOTION_FIXED_RIGHT_45_VALUE);
}

/**
 * @brief 左转1度固定动作
 * @param 无
 * @return 0表示执行成功，非0表示执行失败
 */
int motion_action_left_1deg(void)
{
    return motion_run_low_speed_calibrated_action('A', MOTION_FIXED_LEFT_1_VALUE);
}

/**
 * @brief 右转1度固定动作
 * @param 无
 * @return 0表示执行成功，非0表示执行失败
 */
int motion_action_right_1deg(void)
{
    return motion_run_low_speed_calibrated_action('D', MOTION_FIXED_RIGHT_1_VALUE);
}

/**
 * @brief 前进约20厘米固定动作
 * @param 无
 * @return 0表示执行成功，非0表示执行失败
 */
int motion_action_forward_20cm(void)
{
    return motion_run_low_speed_calibrated_action('W', MOTION_FIXED_FORWARD_20CM_VALUE);
}

/**
 * @brief 后退约20厘米固定动作
 * @param 无
 * @return 0表示执行成功，非0表示执行失败
 */
int motion_action_backward_20cm(void)
{
    return motion_run_low_speed_calibrated_action('S', MOTION_FIXED_BACKWARD_20CM_VALUE);
}

/**
 * @brief 前进约5厘米固定动作
 * @param 无
 * @return 0表示执行成功，非0表示执行失败
 */
int motion_action_forward_5cm(void){
    return motion_run_low_speed_calibrated_action('W', MOTION_FIXED_FORWARD_5CM_VALUE);
};

/**
 * @brief 后退约5厘米固定动作
 * @param 无
 * @return 0表示执行成功，非0表示执行失败
 */int motion_action_backward_5cm(void){
    return motion_run_low_speed_calibrated_action('S', MOTION_FIXED_BACKWARD_5CM_VALUE);
};

/**
 * @brief 原地左转约一圈固定动作
 * @param 无
 * @return 0表示执行成功，非0表示执行失败
 */
int motion_action_left_circle(void)
{
    return motion_run_low_speed_calibrated_action('A', MOTION_FIXED_LEFT_CIRCLE_VALUE);
}

/**
 * @brief 原地右转约一圈固定动作
 * @param 无
 * @return 0表示执行成功，非0表示执行失败
 */
int motion_action_right_circle(void)
{
    return motion_run_low_speed_calibrated_action('D', MOTION_FIXED_RIGHT_CIRCLE_VALUE);
}

/**
 * @brief 无信号时向右微调搜索固定动作
 * @param 无
 * @return 0表示执行成功，非0表示执行失败
 */
int motion_action_right_search_step(void)
{
    return motion_run_low_speed_calibrated_action('D', MOTION_FIXED_RIGHT_SEARCH_VALUE);
}

/** 
    * @brief 左转约120度固定动作
    * @param 无
    * @return 0表示执行成功，非0表示执行失败
*/
int motion_action_left_90deg(void){
    return motion_run_low_speed_calibrated_action('A', MOTION_FIXED_LEFT_90_VALUE);
}

/** 
    * @brief 右转约90度固定动作
    * @param 无
    * @return 0表示执行成功，非0表示执行失败
*/
int motion_action_right_90deg(void){
    return motion_run_low_speed_calibrated_action('D', MOTION_FIXED_RIGHT_90_VALUE);
}
