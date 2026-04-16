#ifndef MOTION_H
#define MOTION_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
//内部电机控制的socket路径
#define MOTION_SOCKET_PATH "/tmp/motor_ctl_socket"
//tof测距锁定文件路径
#define MOTION_TOF_ESTOP_LOCK_FILE "/tmp/tof_estop.lock"
//电机速度文件路径
#define MOTION_MOTOR_SPEED_FILE "/tmp/motor_speed"
//tof测距时间戳文件路径
#define MOTION_TOF_MOTION_TS_FILE "/tmp/tof_motion_ts"
//默认线速度，单位毫米每秒
#define MOTION_DEFAULT_LINEAR_SPEED_MMPS 250
//默认转向速度，单位毫米每秒
#define MOTION_DEFAULT_TURN_SPEED_MMPS 180
//默认速度等级
#define MOTION_DEFAULT_SPEED_LEVEL 1
//默认圆形动作持续时间，单位毫秒
#define MOTION_DEFAULT_CIRCLE_DURATION_MS 500

/*
 * 以下参数为当前底盘动作标定值，直接对应原测试命令里的第二个参数。
 * 例如：left 50、right 65、forward 50。
 * 它们不是严格物理意义上的毫米，只是当前车体上实测可用的经验值。
 */
#define MOTION_FIXED_LEFT_45_VALUE        50
#define MOTION_FIXED_RIGHT_45_VALUE       130
#define MOTION_FIXED_FORWARD_20CM_VALUE   1000
#define MOTION_FIND_SINGLE_FORWARD_VALUE  300
#define MOTION_FIXED_BACKWARD_20CM_VALUE  50
#define MOTION_FIXED_LEFT_CIRCLE_VALUE    220
#define MOTION_FIXED_RIGHT_CIRCLE_VALUE   258
#define MOTION_FIXED_RIGHT_SEARCH_VALUE   30
#define MOTION_FIXED_LEFT_1_VALUE         1
#define MOTION_FIXED_RIGHT_1_VALUE        1
#define MOTION_FIXED_FORWARD_5CM_VALUE    10
#define MOTION_FIXED_BACKWARD_5CM_VALUE   100
#define MOTION_FIXED_LEFT_90_VALUE      120
#define MOTION_FIXED_RIGHT_90_VALUE     650

//电机命令定义
enum {
    MOTION_PROTOCOL_MOTOR_COMMAND = 0x10,
    MOTION_PROTOCOL_SERVO_COMMAND = 0x11,
    MOTION_PROTOCOL_ACTION_COMMAND = 0x12,
};

//动作定义
enum {
    MOTION_MOTOR_ACTION_STOP = 0x30,
    MOTION_MOTOR_ACTION_FORWARD = 0x31,
    MOTION_MOTOR_ACTION_BACKWARD = 0x32,
    MOTION_MOTOR_ACTION_LEFT = 0x33,
    MOTION_MOTOR_ACTION_RIGHT = 0x34,
    MOTION_MOTOR_ACTION_LOW = 0x35,
    MOTION_MOTOR_ACTION_MEDIUM = 0x36,
    MOTION_MOTOR_ACTION_HIGH = 0x37,
};


//舵机定义
enum {
    MOTION_SERVO_ACTION_FORWARD = 0x31,
    MOTION_SERVO_ACTION_BACKWARD = 0x32,
    MOTION_SERVO_ACTION_RESET = 0x33,
};

//电机动作控制指令
enum {
    MOTION_ACTION_STOP = 0x30,
    MOTION_ACTION_CIRCLE_L = 0x31,
    MOTION_ACTION_EIGHT = 0x32,
    MOTION_ACTION_Z = 0x33,
    MOTION_ACTION_GREET = 0x34,
    MOTION_ACTION_CIRCLE_R = 0x35,
};


int motion_send_raw(char cmd);
int motion_wait_ms(int duration_ms);
int motion_stop(void);

int motion_forward(void);
int motion_backward(void);
int motion_turn_left(void);
int motion_turn_right(void);

int motion_forward_distance_mm(int distance_mm);
int motion_backward_distance_mm(int distance_mm);
int motion_turn_left_distance_mm(int distance_mm);
int motion_turn_right_distance_mm(int distance_mm);
int motion_circle_left_once(void);
int motion_circle_right_once(void);

int motion_servo_forward(void);
int motion_servo_backward(void);
int motion_servo_reset(void);

int motion_set_speed_level(int speed_level);
int motion_get_speed_level(void);

void motion_set_linear_speed_mmps(int speed_mmps);
void motion_set_turn_speed_mmps(int speed_mmps);
void motion_set_circle_duration_ms(int duration_ms);
int motion_get_linear_speed_mmps(void);
int motion_get_turn_speed_mmps(void);
int motion_get_circle_duration_ms(void);

int motion_is_tof_estop_locked(void);
int motion_translate_protocol_cmd(int command, uint8_t action, char *buf, size_t buf_size);
int motion_run_startup_sequence(void);

int motion_action_left_45deg(void);
int motion_action_right_45deg(void);
int motion_action_forward_20cm(void);
int motion_action_backward_20cm(void);
int motion_action_left_circle(void);
int motion_action_right_circle(void);
int motion_action_right_search_step(void);
int motion_action_left_1deg(void);
int motion_action_right_1deg(void);
int motion_action_forward_5cm(void);
int motion_action_backward_5cm(void);
int motion_action_left_90deg(void);
int motion_action_right_90deg(void);

#ifdef __cplusplus
}
#endif

#endif
