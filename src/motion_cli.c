#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../include/motion.h"

typedef int (*motion_handler_t)(void);

typedef struct {
    const char *name;
    motion_handler_t handler;
} motion_action_entry_t;

static void motion_cli_print_usage(const char *prog)
{
    fprintf(stderr,
            "Usage:\n"
            "  %s raw <char>\n"
            "  %s speed <1|2|3>\n"
            "  %s left <distance>\n"
            "  %s right <distance>\n"
            "  %s forward <distance>\n"
            "  %s backward <distance>\n"
            "  %s action <left45|right45|forward20|backward20|left1|right1|forward5|backward5|circle_left|circle_right|search_right|left90|right90>\n"
            "  %s servo <forward|backward|reset>\n",
            prog, prog, prog, prog, prog, prog, prog, prog);
}

static int motion_cli_parse_int(const char *value, int *out)
{
    char *end = NULL;
    long parsed;

    if (value == NULL || out == NULL || value[0] == '\0') {
        return -1;
    }

    parsed = strtol(value, &end, 10);
    if (end == NULL || *end != '\0') {
        return -1;
    }

    *out = (int)parsed;
    return 0;
}

static int motion_cli_run_action(const char *name)
{
    static const motion_action_entry_t actions[] = {
        {"left45", motion_action_left_45deg},
        {"right45", motion_action_right_45deg},
        {"forward20", motion_action_forward_20cm},
        {"backward20", motion_action_backward_20cm},
        {"left1", motion_action_left_1deg},
        {"right1", motion_action_right_1deg},
        {"forward5", motion_action_forward_5cm},
        {"backward5", motion_action_backward_5cm},
        {"circle_left", motion_action_left_circle},
        {"circle_right", motion_action_right_circle},
        {"search_right", motion_action_right_search_step},
        {"left90", motion_action_left_90deg},
        {"right90", motion_action_right_90deg},
    };
    size_t i;

    if (name == NULL) {
        return -1;
    }

    for (i = 0; i < sizeof(actions) / sizeof(actions[0]); i++) {
        if (strcmp(name, actions[i].name) == 0) {
            return actions[i].handler();
        }
    }

    return -1;
}

static int motion_cli_run_servo(const char *name)
{
    if (name == NULL) {
        return -1;
    }

    if (strcmp(name, "forward") == 0) {
        return motion_servo_forward();
    }
    if (strcmp(name, "backward") == 0) {
        return motion_servo_backward();
    }
    if (strcmp(name, "reset") == 0) {
        return motion_servo_reset();
    }

    return -1;
}

static int motion_cli_run_distance_cmd(const char *name, int distance)
{
    if (name == NULL || distance <= 0) {
        return -1;
    }

    if (strcmp(name, "left") == 0) {
        return motion_turn_left_distance_mm(distance);
    }
    if (strcmp(name, "right") == 0) {
        return motion_turn_right_distance_mm(distance);
    }
    if (strcmp(name, "forward") == 0) {
        return motion_forward_distance_mm(distance);
    }
    if (strcmp(name, "backward") == 0) {
        return motion_backward_distance_mm(distance);
    }

    return -1;
}

int main(int argc, char *argv[])
{
    int ret = -1;

    if (argc != 3) {
        motion_cli_print_usage(argv[0]);
        return 1;
    }

    if (strcmp(argv[1], "raw") == 0) {
        if (argv[2][0] == '\0' || argv[2][1] != '\0') {
            fprintf(stderr, "raw expects a single character command\n");
            motion_cli_print_usage(argv[0]);
            return 1;
        }
        ret = motion_send_raw(argv[2][0]);
    } else if (strcmp(argv[1], "speed") == 0) {
        int speed_level;

        if (motion_cli_parse_int(argv[2], &speed_level) != 0) {
            fprintf(stderr, "invalid speed level: %s\n", argv[2]);
            motion_cli_print_usage(argv[0]);
            return 1;
        }
        ret = motion_set_speed_level(speed_level);
    } else if (strcmp(argv[1], "left") == 0 ||
               strcmp(argv[1], "right") == 0 ||
               strcmp(argv[1], "forward") == 0 ||
               strcmp(argv[1], "backward") == 0) {
        int distance;

        if (motion_cli_parse_int(argv[2], &distance) != 0 || distance <= 0) {
            fprintf(stderr, "invalid distance for %s: %s\n", argv[1], argv[2]);
            motion_cli_print_usage(argv[0]);
            return 1;
        }
        ret = motion_cli_run_distance_cmd(argv[1], distance);
    } else if (strcmp(argv[1], "action") == 0) {
        ret = motion_cli_run_action(argv[2]);
    } else if (strcmp(argv[1], "servo") == 0) {
        ret = motion_cli_run_servo(argv[2]);
    } else {
        fprintf(stderr, "unknown subcommand: %s\n", argv[1]);
        motion_cli_print_usage(argv[0]);
        return 1;
    }

    if (ret != 0) {
        fprintf(stderr, "command failed: %s %s (ret=%d)\n", argv[1], argv[2], ret);
        return 1;
    }

    return 0;
}
