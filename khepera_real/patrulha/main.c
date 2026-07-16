/*
 * main.c -- Khepera IV, navegacao tatil ate um alvo sem mapa previo.
 *
 * O IR real e tratado como sensor de toque. A odometria mede todas as
 * manobras: recuo, giro limitado, deslocamento tangencial e teste do caminho.
 * Nenhum estado pode mandar o robo girar varias voltas no proprio eixo.
 */
#include "khepera4.h"
#include "commandline.h"

#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <sys/time.h>
#include <unistd.h>

#define PULSES_PER_MM 147.4
#define WHEELBASE_MM  105.4
/* +1: diferencial negativo gira fisicamente para a esquerda neste robo. */
#define DRIVE_DIFF_SIGN 1.0

#define GOAL_X_DEF    700.0
#define GOAL_Y_DEF      0.0

/* Reduzido para os primeiros testes da nova logica. */
#define SPEED          4500
#define SLOW_SPEED     3200
#define WAIT_US       40000
#define BATT_LOW_MV    6300

#define IRP(index) (khepera4.infrared_proximity.sensor[(index)])

#include "controller_config.h"
#include "controller_core.h"

static volatile sig_atomic_t running = 1;

static void on_stop_signal(int signal_number) {
    (void)signal_number;
    running = 0;
}

static double now_s(void) {
    struct timeval value;
    gettimeofday(&value, NULL);
    return value.tv_sec + value.tv_usec / 1000000.0;
}

static double wrap_angle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

static double clamp_value(double value, double lo, double hi) {
    return value < lo ? lo : (value > hi ? hi : value);
}

static const char *state_name(KhepState state) {
    switch (state) {
        case KHEP_GOAL:          return "GOAL";
        case KHEP_BACKOFF:       return "RECUO";
        case KHEP_TANGENT:       return "GIRO";
        case KHEP_BYPASS:        return "CONTORNO";
        case KHEP_ALIGN_GOAL:    return "ALINHA";
        case KHEP_PROBE:         return "TESTE";
        case KHEP_CLIFF_BACKOFF: return "BEIRADA-RECUO";
        case KHEP_CLIFF_TURN:    return "BEIRADA-GIRO";
        case KHEP_BLOCKED:       return "BLOQUEADO";
        default:                 return "DESCONHECIDO";
    }
}

static void set_leds(int red, int green, int blue) {
    khepera_dspic_write_register8(I2C_RGB_FL_R, red);
    khepera_dspic_write_register8(I2C_RGB_FL_G, green);
    khepera_dspic_write_register8(I2C_RGB_FL_B, blue);
    khepera_dspic_write_register8(I2C_RGB_FR_R, red);
    khepera_dspic_write_register8(I2C_RGB_FR_G, green);
    khepera_dspic_write_register8(I2C_RGB_FR_B, blue);
    khepera_dspic_write_register8(I2C_RGB_B_R, red);
    khepera_dspic_write_register8(I2C_RGB_B_G, green);
    khepera_dspic_write_register8(I2C_RGB_B_B, blue);
}

static int ground_minimum(void) {
    int minimum = IRP(8);
    if (IRP(9) < minimum) minimum = IRP(9);
    if (IRP(10) < minimum) minimum = IRP(10);
    if (IRP(11) < minimum) minimum = IRP(11);
    return minimum;
}

static void stop_motors(void) {
    khepera4_drive_set_speed_differential(SPEED, 0.0, 0.0);
    khepera4_drive_stop();
    khepera4_drive_idle();
    set_leds(0, 0, 0);
}

static void run_diagnostics(void) {
    printf("DIAG: leitura bruta, sem movimento. Ctrl-C para sair.\n");
    while (running) {
        khepera4_infrared_proximity();
        khepera4_drive_get_current_position();
        khepera4_battery();
        printf("IR[BL L FL F FR R BR B]=%d %d %d %d %d %d %d %d | "
               "chao[%d %d %d %d] | enc L/R=%d/%d | bat=%umV\n",
               IRP(0), IRP(1), IRP(2), IRP(3), IRP(4), IRP(5), IRP(6), IRP(7),
               IRP(8), IRP(9), IRP(10), IRP(11),
               khepera4.motor_left.current_position,
               khepera4.motor_right.current_position,
               khepera4.battery.voltage);
        fflush(stdout);
        usleep(400000);
    }
}

static int read_line_if_available(char *line, size_t size) {
    fd_set read_fds;
    struct timeval timeout;
    FD_ZERO(&read_fds);
    FD_SET(STDIN_FILENO, &read_fds);
    timeout.tv_sec = 0;
    timeout.tv_usec = 0;
    if (select(STDIN_FILENO + 1, &read_fds, NULL, NULL, &timeout) <= 0)
        return 0;
    return fgets(line, size, stdin) != NULL;
}

static int read_manual_command(double *forward, double *diff, int *quit) {
    char line[96], operation[16];
    double a, b;
    if (!read_line_if_available(line, sizeof(line))) return 0;
    if (sscanf(line, " %15s", operation) != 1) return 0;

    if (strcmp(operation, "v") == 0 &&
        sscanf(line, " %*s %lf %lf", &a, &b) == 2) {
        *forward = clamp_value(a, -1.0, 1.0);
        *diff = clamp_value(b, -DIFF_MAX, DIFF_MAX);
        return 1;
    }
    if (strcmp(operation, "s") == 0 || strcmp(operation, "stop") == 0) {
        *forward = 0.0;
        *diff = 0.0;
        return 1;
    }
    if (strcmp(operation, "q") == 0 || strcmp(operation, "quit") == 0) {
        *forward = 0.0;
        *diff = 0.0;
        *quit = 1;
        return 1;
    }
    return 0;
}

static void run_manual(void) {
    double target_forward = 0.0, target_diff = 0.0;
    double last_command = now_s();
    int encoder_left, encoder_right;
    double x = 0.0, y = 0.0, th = 0.0;
    long tick = 0;

    khepera4_drive_start();
    khepera4_drive_reset_position();
    usleep(120000);
    khepera4_drive_get_current_position();
    encoder_left = khepera4.motor_left.current_position;
    encoder_right = khepera4.motor_right.current_position;
    set_leds(35, 25, 0);
    printf("TELEOP: v <frente> <giro>, s para, q sai. Timeout de 0.8s.\n");

    while (running) {
        int quit = 0;
        int next_left, next_right, floor;
        double dl, dr, dc, dth;
        double command_forward = target_forward;
        const char *guard = "OK";

        if (read_manual_command(&target_forward, &target_diff, &quit))
            last_command = now_s();
        if (quit) break;

        khepera4_infrared_proximity();
        khepera4_drive_get_current_position();
        if ((tick % 25) == 0) khepera4_battery();

        next_left = khepera4.motor_left.current_position;
        next_right = khepera4.motor_right.current_position;
        dl = (next_left - encoder_left) / PULSES_PER_MM;
        dr = (next_right - encoder_right) / PULSES_PER_MM;
        encoder_left = next_left;
        encoder_right = next_right;
        dc = 0.5 * (dl + dr);
        dth = DRIVE_DIFF_SIGN * (dr - dl) / WHEELBASE_MM;
        x += dc * cos(th + 0.5 * dth);
        y += dc * sin(th + 0.5 * dth);
        th = wrap_angle(th + dth);

        floor = ground_minimum();
        if (now_s() - last_command > 0.8) {
            command_forward = 0.0;
            target_diff = 0.0;
            guard = "STALE";
        }
        if (floor < CLIFF_THR && command_forward > 0.0) {
            command_forward = 0.0;
            guard = "BEIRADA";
        }

        khepera4_drive_set_speed_differential(
            SLOW_SPEED, command_forward, DRIVE_DIFF_SIGN * target_diff);
        if ((++tick % 8) == 0) {
            printf("[MANUAL] pose=(%.0f,%.0f,%.0fd) fwd=%+.2f diff=%+.2f "
                   "guard=%s | F/FL/FR=%d/%d/%d | chao=%d bat=%umV\n",
                   x, y, th * 180.0 / M_PI, command_forward, target_diff,
                   guard, IRP(3), IRP(2), IRP(4), floor,
                   khepera4.battery.voltage);
            fflush(stdout);
        }
        usleep(WAIT_US);
    }

    stop_motors();
    printf("TELEOP encerrado. Motores parados.\n");
}

static int read_mission_command(double *goal_x, double *goal_y) {
    char line[96], operation[16];
    if (!read_line_if_available(line, sizeof(line))) return 0;
    if (sscanf(line, " %15s", operation) != 1) return 0;
    if (strcmp(operation, "goto") == 0 &&
        sscanf(line, " %*s %lf %lf", goal_x, goal_y) == 2) return 1;
    if (strcmp(operation, "stop") == 0) return 2;
    if (strcmp(operation, "quit") == 0 || strcmp(operation, "q") == 0)
        return 3;
    return 0;
}

int main(int argc, char *argv[]) {
    double goal_x, goal_y, trim;
    double x = 0.0, y = 0.0, th = 0.0;
    double previous_time;
    double origin_x = 0.0, origin_y = 0.0;
    int encoder_left, encoder_right;
    int loop_mode, mission_mode, slow_mode, have_goal;
    int motor_speed, led_mode = -1, battery_tick = 0;
    long tick = 0;
    KhepController controller;

    commandline_init();
    commandline_parse(argc, argv);
    signal(SIGINT, on_stop_signal);
    signal(SIGTERM, on_stop_signal);
    signal(SIGHUP, on_stop_signal);
    khepera4_init();

    if (commandline_option_provided("-d", "--diag")) {
        run_diagnostics();
        return 0;
    }
    if (commandline_option_provided("-m", "--teleop")) {
        run_manual();
        return 0;
    }

    trim = commandline_option_value_float("-t", "--trim", 0.0);
    goal_x = commandline_option_value_float("-x", "--gx", GOAL_X_DEF);
    goal_y = commandline_option_value_float("-y", "--gy", GOAL_Y_DEF);
    loop_mode = commandline_option_provided("-l", "--loop");
    mission_mode = commandline_option_provided("-M", "--mission");
    slow_mode = commandline_option_provided("-w", "--slow");
    motor_speed = slow_mode ? SLOW_SPEED : SPEED;
    have_goal = !mission_mode;

    khepera4_drive_start();
    khepera4_drive_reset_position();
    usleep(120000);
    khepera4_drive_get_current_position();
    encoder_left = khepera4.motor_left.current_position;
    encoder_right = khepera4.motor_right.current_position;
    khep_ctrl_init(&controller, goal_x, goal_y);
    previous_time = now_s();
    set_leds(0, 0, 30);

    if (mission_mode)
        printf("MISSAO TATIL: aguardando 'goto X Y'.\n");
    else
        printf("NAVEGADOR TATIL: B=(%.0f,%.0f)mm speed=%d loop=%d trim=%.3f.\n",
               goal_x, goal_y, motor_speed, loop_mode, trim);
    printf("IR livre<%d, contato>=%d, contato forte>=%d. Ctrl-C para parar.\n",
           IR_CLEAR_RAW, IR_CONTACT_RAW, IR_HARD_CONTACT_RAW);
    fflush(stdout);

    while (running) {
        double current_time = now_s();
        double dt = current_time - previous_time;
        int next_left, next_right, floor;
        double dl, dr, dc, dth;
        KhepOutput output;

        previous_time = current_time;
        if (dt <= 0.0 || dt > 0.5) dt = WAIT_US / 1000000.0;

        khepera4_infrared_proximity();
        khepera4_drive_get_current_position();
        if ((battery_tick++ % 50) == 0) khepera4_battery();

        next_left = khepera4.motor_left.current_position;
        next_right = khepera4.motor_right.current_position;
        dl = (next_left - encoder_left) / PULSES_PER_MM;
        dr = (next_right - encoder_right) / PULSES_PER_MM;
        encoder_left = next_left;
        encoder_right = next_right;
        dc = 0.5 * (dl + dr);
        dth = DRIVE_DIFF_SIGN * (dr - dl) / WHEELBASE_MM;
        x += dc * cos(th + 0.5 * dth);
        y += dc * sin(th + 0.5 * dth);
        th = wrap_angle(th + dth);

        if (mission_mode) {
            double next_goal_x, next_goal_y;
            int mission_command = read_mission_command(&next_goal_x, &next_goal_y);
            if (mission_command == 1) {
                khep_ctrl_set_segment(&controller, x, y, next_goal_x, next_goal_y);
                have_goal = 1;
                printf(">> GOTO (%.0f,%.0f)\n", next_goal_x, next_goal_y);
            } else if (mission_command == 2) {
                have_goal = 0;
                printf(">> STOP\n");
            } else if (mission_command == 3) {
                break;
            }
            if (!have_goal) {
                khepera4_drive_set_speed_differential(motor_speed, 0.0, 0.0);
                usleep(WAIT_US);
                continue;
            }
        }

        floor = ground_minimum();
        output = khep_ctrl_step(
            &controller, x, y, th, dc, dt, current_time,
            IRP(3), IRP(2), IRP(4), IRP(1), IRP(5), floor);

        if (output.contact_event) {
            printf(">> CONTATO #%d mapa=(%.0f,%.0f) IR=%d/%d/%d L/R=%d/%d "
                   "contorno=%s%s\n",
                   output.contact_index, output.contact_x, output.contact_y,
                   output.F, output.FL, output.FR, output.L, output.R,
                   output.bypass_side > 0 ? "ESQUERDA" : "DIREITA",
                   output.loop_detected ? " LOOP" : "");
        }
        if (output.side_switched) {
            printf("!! lado anterior sem progresso; tentando pela %s\n",
                   output.bypass_side > 0 ? "ESQUERDA" : "DIREITA");
        }
        if (output.obstacle_cleared) {
            printf("<< caminho confirmado livre; retomando GOAL\n");
        }
        if (output.blocked) {
            printf("!! BLOQUEADO: controlador solicitou parada segura\n");
        }
        if (output.state_changed) {
            printf("-- estado: %s\n", state_name(output.state));
        }

        if (output.d_goal < ARRIVE_MM) {
            khepera4_drive_set_speed_differential(motor_speed, 0.0, 0.0);
            printf("== CHEGOU em (%.0f,%.0f), pose=(%.0f,%.0f,%.0fdeg) ==\n",
                   controller.bx, controller.by, x, y, th * 180.0 / M_PI);
            fflush(stdout);
            if (mission_mode) {
                have_goal = 0;
                khep_ctrl_set_segment(&controller, x, y, x, y);
                set_leds(0, 40, 0);
                usleep(200000);
                continue;
            }
            if (!loop_mode) break;
            if (hypot(controller.bx - origin_x, controller.by - origin_y) < ARRIVE_MM)
                khep_ctrl_set_segment(&controller, x, y, goal_x, goal_y);
            else
                khep_ctrl_set_segment(&controller, x, y, origin_x, origin_y);
            set_leds(0, 40, 0);
            usleep(400000);
            continue;
        }

        if (output.blocked) {
            khepera4_drive_set_speed_differential(motor_speed, 0.0, 0.0);
            break;
        }

        if (output.beirada) {
            if (led_mode != 3) { set_leds(40, 0, 0); led_mode = 3; }
        } else if (output.state == KHEP_GOAL || output.state == KHEP_PROBE) {
            if (led_mode != 0) { set_leds(0, 30, 0); led_mode = 0; }
        } else {
            if (led_mode != 2) { set_leds(0, 0, 40); led_mode = 2; }
        }

        khepera4_drive_set_speed_differential(
            motor_speed, output.fwd_s,
            DRIVE_DIFF_SIGN * (output.diff_s + trim));

        if ((++tick % 8) == 0) {
            printf("[%s-%s] pose=(%.0f,%.0f,%.0fd) d_goal=%.0f d_line=%+.0f "
                   "lat=%+.0f alvo=%+.0fd herr=%+.0fd cmd=(%+.2f,%+.2f) | "
                   "F/FL/FR=%d/%d/%d L/R=%d/%d | "
                   "chao=%d bat=%umV\n",
                   state_name(output.state),
                   output.bypass_side > 0 ? "E" : "D",
                   x, y, th * 180.0 / M_PI, output.d_goal, output.d_line,
                   output.lateral_progress, output.heading_target * 180.0 / M_PI,
                   output.herr * 180.0 / M_PI, output.fwd_s, output.diff_s,
                   output.F, output.FL, output.FR, output.L, output.R, floor,
                   khepera4.battery.voltage);
            if (khepera4.battery.voltage > 0 &&
                khepera4.battery.voltage < BATT_LOW_MV)
                printf("!! BATERIA BAIXA: %umV\n", khepera4.battery.voltage);
            fflush(stdout);
        }
        usleep(WAIT_US);
    }

    stop_motors();
    printf("Motores parados. Encerrado com seguranca.\n");
    return 0;
}
