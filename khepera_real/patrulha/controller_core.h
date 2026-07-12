/*
 * controller_core.h -- nucleo compartilhado do controlador Bug2 do Khepera.
 *
 * Este arquivo nao depende do hardware. Ele recebe pose, sensores e tempo,
 * devolve comando diferencial normalizado e telemetria logica. O main.c do
 * robo real e o visualizador/simulador OpenGL incluem este mesmo header, para
 * testar localmente exatamente a logica que sera compilada junto do robo.
 */
#ifndef KHEPERA_CONTROLLER_CORE_H
#define KHEPERA_CONTROLLER_CORE_H

#include <math.h>
#include <string.h>

#include "controller_config.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef ARRIVE_MM
#define ARRIVE_MM 90.0
#endif
#ifndef KP_HEAD
#define KP_HEAD 1.2
#endif
#ifndef HEAD_SLOW
#define HEAD_SLOW 1.2
#endif
#ifndef LOOKAHEAD_MM
#define LOOKAHEAD_MM 120.0
#endif
#ifndef USE_POINT_P
#define USE_POINT_P 1
#endif
#ifndef K_CT
#define K_CT 1.0
#endif
#ifndef KI_CT
#define KI_CT 0.0015
#endif
#ifndef CT_INT_MAX
#define CT_INT_MAX 250.0
#endif
#ifndef IR_BASE
#define IR_BASE 120
#endif
#ifndef FRONT_SPAN
#define FRONT_SPAN 450.0
#endif
#ifndef FRONT_BLOCK
#define FRONT_BLOCK 200
#endif
#ifndef FRONT_CRIT
#define FRONT_CRIT 320
#endif
#ifndef FRONT_CLEAR
#define FRONT_CLEAR 90
#endif
#ifndef K_AVOID
#define K_AVOID 0.0028
#endif
#ifndef K_SIDE_AVOID
#define K_SIDE_AVOID 0.0065
#endif
#ifndef FRONT_TURN_BIAS
#define FRONT_TURN_BIAS 0.42
#endif
#ifndef SIDE_CRIT
#define SIDE_CRIT 260
#endif
#ifndef WALL_TARGET
#define WALL_TARGET 150
#endif
#ifndef WALL_LOST
#define WALL_LOST 45
#endif
#ifndef WALL_MIN_MM
#define WALL_MIN_MM 180.0
#endif
#ifndef WALL_NO_PROGRESS_MM
#define WALL_NO_PROGRESS_MM 1400.0
#endif
#ifndef KP_WALL
#define KP_WALL 0.0045
#endif
#ifndef STEER_HARD
#define STEER_HARD 0.75
#endif
#ifndef LINE_EPS
#define LINE_EPS 60.0
#endif
#ifndef PROGRESS_MM
#define PROGRESS_MM 40.0
#endif
#ifndef TURN_SPEED
#define TURN_SPEED 0.5
#endif
#ifndef DIFF_MAX
#define DIFF_MAX 0.90
#endif
#ifndef ALPHA
#define ALPHA 0.4
#endif
#ifndef CLIFF_THR
#define CLIFF_THR 500
#endif

typedef enum {
    KHEP_GOAL = 0,
    KHEP_WALLF = 1,
    KHEP_BEIRADA = 2
} KhepState;

typedef struct {
    double ax, ay, bx, by;
    KhepState state;
    double fwd_s, diff_s;
    double cliff_start;
    int wall_left;
    double hit_dgoal, wallf_dist;
    double best_wall_dgoal, no_progress_dist;
    int left_line;
    double ct_int;
} KhepController;

typedef struct {
    double forward, diff;
    double fwd_s, diff_s;
    double d_goal, d_line, herr;
    double ux, uy;
    int F, FL, FR, L, R;
    int fb, Lc, Rc;
    int wall_left;
    int entered_wall;
    int left_wall;
    int leave_by_goal;
    int beirada;
    KhepState state;
} KhepOutput;

static double khep_clampd(double v, double lo, double hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

static double khep_wrap(double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

static int khep_excess(int v) {
    int e = v - IR_BASE;
    return e > 0 ? e : 0;
}

static void khep_ctrl_set_segment(KhepController *c,
                                  double ax, double ay,
                                  double bx, double by) {
    c->ax = ax;
    c->ay = ay;
    c->bx = bx;
    c->by = by;
    c->state = KHEP_GOAL;
    c->fwd_s = 0.0;
    c->diff_s = 0.0;
    c->cliff_start = 0.0;
    c->wall_left = 0;
    c->hit_dgoal = 0.0;
    c->wallf_dist = 0.0;
    c->best_wall_dgoal = 0.0;
    c->no_progress_dist = 0.0;
    c->left_line = 0;
    c->ct_int = 0.0;
}

static void khep_ctrl_init(KhepController *c, double bx, double by) {
    memset(c, 0, sizeof(*c));
    khep_ctrl_set_segment(c, 0.0, 0.0, bx, by);
}

static KhepOutput khep_ctrl_step(KhepController *c,
                                 double x, double y, double th,
                                 double dc, double dt, double t_now,
                                 int rawF, int rawFL, int rawFR,
                                 int rawL, int rawR, int gmin) {
    KhepOutput out;
    double vx, vy, vlen, px, py, bearing;
    double forward = 0.0, diff = 0.0;

    memset(&out, 0, sizeof(out));

    out.F = khep_excess(rawF);
    out.FL = khep_excess(rawFL);
    out.FR = khep_excess(rawFR);
    out.L = khep_excess(rawL);
    out.R = khep_excess(rawR);
    out.Lc = out.FL > out.L ? out.FL : out.L;
    out.Rc = out.FR > out.R ? out.FR : out.R;
    out.fb = out.F;
    if (out.FL > out.fb) out.fb = out.FL;
    if (out.FR > out.fb) out.fb = out.FR;

    vx = c->bx - c->ax;
    vy = c->by - c->ay;
    vlen = hypot(vx, vy);
    if (vlen < 1e-6) vlen = 1e-6;
    out.ux = vx / vlen;
    out.uy = vy / vlen;
    px = x - c->ax;
    py = y - c->ay;
    out.d_line = out.ux * py - out.uy * px;
    out.d_goal = hypot(c->bx - x, c->by - y);
    bearing = atan2(c->by - y, c->bx - x);
    out.herr = khep_wrap(bearing - th);

    if (c->state != KHEP_BEIRADA && gmin < CLIFF_THR) {
        c->state = KHEP_BEIRADA;
        c->cliff_start = t_now;
    }

    if (c->state == KHEP_BEIRADA) {
        double e = t_now - c->cliff_start;
        out.beirada = 1;
        if (e < 0.5) {
            forward = -0.6;
            diff = 0.0;
        } else if (e < 1.3) {
            forward = 0.0;
            diff = TURN_SPEED;
        } else {
            c->state = KHEP_GOAL;
            c->fwd_s = 0.0;
            c->diff_s = 0.0;
            c->ct_int = 0.0;
            c->wallf_dist = 0.0;
        }
        out.forward = forward;
        out.diff = diff;
        out.fwd_s = forward;
        out.diff_s = diff;
        out.state = c->state;
        out.wall_left = c->wall_left;
        return out;
    }

    if (c->state == KHEP_GOAL) {
        double theta_line = atan2(out.uy, out.ux);
        double ct_cmd = K_CT * out.d_line + KI_CT * c->ct_int;
        double cross_corr = atan2(ct_cmd, LOOKAHEAD_MM);
        double head_ref = khep_wrap(theta_line - cross_corr);
        double herr_ctrl = USE_POINT_P ? khep_wrap(head_ref - th) : out.herr;
        double left_pressure = (double)out.FL + 0.75 * (double)out.L;
        double right_pressure = (double)out.FR + 0.75 * (double)out.R;
        double side_guard = 0.0;
        if (out.d_goal < 650.0) {
            herr_ctrl = out.herr;
        }
        double fr_scale;

        if (out.F > FRONT_CLEAR || out.Lc > SIDE_CRIT || out.Rc > SIDE_CRIT) {
            side_guard += K_AVOID * (double)(out.Lc - out.Rc);
        }
        if (out.Lc > SIDE_CRIT || out.Rc > SIDE_CRIT) {
            side_guard += K_SIDE_AVOID * (left_pressure - right_pressure);
        }
        if (out.F > FRONT_CLEAR) {
            if (right_pressure > left_pressure + 25.0) {
                side_guard -= FRONT_TURN_BIAS;
            } else if (left_pressure > right_pressure + 25.0) {
                side_guard += FRONT_TURN_BIAS;
            } else {
                side_guard += (out.herr >= 0.0) ? -FRONT_TURN_BIAS : FRONT_TURN_BIAS;
            }
        }

        diff = -KP_HEAD * herr_ctrl + side_guard;
        diff = khep_clampd(diff, -DIFF_MAX, DIFF_MAX);
        forward = khep_clampd(1.0 - fabs(herr_ctrl) / HEAD_SLOW, 0.12, 1.0);
        fr_scale = khep_clampd(1.0 - out.fb / FRONT_SPAN, 0.25, 1.0);
        if (fr_scale < forward) forward = fr_scale;
        if (out.Lc > SIDE_CRIT || out.Rc > SIDE_CRIT) {
            double side_scale = khep_clampd(1.0 - ((double)(out.Lc > out.Rc ? out.Lc : out.Rc)) / FRONT_SPAN,
                                           0.10, 0.70);
            if (side_scale < forward) forward = side_scale;
        }

        if (fabs(diff) < DIFF_MAX) {
            c->ct_int = khep_clampd(c->ct_int + out.d_line * dt,
                                    -CT_INT_MAX, CT_INT_MAX);
        }

        if ((out.F > FRONT_BLOCK || (out.FL > FRONT_CRIT && out.FR > FRONT_CRIT)) &&
            (out.d_goal > 3.0 * ARRIVE_MM || out.fb > FRONT_CRIT)) {
            c->state = KHEP_WALLF;
            c->hit_dgoal = out.d_goal;
            c->best_wall_dgoal = out.d_goal;
            c->no_progress_dist = 0.0;
            c->wallf_dist = 0.0;
            c->left_line = 0;
            c->ct_int = 0.0;
            if (out.Lc == out.Rc)
                c->wall_left = (out.herr < 0.0);
            else
                c->wall_left = (out.Lc > out.Rc);
            out.entered_wall = 1;
        }
    } else {
        int side, other, err;
        double goal_diff;
        c->wallf_dist += fabs(dc);
        if (out.d_goal < c->best_wall_dgoal - 25.0) {
            c->best_wall_dgoal = out.d_goal;
            c->no_progress_dist = 0.0;
        } else {
            c->no_progress_dist += fabs(dc);
        }
        if (c->no_progress_dist > WALL_NO_PROGRESS_MM) {
            c->wall_left = !c->wall_left;
            c->hit_dgoal = out.d_goal;
            c->best_wall_dgoal = out.d_goal;
            c->wallf_dist = 0.0;
            c->no_progress_dist = 0.0;
            c->left_line = 0;
        }

        side = c->wall_left ? out.Lc : out.Rc;
        other = c->wall_left ? out.Rc : out.Lc;
        err = WALL_TARGET - side;

        if (c->wallf_dist > WALL_MIN_MM &&
            out.F < FRONT_CLEAR && out.FL < FRONT_BLOCK && out.FR < FRONT_BLOCK &&
            (fabs(out.herr) < 1.75 || c->no_progress_dist > 0.45 * WALL_NO_PROGRESS_MM) &&
            out.d_goal < c->hit_dgoal + 320.0) {
            c->state = KHEP_GOAL;
            c->fwd_s = 0.0;
            c->diff_s = 0.0;
            c->ct_int = 0.0;
            out.left_wall = 1;
            out.leave_by_goal = 1;
        }

        if (out.F > FRONT_CRIT && out.Lc > SIDE_CRIT && out.Rc > SIDE_CRIT) {
            forward = -0.10;
            diff = (out.Lc > out.Rc) ? +STEER_HARD : -STEER_HARD;
        } else if (other > SIDE_CRIT) {
            forward = 0.08;
            diff = c->wall_left ? -STEER_HARD : +STEER_HARD;
        } else if (side > FRONT_CRIT) {
            forward = 0.12;
            diff = c->wall_left ? +STEER_HARD : -STEER_HARD;
        } else if (side < WALL_LOST && out.F < FRONT_CLEAR) {
            forward = 0.22;
            diff = c->wall_left ? -0.55 * STEER_HARD : +0.55 * STEER_HARD;
        } else if (out.F > FRONT_CRIT) {
            forward = 0.06;
            diff = c->wall_left ? +STEER_HARD : -STEER_HARD;
        } else {
            double d = KP_WALL * (double)err;
            forward = khep_clampd(1.0 - out.F / FRONT_SPAN, 0.30, 0.85);
            diff = c->wall_left ? -d : +d;
        }
        goal_diff = khep_clampd(-0.65 * KP_HEAD * out.herr, -DIFF_MAX, DIFF_MAX);
        if (c->wallf_dist > 2.0 * WALL_MIN_MM && out.F < FRONT_CLEAR && fabs(out.herr) < 1.25) {
            diff = 0.70 * diff + 0.30 * goal_diff;
        }

        if (fabs(out.d_line) > 1.5 * LINE_EPS) c->left_line = 1;
        {
            int on_leave_line = (c->left_line || c->wallf_dist > WALL_MIN_MM) &&
                                fabs(out.d_line) < LINE_EPS;
            int target_clear = c->wallf_dist > 2.0 * WALL_MIN_MM &&
                               fabs(out.herr) < 0.90 &&
                               out.fb < FRONT_CLEAR;
            int near_goal_release = out.d_goal < 3.0 * ARRIVE_MM &&
                                    out.fb < FRONT_BLOCK;
            if (near_goal_release ||
                ((on_leave_line || target_clear) &&
                 out.d_goal < c->hit_dgoal - PROGRESS_MM &&
                 out.fb < FRONT_CLEAR)) {
            c->state = KHEP_GOAL;
            c->fwd_s = 0.0;
            c->diff_s = 0.0;
            out.left_wall = 1;
            out.leave_by_goal = (target_clear || near_goal_release) && !on_leave_line;
            }
        }
    }

    diff = khep_clampd(diff, -DIFF_MAX, DIFF_MAX);
    c->fwd_s = (1.0 - ALPHA) * c->fwd_s + ALPHA * forward;
    c->diff_s = (1.0 - ALPHA) * c->diff_s + ALPHA * diff;

    out.forward = forward;
    out.diff = diff;
    out.fwd_s = c->fwd_s;
    out.diff_s = c->diff_s;
    out.state = c->state;
    out.wall_left = c->wall_left;
    return out;
}

#endif
