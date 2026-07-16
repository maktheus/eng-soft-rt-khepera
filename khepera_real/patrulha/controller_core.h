/*
 * controller_core.h -- navegador tactil sem mapa previo.
 *
 * O nucleo nao conhece o ambiente. Ele recebe pose por odometria e IR bruto.
 * Um obstaculo e descoberto pelo toque, guardado como ponto de contato e
 * contornado por manobras curtas, limitadas e mensuradas pela odometria.
 */
#ifndef KHEPERA_CONTROLLER_CORE_H
#define KHEPERA_CONTROLLER_CORE_H

#include <math.h>
#include <string.h>

#include "controller_config.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

typedef enum {
    KHEP_GOAL = 0,
    KHEP_BACKOFF,
    KHEP_TANGENT,
    KHEP_BYPASS,
    KHEP_ALIGN_GOAL,
    KHEP_PROBE,
    KHEP_CLIFF_BACKOFF,
    KHEP_CLIFF_TURN,
    KHEP_BLOCKED
} KhepState;

typedef struct {
    double ax, ay, bx, by;
    KhepState state;
    KhepState previous_state;
    double state_started;
    double state_distance;
    double state_rotation;
    double last_th;
    int last_th_valid;

    double fwd_s, diff_s;
    double heading_target;
    double bypass_heading;
    double episode_heading;
    double contact_heading;
    double hit_x, hit_y, hit_dgoal;
    double best_dgoal;
    double episode_path;
    double lateral_progress_mark;
    double bypass_required;
    double no_motion_time;

    int bypass_side;       /* +1: passa pela esquerda; -1: pela direita */
    int preferred_side;
    int episode_active;
    int episode_contacts;
    int side_switches;
    int lateral_stall_contacts;
    int loop_contacts;
    int contact_samples;
    int cliff_samples;

    int map_count;
    double map_x[CONTACT_MAP_CAP];
    double map_y[CONTACT_MAP_CAP];
} KhepController;

typedef struct {
    double forward, diff;
    double fwd_s, diff_s;
    double d_goal, d_line, herr, lateral_progress, heading_target;
    int F, FL, FR, L, R;
    int front_peak, all_peak;
    int contact;
    int contact_event;
    int contact_index;
    double contact_x, contact_y;
    int state_changed;
    int side_switched;
    int loop_detected;
    int obstacle_cleared;
    int blocked;
    int beirada;
    int bypass_side;
    int wall_left;
    int entered_wall;
    int left_wall;
    int leave_by_goal;
    int flipped_wall;
    int spin_stuck;
    KhepState state;
} KhepOutput;

static double khep_clampd(double value, double lo, double hi) {
    return value < lo ? lo : (value > hi ? hi : value);
}

static double khep_wrap(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

static double khep_sign(double value) {
    return value < 0.0 ? -1.0 : 1.0;
}

static double khep_slew(double current, double target, double rate, double dt) {
    double max_step = rate * khep_clampd(dt, 0.0, 0.20);
    double delta = target - current;
    if (delta > max_step) delta = max_step;
    if (delta < -max_step) delta = -max_step;
    return current + delta;
}

static void khep_enter_state(KhepController *c, KhepState state,
                             double t_now) {
    c->previous_state = c->state;
    c->state = state;
    c->state_started = t_now;
    c->state_distance = 0.0;
    c->state_rotation = 0.0;
    c->no_motion_time = 0.0;
}

static void khep_reset_episode(KhepController *c) {
    c->episode_active = 0;
    c->episode_contacts = 0;
    c->side_switches = 0;
    c->episode_path = 0.0;
    c->lateral_progress_mark = 0.0;
    c->lateral_stall_contacts = 0;
    c->loop_contacts = 0;
    c->hit_dgoal = 0.0;
    c->best_dgoal = 0.0;
    c->bypass_required = BYPASS_BASE_MM;
}

static void khep_ctrl_set_segment(KhepController *c,
                                  double ax, double ay,
                                  double bx, double by) {
    c->ax = ax;
    c->ay = ay;
    c->bx = bx;
    c->by = by;
    c->state = KHEP_GOAL;
    c->previous_state = KHEP_GOAL;
    c->state_started = 0.0;
    c->state_distance = 0.0;
    c->state_rotation = 0.0;
    c->last_th_valid = 0;
    c->fwd_s = 0.0;
    c->diff_s = 0.0;
    c->contact_samples = 0;
    c->cliff_samples = 0;
    if (c->preferred_side == 0) c->preferred_side = +1;
    c->bypass_side = c->preferred_side;
    khep_reset_episode(c);
}

static void khep_ctrl_init(KhepController *c, double bx, double by) {
    memset(c, 0, sizeof(*c));
    c->preferred_side = +1;
    khep_ctrl_set_segment(c, 0.0, 0.0, bx, by);
}

static int khep_contact_near_old_point(const KhepController *c,
                                       double x, double y) {
    int i;
    int oldest = c->map_count > CONTACT_MAP_CAP ? c->map_count - CONTACT_MAP_CAP : 0;
    int newest = c->map_count - 2;
    for (i = oldest; i <= newest; ++i) {
        int slot = i % CONTACT_MAP_CAP;
        if (hypot(x - c->map_x[slot], y - c->map_y[slot]) < LOOP_RADIUS_MM)
            return 1;
    }
    return 0;
}

static int khep_record_contact(KhepController *c, double x, double y) {
    int loop = khep_contact_near_old_point(c, x, y);
    int slot;

    if (c->map_count > 0) {
        int last = (c->map_count - 1) % CONTACT_MAP_CAP;
        if (hypot(x - c->map_x[last], y - c->map_y[last]) < CONTACT_MAP_MERGE_MM)
            return 1;
    }

    slot = c->map_count % CONTACT_MAP_CAP;
    c->map_x[slot] = x;
    c->map_y[slot] = y;
    c->map_count++;
    return loop;
}

static void khep_contact_point(const KhepOutput *out,
                               double x, double y, double th,
                               double *cx, double *cy) {
    double rel = 0.0;
    int peak = out->F;
    if (out->FL > peak) { peak = out->FL; rel = +0.55; }
    if (out->FR > peak) { peak = out->FR; rel = -0.55; }
    *cx = x + 70.0 * cos(th + rel);
    *cy = y + 70.0 * sin(th + rel);
}

static void khep_choose_first_side(KhepController *c, const KhepOutput *out) {
    int left = out->FL + out->L;
    int right = out->FR + out->R;
    if (left > right + IR_SIDE_BIAS_RAW)
        c->bypass_side = -1;
    else if (right > left + IR_SIDE_BIAS_RAW)
        c->bypass_side = +1;
    else
        c->bypass_side = c->preferred_side;
}

static double khep_lateral_progress(const KhepController *c,
                                    double x, double y) {
    double nx = -sin(c->episode_heading);
    double ny = cos(c->episode_heading);
    return c->bypass_side * ((x - c->hit_x) * nx + (y - c->hit_y) * ny);
}

static void khep_switch_side(KhepController *c, KhepOutput *out) {
    c->bypass_side = -c->bypass_side;
    c->preferred_side = c->bypass_side;
    c->side_switches++;
    c->episode_contacts = 0;
    c->episode_path = 0.0;
    c->lateral_progress_mark = 0.0;
    c->lateral_stall_contacts = 0;
    c->loop_contacts = 0;
    c->bypass_required = BYPASS_BASE_MM;
    c->bypass_heading = khep_wrap(
        c->episode_heading + c->bypass_side * TANGENT_ANGLE_RAD);
    c->heading_target = c->bypass_heading;
    out->side_switched = 1;
    out->flipped_wall = 1;
}

static void khep_begin_contact(KhepController *c, KhepOutput *out,
                               double x, double y, double th,
                               double t_now) {
    double cx, cy;
    int should_switch = 0;
    int new_episode = !c->episode_active;

    khep_contact_point(out, x, y, th, &cx, &cy);
    out->loop_detected = khep_record_contact(c, cx, cy);
    out->contact_x = cx;
    out->contact_y = cy;
    out->contact_index = c->map_count;

    if (!c->episode_active) {
        c->episode_active = 1;
        c->episode_contacts = 0;
        c->side_switches = 0;
        c->episode_path = 0.0;
        c->lateral_progress_mark = 0.0;
        c->lateral_stall_contacts = 0;
        c->loop_contacts = 0;
        c->hit_x = x;
        c->hit_y = y;
        c->hit_dgoal = out->d_goal;
        c->best_dgoal = out->d_goal;
        c->episode_heading = th;
        khep_choose_first_side(c, out);
        c->bypass_heading = khep_wrap(
            c->episode_heading + c->bypass_side * TANGENT_ANGLE_RAD);
        out->entered_wall = 1;
    }

    if (!new_episode) {
        double lateral = khep_lateral_progress(c, x, y);
        if (lateral >= c->lateral_progress_mark + LATERAL_PROGRESS_STEP_MM) {
            c->lateral_progress_mark = lateral;
            c->lateral_stall_contacts = 0;
        } else {
            c->lateral_stall_contacts++;
        }
        if (out->loop_detected) c->loop_contacts++;
    }

    c->episode_contacts++;
    if (c->side_switches == 0 &&
        (c->lateral_stall_contacts >= SIDE_STALL_CONTACTS ||
         c->loop_contacts >= SIDE_LOOP_CONTACTS))
        should_switch = 1;

    if (should_switch) {
        khep_switch_side(c, out);
        c->episode_contacts = 1;
    }

    c->contact_heading = th;
    c->heading_target = c->bypass_heading;
    c->bypass_required = khep_clampd(
        BYPASS_BASE_MM + (c->episode_contacts - 1) * BYPASS_GROW_MM,
        BYPASS_BASE_MM, BYPASS_MAX_SEGMENT_MM);
    c->fwd_s = 0.0;
    c->diff_s = 0.0;
    out->contact_event = 1;
    khep_enter_state(c, KHEP_BACKOFF, t_now);
}

static void khep_heading_arc(double error, double base_forward,
                             double kp, double diff_limit,
                             double *forward, double *diff) {
    double turn = khep_clampd(-kp * error, -diff_limit, diff_limit);
    double slow = khep_clampd(1.0 - fabs(error) / GOAL_SLOW_ANGLE_RAD,
                              0.42, 1.0);
    *forward = base_forward * slow;
    if (*forward < GOAL_FWD_MIN) *forward = GOAL_FWD_MIN;
    turn = khep_clampd(turn, -*forward * 0.92, *forward * 0.92);
    *diff = turn;
}

static KhepOutput khep_ctrl_step(KhepController *c,
                                 double x, double y, double th,
                                 double dc, double dt, double t_now,
                                 int rawF, int rawFL, int rawFR,
                                 int rawL, int rawR, int gmin) {
    KhepOutput out;
    double vx, vy, vlen, px, py;
    double dth = 0.0;
    double forward = 0.0, diff = 0.0;
    double elapsed;
    int contact_peak, contact_candidate, hard_contact, confirmed_contact;
    KhepState state_at_start = c->state;

    memset(&out, 0, sizeof(out));
    out.F = rawF;
    out.FL = rawFL;
    out.FR = rawFR;
    out.L = rawL;
    out.R = rawR;
    out.front_peak = rawF;
    if (rawFL > out.front_peak) out.front_peak = rawFL;
    if (rawFR > out.front_peak) out.front_peak = rawFR;
    out.all_peak = out.front_peak;
    if (rawL > out.all_peak) out.all_peak = rawL;
    if (rawR > out.all_peak) out.all_peak = rawR;

    if (c->last_th_valid)
        dth = khep_wrap(th - c->last_th);
    else
        c->last_th_valid = 1;
    c->last_th = th;
    c->state_distance += fabs(dc);
    c->state_rotation += fabs(dth);
    if (c->episode_active) c->episode_path += fabs(dc);

    vx = c->bx - c->ax;
    vy = c->by - c->ay;
    vlen = hypot(vx, vy);
    if (vlen < 1e-6) vlen = 1e-6;
    px = x - c->ax;
    py = y - c->ay;
    out.d_line = (vx / vlen) * py - (vy / vlen) * px;
    out.d_goal = hypot(c->bx - x, c->by - y);
    out.herr = khep_wrap(atan2(c->by - y, c->bx - x) - th);
    if (c->episode_active)
        out.lateral_progress = khep_lateral_progress(c, x, y);
    if (c->episode_active && out.d_goal < c->best_dgoal)
        c->best_dgoal = out.d_goal;

    if (fabs(dc) < MOTION_EPS_MM &&
        (fabs(c->fwd_s) > 0.10 || fabs(c->diff_s) > 0.10))
        c->no_motion_time += dt;
    else
        c->no_motion_time = 0.0;

    /* O diagonal voltado para a parede e pressao lateral, nao nova colisao. */
    contact_peak = out.front_peak;
    if (c->episode_active &&
        (c->state == KHEP_TANGENT || c->state == KHEP_BYPASS ||
         c->state == KHEP_ALIGN_GOAL || c->state == KHEP_PROBE)) {
        contact_peak = out.F;
        if (c->bypass_side > 0) {
            if (out.FL > contact_peak) contact_peak = out.FL;
        } else {
            if (out.FR > contact_peak) contact_peak = out.FR;
        }
    }
    contact_candidate = contact_peak >= IR_CONTACT_RAW;
    hard_contact = contact_peak >= IR_HARD_CONTACT_RAW;
    if (contact_candidate)
        c->contact_samples++;
    else
        c->contact_samples = 0;
    confirmed_contact = hard_contact ||
                        c->contact_samples >= CONTACT_CONFIRM_TICKS;
    out.contact = confirmed_contact;

    if (c->state == KHEP_CLIFF_BACKOFF || c->state == KHEP_CLIFF_TURN) {
        c->cliff_samples = 0;
    } else if (gmin < CLIFF_THR) {
        c->cliff_samples++;
    } else {
        c->cliff_samples = 0;
    }

    if (c->cliff_samples >= CLIFF_CONFIRM_TICKS &&
        c->state != KHEP_BLOCKED) {
        c->preferred_side = -c->preferred_side;
        c->heading_target = khep_wrap(th + c->preferred_side * (0.5 * M_PI));
        c->fwd_s = 0.0;
        c->diff_s = 0.0;
        c->cliff_samples = 0;
        khep_reset_episode(c);
        khep_enter_state(c, KHEP_CLIFF_BACKOFF, t_now);
    }

    if (confirmed_contact &&
        (c->state == KHEP_GOAL || c->state == KHEP_TANGENT ||
         c->state == KHEP_BYPASS || c->state == KHEP_ALIGN_GOAL ||
         c->state == KHEP_PROBE)) {
        khep_begin_contact(c, &out, x, y, th, t_now);
        c->contact_samples = 0;
    }

    elapsed = t_now - c->state_started;
    switch (c->state) {
        case KHEP_GOAL: {
            double goal_error = 0.0;
            if (fabs(out.herr) > GOAL_DEADBAND_RAD)
                goal_error = out.herr - khep_sign(out.herr) * GOAL_DEADBAND_RAD;
            khep_heading_arc(goal_error, GOAL_FWD, GOAL_KP_HEAD,
                             GOAL_DIFF_MAX, &forward, &diff);
            if (out.front_peak > IR_NEAR_RAW) {
                double scale = khep_clampd(
                    (IR_CONTACT_RAW - out.front_peak) /
                    (double)(IR_CONTACT_RAW - IR_NEAR_RAW), 0.45, 1.0);
                forward *= scale;
                diff = khep_clampd(diff, -forward * 0.92, forward * 0.92);
            }
            break;
        }

        case KHEP_BACKOFF:
            forward = BACKOFF_FWD;
            diff = 0.0;
            if (c->state_distance >= BACKOFF_MM || elapsed >= BACKOFF_TIMEOUT_S) {
                khep_enter_state(c, KHEP_TANGENT, t_now);
                forward = 0.0;
            }
            break;

        case KHEP_TANGENT: {
            double err = khep_wrap(c->heading_target - th);
            khep_heading_arc(err, TURN_FWD, TURN_KP_HEAD,
                             TURN_DIFF_MAX, &forward, &diff);
            if (fabs(err) < TURN_TOL_RAD) {
                khep_enter_state(c, KHEP_BYPASS, t_now);
                forward = 0.0;
                diff = 0.0;
            } else if (elapsed >= TURN_TIMEOUT_S ||
                       c->state_rotation >= TURN_ROTATION_LIMIT_RAD) {
                if (fabs(err) <= TURN_FALLBACK_RAD) {
                    khep_enter_state(c, KHEP_BYPASS, t_now);
                    forward = 0.0;
                    diff = 0.0;
                } else {
                    khep_reset_episode(c);
                    khep_enter_state(c, KHEP_GOAL, t_now);
                    forward = 0.0;
                    diff = 0.0;
                }
            }
            break;
        }

        case KHEP_BYPASS: {
            int wall_raw = c->bypass_side > 0 ? out.R : out.L;
            double pressure;
            double target;
            if (c->bypass_side > 0 && out.FR > wall_raw) wall_raw = out.FR;
            if (c->bypass_side < 0 && out.FL > wall_raw) wall_raw = out.FL;
            pressure = khep_clampd(
                (wall_raw - IR_NEAR_RAW) /
                (double)(IR_HARD_CONTACT_RAW - IR_NEAR_RAW), 0.0, 1.0);
            target = khep_wrap(
                c->heading_target - c->bypass_side * BYPASS_INWARD_RAD +
                c->bypass_side * BYPASS_AWAY_RAD * pressure);
            double err = khep_wrap(target - th);
            khep_heading_arc(err, BYPASS_FWD, BYPASS_KP_HEAD,
                             BYPASS_DIFF_MAX, &forward, &diff);
            if (wall_raw >= IR_CONTACT_RAW) forward *= BYPASS_TOUCH_SCALE;
            diff = khep_clampd(diff, -forward * 0.92, forward * 0.92);

            if (c->state_distance >= c->bypass_required) {
                c->heading_target = c->episode_heading;
                khep_enter_state(c, KHEP_ALIGN_GOAL, t_now);
                forward = 0.0;
                diff = 0.0;
            } else if (c->no_motion_time >= STUCK_TIMEOUT_S) {
                khep_reset_episode(c);
                khep_enter_state(c, KHEP_GOAL, t_now);
            }
            break;
        }

        case KHEP_ALIGN_GOAL: {
            int wall_front = c->bypass_side > 0 ? out.FR : out.FL;
            double err = khep_wrap(c->heading_target - th);

            /* Ainda estamos ao lado da caixa: nao corte o canto para dentro. */
            if (wall_front >= IR_CONTACT_RAW) {
                khep_reset_episode(c);
                khep_enter_state(c, KHEP_GOAL, t_now);
                forward = 0.0;
                diff = 0.0;
                break;
            }

            khep_heading_arc(err, TURN_FWD, TURN_KP_HEAD,
                             TURN_DIFF_MAX, &forward, &diff);
            if (fabs(err) < TURN_TOL_RAD ||
                ((elapsed >= TURN_TIMEOUT_S || c->state_rotation >= M_PI) &&
                 fabs(err) <= TURN_FALLBACK_RAD)) {
                khep_enter_state(c, KHEP_PROBE, t_now);
                forward = 0.0;
                diff = 0.0;
            } else if (elapsed >= TURN_TIMEOUT_S || c->state_rotation >= M_PI) {
                khep_reset_episode(c);
                khep_enter_state(c, KHEP_GOAL, t_now);
                forward = 0.0;
                diff = 0.0;
            }
            break;
        }

        case KHEP_PROBE: {
            double err = khep_wrap(c->heading_target - th);
            khep_heading_arc(err, PROBE_FWD, GOAL_KP_HEAD,
                             GOAL_DIFF_MAX, &forward, &diff);
            if (c->state_distance >= PROBE_CLEAR_MM ||
                elapsed >= PROBE_TIMEOUT_S) {
                khep_reset_episode(c);
                khep_enter_state(c, KHEP_GOAL, t_now);
                out.obstacle_cleared = 1;
                out.left_wall = 1;
                out.leave_by_goal = 1;
            }
            break;
        }

        case KHEP_CLIFF_BACKOFF:
            out.beirada = 1;
            forward = -0.28;
            diff = 0.0;
            if (c->state_distance >= 120.0 || elapsed >= 2.0) {
                khep_enter_state(c, KHEP_CLIFF_TURN, t_now);
                forward = 0.0;
            }
            break;

        case KHEP_CLIFF_TURN: {
            double err = khep_wrap(c->heading_target - th);
            out.beirada = 1;
            forward = TURN_FWD;
            diff = -khep_sign(err) * TURN_DIFF_MAX;
            if (fabs(err) < TURN_TOL_RAD || elapsed >= TURN_TIMEOUT_S ||
                c->state_rotation >= TURN_ROTATION_LIMIT_RAD) {
                khep_enter_state(c, KHEP_GOAL, t_now);
                forward = 0.0;
                diff = 0.0;
            }
            break;
        }

        case KHEP_BLOCKED:
            out.blocked = 1;
            forward = 0.0;
            diff = 0.0;
            c->fwd_s = 0.0;
            c->diff_s = 0.0;
            break;
    }

    diff = khep_clampd(diff, -DIFF_MAX, DIFF_MAX);
    if (c->state == KHEP_BACKOFF || c->state == KHEP_CLIFF_BACKOFF) {
        c->fwd_s = khep_slew(c->fwd_s, forward, FWD_SLEW_PER_S, dt);
        c->diff_s = 0.0;
    } else {
        c->fwd_s = khep_slew(c->fwd_s, forward, FWD_SLEW_PER_S, dt);
        c->diff_s = khep_slew(c->diff_s, diff, DIFF_SLEW_PER_S, dt);
    }

    out.forward = forward;
    out.diff = diff;
    out.fwd_s = c->fwd_s;
    out.diff_s = c->diff_s;
    out.heading_target = c->state == KHEP_GOAL
        ? atan2(c->by - y, c->bx - x)
        : c->heading_target;
    out.state = c->state;
    out.state_changed = c->state != state_at_start;
    out.bypass_side = c->bypass_side;
    out.wall_left = c->bypass_side < 0;
    return out;
}

#endif
