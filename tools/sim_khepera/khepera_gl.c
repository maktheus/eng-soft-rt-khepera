/*
 * khepera_gl.c -- simulador visual OpenGL do controlador real do Khepera.
 *
 * Este programa inclui diretamente:
 *   ../../khepera_real/patrulha/controller_core.h
 *
 * Ou seja: a janela OpenGL e o robo fisico usam a mesma maquina de estados
 * GOAL/WALLF/BEIRADA. A simulacao troca apenas hardware por sensores raycast
 * e cinematica diferencial aproximada.
 *
 * Uso:
 *   khepera_gl.exe [mundos.json]                 janela interativa
 *   khepera_gl.exe --batch outdir [mundos.json] [--save-all]
 *                                                gera summary.txt e imagens
 *
 * Teclas na janela:
 *   N/P    proximo/anterior
 *   setas  proximo/anterior
 *   R      reinicia cenario
 *   F      alterna tempo real / rapido
 *   Espaco pausa/continua
 *   Esc    sai
 */

#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <gl/GL.h>

#include <errno.h>
#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../../khepera_real/patrulha/controller_core.h"

#ifdef _WIN32
#include <direct.h>
#define MKDIR(path) _mkdir(path)
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define ROBOT_BODY_RADIUS_MM      70.0
#define ROBOT_CLEARANCE_RADIUS_MM 90.0
#define SENSOR_MAX_MM             350.0
#define IR_EXCESS_MAX   520.0
#define DT_S            0.04
#define MAX_TIME_S      900.0
#define LINEAR_MM_S     120.0
#define ANGULAR_RAD_S   2.35
#define MAX_TRAIL       20000
#define MAX_OBS         64
#define MAX_SCENARIOS   1024
#define MAX_WAYPOINTS   256
#define PLAN_STEP_MM    120.0
#define PLAN_CLEARANCE_MM 150.0
#define WAYPOINT_REACH_MM 70.0
#define BATCH_W         1600
#define BATCH_H         1000

typedef struct {
    double x, y, w, h;
    char name[48];
} Rect;

typedef struct {
    char name[64];
    double gx, gy;
    double xmin, xmax, ymin, ymax;
    int obstacle_count;
    Rect obstacles[MAX_OBS];
} Scenario;

typedef struct {
    double x, y;
    KhepState state;
    int return_leg;
} TrailPoint;

typedef struct {
    double x, y;
} Waypoint;

typedef struct {
    KhepController ctrl;
    double x, y, th;
    double t;
    double path_mm;
    double min_goal_mm;
    double final_goal_mm;
    int arrived;
    int collided;
    int timeout;
    int wall_entries;
    TrailPoint trail[MAX_TRAIL];
    int trail_count;
    Waypoint waypoints[MAX_WAYPOINTS];
    int waypoint_count;
    int waypoint_index;
    int return_leg;
    int reached_b;
    double active_start_x, active_start_y;
    double active_goal_x, active_goal_y;
    KhepOutput last;
} Sim;

static Scenario g_scenarios[MAX_SCENARIOS];
static int g_scenario_count = 0;
static int g_current = 0;
static Sim g_sim;
static int g_paused = 0;
static int g_steps_per_tick = 1;
static int g_batch_save_all = 0;
static HWND g_hwnd = NULL;
static HDC g_hdc = NULL;
static HGLRC g_glrc = NULL;
static int g_win_w = 1400;
static int g_win_h = 900;

static double clamp_local(double v, double lo, double hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

static void copy_text(char *dst, size_t cap, const char *src) {
    if (cap == 0) return;
    if (!src) src = "";
    strncpy(dst, src, cap - 1);
    dst[cap - 1] = '\0';
}

static void safe_file_name(char *dst, size_t cap, const char *src) {
    size_t i;
    size_t j = 0;
    if (cap == 0) return;
    if (!src) src = "";
    for (i = 0; src[i] && j + 1 < cap; ++i) {
        unsigned char ch = (unsigned char)src[i];
        dst[j++] = (isalnum(ch) || ch == '_' || ch == '-') ? (char)ch : '_';
    }
    dst[j] = '\0';
    if (j == 0) copy_text(dst, cap, "cenario");
}

static void add_rect(Scenario *sc, double x, double y, double w, double h, const char *name) {
    Rect r;
    if (sc->obstacle_count >= MAX_OBS) return;
    r.x = x;
    r.y = y;
    r.w = w;
    r.h = h;
    copy_text(r.name, sizeof(r.name), name);
    sc->obstacles[sc->obstacle_count++] = r;
}

static void set_scenario_name(Scenario *sc, const char *name) {
    copy_text(sc->name, sizeof(sc->name), name);
}

static void init_scenarios(void) {
    Scenario *sc;

    sc = &g_scenarios[0];
    memset(sc, 0, sizeof(*sc));
    set_scenario_name(sc, "single_box_2m");
    sc->gx = 2200.0;
    sc->gy = 0.0;
    sc->xmin = -300.0; sc->xmax = 2600.0; sc->ymin = -900.0; sc->ymax = 900.0;
    add_rect(sc, 950.0, 0.0, 240.0, 320.0, "box_240x320");

    sc = &g_scenarios[1];
    memset(sc, 0, sizeof(*sc));
    set_scenario_name(sc, "corridor");
    sc->gx = 3000.0;
    sc->gy = 0.0;
    sc->xmin = -300.0; sc->xmax = 3400.0; sc->ymin = -900.0; sc->ymax = 900.0;
    add_rect(sc, 650.0, 360.0, 700.0, 160.0, "wall_top_1");
    add_rect(sc, 650.0, -360.0, 700.0, 160.0, "wall_bottom_1");
    add_rect(sc, 1450.0, 360.0, 700.0, 160.0, "wall_top_2");
    add_rect(sc, 1450.0, -360.0, 700.0, 160.0, "wall_bottom_2");
    add_rect(sc, 2300.0, 250.0, 220.0, 420.0, "offset_top");
    add_rect(sc, 2700.0, -250.0, 220.0, 420.0, "offset_bottom");

    sc = &g_scenarios[2];
    memset(sc, 0, sizeof(*sc));
    set_scenario_name(sc, "two_boxes_2_6m");
    sc->gx = 2600.0;
    sc->gy = 500.0;
    sc->xmin = -300.0; sc->xmax = 3000.0; sc->ymin = -800.0; sc->ymax = 1300.0;
    add_rect(sc, 850.0, 0.0, 220.0, 300.0, "first");
    add_rect(sc, 1650.0, 500.0, 240.0, 320.0, "second");

    sc = &g_scenarios[3];
    memset(sc, 0, sizeof(*sc));
    set_scenario_name(sc, "webots_3m");
    sc->gx = 2000.0;
    sc->gy = 2000.0;
    sc->xmin = -500.0; sc->xmax = 2500.0; sc->ymin = -500.0; sc->ymax = 2500.0;
    add_rect(sc, 500.0, 500.0, 200.0, 200.0, "box_200");
    add_rect(sc, 1000.0, 1000.0, 250.0, 250.0, "box_250");
    add_rect(sc, 1500.0, 1500.0, 200.0, 200.0, "box_200");
    add_rect(sc, 1400.0, 700.0, 150.0, 150.0, "box_150");
    add_rect(sc, 700.0, 1400.0, 150.0, 150.0, "box_150");

    sc = &g_scenarios[4];
    memset(sc, 0, sizeof(*sc));
    set_scenario_name(sc, "long_slalom");
    sc->gx = 2900.0;
    sc->gy = 500.0;
    sc->xmin = -300.0; sc->xmax = 3300.0; sc->ymin = -700.0; sc->ymax = 1300.0;
    add_rect(sc, 450.0, -120.0, 180.0, 360.0, "s1");
    add_rect(sc, 850.0, 360.0, 220.0, 300.0, "s2");
    add_rect(sc, 1300.0, -180.0, 180.0, 360.0, "s3");
    add_rect(sc, 1750.0, 440.0, 240.0, 320.0, "s4");
    add_rect(sc, 2200.0, -80.0, 200.0, 360.0, "s5");
    add_rect(sc, 2550.0, 720.0, 180.0, 260.0, "s6");

    sc = &g_scenarios[5];
    memset(sc, 0, sizeof(*sc));
    set_scenario_name(sc, "real_gate");
    sc->gx = 2600.0;
    sc->gy = 0.0;
    sc->xmin = -300.0; sc->xmax = 3000.0; sc->ymin = -900.0; sc->ymax = 900.0;
    add_rect(sc, 900.0, 260.0, 240.0, 520.0, "gate_top");
    add_rect(sc, 900.0, -260.0, 240.0, 520.0, "gate_bottom");
    add_rect(sc, 1550.0, 0.0, 220.0, 280.0, "center");
    add_rect(sc, 2150.0, -240.0, 220.0, 320.0, "exit");

    sc = &g_scenarios[6];
    memset(sc, 0, sizeof(*sc));
    set_scenario_name(sc, "u_trap");
    sc->gx = 2500.0;
    sc->gy = -350.0;
    sc->xmin = -300.0; sc->xmax = 3100.0; sc->ymin = -1300.0; sc->ymax = 1000.0;
    add_rect(sc, 900.0, 0.0, 180.0, 900.0, "u_left");
    add_rect(sc, 1450.0, 360.0, 900.0, 180.0, "u_top");
    add_rect(sc, 2000.0, 0.0, 180.0, 900.0, "u_right");
    add_rect(sc, 1450.0, -650.0, 320.0, 220.0, "island");
    add_rect(sc, 2350.0, -150.0, 180.0, 300.0, "exit_block");

    sc = &g_scenarios[7];
    memset(sc, 0, sizeof(*sc));
    set_scenario_name(sc, "crowded_lab");
    sc->gx = 3200.0;
    sc->gy = 1200.0;
    sc->xmin = -400.0; sc->xmax = 3600.0; sc->ymin = -1100.0; sc->ymax = 1800.0;
    add_rect(sc, 500.0, 120.0, 180.0, 300.0, "c1");
    add_rect(sc, 820.0, 650.0, 220.0, 220.0, "c2");
    add_rect(sc, 980.0, -520.0, 260.0, 260.0, "c3");
    add_rect(sc, 1370.0, 250.0, 200.0, 420.0, "c4");
    add_rect(sc, 1760.0, 840.0, 260.0, 220.0, "c5");
    add_rect(sc, 1990.0, -260.0, 220.0, 380.0, "c6");
    add_rect(sc, 2380.0, 420.0, 240.0, 260.0, "c7");
    add_rect(sc, 2680.0, 1050.0, 220.0, 260.0, "c8");
    add_rect(sc, 2920.0, 200.0, 180.0, 420.0, "c9");
    add_rect(sc, 3150.0, -520.0, 260.0, 260.0, "c10");

    g_scenario_count = 8;
}

typedef struct {
    const char *p;
    const char *end;
    char error[160];
} JsonReader;

static void json_error(JsonReader *r, const char *msg) {
    if (!r->error[0]) copy_text(r->error, sizeof(r->error), msg);
}

static void json_ws(JsonReader *r) {
    while (r->p < r->end && isspace((unsigned char)*r->p)) r->p++;
}

static int json_ch(JsonReader *r, char ch) {
    json_ws(r);
    if (r->p < r->end && *r->p == ch) {
        r->p++;
        return 1;
    }
    return 0;
}

static int json_expect(JsonReader *r, char ch) {
    if (json_ch(r, ch)) return 1;
    json_error(r, "JSON inesperado: delimitador ausente");
    return 0;
}

static int json_string(JsonReader *r, char *out, size_t cap) {
    size_t n = 0;
    json_ws(r);
    if (r->p >= r->end || *r->p != '"') {
        json_error(r, "JSON inesperado: string esperada");
        return 0;
    }
    r->p++;
    while (r->p < r->end && *r->p != '"') {
        char c = *r->p++;
        if (c == '\\' && r->p < r->end) {
            c = *r->p++;
            if (c == 'n') c = '\n';
            else if (c == 't') c = '\t';
        }
        if (n + 1 < cap) out[n++] = c;
    }
    if (r->p >= r->end) {
        json_error(r, "JSON inesperado: string sem fechamento");
        return 0;
    }
    r->p++;
    if (cap) out[n] = '\0';
    return 1;
}

static int json_number(JsonReader *r, double *out) {
    char *tail;
    json_ws(r);
    if (r->p >= r->end) {
        json_error(r, "JSON inesperado: numero esperado");
        return 0;
    }
    *out = strtod(r->p, &tail);
    if (tail == r->p) {
        json_error(r, "JSON inesperado: numero invalido");
        return 0;
    }
    r->p = tail;
    return 1;
}

static void json_skip_value(JsonReader *r);

static void json_skip_string(JsonReader *r) {
    char tmp[8];
    json_string(r, tmp, sizeof(tmp));
}

static void json_skip_array(JsonReader *r) {
    if (!json_expect(r, '[')) return;
    if (json_ch(r, ']')) return;
    while (r->p < r->end && !r->error[0]) {
        json_skip_value(r);
        if (json_ch(r, ']')) return;
        if (!json_expect(r, ',')) return;
    }
}

static void json_skip_object(JsonReader *r) {
    char key[64];
    if (!json_expect(r, '{')) return;
    if (json_ch(r, '}')) return;
    while (r->p < r->end && !r->error[0]) {
        if (!json_string(r, key, sizeof(key))) return;
        if (!json_expect(r, ':')) return;
        json_skip_value(r);
        if (json_ch(r, '}')) return;
        if (!json_expect(r, ',')) return;
    }
}

static void json_skip_value(JsonReader *r) {
    json_ws(r);
    if (r->p >= r->end) return;
    if (*r->p == '"') json_skip_string(r);
    else if (*r->p == '{') json_skip_object(r);
    else if (*r->p == '[') json_skip_array(r);
    else {
        while (r->p < r->end && !isspace((unsigned char)*r->p) &&
               *r->p != ',' && *r->p != ']' && *r->p != '}') {
            r->p++;
        }
    }
}

static int json_number_array(JsonReader *r, double *vals, int count) {
    int i;
    if (!json_expect(r, '[')) return 0;
    for (i = 0; i < count; ++i) {
        if (!json_number(r, &vals[i])) return 0;
        if (i + 1 < count && !json_expect(r, ',')) return 0;
    }
    if (!json_expect(r, ']')) return 0;
    return 1;
}

static int parse_obstacle(JsonReader *r, Scenario *sc) {
    char key[64], name[48] = "";
    double x = 0, y = 0, w = 0, h = 0;
    int have_rect = 0;

    if (!json_expect(r, '{')) return 0;
    while (!json_ch(r, '}') && !r->error[0]) {
        if (!json_string(r, key, sizeof(key))) return 0;
        if (!json_expect(r, ':')) return 0;
        if (strcmp(key, "name") == 0) {
            if (!json_string(r, name, sizeof(name))) return 0;
        } else if (strcmp(key, "rect") == 0) {
            double vals[4];
            if (!json_number_array(r, vals, 4)) return 0;
            x = vals[0]; y = vals[1]; w = vals[2]; h = vals[3];
            have_rect = 1;
        } else {
            json_skip_value(r);
        }
        json_ch(r, ',');
    }
    if (!have_rect || w <= 0 || h <= 0) {
        json_error(r, "obstaculo invalido: use rect [x,y,w,h] com dimensoes positivas");
        return 0;
    }
    add_rect(sc, x, y, w, h, name[0] ? name : "obs");
    return 1;
}

static int parse_obstacles(JsonReader *r, Scenario *sc) {
    if (!json_expect(r, '[')) return 0;
    if (json_ch(r, ']')) return 1;
    while (!r->error[0]) {
        if (sc->obstacle_count >= MAX_OBS) {
            json_error(r, "obstaculos demais no cenario");
            return 0;
        }
        if (!parse_obstacle(r, sc)) return 0;
        if (json_ch(r, ']')) return 1;
        if (!json_expect(r, ',')) return 0;
    }
    return 0;
}

static int scenario_valid(JsonReader *r, const Scenario *sc) {
    int i;
    if (!sc->name[0]) {
        json_error(r, "cenario sem nome");
        return 0;
    }
    if (sc->xmax <= sc->xmin || sc->ymax <= sc->ymin) {
        json_error(r, "bounds invalidos");
        return 0;
    }
    if (sc->gx < sc->xmin || sc->gx > sc->xmax || sc->gy < sc->ymin || sc->gy > sc->ymax) {
        json_error(r, "goal fora dos bounds");
        return 0;
    }
    for (i = 0; i < sc->obstacle_count; ++i) {
        Rect o = sc->obstacles[i];
        if (fabs(o.x) < 0.5 * o.w + ROBOT_CLEARANCE_RADIUS_MM &&
            fabs(o.y) < 0.5 * o.h + ROBOT_CLEARANCE_RADIUS_MM) {
            json_error(r, "obstaculo invade a area inicial do robo");
            return 0;
        }
    }
    return 1;
}

static int parse_scenario(JsonReader *r, Scenario *sc) {
    char key[64];
    int have_goal = 0, have_bounds = 0;
    memset(sc, 0, sizeof(*sc));
    if (!json_expect(r, '{')) return 0;
    while (!json_ch(r, '}') && !r->error[0]) {
        if (!json_string(r, key, sizeof(key))) return 0;
        if (!json_expect(r, ':')) return 0;
        if (strcmp(key, "name") == 0) {
            if (!json_string(r, sc->name, sizeof(sc->name))) return 0;
        } else if (strcmp(key, "goal") == 0) {
            double vals[2];
            if (!json_number_array(r, vals, 2)) return 0;
            sc->gx = vals[0]; sc->gy = vals[1]; have_goal = 1;
        } else if (strcmp(key, "bounds") == 0) {
            double vals[4];
            if (!json_number_array(r, vals, 4)) return 0;
            sc->xmin = vals[0]; sc->xmax = vals[1]; sc->ymin = vals[2]; sc->ymax = vals[3];
            have_bounds = 1;
        } else if (strcmp(key, "obstacles") == 0) {
            if (!parse_obstacles(r, sc)) return 0;
        } else {
            json_skip_value(r);
        }
        json_ch(r, ',');
    }
    if (!have_goal || !have_bounds) {
        json_error(r, "cenario precisa de goal e bounds");
        return 0;
    }
    return scenario_valid(r, sc);
}

static int parse_scenarios_array(JsonReader *r) {
    if (!json_expect(r, '[')) return 0;
    g_scenario_count = 0;
    if (json_ch(r, ']')) return 1;
    while (!r->error[0]) {
        if (g_scenario_count >= MAX_SCENARIOS) {
            json_error(r, "cenarios demais no JSON");
            return 0;
        }
        if (!parse_scenario(r, &g_scenarios[g_scenario_count])) return 0;
        g_scenario_count++;
        if (json_ch(r, ']')) return 1;
        if (!json_expect(r, ',')) return 0;
    }
    return 0;
}

static int parse_worlds_json(JsonReader *r) {
    char key[64];
    int found = 0;
    if (!json_expect(r, '{')) return 0;
    while (!json_ch(r, '}') && !r->error[0]) {
        if (!json_string(r, key, sizeof(key))) return 0;
        if (!json_expect(r, ':')) return 0;
        if (strcmp(key, "scenarios") == 0) {
            if (!parse_scenarios_array(r)) return 0;
            found = 1;
        } else {
            json_skip_value(r);
        }
        json_ch(r, ',');
    }
    if (!found || g_scenario_count <= 0) {
        json_error(r, "JSON precisa conter array 'scenarios'");
        return 0;
    }
    return 1;
}

static char *read_file_text(const char *path, long *len_out) {
    FILE *f = fopen(path, "rb");
    char *buf;
    long len;
    if (!f) return NULL;
    fseek(f, 0, SEEK_END);
    len = ftell(f);
    fseek(f, 0, SEEK_SET);
    buf = (char *)malloc((size_t)len + 1u);
    if (!buf) {
        fclose(f);
        return NULL;
    }
    if (fread(buf, 1, (size_t)len, f) != (size_t)len) {
        fclose(f);
        free(buf);
        return NULL;
    }
    fclose(f);
    buf[len] = '\0';
    if (len_out) *len_out = len;
    return buf;
}

static int load_scenarios_json(const char *path) {
    long len = 0;
    char *text = read_file_text(path, &len);
    JsonReader r;
    if (!text) return 0;
    memset(&r, 0, sizeof(r));
    r.p = text;
    r.end = text + len;
    if (!parse_worlds_json(&r)) {
        fprintf(stderr, "Erro lendo %s: %s\n", path, r.error[0] ? r.error : "JSON invalido");
        free(text);
        return 0;
    }
    free(text);
    return 1;
}

static int ensure_dir(const char *path) {
    if (MKDIR(path) == 0 || errno == EEXIST) return 1;
    return 0;
}

static double rect_min_dist_to_point(Rect r, double x, double y) {
    double dx = fabs(x - r.x) - 0.5 * r.w;
    double dy = fabs(y - r.y) - 0.5 * r.h;
    double ox = dx > 0.0 ? dx : 0.0;
    double oy = dy > 0.0 ? dy : 0.0;
    return hypot(ox, oy);
}

static int collides(const Scenario *sc, double x, double y) {
    int i;
    for (i = 0; i < sc->obstacle_count; ++i) {
        if (rect_min_dist_to_point(sc->obstacles[i], x, y) <= ROBOT_BODY_RADIUS_MM) return 1;
    }
    return 0;
}

typedef struct {
    double g, f;
    double clearance;
    int parent;
    unsigned char blocked, closed, opened;
} PlanNode;

typedef struct {
    int idx;
    double f;
} HeapItem;

static int plan_blocked(const Scenario *sc, double x, double y) {
    int i;
    if (x < sc->xmin || x > sc->xmax || y < sc->ymin || y > sc->ymax) return 1;
    for (i = 0; i < sc->obstacle_count; ++i) {
        if (rect_min_dist_to_point(sc->obstacles[i], x, y) <= PLAN_CLEARANCE_MM) return 1;
    }
    return 0;
}

static int plan_line_clear(const Scenario *sc, double ax, double ay, double bx, double by) {
    double dx = bx - ax;
    double dy = by - ay;
    double dist = hypot(dx, dy);
    int steps = (int)ceil(dist / 35.0);
    int i;
    if (steps < 1) steps = 1;
    for (i = 0; i <= steps; ++i) {
        double t = (double)i / (double)steps;
        if (plan_blocked(sc, ax + dx * t, ay + dy * t)) return 0;
    }
    return 1;
}

static void heap_swap(HeapItem *a, HeapItem *b) {
    HeapItem t = *a;
    *a = *b;
    *b = t;
}

static int heap_push(HeapItem *heap, int *count, int cap, int idx, double f) {
    int i;
    if (*count >= cap) return 0;
    i = (*count)++;
    heap[i].idx = idx;
    heap[i].f = f;
    while (i > 0) {
        int p = (i - 1) / 2;
        if (heap[p].f <= heap[i].f) break;
        heap_swap(&heap[p], &heap[i]);
        i = p;
    }
    return 1;
}

static int heap_pop(HeapItem *heap, int *count) {
    int out;
    int i = 0;
    if (*count <= 0) return -1;
    out = heap[0].idx;
    heap[0] = heap[--(*count)];
    for (;;) {
        int l = i * 2 + 1;
        int r = l + 1;
        int s = i;
        if (l < *count && heap[l].f < heap[s].f) s = l;
        if (r < *count && heap[r].f < heap[s].f) s = r;
        if (s == i) break;
        heap_swap(&heap[s], &heap[i]);
        i = s;
    }
    return out;
}

static int nearest_free_idx(const Scenario *sc, int ix, int iy, int nx, int ny,
                            double xmin, double ymin, PlanNode *nodes) {
    int r;
    for (r = 0; r < nx || r < ny; ++r) {
        int y0 = iy - r, y1 = iy + r;
        int x0 = ix - r, x1 = ix + r;
        int x, y;
        for (y = y0; y <= y1; ++y) {
            for (x = x0; x <= x1; ++x) {
                int idx;
                double px, py;
                if (x < 0 || y < 0 || x >= nx || y >= ny) continue;
                if (x != x0 && x != x1 && y != y0 && y != y1) continue;
                idx = y * nx + x;
                px = xmin + (double)x * PLAN_STEP_MM;
                py = ymin + (double)y * PLAN_STEP_MM;
                if (!nodes[idx].blocked && !plan_blocked(sc, px, py)) return idx;
            }
        }
    }
    return -1;
}

static int build_waypoints(const Scenario *sc,
                           double ax, double ay,
                           double bx, double by,
                           Waypoint *out, int cap) {
    int nx = (int)floor((sc->xmax - sc->xmin) / PLAN_STEP_MM) + 1;
    int ny = (int)floor((sc->ymax - sc->ymin) / PLAN_STEP_MM) + 1;
    int total = nx * ny;
    PlanNode *nodes;
    HeapItem *heap;
    int heap_count = 0;
    int six, siy, gix, giy, start, goal, found = 0;
    int i, idx;
    int *path;
    int path_count = 0;
    int rev_count = 0;
    Waypoint rev[MAX_WAYPOINTS * 4];
    const int dirs[8][2] = {
        {1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}
    };

    if (nx <= 2 || ny <= 2 || total <= 0) return 0;
    nodes = (PlanNode *)calloc((size_t)total, sizeof(PlanNode));
    heap = (HeapItem *)malloc((size_t)total * sizeof(HeapItem));
    path = (int *)malloc((size_t)total * sizeof(int));
    if (!nodes || !heap || !path) {
        free(nodes); free(heap); free(path);
        return 0;
    }

    for (i = 0; i < total; ++i) {
        int x = i % nx;
        int y = i / nx;
        double px = sc->xmin + (double)x * PLAN_STEP_MM;
        double py = sc->ymin + (double)y * PLAN_STEP_MM;
        double clearance = 1.0e9;
        int oi;
        for (oi = 0; oi < sc->obstacle_count; ++oi) {
            double d = rect_min_dist_to_point(sc->obstacles[oi], px, py);
            if (d < clearance) clearance = d;
        }
        nodes[i].g = 1.0e30;
        nodes[i].f = 1.0e30;
        nodes[i].clearance = clearance;
        nodes[i].parent = -1;
        nodes[i].blocked = (unsigned char)plan_blocked(sc, px, py);
    }

    six = (int)floor((ax - sc->xmin) / PLAN_STEP_MM + 0.5);
    siy = (int)floor((ay - sc->ymin) / PLAN_STEP_MM + 0.5);
    gix = (int)floor((bx - sc->xmin) / PLAN_STEP_MM + 0.5);
    giy = (int)floor((by - sc->ymin) / PLAN_STEP_MM + 0.5);
    start = nearest_free_idx(sc, six, siy, nx, ny, sc->xmin, sc->ymin, nodes);
    goal = nearest_free_idx(sc, gix, giy, nx, ny, sc->xmin, sc->ymin, nodes);
    if (start < 0 || goal < 0) {
        free(nodes); free(heap); free(path);
        return 0;
    }

    nodes[start].g = 0.0;
    nodes[start].f = hypot((double)(start % nx - goal % nx), (double)(start / nx - goal / nx));
    heap_push(heap, &heap_count, total, start, nodes[start].f);

    while (heap_count > 0) {
        int cur = heap_pop(heap, &heap_count);
        int cx, cy, d;
        if (cur < 0) break;
        if (nodes[cur].closed) continue;
        nodes[cur].closed = 1;
        if (cur == goal) {
            found = 1;
            break;
        }
        cx = cur % nx;
        cy = cur / nx;
        for (d = 0; d < 8; ++d) {
            int tx = cx + dirs[d][0];
            int ty = cy + dirs[d][1];
            int ni;
            double step_cost, ng, h;
            if (tx < 0 || ty < 0 || tx >= nx || ty >= ny) continue;
            ni = ty * nx + tx;
            if (nodes[ni].blocked || nodes[ni].closed) continue;
            if (dirs[d][0] != 0 && dirs[d][1] != 0) {
                int a = cy * nx + tx;
                int b = ty * nx + cx;
                if (nodes[a].blocked || nodes[b].blocked) continue;
            }
            step_cost = (dirs[d][0] != 0 && dirs[d][1] != 0) ? 1.41421356237 : 1.0;
            {
                double c = nodes[ni].clearance;
                double penalty = 0.0;
                if (c < 360.0) penalty += 1.5 * (360.0 - c) / 360.0;
                if (c < 260.0) penalty += 2.0 * (260.0 - c) / 260.0;
                ng = nodes[cur].g + step_cost * (1.0 + penalty);
            }
            if (ng >= nodes[ni].g) continue;
            nodes[ni].g = ng;
            nodes[ni].parent = cur;
            h = hypot((double)(tx - goal % nx), (double)(ty - goal / nx));
            nodes[ni].f = ng + h;
            heap_push(heap, &heap_count, total, ni, nodes[ni].f);
            nodes[ni].opened = 1;
        }
    }

    if (!found) {
        free(nodes); free(heap); free(path);
        return 0;
    }

    idx = goal;
    while (idx >= 0 && path_count < total) {
        path[path_count++] = idx;
        if (idx == start) break;
        idx = nodes[idx].parent;
    }
    if (path_count <= 1) {
        free(nodes); free(heap); free(path);
        return 0;
    }

    for (i = path_count - 1; i >= 0 && rev_count < (int)(sizeof(rev) / sizeof(rev[0])); --i) {
        int p = path[i];
        rev[rev_count].x = sc->xmin + (double)(p % nx) * PLAN_STEP_MM;
        rev[rev_count].y = sc->ymin + (double)(p / nx) * PLAN_STEP_MM;
        rev_count++;
    }

    {
        int out_count = 0;
        int anchor = 0;
        while (anchor < rev_count - 1 && out_count < cap - 1) {
            int next = rev_count - 1;
            int j;
            for (j = rev_count - 1; j > anchor + 1; --j) {
                if (plan_line_clear(sc, rev[anchor].x, rev[anchor].y, rev[j].x, rev[j].y)) {
                    next = j;
                    break;
                }
            }
            out[out_count++] = rev[next];
            anchor = next;
        }
        if (out_count == 0 ||
            hypot(out[out_count - 1].x - bx, out[out_count - 1].y - by) > ARRIVE_MM) {
            out[out_count].x = bx;
            out[out_count].y = by;
            out_count++;
        } else {
            out[out_count - 1].x = bx;
            out[out_count - 1].y = by;
        }
        free(nodes); free(heap); free(path);
        return out_count;
    }
}

static double active_goal_dist(void) {
    return hypot(g_sim.active_goal_x - g_sim.x, g_sim.active_goal_y - g_sim.y);
}

static double current_waypoint_dist(void) {
    Waypoint wp;
    if (g_sim.waypoint_count <= 0) return 0.0;
    if (g_sim.waypoint_index < 0) g_sim.waypoint_index = 0;
    if (g_sim.waypoint_index >= g_sim.waypoint_count) {
        g_sim.waypoint_index = g_sim.waypoint_count - 1;
    }
    wp = g_sim.waypoints[g_sim.waypoint_index];
    return hypot(wp.x - g_sim.x, wp.y - g_sim.y);
}

static void set_waypoint_target(void) {
    Waypoint wp;
    if (g_sim.waypoint_count <= 0) return;
    if (g_sim.waypoint_index < 0) g_sim.waypoint_index = 0;
    if (g_sim.waypoint_index >= g_sim.waypoint_count) {
        g_sim.waypoint_index = g_sim.waypoint_count - 1;
    }
    wp = g_sim.waypoints[g_sim.waypoint_index];
    khep_ctrl_set_segment(&g_sim.ctrl, g_sim.x, g_sim.y, wp.x, wp.y);
}

static void configure_leg(const Scenario *sc,
                          double ax, double ay,
                          double bx, double by,
                          int return_leg) {
    g_sim.return_leg = return_leg;
    g_sim.active_start_x = ax;
    g_sim.active_start_y = ay;
    g_sim.active_goal_x = bx;
    g_sim.active_goal_y = by;
    g_sim.waypoint_count = build_waypoints(sc, ax, ay, bx, by,
                                           g_sim.waypoints, MAX_WAYPOINTS);
    if (g_sim.waypoint_count <= 0) {
        g_sim.waypoint_count = 1;
        g_sim.waypoints[0].x = bx;
        g_sim.waypoints[0].y = by;
    }
    g_sim.waypoint_index = 0;
    set_waypoint_target();
}

static int maybe_advance_waypoint(const Scenario *sc) {
    int changed = 0;
    int i;
    if (g_sim.waypoint_count <= 0) return 0;

    while (g_sim.waypoint_index + 1 < g_sim.waypoint_count &&
           current_waypoint_dist() <= WAYPOINT_REACH_MM) {
        g_sim.waypoint_index++;
        changed = 1;
    }

    for (i = g_sim.waypoint_count - 1; i > g_sim.waypoint_index; --i) {
        Waypoint wp = g_sim.waypoints[i];
        if (plan_line_clear(sc, g_sim.x, g_sim.y, wp.x, wp.y)) {
            g_sim.waypoint_index = i;
            changed = 1;
            break;
        }
    }

    if (changed) set_waypoint_target();
    return changed;
}

static int handle_active_goal(const Scenario *sc, double d_final) {
    if (d_final >= ARRIVE_MM) return 0;
    if (!g_sim.return_leg) {
        g_sim.reached_b = 1;
        g_sim.final_goal_mm = d_final;
        configure_leg(sc, g_sim.x, g_sim.y, 0.0, 0.0, 1);
        return 1;
    }
    g_sim.arrived = 1;
    g_sim.final_goal_mm = d_final;
    return 1;
}

static double cast_sensor(const Scenario *sc, double x, double y, double th, double rel) {
    double a = th + rel;
    double step = 4.0;
    double d;
    for (d = 0.0; d <= SENSOR_MAX_MM; d += step) {
        double px = x + cos(a) * d;
        double py = y + sin(a) * d;
        int i;
        if (px < sc->xmin || px > sc->xmax || py < sc->ymin || py > sc->ymax) return d;
        for (i = 0; i < sc->obstacle_count; ++i) {
            Rect r = sc->obstacles[i];
            if (fabs(px - r.x) <= 0.5 * r.w && fabs(py - r.y) <= 0.5 * r.h) return d;
        }
    }
    return SENSOR_MAX_MM;
}

static int ir_from_dist(double dist) {
    if (dist >= SENSOR_MAX_MM) return IR_BASE;
    return IR_BASE + (int)clamp_local(IR_EXCESS_MAX * (1.0 - dist / SENSOR_MAX_MM), 0.0, IR_EXCESS_MAX);
}

static void read_ir(const Scenario *sc, double x, double y, double th,
                    int *rawF, int *rawFL, int *rawFR, int *rawL, int *rawR) {
    *rawF  = ir_from_dist(cast_sensor(sc, x, y, th, 0.0));
    *rawFL = ir_from_dist(cast_sensor(sc, x, y, th, 35.0 * M_PI / 180.0));
    *rawFR = ir_from_dist(cast_sensor(sc, x, y, th, -35.0 * M_PI / 180.0));
    *rawL  = ir_from_dist(cast_sensor(sc, x, y, th, 80.0 * M_PI / 180.0));
    *rawR  = ir_from_dist(cast_sensor(sc, x, y, th, -80.0 * M_PI / 180.0));
}

static void reset_sim(int idx) {
    Scenario *sc = &g_scenarios[idx];
    memset(&g_sim, 0, sizeof(g_sim));
    g_sim.x = 0.0;
    g_sim.y = 0.0;
    g_sim.th = 0.0;
    khep_ctrl_init(&g_sim.ctrl, sc->gx, sc->gy);
    configure_leg(sc, 0.0, 0.0, sc->gx, sc->gy, 0);
    g_sim.min_goal_mm = active_goal_dist();
    g_sim.final_goal_mm = g_sim.min_goal_mm;
    g_sim.trail[0].x = g_sim.x;
    g_sim.trail[0].y = g_sim.y;
    g_sim.trail[0].state = KHEP_GOAL;
    g_sim.trail[0].return_leg = 0;
    g_sim.trail_count = 1;
}

static void sim_step(const Scenario *sc) {
    int rawF, rawFL, rawFR, rawL, rawR;
    int gmin = 960;
    double old_x, old_y, dc, moved, d_final;

    if (g_sim.arrived || g_sim.collided || g_sim.timeout) return;

    d_final = active_goal_dist();
    if (d_final < g_sim.min_goal_mm) g_sim.min_goal_mm = d_final;
    if (handle_active_goal(sc, d_final)) return;
    maybe_advance_waypoint(sc);

    read_ir(sc, g_sim.x, g_sim.y, g_sim.th, &rawF, &rawFL, &rawFR, &rawL, &rawR);
    old_x = g_sim.x;
    old_y = g_sim.y;
    dc = 0.0;
    if (g_sim.trail_count > 1) {
        TrailPoint p = g_sim.trail[g_sim.trail_count - 2];
        dc = hypot(g_sim.x - p.x, g_sim.y - p.y);
    }

    g_sim.last = khep_ctrl_step(&g_sim.ctrl, g_sim.x, g_sim.y, g_sim.th, dc, DT_S, g_sim.t,
                                rawF, rawFL, rawFR, rawL, rawR, gmin);
    if (g_sim.last.entered_wall) g_sim.wall_entries++;

    g_sim.x += cos(g_sim.th) * (g_sim.last.fwd_s * LINEAR_MM_S * DT_S);
    g_sim.y += sin(g_sim.th) * (g_sim.last.fwd_s * LINEAR_MM_S * DT_S);
    g_sim.th = khep_wrap(g_sim.th - g_sim.last.diff_s * ANGULAR_RAD_S * DT_S);
    moved = hypot(g_sim.x - old_x, g_sim.y - old_y);
    g_sim.path_mm += moved;
    g_sim.t += DT_S;

    d_final = active_goal_dist();
    if (d_final < g_sim.min_goal_mm) g_sim.min_goal_mm = d_final;
    if (handle_active_goal(sc, d_final)) return;
    if (collides(sc, g_sim.x, g_sim.y)) {
        g_sim.collided = 1;
        g_sim.final_goal_mm = active_goal_dist();
    }
    if (g_sim.t >= MAX_TIME_S) {
        g_sim.timeout = 1;
        g_sim.final_goal_mm = active_goal_dist();
    }
    if (!g_sim.arrived && !g_sim.collided && !g_sim.timeout) maybe_advance_waypoint(sc);

    if (g_sim.trail_count < MAX_TRAIL) {
        TrailPoint *p = &g_sim.trail[g_sim.trail_count++];
        p->x = g_sim.x;
        p->y = g_sim.y;
        p->state = g_sim.last.state;
        p->return_leg = g_sim.return_leg;
    }
}

static void gl_color(double r, double g, double b) {
    glColor3d(r, g, b);
}

static void draw_rect(Rect r, double rr, double gg, double bb) {
    gl_color(rr, gg, bb);
    glBegin(GL_QUADS);
    glVertex2d(r.x - 0.5 * r.w, r.y - 0.5 * r.h);
    glVertex2d(r.x + 0.5 * r.w, r.y - 0.5 * r.h);
    glVertex2d(r.x + 0.5 * r.w, r.y + 0.5 * r.h);
    glVertex2d(r.x - 0.5 * r.w, r.y + 0.5 * r.h);
    glEnd();
}

static void draw_circle(double x, double y, double r, int filled) {
    int i;
    if (filled) glBegin(GL_TRIANGLE_FAN); else glBegin(GL_LINE_LOOP);
    if (filled) glVertex2d(x, y);
    for (i = 0; i <= 48; ++i) {
        double a = 2.0 * M_PI * (double)i / 48.0;
        glVertex2d(x + cos(a) * r, y + sin(a) * r);
    }
    glEnd();
}

static void draw_sensor_rays(const Scenario *sc, const Sim *sim) {
    const double rels[] = {0.0, 35.0, -35.0, 80.0, -80.0};
    int i;
    glLineWidth(1.5f);
    for (i = 0; i < 5; ++i) {
        double rel = rels[i] * M_PI / 180.0;
        double a = sim->th + rel;
        double d = cast_sensor(sc, sim->x, sim->y, sim->th, rel);
        double hit = d < SENSOR_MAX_MM ? 1.0 : 0.0;
        if (hit) gl_color(0.90, 0.24, 0.18);
        else gl_color(0.95, 0.68, 0.20);
        glBegin(GL_LINES);
        glVertex2d(sim->x, sim->y);
        glVertex2d(sim->x + cos(a) * d, sim->y + sin(a) * d);
        glEnd();
        if (hit) {
            draw_circle(sim->x + cos(a) * d, sim->y + sin(a) * d, 14.0, 1);
        }
    }
}

static void draw_scene(const Scenario *sc, const Sim *sim, int width, int height) {
    double sx = (sc->xmax - sc->xmin) / (double)width;
    double sy = (sc->ymax - sc->ymin) / (double)height;
    double margin = (sx > sy ? sx : sy) * 18.0;
    int i;

    glViewport(0, 0, width, height);
    glClearColor(0.965f, 0.955f, 0.925f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(sc->xmin - margin, sc->xmax + margin, sc->ymin - margin, sc->ymax + margin, -1.0, 1.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glLineWidth(1.0f);
    gl_color(0.88, 0.86, 0.80);
    glBegin(GL_LINES);
    for (i = (int)ceil(sc->xmin / 250.0) * 250; i <= (int)sc->xmax; i += 250) {
        glVertex2d(i, sc->ymin); glVertex2d(i, sc->ymax);
    }
    for (i = (int)ceil(sc->ymin / 250.0) * 250; i <= (int)sc->ymax; i += 250) {
        glVertex2d(sc->xmin, i); glVertex2d(sc->xmax, i);
    }
    glEnd();

    glLineWidth(2.0f);
    gl_color(0.25, 0.30, 0.32);
    glBegin(GL_LINE_LOOP);
    glVertex2d(sc->xmin, sc->ymin);
    glVertex2d(sc->xmax, sc->ymin);
    glVertex2d(sc->xmax, sc->ymax);
    glVertex2d(sc->xmin, sc->ymax);
    glEnd();

    glLineWidth(2.0f);
    gl_color(0.48, 0.54, 0.55);
    glBegin(GL_LINES);
    glVertex2d(0.0, 0.0); glVertex2d(sc->gx, sc->gy);
    glEnd();

    for (i = 0; i < sc->obstacle_count; ++i) {
        Rect r = sc->obstacles[i];
        Rect inflated = r;
        inflated.w += 2.0 * ROBOT_CLEARANCE_RADIUS_MM;
        inflated.h += 2.0 * ROBOT_CLEARANCE_RADIUS_MM;
        draw_rect(r, 0.47, 0.30, 0.22);
        gl_color(0.74, 0.65, 0.61);
        glLineWidth(1.0f);
        glBegin(GL_LINE_LOOP);
        glVertex2d(inflated.x - 0.5 * inflated.w, inflated.y - 0.5 * inflated.h);
        glVertex2d(inflated.x + 0.5 * inflated.w, inflated.y - 0.5 * inflated.h);
        glVertex2d(inflated.x + 0.5 * inflated.w, inflated.y + 0.5 * inflated.h);
        glVertex2d(inflated.x - 0.5 * inflated.w, inflated.y + 0.5 * inflated.h);
        glEnd();
    }

    if (sim->waypoint_count > 0) {
        glLineWidth(2.0f);
        if (sim->return_leg) gl_color(0.56, 0.20, 0.70);
        else gl_color(0.32, 0.24, 0.72);
        glBegin(GL_LINE_STRIP);
        glVertex2d(sim->active_start_x, sim->active_start_y);
        for (i = 0; i < sim->waypoint_count; ++i) {
            glVertex2d(sim->waypoints[i].x, sim->waypoints[i].y);
        }
        glEnd();
        if (sim->return_leg) gl_color(0.56, 0.20, 0.70);
        else gl_color(0.32, 0.24, 0.72);
        for (i = 0; i < sim->waypoint_count - 1; ++i) {
            draw_circle(sim->waypoints[i].x, sim->waypoints[i].y, 13.0, 1);
        }
    }

    if (sim->trail_count > 1) {
        glLineWidth(3.0f);
        for (i = 1; i < sim->trail_count; ++i) {
            if (sim->trail[i].state == KHEP_WALLF) gl_color(0.08, 0.40, 0.75);
            else if (sim->trail[i].return_leg) gl_color(0.45, 0.22, 0.58);
            else gl_color(0.18, 0.49, 0.20);
            glBegin(GL_LINES);
            glVertex2d(sim->trail[i - 1].x, sim->trail[i - 1].y);
            glVertex2d(sim->trail[i].x, sim->trail[i].y);
            glEnd();
        }
    }

    draw_sensor_rays(sc, sim);

    gl_color(0.00, 0.65, 0.46);
    draw_circle(0.0, 0.0, ROBOT_BODY_RADIUS_MM * 0.48, 1);
    gl_color(0.88, 0.12, 0.15);
    draw_circle(sc->gx, sc->gy, ROBOT_BODY_RADIUS_MM * 0.85, 1);
    gl_color(0.36, 0.27, 0.18);
    draw_circle(sim->x, sim->y, ROBOT_CLEARANCE_RADIUS_MM, 0);
    gl_color(1.00, 0.78, 0.16);
    draw_circle(sim->x, sim->y, ROBOT_BODY_RADIUS_MM, 1);
    gl_color(0.12, 0.16, 0.18);
    glLineWidth(3.0f);
    glBegin(GL_LINES);
    glVertex2d(sim->x, sim->y);
    glVertex2d(sim->x + cos(sim->th) * ROBOT_BODY_RADIUS_MM * 1.45,
               sim->y + sin(sim->th) * ROBOT_BODY_RADIUS_MM * 1.45);
    glEnd();

    glFlush();
}

static int save_bmp(const char *path, int width, int height) {
    FILE *f;
    unsigned char *pixels;
    unsigned char file_header[14] = {'B','M'};
    unsigned char info_header[40] = {0};
    int row_stride = width * 3;
    int row_pad = (4 - (row_stride % 4)) % 4;
    int image_size = (row_stride + row_pad) * height;
    int file_size = 14 + 40 + image_size;
    int y, x;

    pixels = (unsigned char *)malloc((size_t)width * (size_t)height * 3u);
    if (!pixels) return 0;
    glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, pixels);

    file_header[2] = (unsigned char)(file_size);
    file_header[3] = (unsigned char)(file_size >> 8);
    file_header[4] = (unsigned char)(file_size >> 16);
    file_header[5] = (unsigned char)(file_size >> 24);
    file_header[10] = 54;

    info_header[0] = 40;
    info_header[4] = (unsigned char)(width);
    info_header[5] = (unsigned char)(width >> 8);
    info_header[6] = (unsigned char)(width >> 16);
    info_header[7] = (unsigned char)(width >> 24);
    info_header[8] = (unsigned char)(height);
    info_header[9] = (unsigned char)(height >> 8);
    info_header[10] = (unsigned char)(height >> 16);
    info_header[11] = (unsigned char)(height >> 24);
    info_header[12] = 1;
    info_header[14] = 24;

    f = fopen(path, "wb");
    if (!f) {
        free(pixels);
        return 0;
    }
    fwrite(file_header, 1, sizeof(file_header), f);
    fwrite(info_header, 1, sizeof(info_header), f);
    for (y = 0; y < height; ++y) {
        static const unsigned char pad[3] = {0, 0, 0};
        unsigned char *row = pixels + (size_t)y * (size_t)width * 3u;
        for (x = 0; x < width; ++x) {
            unsigned char bgr[3];
            bgr[0] = row[x * 3 + 2];
            bgr[1] = row[x * 3 + 1];
            bgr[2] = row[x * 3 + 0];
            fwrite(bgr, 1, 3, f);
        }
        fwrite(pad, 1, (size_t)row_pad, f);
    }
    fclose(f);
    free(pixels);
    return 1;
}

static const char *status_text(const Sim *sim) {
    if (sim->arrived) return "ARRIVED";
    if (sim->collided) return "COLLISION";
    if (sim->timeout) return "TIMEOUT";
    return "RUNNING";
}

static void update_window_title(void) {
    char title[256];
    const Scenario *sc = &g_scenarios[g_current];
    snprintf(title, sizeof(title),
             "Khepera OpenGL - [%d/%d] %s | %s | %s | t=%.1fs goal=%.0fmm wp=%d/%d path=%.0fmm walls=%d | %dx | N/P Setas R F Space Esc",
             g_current + 1, g_scenario_count, sc->name, status_text(&g_sim),
             g_sim.return_leg ? "B->A" : "A->B", g_sim.t, active_goal_dist(),
             g_sim.waypoint_index + 1, g_sim.waypoint_count, g_sim.path_mm, g_sim.wall_entries, g_steps_per_tick);
    SetWindowTextA(g_hwnd, title);
}

static void run_until_done(const Scenario *sc) {
    int max_steps = (int)(MAX_TIME_S / DT_S) + 5;
    int i;
    for (i = 0; i < max_steps && !g_sim.arrived && !g_sim.collided && !g_sim.timeout; ++i) {
        sim_step(sc);
    }
}

static int setup_gl(HWND hwnd) {
    PIXELFORMATDESCRIPTOR pfd;
    int pf;
    memset(&pfd, 0, sizeof(pfd));
    pfd.nSize = sizeof(pfd);
    pfd.nVersion = 1;
    pfd.dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER;
    pfd.iPixelType = PFD_TYPE_RGBA;
    pfd.cColorBits = 24;
    pfd.cDepthBits = 0;
    pfd.iLayerType = PFD_MAIN_PLANE;
    g_hdc = GetDC(hwnd);
    pf = ChoosePixelFormat(g_hdc, &pfd);
    if (!pf || !SetPixelFormat(g_hdc, pf, &pfd)) return 0;
    g_glrc = wglCreateContext(g_hdc);
    if (!g_glrc || !wglMakeCurrent(g_hdc, g_glrc)) return 0;
    return 1;
}

static void teardown_gl(void) {
    if (g_glrc) {
        wglMakeCurrent(NULL, NULL);
        wglDeleteContext(g_glrc);
        g_glrc = NULL;
    }
    if (g_hwnd && g_hdc) {
        ReleaseDC(g_hwnd, g_hdc);
        g_hdc = NULL;
    }
}

static LRESULT CALLBACK wnd_proc(HWND hwnd, UINT msg, WPARAM wp, LPARAM lp) {
    switch (msg) {
    case WM_CREATE:
        if (!setup_gl(hwnd)) return -1;
        SetTimer(hwnd, 1, (UINT)(DT_S * 1000.0), NULL);
        return 0;
    case WM_SIZE:
        g_win_w = LOWORD(lp);
        g_win_h = HIWORD(lp);
        return 0;
    case WM_TIMER:
        if (!g_paused) {
            int i;
            for (i = 0; i < g_steps_per_tick; ++i) sim_step(&g_scenarios[g_current]);
            update_window_title();
            InvalidateRect(hwnd, NULL, FALSE);
        }
        return 0;
    case WM_KEYDOWN:
        if (wp == VK_ESCAPE) DestroyWindow(hwnd);
        else if (wp == VK_SPACE) g_paused = !g_paused;
        else if (wp == 'R') reset_sim(g_current);
        else if (wp == 'F') g_steps_per_tick = (g_steps_per_tick == 1) ? 5 : 1;
        else if (wp == 'N' || wp == VK_RIGHT || wp == VK_DOWN) {
            g_current = (g_current + 1) % g_scenario_count;
            reset_sim(g_current);
        }
        else if (wp == 'P' || wp == VK_LEFT || wp == VK_UP) {
            g_current = (g_current + g_scenario_count - 1) % g_scenario_count;
            reset_sim(g_current);
        }
        else if (wp >= '1' && wp <= '9') {
            int idx = (int)(wp - '1');
            if (idx < g_scenario_count) {
                g_current = idx;
                reset_sim(g_current);
            }
        }
        update_window_title();
        InvalidateRect(hwnd, NULL, FALSE);
        return 0;
    case WM_PAINT:
        {
            PAINTSTRUCT ps;
            BeginPaint(hwnd, &ps);
            draw_scene(&g_scenarios[g_current], &g_sim, g_win_w, g_win_h);
            SwapBuffers(g_hdc);
            EndPaint(hwnd, &ps);
        }
        return 0;
    case WM_DESTROY:
        KillTimer(hwnd, 1);
        teardown_gl();
        PostQuitMessage(0);
        return 0;
    default:
        return DefWindowProc(hwnd, msg, wp, lp);
    }
}

static int create_window(int visible, int width, int height) {
    WNDCLASSA wc;
    DWORD style = WS_OVERLAPPEDWINDOW;
    memset(&wc, 0, sizeof(wc));
    wc.style = CS_OWNDC;
    wc.lpfnWndProc = wnd_proc;
    wc.hInstance = GetModuleHandle(NULL);
    wc.hCursor = LoadCursor(NULL, IDC_ARROW);
    wc.lpszClassName = "KheperaOpenGLSim";
    if (!RegisterClassA(&wc) && GetLastError() != ERROR_CLASS_ALREADY_EXISTS) return 0;
    g_hwnd = CreateWindowA(wc.lpszClassName, "Khepera OpenGL",
                           style, CW_USEDEFAULT, CW_USEDEFAULT, width, height,
                           NULL, NULL, wc.hInstance, NULL);
    if (!g_hwnd) return 0;
    if (visible) ShowWindow(g_hwnd, SW_SHOW);
    UpdateWindow(g_hwnd);
    return 1;
}

static int run_batch(const char *out_dir) {
    FILE *summary;
    char path[512];
    int i;

    ensure_dir("tools");
    ensure_dir("tools/sim_khepera");
    ensure_dir(out_dir);
    if (!create_window(0, BATCH_W, BATCH_H)) return 1;
    snprintf(path, sizeof(path), "%s/summary.txt", out_dir);
    summary = fopen(path, "w");
    if (!summary) return 1;
    fprintf(summary, "scenario,status,time_s,path_mm,min_goal_mm,final_goal_mm,wall_entries,samples\n");
    printf("scenario,status,time_s,path_mm,min_goal_mm,final_goal_mm,wall_entries\n");

    for (i = 0; i < g_scenario_count; ++i) {
        Scenario *sc = &g_scenarios[i];
        reset_sim(i);
        run_until_done(sc);
        if (g_batch_save_all || !g_sim.arrived || i == 0 || i == g_scenario_count - 1) {
            char file_name[96];
            draw_scene(sc, &g_sim, BATCH_W, BATCH_H);
            glFinish();
            safe_file_name(file_name, sizeof(file_name), sc->name);
            snprintf(path, sizeof(path), "%s/%s.bmp", out_dir, file_name);
            save_bmp(path, BATCH_W, BATCH_H);
        }
        fprintf(summary, "%s,%s,%.2f,%.1f,%.1f,%.1f,%d,%d\n",
                sc->name, status_text(&g_sim), g_sim.t, g_sim.path_mm,
                g_sim.min_goal_mm, g_sim.final_goal_mm, g_sim.wall_entries,
                g_sim.trail_count);
        printf("%s,%s,%.2f,%.1f,%.1f,%.1f,%d\n",
               sc->name, status_text(&g_sim), g_sim.t, g_sim.path_mm,
               g_sim.min_goal_mm, g_sim.final_goal_mm, g_sim.wall_entries);
    }
    fclose(summary);
    teardown_gl();
    DestroyWindow(g_hwnd);
    return 0;
}

static int run_interactive(void) {
    MSG msg;
    if (!create_window(1, g_win_w, g_win_h)) return 1;
    update_window_title();
    while (GetMessage(&msg, NULL, 0, 0) > 0) {
        TranslateMessage(&msg);
        DispatchMessage(&msg);
    }
    return 0;
}

int main(int argc, char **argv) {
    const char *default_worlds = "tools/sim_khepera/worlds_1000.json";
    const char *worlds_path = default_worlds;
    int explicit_worlds = 0;

    init_scenarios();

    if (argc >= 2 && strcmp(argv[1], "--batch") == 0) {
        const char *out_dir = argc >= 3 ? argv[2] : "tools/sim_khepera/out_gl";
        if (argc >= 4) {
            worlds_path = argv[3];
            explicit_worlds = 1;
        }
        if (argc >= 5 && strcmp(argv[4], "--save-all") == 0) {
            g_batch_save_all = 1;
        }
        if (!load_scenarios_json(worlds_path) && explicit_worlds) return 1;
        reset_sim(0);
        return run_batch(out_dir);
    }
    if (argc >= 2) {
        worlds_path = argv[1];
        explicit_worlds = 1;
    }
    if (!load_scenarios_json(worlds_path) && explicit_worlds) return 1;
    reset_sim(0);
    return run_interactive();
}
