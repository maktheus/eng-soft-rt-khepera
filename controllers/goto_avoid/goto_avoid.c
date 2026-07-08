/*
 * goto_avoid.c  —  Khepera IV: navegacao ponto-a-ponto com desvio de obstaculos
 * ---------------------------------------------------------------------------
 * Disciplina: Engenharia de Software de Tempo Real  (CETELI/UFAM)
 *
 * v2 — MAQUINA DE ESTADOS (corrige o "minimo local" da v1).
 *
 *   Campo potencial puro TRAVA quando o alvo fica atras de um obstaculo
 *   (a forca que puxa anula a que empurra). A cura classica e PARAR de ser
 *   100% reativo e se COMPROMETER com um contorno (ideia dos algoritmos Bug
 *   e do wall-following). Por isso 3 estados:
 *
 *     NAVEGAR  -> vai pro alvo (controle P de rumo) + desvio reativo leve
 *     GIRAR    -> parede na frente: trava 1 direcao e gira ate limpar
 *     CONTORNAR-> anda reto uma distancia fixa p/ passar a caixa, e re-mira
 *
 *   Tudo num laco periodico (wb_robot_step) = tarefa periodica de tempo real.
 */

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/gps.h>
#include <webots/inertial_unit.h>

#include <stdio.h>
#include <math.h>

#define MAX_SPEED 47.6              /* rad/s — velocidade max das rodas         */
#define CRUISE   (0.25 * MAX_SPEED) /* velocidade de cruzeiro (~0.25 m/s)       */

#define GOAL_X   1.0                /* ponto B (bate com o disco vermelho)      */
#define GOAL_Y   1.0
#define GOAL_TOL 0.07               /* 7 cm: "chegou"                           */

/* limiares calibrados pela MEDICAO real (piso reflete -> base ~90-120):       */
#define IR_BASE  100.0              /* leitura de base em espaco aberto         */
#define IR_TRIG  210.0              /* >210: obstaculo de verdade -> GIRAR      */
#define IR_CLEAR 160.0              /* <160: frente limpou (volta p/ base)      */

#define K_HEADING 3.0               /* ganho P do rumo (go-to-goal)             */
#define K_AVOID   0.05              /* ganho do desvio reativo                  */
#define SPIN      6.0               /* rad/s p/ girar no lugar no estado GIRAR  */
#define PASS_DIST 0.30              /* m a percorrer no estado CONTORNAR        */

typedef enum { NAVEGAR, GIRAR, CONTORNAR } State;

static double clampd(double v, double lo, double hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}
static double wrap_pi(double a) {
  while (a >  M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}
static double maxd(double a, double b) { return a > b ? a : b; }

int main(void) {
  wb_robot_init();
  setvbuf(stdout, NULL, _IONBF, 0);  /* sem buffer: telemetria sai na hora     */
  FILE *logf = fopen("U:\\ufam\\eng_soft_rt\\controllers\\goto_avoid\\run.log", "w");
  const int dt = (int)wb_robot_get_basic_time_step();

  /* --- atuadores: rodas em modo VELOCIDADE --- */
  WbDeviceTag left  = wb_robot_get_device("left wheel motor");
  WbDeviceTag right = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left,  INFINITY);
  wb_motor_set_position(right, INFINITY);
  wb_motor_set_velocity(left,  0.0);
  wb_motor_set_velocity(right, 0.0);

  /* --- sensores IR do anel --- */
  const char *ir_names[8] = {
    "rear left infrared sensor",  "left infrared sensor",
    "front left infrared sensor", "front infrared sensor",
    "front right infrared sensor","right infrared sensor",
    "rear right infrared sensor", "rear infrared sensor"};
  WbDeviceTag ir[8];
  for (int i = 0; i < 8; i++) {
    ir[i] = wb_robot_get_device(ir_names[i]);
    wb_distance_sensor_enable(ir[i], dt);
  }

  /* --- localizacao: GPS + IMU --- */
  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, dt);
  WbDeviceTag imu = wb_robot_get_device("inertial unit");
  wb_inertial_unit_enable(imu, dt);

  State state = NAVEGAR;
  int    escape_dir = 1;     /* +1 = gira p/ esquerda, -1 = p/ direita         */
  double pass_x = 0, pass_y = 0;
  const char *snames[3] = {"NAV", "GIR", "CON"};

  printf("[goto_avoid v2] objetivo: ir ate (%.2f, %.2f)\n", GOAL_X, GOAL_Y);
  if (logf) { fprintf(logf, "[goto_avoid v2] objetivo: ir ate (%.2f, %.2f)\n", GOAL_X, GOAL_Y); fflush(logf); }
  double last_print = -1.0;

  /* ========================= LACO DE CONTROLE ========================= */
  while (wb_robot_step(dt) != -1) {

    /* ---- (1) PERCEPCAO ---- */
    const double *pos = wb_gps_get_values(gps);
    double yaw = wb_inertial_unit_get_roll_pitch_yaw(imu)[2];
    double dx = GOAL_X - pos[0], dy = GOAL_Y - pos[1];
    double dist = sqrt(dx * dx + dy * dy);

    if (dist < GOAL_TOL) {                       /* chegou -> para tudo */
      wb_motor_set_velocity(left, 0.0);
      wb_motor_set_velocity(right, 0.0);
      printf(">> CHEGUEI em (%.2f, %.2f) | t = %.1f s\n", pos[0], pos[1], wb_robot_get_time());
      if (logf) { fprintf(logf, ">> CHEGUEI em (%.2f, %.2f) | t = %.1f s\n", pos[0], pos[1], wb_robot_get_time()); fflush(logf); fclose(logf); logf = NULL; }
      break;
    }

    double f  = wb_distance_sensor_get_value(ir[3]);   /* frente       */
    double fl = wb_distance_sensor_get_value(ir[2]);   /* frente-esq   */
    double fr = wb_distance_sensor_get_value(ir[4]);   /* frente-dir   */
    double l  = wb_distance_sensor_get_value(ir[1]);   /* esquerda     */
    double r  = wb_distance_sensor_get_value(ir[5]);   /* direita      */
    double block_l = clampd((fl - IR_BASE) + 0.5 * (l - IR_BASE), 0, 1e9);
    double block_r = clampd((fr - IR_BASE) + 0.5 * (r - IR_BASE), 0, 1e9);
    double front   = maxd(f, maxd(fl, fr));            /* "tem parede a frente?" */

    double goal_dir = atan2(dy, dx);
    double err = wrap_pi(goal_dir - yaw);

    /* ---- (2) MAQUINA DE ESTADOS ---- */
    double forward = 0.0, steer = 0.0;
    switch (state) {

      case NAVEGAR:
        /* vai pro alvo (P de rumo) + desvio reativo leve */
        steer = K_HEADING * err + K_AVOID * (block_r - block_l);
        forward = CRUISE;
        forward *= (1.0 - 0.7 * clampd((f - IR_BASE) / (IR_TRIG - IR_BASE), 0, 1));
        forward *= clampd(1.0 - 0.5 * fabs(err), 0.3, 1.0);
        if (front > IR_TRIG) {                 /* parede! decide UMA vez e trava */
          escape_dir = (block_l <= block_r) ? +1 : -1;  /* gira p/ o lado livre */
          state = GIRAR;
        }
        break;

      case GIRAR:
        /* gira no lugar, MESMA direcao, ate a frente limpar de verdade */
        forward = 0.0;
        steer = escape_dir * SPIN;
        if (f < IR_CLEAR && fl < IR_CLEAR && fr < IR_CLEAR) {
          pass_x = pos[0];
          pass_y = pos[1];
          state = CONTORNAR;
        }
        break;

      case CONTORNAR:
        /* anda reto p/ passar a caixa; so desvio leve; re-gira se topar outra */
        forward = 0.7 * CRUISE;
        steer = K_AVOID * (block_r - block_l);
        if (front > IR_TRIG) {
          escape_dir = (block_l <= block_r) ? +1 : -1;
          state = GIRAR;
        } else {
          double tx = pos[0] - pass_x, ty = pos[1] - pass_y;
          if (sqrt(tx * tx + ty * ty) > PASS_DIST)
            state = NAVEGAR;                    /* contornou -> re-mira no alvo */
        }
        break;
    }

    /* ---- (3) CINEMATICA DIFERENCIAL ---- */
    double vl = clampd(forward - steer, -MAX_SPEED, MAX_SPEED);
    double vr = clampd(forward + steer, -MAX_SPEED, MAX_SPEED);
    wb_motor_set_velocity(left,  vl);
    wb_motor_set_velocity(right, vr);

    /* ---- telemetria (1x/s) ---- */
    double t = wb_robot_get_time();
    if (t - last_print >= 1.0) {
      last_print = t;
      printf("[%s] t=%4.1fs pos=(%+.2f,%+.2f) dist=%.2f err=%+.2f IR[L/F/R]=%3.0f/%3.0f/%3.0f rodas=%+.1f/%+.1f\n",
             snames[state], t, pos[0], pos[1], dist, err, fl, f, fr, vl, vr);
      if (logf) { fprintf(logf, "[%s] t=%4.1fs pos=(%+.2f,%+.2f) dist=%.2f err=%+.2f IR[L/F/R]=%3.0f/%3.0f/%3.0f rodas=%+.1f/%+.1f\n",
             snames[state], t, pos[0], pos[1], dist, err, fl, f, fr, vl, vr); fflush(logf); }
    }
  }

  wb_robot_cleanup();
  return 0;
}
