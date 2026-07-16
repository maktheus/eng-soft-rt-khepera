/*
 * controller_config.h -- parametros do navegador tactil do Khepera IV.
 *
 * Valores escolhidos a partir das leituras do robo real:
 *   livre: normalmente abaixo de 20
 *   toque: 175..1023 nos sensores proximos ao contato
 */
#ifndef KHEPERA_CONTROLLER_CONFIG_H
#define KHEPERA_CONTROLLER_CONFIG_H

/* Alvo e seguranca. */
#define ARRIVE_MM                 90.0
#define CLIFF_THR                 500
#define CLIFF_CONFIRM_TICKS         3
#define DIFF_MAX                  0.55

/* IR bruto. Nao existe mais IR_BASE nem distancia IR ficticia. */
#define IR_NEAR_RAW               55
#define IR_CONTACT_RAW            180
#define IR_HARD_CONTACT_RAW       400
#define IR_CLEAR_RAW              40
#define IR_SIDE_BIAS_RAW          35
#define CONTACT_CONFIRM_TICKS     3

/* Navegacao normal ate o alvo. Os dois motores permanecem para a frente. */
#define GOAL_FWD                  0.32
#define GOAL_FWD_MIN              0.13
#define GOAL_KP_HEAD              0.22
#define GOAL_DIFF_MAX             0.08
#define GOAL_DEADBAND_RAD         0.10
#define GOAL_SLOW_ANGLE_RAD       0.80

/* Resposta ao toque, medida por odometria em vez de temporizadores. */
#define BACKOFF_MM                85.0
#define BACKOFF_FWD              -0.22
#define BACKOFF_TIMEOUT_S          2.0
#define TANGENT_ANGLE_RAD          1.40
#define TURN_TOL_RAD               0.14
#define TURN_FALLBACK_RAD          0.32
#define TURN_FWD                   0.14
#define TURN_KP_HEAD               0.45
#define TURN_DIFF_MAX              0.18
#define TURN_TIMEOUT_S             4.0
#define TURN_ROTATION_LIMIT_RAD     2.60

/* Contorno sem visao continua da parede. */
#define BYPASS_BASE_MM             320.0
#define BYPASS_GROW_MM             140.0
#define BYPASS_MAX_SEGMENT_MM      700.0
#define BYPASS_FWD                  0.24
#define BYPASS_KP_HEAD              0.65
#define BYPASS_DIFF_MAX             0.20
#define BYPASS_INWARD_RAD           0.06
#define BYPASS_AWAY_RAD             0.65
#define BYPASS_TOUCH_SCALE          0.52

/* Teste do caminho para B depois de cada trecho tangencial. */
#define PROBE_CLEAR_MM              280.0
#define PROBE_FWD                   0.22
#define PROBE_TIMEOUT_S              6.0
#define PROGRESS_MM                  80.0

/* Memoria tatil e recuperacao. */
#define CONTACT_MAP_CAP             48
#define CONTACT_MAP_MERGE_MM        90.0
#define LOOP_RADIUS_MM              150.0
#define SIDE_STALL_CONTACTS            4
#define SIDE_LOOP_CONTACTS             3
#define LATERAL_PROGRESS_STEP_MM    120.0
#define STUCK_TIMEOUT_S               1.8
#define MOTION_EPS_MM                 0.30

/* Suavizacao dos comandos, em unidades normalizadas por segundo. */
#define FWD_SLEW_PER_S                2.8
#define DIFF_SLEW_PER_S               3.8

/* O raycast simulado parte do centro: 70 mm de raio + 12 mm de alcance. */
#define SIM_IR_FREE_RAW              10
#define SIM_IR_RANGE_MM              82.0
#define SIM_IR_EXCESS_MAX           900.0

#endif
