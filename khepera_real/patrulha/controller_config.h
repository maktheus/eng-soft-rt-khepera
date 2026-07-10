/*
 * controller_config.h -- parametros compartilhados pelo robo e simulador.
 *
 * A interface web pode editar estes #defines no robo antes de recompilar.
 * O visualizador OpenGL tambem inclui este arquivo pelo controller_core.h.
 */
#ifndef KHEPERA_CONTROLLER_CONFIG_H
#define KHEPERA_CONTROLLER_CONFIG_H

/* ---------- chegada ---------- */
#define ARRIVE_MM     90.0     /* raio de chegada em B                       */

/* ---------- rumo (go-to-goal) ---------- */
#define KP_HEAD       1.2      /* ganho de rumo (erro rad -> diff)           */
#define HEAD_SLOW     1.2      /* rad de erro p/ (quase) girar parado        */

/* ---------- rastreamento da linha: ponto de controle virtual P ---------- */
#define USE_POINT_P   1        /* 1 = ponto-P (segue a linha); 0 = mira em B */
#define LOOKAHEAD_MM  120.0    /* "d": distancia do ponto P                  */
#define K_CT          1.0      /* ganho proporcional do erro lateral         */
#define KI_CT         0.0015   /* ganho integral do erro lateral             */
#define CT_INT_MAX    250.0    /* limite anti-windup do integrador           */

/* ---------- IR (em "excesso" = leitura - IR_BASE) ---------- */
#define IR_BASE       120
#define FRONT_SPAN    450.0
#define FRONT_BLOCK   200      /* frente travada -> entra em WALLF           */
#define FRONT_CRIT    320      /* quina interna -> pivota p/ longe           */
#define FRONT_CLEAR   90       /* frente livre p/ poder largar a parede      */
#define K_AVOID       0.0028   /* desvio lateral ainda no estado GOAL        */

/* ---------- seguimento de parede ---------- */
#define WALL_TARGET   150      /* distancia-alvo do lado seguido (excesso)   */
#define WALL_LOST     45       /* abaixo disso considera quina externa       */
#define WALL_MIN_MM   180.0    /* minimo de contorno antes de relargar linha */
#define WALL_NO_PROGRESS_MM 1400.0 /* inverte contorno se ficar orbitando     */
#define KP_WALL       0.0045
#define STEER_HARD    0.75

/* ---------- leave condition (Bug2) ---------- */
#define LINE_EPS      60.0     /* mm: "esta sobre a linha A->B"              */
#define PROGRESS_MM   40.0     /* precisa estar mais perto de B que no hit   */

/* ---------- seguranca / filtro ---------- */
#define TURN_SPEED    0.5
#define DIFF_MAX      0.90
#define ALPHA         0.4
#define CLIFF_THR     500      /* IR de chao abaixo disso = beirada          */

#endif
