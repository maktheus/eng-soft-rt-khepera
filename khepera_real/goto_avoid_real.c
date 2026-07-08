/*
 * goto_avoid_real.c  --  Khepera IV (robo FISICO) -- biblioteca libkhepera
 * ---------------------------------------------------------------------------
 * Missao (B): PATRULHA "vai-e-volta" INFINITA por odometria (dead-reckoning),
 *             DESVIANDO de qualquer obstaculo (camada reativa por IR).
 *
 * IMPORTANTE: este NAO e um controlador do Webots. Usa a API real do robo
 *             (<khepera/khepera.h>), com motores/encoders/IR de hardware.
 *
 * Constantes CONFIRMADAS (manual K-Team / libkhepera):
 *   - encoder ....... 147,4 pulsos/mm  ->  PULSE_TO_MM = 1/147.4
 *   - velocidade .... 1 unidade kh4 ~= 0,678181 mm/s (KH4_SPEED_TO_MM_S)
 * Itens marcados [CALIBRAR]/[VERIFICAR] devem ser conferidos no proprio robo
 * (a telemetria ja imprime os IR e o estado para facilitar a calibracao).
 *
 * Compilar NO robo:  make
 *   (ou)  gcc -O2 -Wall -o goto_avoid_real goto_avoid_real.c -lkhepera -lpthread -lm
 * Rodar:             ./goto_avoid_real
 * Parar:             Ctrl-C  (faz parada segura dos motores)
 */

#include <khepera/khepera.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>

static knet_dev_t *dsPic;
static volatile sig_atomic_t running = 1;

/* ----------------------------- parametros ----------------------------- */
#define PULSE_TO_MM    (1.0/147.4)  /* CONFIRMADO: 147,4 pulsos por mm        */
#define SPEED_TO_MM_S  0.678181     /* CONFIRMADO: mm/s por unidade de speed   */
#define WHEELBASE_MM   105.4        /* [VERIFICAR] distancia entre as rodas    */

#define CRUISE         150          /* avanco ~100 mm/s  [CALIBRAR: comece baixo] */
#define TURN           120          /* velocidade de giro no lugar             */
#define PATROL_MM      1000.0       /* comprimento de cada perna (ida = volta) */
#define IR_OBSTACLE    500          /* [CALIBRAR] frente bloqueada (maior=+perto) */
#define IR_CLEAR       300          /* [CALIBRAR] frente liberada de novo      */
#define LOOP_US        30000        /* periodo do laco (30 ms)                 */

/* Ordem dos sensores IR ao redor do corpo. [VERIFICAR no robo pela telemetria:
   ponha a mao na frente e veja qual indice sobe].                            */
enum { IR_BL=0, IR_L=1, IR_FL=2, IR_F=3, IR_FR=4, IR_R=5, IR_BR=6, IR_B=7 };

static void on_sigint(int s){ (void)s; running = 0; }

/* combina 2 bytes (little-endian) no valor de 16 bits do sensor i */
static int ir_val(const char *buf, int i){
  return ((unsigned char)buf[2*i]) | (((unsigned char)buf[2*i+1]) << 8);
}

int main(int argc, char *argv[]){
  signal(SIGINT,  on_sigint);
  signal(SIGTERM, on_sigint);

  if (kb_init(argc, argv) < 0){ fprintf(stderr, "Erro: kb_init falhou\n"); return 1; }
  dsPic = knet_open("Khepera4:dsPic", KNET_BUS_I2C, 0, NULL);
  if (dsPic == NULL){ fprintf(stderr, "Erro: knet_open(dsPic) falhou\n"); return 1; }

  kh4_SetMode(kh4RegSpeed, dsPic);   /* controle por velocidade (PID interno)  */
  kh4_set_speed(0, 0, dsPic);

  char irbuf[256];
  int pL, pR, pL0, pR0;
  kh4_get_position(&pL0, &pR0, dsPic);

  enum { ANDAR, DESVIAR, MEIA_VOLTA } estado = ANDAR;
  double leg = 0.0;        /* distancia acumulada na perna atual (mm)         */
  double turn_acc = 0.0;   /* rotacao acumulada na manobra (rad)              */
  int    turn_dir = 1;     /* +1 = gira para a esquerda                       */
  long   tick = 0;

  printf("Patrulha vai-e-volta iniciada (%.0f mm por perna). Ctrl-C para parar.\n", PATROL_MM);

  while (running){
    /* ---------- (1) ODOMETRIA: delta dos encoders desde o ultimo ciclo ---- */
    kh4_get_position(&pL, &pR, dsPic);
    double dL = (pL - pL0) * PULSE_TO_MM;
    double dR = (pR - pR0) * PULSE_TO_MM;
    pL0 = pL; pR0 = pR;
    double dfwd = 0.5 * (dL + dR);              /* avanco medio (mm)           */
    double drot = (dR - dL) / WHEELBASE_MM;     /* rotacao (rad)              */

    /* ---------- (2) PERCEPCAO: IR frontais e laterais ---------- */
    kh4_proximity_ir(irbuf, dsPic);
    int irF  = ir_val(irbuf, IR_F);
    int irFL = ir_val(irbuf, IR_FL);
    int irFR = ir_val(irbuf, IR_FR);
    int irL  = ir_val(irbuf, IR_L);
    int irR  = ir_val(irbuf, IR_R);
    int front = irF; if (irFL > front) front = irFL; if (irFR > front) front = irFR;

    /* ---------- (3) MAQUINA DE ESTADOS ---------- */
    int sl = 0, sr = 0;
    switch (estado){
      case ANDAR:
        if (front > IR_OBSTACLE){                 /* obstaculo -> desviar      */
          turn_dir = (irFL + irL <= irFR + irR) ? +1 : -1;  /* vai p/ lado livre */
          estado = DESVIAR;
          sl = -turn_dir*TURN; sr = turn_dir*TURN;
        } else {
          if (dfwd > 0) leg += dfwd;              /* conta a perna             */
          if (leg >= PATROL_MM){                  /* fim da perna -> meia-volta */
            estado = MEIA_VOLTA; turn_acc = 0; turn_dir = +1;
            sl = -TURN; sr = TURN;
          } else { sl = CRUISE; sr = CRUISE; }    /* segue em frente           */
        }
        break;

      case DESVIAR:                               /* gira no lugar ate limpar  */
        sl = -turn_dir*TURN; sr = turn_dir*TURN;
        if (front < IR_CLEAR) estado = ANDAR;     /* retoma a perna            */
        break;

      case MEIA_VOLTA:                            /* gira ~180 graus           */
        turn_acc += fabs(drot);
        sl = -turn_dir*TURN; sr = turn_dir*TURN;
        if (turn_acc >= M_PI){ estado = ANDAR; leg = 0.0; }
        break;
    }

    kh4_set_speed(sl, sr, dsPic);

    /* ---------- (4) TELEMETRIA (~1 Hz) para calibracao ---------- */
    if (++tick % 33 == 0){
      const char *nm[3] = {"ANDAR  ", "DESVIAR", "M-VOLTA"};
      printf("[%s] perna=%4.0fmm  IR F/FL/FR=%4d/%4d/%4d  L/R=%4d/%4d  rodas=%4d/%4d\n",
             nm[estado], leg, irF, irFL, irFR, irL, irR, sl, sr);
      fflush(stdout);
    }
    usleep(LOOP_US);
  }

  /* ---------- parada segura ---------- */
  kh4_set_speed(0, 0, dsPic);
  kh4_SetMode(kh4RegIdle, dsPic);
  printf("\nMotores parados. Encerrado com seguranca.\n");
  return 0;
}
