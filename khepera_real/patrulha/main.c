/*
 * main.c -- Khepera IV: IR DO PONTO A AO PONTO B desviando de obstaculos
 * ===========================================================================
 * OBJETIVO: navegar de A (origem, onde o robo liga) ate B (alvo em mm),
 * contornando qualquer obstaculo e VOLTANDO para a linha A->B -- ou seja,
 * um go-to-goal de verdade, nao um passeio reativo.
 *
 * ALGORITMO: Bug2 (Lumelsky & Stepanov, 1987) -- o padrao de competicao p/
 * "chegar num ponto com sensores locais":
 *   1) MIRA em B e anda reto pela LINHA A->B (controle de rumo por odometria).
 *   2) Bateu num obstaculo?  -> SEGUE A PAREDE (wall-following) contornando-o,
 *      MAS lembrando a linha A->B.
 *   3) LEAVE CONDITION: no instante em que RECRUZA a linha A->B num ponto MAIS
 *      PERTO de B do que o ponto onde bateu -> larga a parede e volta a mirar B.
 *   4) Repete ate chegar em B.
 * O passo 3 e o que faz o robo VOLTAR PRO EIXO em vez de sair vagando (era o
 * que faltava: sem alvo, o reativo puro nunca "volta pro meio").
 *
 * POSE (x,y,theta) por ODOMETRIA (encoders): 147,4 pulsos/mm, base 105,4 mm.
 * (Sem GPS como no Webots -> dead reckoning pelos encoders das rodas.)
 *
 * Convencao de giro (do driver: L=fwd+diff, R=fwd-diff):
 *     diff > 0  => vira DIREITA (theta DIMINUI)   diff < 0 => ESQUERDA (theta sobe)
 *
 * Uso:
 *   ./patrulha                 vai ate (700,0) mm e para
 *   ./patrulha --gx 1000 --gy 300   alvo B em (1000,300) mm
 *   ./patrulha --loop          vai-e-volta A<->B pra sempre (bom pra demo)
 *   ./patrulha --diag          so le sensores/encoders, nao move
 *   ./patrulha --teleop        controle manual pela serial (comandos v/s/q)
 *   ./patrulha --mission       recebe waypoints "goto X Y" pela serial, com a
 *                              odometria continua (usado pelo planejador A* da
 *                              interface sobre o mapa gerado)
 *   ./patrulha --slow          metade da velocidade linear -> contorno lento e
 *                              limpo (usado na geracao automatica de mapa A->B)
 *   ./patrulha -t 0.03         trim se o robo puxa pra um lado
 */
#include "khepera4.h"
#include "commandline.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <math.h>
#include <sys/time.h>
#include <sys/select.h>

/* ---------- odometria ---------- */
#define PULSES_PER_MM 147.4
#define WHEELBASE_MM  105.4

/* ---------- alvo / chegada ---------- */
#define GOAL_X_DEF    700.0    /* B padrao (mm a frente de A)                */
#define GOAL_Y_DEF    0.0

/* ---------- geral ---------- */
#define SPEED         12000
#define WAIT_US       40000
#define BATT_LOW_MV   6300

#include "controller_config.h"

static volatile sig_atomic_t running = 1;
static void on_sigint(int s){ (void)s; running = 0; }
static double clampd(double v,double lo,double hi){ return v<lo?lo:(v>hi?hi:v); }
static int    excess(int v){ int e=v-IR_BASE; return e>0?e:0; }
static double now_s(void){ struct timeval t; gettimeofday(&t,NULL); return t.tv_sec + t.tv_usec/1e6; }
static double wrap(double a){ while(a>M_PI)a-=2*M_PI; while(a<-M_PI)a+=2*M_PI; return a; }
#define IRP(i) (khepera4.infrared_proximity.sensor[i])

#include "controller_core.h"

static void leds(int r,int g,int b){
    khepera_dspic_write_register8(I2C_RGB_FL_R,r); khepera_dspic_write_register8(I2C_RGB_FL_G,g); khepera_dspic_write_register8(I2C_RGB_FL_B,b);
    khepera_dspic_write_register8(I2C_RGB_FR_R,r); khepera_dspic_write_register8(I2C_RGB_FR_G,g); khepera_dspic_write_register8(I2C_RGB_FR_B,b);
    khepera_dspic_write_register8(I2C_RGB_B_R,r);  khepera_dspic_write_register8(I2C_RGB_B_G,g);  khepera_dspic_write_register8(I2C_RGB_B_B,b);
}

static void diag(void){
    printf("DIAG: sensores + encoders (sem mover). Ctrl-C p/ sair.\n");
    while(running){
        khepera4_infrared_proximity(); khepera4_battery(); khepera4_drive_get_current_position();
        printf("IR[BL L FL F FR R BR B]=%d %d %d %d %d %d %d %d | chao[%d %d %d %d] | enc L/R=%d/%d | bat=%umV\n",
               IRP(0),IRP(1),IRP(2),IRP(3),IRP(4),IRP(5),IRP(6),IRP(7), IRP(8),IRP(9),IRP(10),IRP(11),
               khepera4.motor_left.current_position, khepera4.motor_right.current_position, khepera4.battery.voltage);
        fflush(stdout); usleep(400000);
    }
}

static int teleop_read_command(double *fwd, double *diff, int *quit){
    fd_set rfds;
    struct timeval tv;
    char line[96], op[16];
    double a=0, b=0;

    FD_ZERO(&rfds);
    FD_SET(STDIN_FILENO, &rfds);
    tv.tv_sec = 0;
    tv.tv_usec = 0;
    if(select(STDIN_FILENO + 1, &rfds, NULL, NULL, &tv) <= 0) return 0;
    if(!fgets(line, sizeof(line), stdin)) return 0;
    if(sscanf(line, " %15s %lf %lf", op, &a, &b) < 1) return 0;

    if(strcmp(op, "v") == 0 && sscanf(line, " %*s %lf %lf", &a, &b) == 2){
        *fwd = clampd(a, -1.0, 1.0);
        *diff = clampd(b, -DIFF_MAX, DIFF_MAX);
        return 1;
    }
    if(strcmp(op, "s") == 0 || strcmp(op, "stop") == 0){
        *fwd = 0.0;
        *diff = 0.0;
        return 1;
    }
    if(strcmp(op, "q") == 0 || strcmp(op, "quit") == 0){
        *fwd = 0.0;
        *diff = 0.0;
        *quit = 1;
        return 1;
    }
    return 0;
}

/* Modo missao: le do stdin (nao bloqueia) uma das linhas:
 *   goto X Y   -> novo alvo em (X,Y) mm            (retorna 1, preenche *gx,*gy)
 *   stop       -> para e aguarda proximo goto      (retorna 2)
 *   quit | q   -> encerra a missao                 (retorna 3)
 * Retorna 0 quando nao ha comando. Usado pela interface p/ enviar os
 * waypoints do planejador A* sobre o mapa, com a odometria continua. */
static int mission_read(double *gx, double *gy){
    fd_set rfds; struct timeval tv; char line[96], op[16]; double a=0,b=0;
    FD_ZERO(&rfds); FD_SET(STDIN_FILENO,&rfds); tv.tv_sec=0; tv.tv_usec=0;
    if(select(STDIN_FILENO+1,&rfds,NULL,NULL,&tv)<=0) return 0;
    if(!fgets(line,sizeof(line),stdin)) return 0;
    if(sscanf(line," %15s",op)<1) return 0;
    if(strcmp(op,"goto")==0 && sscanf(line," %*s %lf %lf",&a,&b)==2){ *gx=a; *gy=b; return 1; }
    if(strcmp(op,"stop")==0) return 2;
    if(strcmp(op,"quit")==0 || strcmp(op,"q")==0) return 3;
    return 0;
}

static void teleop(void){
    double target_fwd = 0.0, target_diff = 0.0;
    double last_cmd = now_s();
    long tick = 0;
    int batt_tick = 0;

    khepera4_drive_start();
    khepera4_drive_reset_position(); usleep(120000);
    khepera4_drive_get_current_position();
    int enc_l0 = khepera4.motor_left.current_position;
    int enc_r0 = khepera4.motor_right.current_position;
    double x=0, y=0, th=0;
    leds(35, 25, 0);
    printf("TELEOP: comandos pela serial: v <fwd -1..1> <diff -0.9..0.9>, s=para, q=sair.\n");
    printf("TELEOP: seguranca ativa: sem comando por 0.8s => motores parados.\n");
    fflush(stdout);

    while(running){
        int quit = 0;
        if(teleop_read_command(&target_fwd, &target_diff, &quit)){
            last_cmd = now_s();
            if(quit) break;
        }

        khepera4_infrared_proximity();
        khepera4_drive_get_current_position();
        if((batt_tick++ % 25)==0) khepera4_battery();

        int el = khepera4.motor_left.current_position;
        int er = khepera4.motor_right.current_position;
        double dl = (double)(el-enc_l0)/PULSES_PER_MM;
        double dr = (double)(er-enc_r0)/PULSES_PER_MM;
        enc_l0=el; enc_r0=er;
        double dc  = 0.5*(dl+dr);
        double dth = (dr-dl)/WHEELBASE_MM;
        x  += dc*cos(th+0.5*dth);
        y  += dc*sin(th+0.5*dth);
        th  = wrap(th+dth);

        int F=excess(IRP(3)), FL=excess(IRP(2)), FR=excess(IRP(4));
        int gmin=IRP(8); if(IRP(9)<gmin)gmin=IRP(9); if(IRP(10)<gmin)gmin=IRP(10); if(IRP(11)<gmin)gmin=IRP(11);
        double fwd = target_fwd, diff = target_diff;
        const char *guard = "OK";

        if(now_s() - last_cmd > 0.8){
            fwd = 0.0;
            diff = 0.0;
            guard = "STALE";
        }
        if(gmin < CLIFF_THR && fwd > 0.0){
            fwd = 0.0;
            guard = "BEIRADA";
        }

        khepera4_drive_set_speed_differential(SPEED, fwd, diff);

        if(++tick % 8 == 0){
            printf("[MANUAL] pose=(%.0f,%.0f,%.0fd) fwd=%+.2f diff=%+.2f guard=%s | F/FL/FR=%d/%d/%d | chao=%d bat=%umV\n",
                   x, y, th*180/M_PI, fwd, diff, guard, F+IR_BASE, FL+IR_BASE, FR+IR_BASE, gmin, khepera4.battery.voltage);
            fflush(stdout);
        }
        usleep(WAIT_US);
    }

    khepera4_drive_set_speed_differential(SPEED, 0.0, 0.0);
    khepera4_drive_stop();
    khepera4_drive_idle();
    leds(0,0,0);
    printf("\nTELEOP encerrado. Motores parados.\n");
    fflush(stdout);
}

int main(int argc, char *argv[]){
    commandline_init(); commandline_parse(argc, argv);
    signal(SIGINT,on_sigint); signal(SIGTERM,on_sigint); signal(SIGHUP,on_sigint);

    khepera4_init();
    if (commandline_option_provided("-d","--diag")){ diag(); return 0; }
    if (commandline_option_provided("-m","--teleop")){ teleop(); return 0; }

    double trim = commandline_option_value_float("-t","--trim",0.0);
    double gx   = commandline_option_value_float("-x","--gx",GOAL_X_DEF);
    double gy   = commandline_option_value_float("-y","--gy",GOAL_Y_DEF);
    int    loop = commandline_option_provided("-l","--loop");
    int    mission = commandline_option_provided("-M","--mission");  /* waypoints via stdin */
    int    slow = commandline_option_provided("-w","--slow");        /* contorno lento p/ mapear */
    double vscale = slow ? 0.5 : 1.0;                                 /* escala da velocidade linear */

    khepera4_drive_start();
    khepera4_drive_reset_position(); usleep(120000);
    khepera4_drive_get_current_position();
    int enc_l0 = khepera4.motor_left.current_position;
    int enc_r0 = khepera4.motor_right.current_position;

    KhepController ctrl;
    double x=0, y=0, th=0;               /* pose por odometria (mm, mm, rad) */

    khep_ctrl_init(&ctrl, gx, gy);
    int led_state=-1;
    int    have_goal = !mission;          /* modo missao comeca sem alvo (aguarda goto) */
    long tick=0; int batt_tick=0;
    double t_prev = now_s();

    leds(0,0,30);
    if(mission)
        printf("MISSAO: aguardando 'goto X Y' pela serial (stop/quit). Ctrl-C p/ parar.\n");
    else
        printf("A->B (Bug2, odometria). B=(%.0f,%.0f)mm loop=%d trim=%.3f. Ctrl-C p/ parar.\n", ctrl.bx,ctrl.by,loop,trim);

    while(running){
        double t_now=now_s(); double dt=t_now-t_prev; t_prev=t_now;
        if(dt<=0||dt>0.5) dt=WAIT_US/1e6;

        /* ---------- leitura de sensores + odometria ---------- */
        khepera4_infrared_proximity();
        khepera4_drive_get_current_position();
        if((batt_tick++ % 50)==0) khepera4_battery();

        int el = khepera4.motor_left.current_position;
        int er = khepera4.motor_right.current_position;
        double dl = (double)(el-enc_l0)/PULSES_PER_MM;   /* mm roda esq */
        double dr = (double)(er-enc_r0)/PULSES_PER_MM;   /* mm roda dir */
        enc_l0=el; enc_r0=er;
        double dc  = 0.5*(dl+dr);
        double dth = (dr-dl)/WHEELBASE_MM;               /* dir>esq => theta sobe (esq) */
        x  += dc*cos(th+0.5*dth);
        y  += dc*sin(th+0.5*dth);
        th  = wrap(th+dth);

        /* ---------- modo missao: recebe waypoints do planejador (A* no mapa) ---------- */
        if(mission){
            double ngx,ngy; int mc=mission_read(&ngx,&ngy);
            if(mc==1){                                   /* goto X Y: novo leg A(atual)->B */
                khep_ctrl_set_segment(&ctrl, x, y, ngx, ngy);
                have_goal=1;
                printf(">> GOTO (%.0f,%.0f)\n", ctrl.bx,ctrl.by); fflush(stdout);
            } else if(mc==2){                            /* stop: aguarda proximo goto */
                have_goal=0; printf(">> STOP\n"); fflush(stdout);
            } else if(mc==3){                            /* quit: encerra a missao */
                break;
            }
            if(!have_goal){
                khepera4_drive_set_speed_differential(SPEED,0,0);
                usleep(WAIT_US); continue;
            }
        }

        int rawF=IRP(3), rawFL=IRP(2), rawFR=IRP(4), rawL=IRP(1), rawR=IRP(5);
        int gmin=IRP(8); if(IRP(9)<gmin)gmin=IRP(9); if(IRP(10)<gmin)gmin=IRP(10); if(IRP(11)<gmin)gmin=IRP(11);
        KhepOutput co = khep_ctrl_step(&ctrl, x, y, th, dc, dt, t_now,
                                       rawF, rawFL, rawFR, rawL, rawR, gmin);

        if(co.entered_wall){
            printf(">> WALLF: parede a %s (hit d_goal=%.0f, herr=%.0fdeg)\n",
                   co.wall_left?"ESQUERDA":"DIREITA", co.d_goal, co.herr*180/M_PI); fflush(stdout);
        }
        if(co.left_wall){
            if(co.leave_by_goal)
                printf("<< alvo livre e progresso feito (d_goal=%.0f) -> GOAL\n", co.d_goal);
            else
                printf("<< recruzou a linha mais perto de B (d_goal=%.0f) -> GOAL\n", co.d_goal);
            fflush(stdout);
        }

        /* ---------- CHEGOU? ---------- */
        if(co.d_goal < ARRIVE_MM){
            khepera4_drive_set_speed_differential(SPEED,0,0);
            printf("== CHEGOU em (%.0f,%.0f)  pose=(%.0f,%.0f,%.0fdeg) ==\n", ctrl.bx,ctrl.by,x,y,th*180/M_PI); fflush(stdout);
            if(mission){        /* chegou no waypoint: aguarda o proximo goto */
                have_goal=0; khep_ctrl_set_segment(&ctrl, x, y, x, y);
                leds(0,40,0); usleep(200000); continue;
            }
            if(!loop) break;
            {   /* inverte o segmento */
                double old_ax=ctrl.ax, old_ay=ctrl.ay;
                khep_ctrl_set_segment(&ctrl, ctrl.bx, ctrl.by, old_ax, old_ay);
            }
            leds(0,40,0); usleep(600000); continue;
        }

        /* ---------- ANTI-QUEDA (prioridade maxima) ---------- */
        if(co.beirada){
            if(led_state!=3){ leds(40,0,0); led_state=3; }
            khepera4_drive_set_speed_differential(SPEED, co.fwd_s, co.diff_s);
            if(++tick%8==0){ printf("[BEIRADA] chao=%d\n",gmin); fflush(stdout); }
            usleep(WAIT_US); continue;
        }

        if(co.state==KHEP_GOAL){
            if(led_state!=0){ leds(0,30,0); led_state=0; }              /* verde: rumo a B */
        } else {
            if(led_state!=2){ leds(0,0,40); led_state=2; }             /* azul: contornando */
        }

        khepera4_drive_set_speed_differential(SPEED, co.fwd_s*vscale, co.diff_s + trim);

        if(++tick%15==0){
            printf("[%s%s] pose=(%.0f,%.0f,%.0fd) d_goal=%.0f d_line=%+.0f herr=%+.0fd | F/FL/FR=%d/%d/%d | chao=%d bat=%umV\n",
                   co.state==KHEP_GOAL?"GOAL ":"WALLF", co.state==KHEP_WALLF?(co.wall_left?"-E":"-D"):"",
                   x,y,th*180/M_PI, co.d_goal, co.d_line, co.herr*180/M_PI,
                   co.F+IR_BASE,co.FL+IR_BASE,co.FR+IR_BASE, gmin, khepera4.battery.voltage);
            if(khepera4.battery.voltage>0 && khepera4.battery.voltage<BATT_LOW_MV) printf("  !! BATERIA BAIXA !!\n");
            fflush(stdout);
        }
        usleep(WAIT_US);
    }

    khepera4_drive_stop(); khepera4_drive_idle(); leds(0,0,0);
    printf("\nMotores parados. Encerrado com seguranca.\n");
    return 0;
}
