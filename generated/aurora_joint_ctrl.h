/* ===== AURORA ROBOTICA - AuroraArm - junta do cotovelo =====
 * ARQUIVO GERADO AUTOMATICAMENTE por src/codegen.py - NAO EDITAR A MAO.
 * Modelo de origem : src/design.py (LQR+integral, Ts=2 ms), src/supervisor.py, src/controller.py
 * Hash dos parametros (SHA-256/16): 9b3923faee6f308a
 * Rastreabilidade (requisito -> funcao):
 *   REQ-CTRL-AA-001 erro nulo em regime ..... aa_joint_step(): estado integral z (acao integral)
 *   REQ-CTRL-AA-002 sobressinal <= 8% ....... ganhos AA_K_* (LQR), rampa de referencia
 *   REQ-CTRL-AA-003 acomodacao <= 300 ms .... ganhos AA_K_* (LQR)
 *   REQ-CTRL-AA-004 |V|<=48 V, |i|<=8 A ..... limitador preditivo (lo/hi) + anti-windup back-calculation
 *   REQ-SAFE-AA-001 torque zero <= 100 ms ... aa_joint_step(): estado STOP_HAND (corrente-zero deadbeat)
 *   REQ-SAFE-AA-002 sem retomada automatica . transicao T5 exige 'rearm' explicito
 *   REQ-SAFE-AA-003 retorno a trajetoria .... RESUME: rampa de r_cmd ate r_target, depois RUN
 *   REQ-SAFE-AA-004 v_efetuador <= 250 mm/s . AA_W_COLLAB: rampa + limitador preditivo de velocidade
 * Premissas de interface: rearm = nivel mantido ate o supervisor consumi-lo; hand = saida do scanner.
 */
#ifndef AURORA_JOINT_CTRL_H
#define AURORA_JOINT_CTRL_H

typedef enum { AA_RUN = 0, AA_STOP_HAND = 1, AA_STOP_WAIT_REARM = 2, AA_RESUME = 3 } aa_state_t;

typedef struct {
    aa_state_t sup;
    unsigned int cnt;      /* ciclos em RESUME */
    float z;               /* integrador do erro de posicao */
    float r_cmd;           /* referencia rampeada [rad, eixo do motor] */
    unsigned char last_tid;/* ultima transicao (cobertura): 0..9 = T0..T9 */
} aa_joint_t;

void  aa_joint_init(aa_joint_t *c);
/* Um ciclo de 2 ms. Retorna tensao de armadura [V]. */
float aa_joint_step(aa_joint_t *c, float theta, float omega, float i_meas, float r_target,
                    int hand, int rearm, int collab);
#endif
