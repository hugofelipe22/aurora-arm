"""Gerador de codigo C (float32, MISRA-friendly: sem malloc, sem recursao, sem libm) a partir do modelo
validado: ganhos/matrizes vem de design.design(), a maquina de estados vem de supervisor.TRANSITIONS e
a ordem das operacoes e a mesma de controller.Ctrl. Saida: generated/aurora_joint_ctrl.{h,c} + sil_main.c."""
import hashlib, os, re
import numpy as np
import supervisor as S
from params import *
from design import design, T_TRACK, Q_DIAG

OUT = os.path.join(os.path.dirname(__file__), '..', 'generated')


def f32(x):
    """Literal float32 C com 9 digitos significativos (round-trip exato)."""
    t = '%.9g' % np.float32(x)
    if not any(ch in t for ch in '.e'):
        t += '.0'
    return t + 'f'


def constants(d):
    Ad, Bd, K = d['Ad'], d['Bd'], d['K']
    return [
        ('AA_K_I', K[0]), ('AA_K_W', K[1]), ('AA_K_E', K[2]), ('AA_K_Z', K[3]),
        ('AA_A_I_I', Ad[0, 0]), ('AA_A_I_W', Ad[0, 1]), ('AA_B_I', Bd[0, 0]),
        ('AA_A_W_I', Ad[1, 0]), ('AA_A_W_W', Ad[1, 1]), ('AA_B_W', Bd[1, 0]),
        ('AA_TS', TS), ('AA_VMAX', VMAX), ('AA_IMAX_EFF', IMAX_EFF), ('AA_W_NOM', W_SS),
        ('AA_W_COLLAB', W_COLLAB), ('AA_KAW', TS / T_TRACK), ('AA_W_FREE', 1e6), ('AA_RESUME_EPS', 1e-3),
    ]


def generate():
    d = design(); cs = constants(d)
    digest = hashlib.sha256(repr([(n, float(np.float32(v))) for n, v in cs]).encode()).hexdigest()[:16]
    hdr = f'''/* ===== AURORA ROBOTICA - AuroraArm - junta do cotovelo =====
 * ARQUIVO GERADO AUTOMATICAMENTE por src/codegen.py - NAO EDITAR A MAO.
 * Modelo de origem : src/design.py (LQR+integral, Ts={TS*1000:g} ms), src/supervisor.py, src/controller.py
 * Hash dos parametros (SHA-256/16): {digest}
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

typedef enum {{ AA_RUN = 0, AA_STOP_HAND = 1, AA_STOP_WAIT_REARM = 2, AA_RESUME = 3 }} aa_state_t;

typedef struct {{
    aa_state_t sup;
    unsigned int cnt;      /* ciclos em RESUME */
    float z;               /* integrador do erro de posicao */
    float r_cmd;           /* referencia rampeada [rad, eixo do motor] */
    unsigned char last_tid;/* ultima transicao (cobertura): 0..9 = T0..T9 */
}} aa_joint_t;

void  aa_joint_init(aa_joint_t *c);
/* Um ciclo de 2 ms. Retorna tensao de armadura [V]. */
float aa_joint_step(aa_joint_t *c, float theta, float omega, float i_meas, float r_target,
                    int hand, int rearm, int collab);
#endif
'''
    consts = '\n'.join(f'#define {n} {f32(v)}' for n, v in cs)
    src = f'''/* ARQUIVO GERADO - ver aurora_joint_ctrl.h (hash {digest}). Compilar com -ffp-contract=off. */
#include "aurora_joint_ctrl.h"

{consts}
#define AA_RESUME_MAX {S.RESUME_MAX}u

static float aa_absf(float x) {{ return (x < 0.0f) ? -x : x; }}
static float aa_min(float a, float b) {{ return (a < b) ? a : b; }}
static float aa_max(float a, float b) {{ return (a > b) ? a : b; }}

void aa_joint_init(aa_joint_t *c)
{{
    c->sup = AA_RUN; c->cnt = 0u; c->z = 0.0f; c->r_cmd = 0.0f; c->last_tid = 0u;
}}

float aa_joint_step(aa_joint_t *c, float theta, float omega, float i_meas, float r_target,
                    int hand, int rearm, int collab)
{{
    float u, base, lim, e, u_raw, lo, hi, wl, wbase, dr;
    int en = 0;
    int resume_done = (aa_absf(c->r_cmd - r_target) < AA_RESUME_EPS) ? 1 : 0;

    /* ---- supervisor (gerado de supervisor.TRANSITIONS) ---- */
    switch (c->sup) {{
    case AA_RUN:
        if (hand) {{ c->sup = AA_STOP_HAND; c->cnt = 0u; en = 0; c->last_tid = 1u; }}
        else      {{ en = 1; c->last_tid = 0u; }}
        break;
    case AA_STOP_HAND:
        if (!hand) {{ c->sup = AA_STOP_WAIT_REARM; c->last_tid = 2u; }} else {{ c->last_tid = 3u; }}
        break;
    case AA_STOP_WAIT_REARM:
        if (hand)       {{ c->sup = AA_STOP_HAND; c->last_tid = 4u; }}
        else if (rearm) {{ c->sup = AA_RESUME; c->cnt = 0u; en = 1; c->last_tid = 5u; }}
        else            {{ c->last_tid = 6u; }}
        break;
    default: /* AA_RESUME */
        if (hand)        {{ c->sup = AA_STOP_HAND; c->cnt = 0u; c->last_tid = 7u; }}
        else if (resume_done || (c->cnt + 1u >= AA_RESUME_MAX)) {{ c->sup = AA_RUN; c->cnt = 0u; en = 1; c->last_tid = 8u; }}
        else             {{ c->cnt = c->cnt + 1u; en = 1; c->last_tid = 9u; }}
        break;
    }}

    base = AA_A_I_I * i_meas + AA_A_I_W * omega;
    if (en != 0) {{
        lim = (collab != 0 ? AA_W_COLLAB : AA_W_NOM) * AA_TS;
        dr = aa_min(aa_max(r_target - c->r_cmd, -lim), lim);
        c->r_cmd = c->r_cmd + dr;
        e = theta - c->r_cmd;
        u_raw = -(AA_K_I * i_meas + AA_K_W * omega + AA_K_E * e + AA_K_Z * c->z);
        lo = aa_max(-AA_VMAX, (-AA_IMAX_EFF - base) / AA_B_I);
        hi = aa_min(AA_VMAX, (AA_IMAX_EFF - base) / AA_B_I);
        wl = (collab != 0) ? AA_W_COLLAB : AA_W_FREE;
        wbase = AA_A_W_I * i_meas + AA_A_W_W * omega;
        lo = aa_max(lo, (-wl - wbase) / AA_B_W);
        hi = aa_min(hi, (wl - wbase) / AA_B_W);
        u = aa_min(aa_max(u_raw, lo), hi);
        c->z = c->z + AA_TS * e + AA_KAW * (u_raw - u) / AA_K_Z;   /* anti-windup (back-calculation) */
    }} else {{
        c->r_cmd = theta; c->z = 0.0f;
        u = aa_min(aa_max(-base / AA_B_I, -AA_VMAX), AA_VMAX);      /* torque zero: corrente-zero deadbeat */
    }}
    return u;
}}
'''
    main = '''/* Harness SIL (gerado): le linhas "theta omega i r hand rearm collab", imprime "u sup tid" (u com 9 digitos significativos). */
#include <stdio.h>
#include "aurora_joint_ctrl.h"
int main(void)
{
    aa_joint_t c; double th, w, i, r; int h, ra, co;
    aa_joint_init(&c);
    while (scanf("%lf %lf %lf %lf %d %d %d", &th, &w, &i, &r, &h, &ra, &co) == 7) {
        float u = aa_joint_step(&c, (float)th, (float)w, (float)i, (float)r, h, ra, co);
        printf("%.9g %d %d\\n", (double)u, (int)c.sup, (int)c.last_tid);
    }
    return 0;
}
'''
    os.makedirs(OUT, exist_ok=True)
    for name, txt in (('aurora_joint_ctrl.h', hdr), ('aurora_joint_ctrl.c', src), ('sil_main.c', main)):
        with open(os.path.join(OUT, name), 'w', newline='\n') as fh:
            fh.write(txt)
    return digest


def parse_constants(path=None):
    """Le de volta as constantes do .c gerado (usado pelo espelho float32 para provar que o espelho usa
    exatamente os dados do codigo gerado)."""
    path = path or os.path.join(OUT, 'aurora_joint_ctrl.c')
    txt = open(path).read()
    return {m.group(1): np.float32(m.group(2)) for m in re.finditer(r'#define (AA_\w+) (-?[0-9.e+-]+)f', txt)}


if __name__ == '__main__':
    print('gerado, hash', generate())
