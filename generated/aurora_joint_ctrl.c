/* ARQUIVO GERADO - ver aurora_joint_ctrl.h (hash 9b3923faee6f308a). Compilar com -ffp-contract=off. */
#include "aurora_joint_ctrl.h"

#define AA_K_I 0.191402391f
#define AA_K_W 0.706640065f
#define AA_K_E 68.2051468f
#define AA_K_Z 1879.56177f
#define AA_A_I_I 0.251248449f
#define AA_A_I_W -0.0774846822f
#define AA_B_I 0.911982417f
#define AA_A_W_I 0.221384808f
#define AA_A_W_W 0.980125368f
#define AA_B_W 0.225477785f
#define AA_TS 0.00200000009f
#define AA_VMAX 48.0f
#define AA_IMAX_EFF 7.5f
#define AA_W_NOM 450.0f
#define AA_W_COLLAB 67.8571396f
#define AA_KAW 0.200000003f
#define AA_W_FREE 1000000.0f
#define AA_RESUME_EPS 0.00100000005f
#define AA_RESUME_MAX 50u

static float aa_absf(float x) { return (x < 0.0f) ? -x : x; }
static float aa_min(float a, float b) { return (a < b) ? a : b; }
static float aa_max(float a, float b) { return (a > b) ? a : b; }

void aa_joint_init(aa_joint_t *c)
{
    c->sup = AA_RUN; c->cnt = 0u; c->z = 0.0f; c->r_cmd = 0.0f; c->last_tid = 0u;
}

float aa_joint_step(aa_joint_t *c, float theta, float omega, float i_meas, float r_target,
                    int hand, int rearm, int collab)
{
    float u, base, lim, e, u_raw, lo, hi, wl, wbase, dr;
    int en = 0;
    int resume_done = (aa_absf(c->r_cmd - r_target) < AA_RESUME_EPS) ? 1 : 0;

    /* ---- supervisor (gerado de supervisor.TRANSITIONS) ---- */
    switch (c->sup) {
    case AA_RUN:
        if (hand) { c->sup = AA_STOP_HAND; c->cnt = 0u; en = 0; c->last_tid = 1u; }
        else      { en = 1; c->last_tid = 0u; }
        break;
    case AA_STOP_HAND:
        if (!hand) { c->sup = AA_STOP_WAIT_REARM; c->last_tid = 2u; } else { c->last_tid = 3u; }
        break;
    case AA_STOP_WAIT_REARM:
        if (hand)       { c->sup = AA_STOP_HAND; c->last_tid = 4u; }
        else if (rearm) { c->sup = AA_RESUME; c->cnt = 0u; en = 1; c->last_tid = 5u; }
        else            { c->last_tid = 6u; }
        break;
    default: /* AA_RESUME */
        if (hand)        { c->sup = AA_STOP_HAND; c->cnt = 0u; c->last_tid = 7u; }
        else if (resume_done || (c->cnt + 1u >= AA_RESUME_MAX)) { c->sup = AA_RUN; c->cnt = 0u; en = 1; c->last_tid = 8u; }
        else             { c->cnt = c->cnt + 1u; en = 1; c->last_tid = 9u; }
        break;
    }

    base = AA_A_I_I * i_meas + AA_A_I_W * omega;
    if (en != 0) {
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
    } else {
        c->r_cmd = theta; c->z = 0.0f;
        u = aa_min(aa_max(-base / AA_B_I, -AA_VMAX), AA_VMAX);      /* torque zero: corrente-zero deadbeat */
    }
    return u;
}
