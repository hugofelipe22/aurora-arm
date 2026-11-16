"""Co-simulacao malha fechada controlador + planta com eventos de ambiente (mao, rearme), atraso do sensor
e ciclos perdidos por jitter. Usada pelos testes, pelo SIL e pela validacao cruzada do modelo temporizado."""
import numpy as np
from params import *
from design import Joint
import supervisor as S

I_ZERO = 0.02 * IMAX   # |i| <= 0,16 A  => "torque zerado"
W_HOLD = 5.0           # rad/s: velocidade abaixo da qual o freio de retencao imobiliza o eixo


def scenario(ctrl, T, r_of_t, hand_of_t=lambda t: 0, rearm_of_t=lambda t: 0, collab_of_t=lambda t: 0,
             delay=T_SENSOR_MAX, skip=(), tload=T_LOAD, x0=None, brake=True):
    p = Joint()
    if x0 is not None:
        p.x = np.array(x0, dtype=float)
    n = int(round(T / TS)); u_prev = 0.0
    keys = ('t', 'th', 'w', 'i', 'ii', 'u', 'sup', 'hand', 'seen', 'r')
    L = {k: [] for k in keys}; inp = []
    for k in range(n):
        t = k * TS
        seen = int(hand_of_t(t - delay)) if t - delay >= 0 else 0
        x = p.x.copy()
        args = (x[2], x[1], x[0], r_of_t(t), seen, int(rearm_of_t(t)), int(collab_of_t(t)))
        if k in skip:
            u = u_prev                               # ciclo perdido: o driver mantem o ultimo comando (ZOH)
        else:
            u = ctrl.step(*args); inp.append(args)
        u_prev = u
        braked = brake and ctrl.sup in (S.STOP_HAND, S.STOP_WAIT) and abs(p.x[0]) <= I_ZERO and abs(p.x[1]) <= W_HOLD
        if braked:                                    # freio de retencao fail-safe (premissa P-BRK): so segura abaixo de W_HOLD
            p.x[1] = 0.0
        ii = p.step(u, 0.0 if braked else tload)
        vals = (t, p.x[2], p.x[1], p.x[0], ii, u, ctrl.sup, hand_of_t(t), seen, float(ctrl.r))
        for kk, v in zip(keys, vals):
            L[kk].append(v)
    out = {k: np.array(v) for k, v in L.items()}
    out['inp'] = inp
    return out


def torque_zero_after(log, t0):
    """Primeiro instante >= t0 a partir do qual |i| <= I_ZERO para sempre (ate o fim do log). None se nunca."""
    ok = np.abs(log['i']) <= I_ZERO
    idx = np.where(~ok)[0]
    last_bad = idx[-1] if len(idx) else -1
    if last_bad + 1 >= len(ok):
        return None
    return max(t0, log['t'][last_bad + 1])
