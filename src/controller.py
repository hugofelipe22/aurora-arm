"""Modelo de referencia do controlador da junta (supervisor + posicao). Parametrizado por dtype:
float64 = modelo de referencia; float32 = espelho bit-a-bit do codigo C gerado (ordem das operacoes
identica a de generated/aurora_joint_ctrl.c, sem FMA)."""
import numpy as np
import supervisor as S
from params import *
from design import design, T_TRACK, Joint


class Ctrl:
    def __init__(self, d=None, dtype=np.float64, buggy=False, antiwindup=True):
        d = d or design()
        f = self.f = dtype
        self.K = [f(k) for k in d['K']]
        self.a0, self.a1 = f(d['Ad'][0, 0]), f(d['Ad'][0, 1])
        self.b0 = f(d['Bd'][0, 0])
        self.c0, self.c1, self.e0 = f(d['Ad'][1, 0]), f(d['Ad'][1, 1]), f(d['Bd'][1, 0])
        self.wfree = f(1e6)
        self.ts, self.vmax, self.imax = f(TS), f(VMAX), f(IMAX_EFF)
        self.wss, self.wcol = f(W_SS), f(W_COLLAB)
        self.kaw = f(TS / T_TRACK)
        self.buggy, self.aw = buggy, antiwindup
        self.sup, self.cnt, self.z, self.r = S.RUN, 0, f(0), f(0)
        self.last_tid = 'T0'

    def step(self, theta, omega, i, r_t, hand, rearm, collab):
        f = self.f
        theta, omega, i, r_t = f(theta), f(omega), f(i), f(r_t)
        resume_done = abs(self.r - r_t) < f(1e-3)
        self.sup, self.cnt, en, _, self.last_tid = S.step(self.sup, hand, rearm, resume_done, self.cnt, self.buggy)
        base = self.a0 * i + self.a1 * omega
        if en:
            vref = self.wcol if collab else self.wss
            lim = vref * self.ts
            dr = min(max(r_t - self.r, -lim), lim)
            self.r = self.r + dr
            e = theta - self.r
            u_raw = -(self.K[0] * i + self.K[1] * omega + self.K[2] * e + self.K[3] * self.z)
            lo = max(-self.vmax, (-self.imax - base) / self.b0)
            hi = min(self.vmax, (self.imax - base) / self.b0)
            wl = self.wcol if collab else self.wfree
            wbase = self.c0 * i + self.c1 * omega
            lo = max(lo, (-wl - wbase) / self.e0)
            hi = min(hi, (wl - wbase) / self.e0)
            u = min(max(u_raw, lo), hi)
            if self.aw:
                self.z = self.z + self.ts * e + self.kaw * (u_raw - u) / self.K[3]
            else:
                self.z = self.z + self.ts * e
        else:                                   # parada de seguranca: corrente-zero (deadbeat)
            self.r = theta
            self.z = f(0)
            u = min(max(-base / self.b0, -self.vmax), self.vmax)
        return float(u)


def run_closed_loop(r_final, T=0.6, tload=T_LOAD, collab=False, antiwindup=True, ctrl=None):
    """Malha fechada: Ctrl + planta (sub-passos). Retorna logs e entradas aplicadas ao controlador."""
    c = ctrl or Ctrl(antiwindup=antiwindup)
    p = Joint(); n = int(round(T / TS)); log = dict(t=[], th=[], w=[], i=[], u=[], r=[], ii=[], inp=[])
    for k in range(n):
        x = p.x.copy()
        u = c.step(x[2], x[1], x[0], r_final, 0, 0, collab)
        ii = p.step(u, tload)
        log['inp'].append((x[2], x[1], x[0], r_final, 0, 0, int(collab)))
        for kk, v in zip(('t', 'th', 'w', 'i', 'u', 'r', 'ii'), (k * TS, p.x[2], p.x[1], p.x[0], u, float(c.r), ii)):
            log[kk].append(v)
    return {k: np.array(v) for k, v in log.items()}
