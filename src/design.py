"""Projeto do controlador: realimentacao de estados + acao integral (LQR discreto), com
limitador preditivo de tensao/corrente e anti-windup por back-calculation."""
import numpy as np
from params import *
from plant import continuous3, c2d, expm

Q_DIAG = (0.0, 1e-3, 2000.0, 5e6)   # pesos [i, w, theta-r, z]
R_W = 1.0
T_TRACK = 0.01                      # s, constante de tempo do back-calculation (Tt)


def dlqr(Ad, Bd, Q, Rm, iters=200000):
    P = Q.copy()
    for _ in range(iters):
        K = np.linalg.solve(Rm + Bd.T @ P @ Bd, Bd.T @ P @ Ad)
        Pn = Q + Ad.T @ P @ (Ad - Bd @ K)
        if np.max(np.abs(Pn - P)) < 1e-12 * max(1, np.max(np.abs(P))):
            P = Pn; break
        P = Pn
    return np.linalg.solve(Rm + Bd.T @ P @ Bd, Bd.T @ P @ Ad)


def design(q=Q_DIAG, rw=R_W):
    A, Bm, Ed, C, D = continuous3()
    Ad, Bd = c2d(A, Bm, TS)
    # perturbacao constante discretizada
    _, Edd = c2d(A, Ed, TS)
    # aumentado: z[k+1] = z[k] + Ts*(theta-r)  (r tratado como deslocamento de theta)
    Aa = np.zeros((4, 4)); Aa[:3, :3] = Ad; Aa[3, 2] = TS; Aa[3, 3] = 1.0
    Ba = np.vstack([Bd, [[0.0]]])
    K = dlqr(Aa, Ba, np.diag(q), np.array([[rw]]))
    return dict(Ad=Ad, Bd=Bd, Edd=Edd, Aa=Aa, Ba=Ba, K=K.flatten())


class Joint:
    """Planta 'verdadeira' simulada com sub-passos exatos (ZOH fino) para ver a corrente intra-amostra."""
    def __init__(self, sub=20):
        A, Bm, Ed, C, D = continuous3()
        self.sub = sub
        self.Af, self.Bf = c2d(A, Bm, TS / sub)
        _, self.Ef = c2d(A, Ed, TS / sub)
        self.x = np.zeros(3)

    def step(self, u, tload):
        imax = 0.0
        for _ in range(self.sub):
            self.x = self.Af @ self.x + self.Bf.flatten() * u - self.Ef.flatten() * tload * 0 + self.Ef.flatten() * tload
            imax = max(imax, abs(self.x[0]))
        return imax


def limits(Ad, Bd, x, imax=IMAX_EFF):
    """Faixa de u que mantem |V|<=VMAX e |i[k+1]|<=imax (limitador preditivo)."""
    base = Ad[0, :] @ x
    lo = (-imax - base) / Bd[0, 0]; hi = (imax - base) / Bd[0, 0]
    return max(-VMAX, lo), min(VMAX, hi)


def simulate(d, r_final, T=0.6, tload=T_LOAD, antiwindup=True, vref=W_SS, tt=T_TRACK, return_u=False):
    """Simula posicao [rad no eixo do motor]. r e rampeada com taxa vref [rad/s]."""
    K = d['K']; Ad, Bd = d['Ad'], d['Bd']
    plant = Joint(); n = int(round(T / TS))
    r = 0.0; z = 0.0
    log = dict(t=[], th=[], w=[], i=[], u=[], r=[], iint=[], uraw=[])
    for k in range(n):
        r_target = r_final
        dr = np.clip(r_target - r, -vref * TS, vref * TS); r = r + dr
        x = plant.x.copy()
        xa = np.array([x[0], x[1], x[2] - r, z])
        # tensao nao saturada
        u_raw = -K @ xa
        lo, hi = limits(Ad, Bd, x)
        u = float(np.clip(u_raw, lo, hi))
        if antiwindup:
            z = z + TS * (x[2] - r) + (TS / tt) * (u_raw - u) / K[3]
        else:
            z = z + TS * (x[2] - r)
        ii = plant.step(u, tload)
        for kk, v in zip(log, (k * TS, plant.x[2], plant.x[1], plant.x[0], u, r, ii, u_raw)):
            log[kk].append(v)
    return {k: np.array(v) for k, v in log.items()}


def metrics(log, r_final, tol=0.02):
    th = log['th']; t = log['t']
    os_ = max(0.0, (th.max() - r_final) / r_final * 100)
    out = np.abs(th - r_final) > tol * r_final
    ts = t[np.where(out)[0][-1]] + TS if out.any() else 0.0
    return dict(overshoot_pct=os_, settling_s=ts, sse=abs(th[-1] - r_final),
                imax_cont=float(log['iint'].max()), umax=float(np.abs(log['u']).max()))


if __name__ == '__main__':
    d = design()
    print('K =', d['K'])
    print('polos MF discretos =', np.abs(np.linalg.eigvals(d['Aa'] - d['Ba'] @ d['K'][None, :])))
    for rf in (0.5, 2.0, 10.0, 50.0):
        for aw in (True, False):
            m = metrics(simulate(d, rf, antiwindup=aw), rf)
            print(rf, aw, {k: round(v, 3) for k, v in m.items()})
