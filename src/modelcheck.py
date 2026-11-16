"""Model checker de estados explicitos (BFS) para o sistema temporizado em tempo discreto (tick = 1 ms).
Equivale a um automato temporizado com relogios inteiros: ambiente (mao, rearme), sensor com atraso
nao-deterministico ate 20 ms, escalonador com ate J ciclos perdidos, supervisor e atuacao (corrente
leva Z ms para zerar). Propriedades: P1 (AG prazo 100 ms), P2 (rearme explicito), P3 (AF retomada)."""
from collections import deque
from math import ceil
import supervisor as S
from params import *

CYCLE = 2      # ms
D_MAX = 20     # ms atraso do sensor
DEADLINE = 100  # ms


def initial():
    # (sup, cnt, phase, hand, s_out, pend, skips, torque, zcd, mon, rearm_latched)
    return (S.RUN, 0, 0, 0, 0, None, 0, 1, 0, -1, 0)


def successors(st, J, Z, buggy=False, env_mask=None):
    """Lista de (estado_seguinte, info). info = dict(tid, consumed, sup_old, hand_seen)."""
    sup, cnt, phase, hand, s_out, pend, skips, torque, zcd, mon, rl = st
    if env_mask == 'no_new_hand':        # premissa de vivacidade: mao nao volta; operador mantem rearme
        hand_opts = [0] if hand == 0 else ([0, 1] if pend is None else [1])
        rearm_opts = [1]
    else:
        hand_opts = [hand] if pend is not None else [hand, 1 - hand]
        rearm_opts = [1] if rl else [0, 1]
    out = []
    for h2 in hand_opts:
        for r2 in rearm_opts:
            pend_opts = [(h2, d) for d in range(D_MAX + 1)] if h2 != hand else [pend]
            for p2 in pend_opts:
                s2 = s_out
                if p2 is not None:
                    tgt, rem = p2
                    if rem == 0:
                        s2, p2 = tgt, None
                    else:
                        p2 = (tgt, rem - 1)
                ph2 = (phase + 1) % CYCLE
                if phase == 0:
                    sched = [(False, skips)] + ([(True, skips + 1)] if skips < J else [])
                else:
                    sched = [(None, skips)]
                for skip, sk2 in sched:
                    if skip is False:
                        for rd in ([0, 1] if sup == S.RESUME else [0]):
                            sup2, cnt2, en, consumed, tid = S.step(sup, s2, r2, rd, cnt, buggy)
                            rl2 = 0 if consumed else r2
                            out.append(_finish(st, sup2, cnt2, ph2, h2, s2, p2, sk2, en, rl2, Z,
                                               dict(tid=tid, consumed=consumed, sup_old=sup, hand_seen=s2)))
                    else:
                        out.append(_finish(st, sup, cnt, ph2, h2, s2, p2, sk2, None, r2, Z,
                                           dict(tid=None, consumed=0, sup_old=sup, hand_seen=s2)))
    return out


def _finish(st, sup2, cnt2, ph2, h2, s2, p2, sk2, enable, rl, Z, info):
    _, _, _, hand, _, _, _, torque, zcd, mon, _ = st
    if enable == 1:
        torque, zcd = 1, 0
    elif enable == 0 and torque == 1 and zcd == 0:
        zcd = Z                               # comando de zero-corrente emitido neste ciclo
        if Z == 0:
            torque = 0
    elif zcd > 0:
        zcd -= 1
        if zcd == 0:
            torque = 0
    # monitor do prazo (mao continuamente presente)
    if h2 == 1:
        mon = 0 if hand == 0 else min(mon + 1, DEADLINE + 1)
    else:
        mon = -1
    return (sup2, cnt2, ph2, h2, s2, p2, sk2, torque, zcd, mon, rl), info


def check(J=1, Z=2, buggy=False):
    init = initial()
    seen = {init: None}; q = deque([init]); n_trans = 0
    worst = 0; cex = None; covered = set(); p2_viol = None
    while q:
        st = q.popleft()
        if st[9] > DEADLINE and st[7] == 1:        # P1 violada: mao presente > 100 ms com torque
            cex = st; break
        for nst, info in successors(st, J, Z, buggy):
            n_trans += 1
            if info['tid']:
                covered.add(info['tid'])
            if nst[0] == S.RESUME and info['sup_old'] != S.RESUME and not (
                    info['sup_old'] == S.STOP_WAIT and info['consumed'] and not info['hand_seen']):
                p2_viol = (st, nst)
            if st[7] == 1 and nst[7] == 0 and st[9] >= 0:
                worst = max(worst, nst[9])
            if nst not in seen:
                seen[nst] = st; q.append(nst)
    res = dict(J=J, Z=Z, buggy=buggy, states=len(seen), transitions=n_trans, p1_ok=cex is None,
               p2_ok=p2_viol is None, wcrt_ms=worst, covered=sorted(covered))
    if cex is not None:
        tr = []; s = cex
        while s is not None:
            tr.append(s); s = seen[s]
        res['counterexample'] = tr[::-1]
    return res


def check_liveness(J=1, Z=2):
    """P3: AG((sup==STOP_WAIT & rearme travado & mao ausente) -> AF(sup==RUN)) sob a premissa de que a
    mao nao volta. Ponto fixo de AF (CTL) no grafo do produto restrito ao ambiente 'no_new_hand'."""
    init = initial(); seen = {init}; q = deque([init])
    while q:
        st = q.popleft()
        for n, _ in successors(st, J, Z, False):
            if n not in seen:
                seen.add(n); q.append(n)
    ante = [s for s in seen if s[0] == S.STOP_WAIT and s[10] == 1 and s[3] == 0 and s[4] == 0]
    sub = {}; q = deque(ante)
    while q:
        st = q.popleft()
        if st in sub:
            continue
        sub[st] = [x[0] for x in successors(st, J, Z, False, env_mask='no_new_hand')]
        for n in sub[st]:
            if n not in sub:
                q.append(n)
    af = {s for s in sub if s[0] == S.RUN}; changed = True
    while changed:
        changed = False
        for s, ns in sub.items():
            if s not in af and ns and all(n in af for n in ns):
                af.add(s); changed = True
    bad = [s for s in ante if s not in af]
    return dict(antecedent_states=len(ante), subgraph_states=len(sub), live_ok=not bad, violations=len(bad))


def torque_zero_time_ms():
    """Z: tempo (ms) para |i| <= 2% de 8 A com comando de corrente-zero deadbeat (pior de varios casos)."""
    import numpy as np
    from design import design, Joint
    d = design(); Ad, Bd = d['Ad'], d['Bd']; worst = 0.0
    for i0, w0 in ((IMAX_EFF, W_SS), (-IMAX_EFF, W_SS), (3.74, W_SS), (IMAX_EFF, 0.0), (-IMAX_EFF, -W_SS)):
        p = Joint(); p.x = np.array([i0, w0, 0.0])
        for k in range(1, 20):
            x = p.x.copy(); u = float(np.clip(-(Ad[0, :] @ x) / Bd[0, 0], -VMAX, VMAX))
            p.step(u, T_LOAD)
            if abs(p.x[0]) <= 0.02 * IMAX:
                break
        worst = max(worst, k * TS * 1000)
    return worst


if __name__ == '__main__':
    Z = int(ceil(torque_zero_time_ms()))
    print('Z (ms) =', Z)
    r = check(J=1, Z=Z, buggy=True)
    print('buggy:', {k: v for k, v in r.items() if k not in ('counterexample', 'covered')}, len(r.get('counterexample', [])))
    r = check(J=1, Z=Z)
    print('fixed:', {k: v for k, v in r.items() if k != 'covered'})
    print(check_liveness(1, Z))
