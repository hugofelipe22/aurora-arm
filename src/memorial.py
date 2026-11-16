"""Memorial de calculo executavel: reproduz os itens (a)-(e) do enunciado e gera figuras e results/memorial.json.
Executar: python src/memorial.py"""
import json, os, sys
from math import ceil
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch
from params import *
from plant import continuous2, continuous3, c2d, ctrb, obsv
from design import design, metrics
from controller import Ctrl, run_closed_loop
import supervisor as S
import modelcheck, sim, sil

ROOT = os.path.join(os.path.dirname(__file__), '..')
FIG = os.path.join(ROOT, 'figs'); os.makedirs(FIG, exist_ok=True)
M = {}
plt.rcParams.update({'figure.dpi': 130, 'axes.grid': True, 'grid.alpha': .3, 'font.size': 9})


def item_a():
    A, B_, C, D = continuous2()
    ev, _ = np.linalg.eig(A)
    ev = np.sort(ev.real)
    tau = -1 / ev
    Co, Ob = ctrb(A, B_), obsv(A, C)
    A3, B3, E3, C3, _ = continuous3()
    M['a'] = dict(A=A.tolist(), B=B_.tolist(), eig=ev.tolist(), tau_fast_ms=float(tau[0] * 1e3), tau_slow_ms=float(tau[1] * 1e3),
                  tau_e_uncoupled_ms=L / R * 1e3, tau_m_uncoupled_s=J / B, tau_m_backemf_ms=J * R / (R * B + KT * KE) * 1e3,
                  separation=float(tau[1] / tau[0]), ctrb=Co.tolist(), rank_ctrb=int(np.linalg.matrix_rank(Co)),
                  det_ctrb=float(np.linalg.det(Co)), obsv=Ob.tolist(), rank_obsv=int(np.linalg.matrix_rank(Ob)),
                  det_obsv=float(np.linalg.det(Ob)), rank_ctrb3=int(np.linalg.matrix_rank(ctrb(A3, B3))),
                  rank_obsv3_theta=int(np.linalg.matrix_rank(obsv(A3, C3))))


def item_b():
    iss = (B * W_SS + T_LOAD) / KT; vss = R * iss + KE * W_SS
    w_max = (VMAX - R * T_LOAD / KT) / (KE + R * B / KT)
    M['b'] = dict(iss=iss, vss=vss, folga_V=VMAX - vss, folga_pct=(VMAX - vss) / VMAX * 100, folga_i=IMAX - iss,
                  folga_i_pct=(IMAX - iss) / IMAX * 100, w_max_carga=w_max, R_iss=R * iss, ke_w=KE * W_SS,
                  i_stall=VMAX / R, torque_estatico_junta=2 * 9.81 * R_EFF / N, torque_estatico_junta_N=2 * 9.81 * R_EFF)


def item_c():
    tm = M['a']['tau_slow_ms']; te = M['a']['tau_fast_ms']; d = design()
    cl = np.abs(np.linalg.eigvals(d['Aa'] - d['Ba'] @ d['K'][None, :]))
    tau_cl = -TS / np.log(cl) * 1e3
    M['c'] = dict(Ts_ms=TS * 1e3, tm_over_Ts=tm / (TS * 1e3), te_over_Ts=te / (TS * 1e3), fs=1 / TS,
                  cl_poles=cl.tolist(), cl_tau_ms=tau_cl.tolist(), samples_per_cl_tau=(tau_cl / (TS * 1e3)).tolist(),
                  K=d['K'].tolist(), Ad=d['Ad'].tolist(), Bd=d['Bd'].tolist(), Ad_eigs=np.abs(np.linalg.eigvals(d['Ad'])).tolist())


def item_d():
    out = {}
    Z = int(ceil(modelcheck.torque_zero_time_ms())); out['Z_ms'] = Z
    bug = modelcheck.check(J=1, Z=Z, buggy=True)
    cex = bug['counterexample']
    ev = []
    for a, b in zip(cex[:-1], cex[1:]):
        if a[0] != b[0] or a[3] != b[3]:
            ev.append(dict(t_ms=len(ev) and cex.index(b), sup=S.NAMES[b[0]], hand=b[3], s_out=b[4], torque=b[7]))
    out['bug'] = dict(states=bug['states'], len_cex=len(cex), events=ev, final=dict(sup=S.NAMES[cex[-1][0]], mon=cex[-1][9], torque=cex[-1][7]))
    rows = []
    for J in (0, 1, 2, 3):
        r = modelcheck.check(J=J, Z=Z)
        rows.append(dict(J=J, states=r['states'], transitions=r['transitions'], p1_ok=r['p1_ok'], p2_ok=r['p2_ok'], wcrt_ms=r['wcrt_ms']))
    out['fixed'] = rows
    out['live'] = modelcheck.check_liveness(1, Z)
    out['analytic_J1'] = 20 + 2 + 2 * 1 + Z
    # varredura de malha fechada: atraso x ciclos perdidos (pior fase)
    th = 0.2; cases = {}
    for nskip in (0, 1, 2, 5, 10, 20, 30, 36, 37, 38, 39, 40):
        worst = 0
        for d_ms in range(0, 21):
            for ph in (0.0, 0.0005, 0.001, 0.0015):
                delay = d_ms / 1000; t0 = th + ph
                k = int(np.ceil((t0 + delay) / TS)); sk = tuple(range(k, k + nskip))
                l = sim.scenario(Ctrl(), 0.5, lambda t: 450.0 * t + 50, lambda t: 1 if t >= t0 else 0, delay=delay, skip=sk,
                                 x0=[3.735, 450.0, 0.0])
                tz = sim.torque_zero_after(l, t0)
                worst = max(worst, (tz - t0) if tz is not None else 1.0)
        cases[nskip] = worst * 1000
    out['cl_sweep_ms'] = cases
    out['max_lost_cycles'] = max(k for k, v in cases.items() if v <= 100)
    M['d'] = out
    # figura: resposta vs atraso
    d_ms = np.arange(0, 21); resp = []
    for d in d_ms:
        w = 0
        for ph in (0.0, 0.0005, 0.001, 0.0015):
            t0 = 0.2 + ph; delay = d / 1000; k = int(np.ceil((t0 + delay) / TS)); sk = (k,)
            l = sim.scenario(Ctrl(), 0.4, lambda t: 450.0 * t + 50, lambda t: 1 if t >= t0 else 0, delay=delay, skip=sk, x0=[3.735, 450.0, 0.0])
            w = max(w, sim.torque_zero_after(l, t0) - t0)
        resp.append(w * 1000)
    fig, ax = plt.subplots(figsize=(5.6, 3))
    ax.plot(d_ms, resp, 'o-', color='#1f5fa8', label='malha fechada (1 ciclo perdido)')
    ax.axhline(100, color='#c0392b', ls='--', label='prazo 100 ms')
    ax.set(xlabel='atraso do sensor [ms]', ylabel='tempo até torque zero [ms]', ylim=(0, 110)); ax.legend(loc='center right')
    fig.tight_layout(); fig.savefig(os.path.join(FIG, 'prazo_vs_atraso.png')); plt.close(fig)


def item_e():
    M['e'] = sil.equivalence()
    # soma dos termos antes do cancelamento (condicionamento) -> limite esperado do erro float32
    ref = sil.stimulus(); d = design(); K = d['K']; c = Ctrl(); worst = 0.0
    for a in ref['inp']:
        c.step(*a)
        e = a[0] - float(c.r); terms = abs(K[0] * a[2]) + abs(K[1] * a[1]) + abs(K[2] * e) + abs(K[3] * float(c.z))
        worst = max(worst, terms)
    M['e']['sum_abs_terms_max'] = worst
    M['e']['err_over_eps_terms'] = M['e']['max_abs_err_V'] / (sil.EPS32 * worst)


def figures():
    # 1) degraus
    fig, axs = plt.subplots(2, 2, figsize=(8, 5))
    for col, rf in enumerate((2.0, 10.0)):
        l = run_closed_loop(rf, T=0.4)
        axs[0, col].plot(l['t'] * 1e3, l['th'], color='#1f5fa8', label='θ'); axs[0, col].plot(l['t'] * 1e3, l['r'], '--', color='gray', label='r (rampeada)')
        axs[0, col].axhline(rf * 1.08, color='#c0392b', ls=':', lw=1); axs[0, col].axhline(rf * 1.02, color='#888', ls=':', lw=1); axs[0, col].axhline(rf * .98, color='#888', ls=':', lw=1)
        axs[0, col].axvline(300, color='#c0392b', ls=':', lw=1)
        axs[0, col].set(title=f'degrau {rf:g} rad (eixo do motor)', ylabel='posição [rad]'); axs[0, col].legend(loc='lower right')
        axs[1, col].plot(l['t'] * 1e3, l['u'], color='#1f5fa8', label='V [V]'); axs[1, col].plot(l['t'] * 1e3, l['i'] * 5, color='#d35400', label='i×5 [A]')
        axs[1, col].axhline(48, color='#c0392b', ls=':'); axs[1, col].axhline(8 * 5, color='#c0392b', ls=':')
        axs[1, col].set(xlabel='t [ms]', ylabel='tensão / corrente'); axs[1, col].legend(loc='upper right')
    fig.tight_layout(); fig.savefig(os.path.join(FIG, 'degraus.png')); plt.close(fig)
    # 2) anti-windup
    fig, axs = plt.subplots(1, 2, figsize=(8, 3))
    for aw, col, lab in ((True, '#1f5fa8', 'com anti-windup'), (False, '#c0392b', 'sem anti-windup')):
        l = run_closed_loop(2.0, T=0.6, antiwindup=aw)
        axs[0].plot(l['t'] * 1e3, l['th'], color=col, label=lab)
    axs[0].axhline(2.0 * 1.08, color='gray', ls=':'); axs[0].set(title='degrau 2 rad', xlabel='t [ms]', ylabel='θ [rad]'); axs[0].legend()
    for aw, col in ((True, '#1f5fa8'), (False, '#c0392b')):
        l = run_closed_loop(10.0, T=0.6, antiwindup=aw)
        axs[1].plot(l['t'] * 1e3, l['th'], color=col)
    axs[1].set(title='degrau 10 rad', xlabel='t [ms]'); fig.tight_layout(); fig.savefig(os.path.join(FIG, 'antiwindup.png')); plt.close(fig)
    # 3) parada de seguranca / rearme / retomada
    c = Ctrl()
    l = sim.scenario(c, 2.5, lambda t: 10.0, lambda t: 1 if 0.4 <= t < 0.6 else 0, lambda t: 1 if 1.0 <= t < 1.1 else 0, delay=0.02, x0=None)
    fig, axs = plt.subplots(3, 1, figsize=(8, 5.4), sharex=True)
    axs[0].plot(l['t'], l['th'], color='#1f5fa8'); axs[0].set_ylabel('θ [rad]')
    axs[1].plot(l['t'], l['i'], color='#d35400'); axs[1].set_ylabel('i [A]')
    axs[2].step(l['t'], l['sup'], where='post', color='#444', label='estado'); axs[2].step(l['t'], l['hand'] * 0.5 - 1, where='post', color='#c0392b', label='mão (real)')
    axs[2].step(l['t'], l['seen'] * 0.5 - 2, where='post', color='#e67e22', label='mão (vista, +20 ms)')
    axs[2].set_yticks([-2, -1, 0, 1, 2, 3]); axs[2].set_yticklabels(['', '', 'RUN', 'STOP_HAND', 'STOP_WAIT', 'RESUME']); axs[2].legend(loc='upper right', fontsize=7)
    axs[2].set_xlabel('t [s]'); fig.tight_layout(); fig.savefig(os.path.join(FIG, 'supervisor_cenario.png')); plt.close(fig)
    # 4) velocidade colaborativa
    l = run_closed_loop(30.0, T=1.5, collab=True)
    fig, ax = plt.subplots(figsize=(5.6, 3))
    ax.plot(l['t'], np.abs(l['w']) / N * R_EFF * 1000, color='#1f5fa8'); ax.axhline(250, color='#c0392b', ls='--', label='250 mm/s')
    ax.set(xlabel='t [s]', ylabel='v do efetuador [mm/s]'); ax.legend(); fig.tight_layout(); fig.savefig(os.path.join(FIG, 'velocidade_colab.png')); plt.close(fig)
    # 5) autômato
    fig, ax = plt.subplots(figsize=(8, 4)); ax.axis('off'); ax.set_xlim(0, 10); ax.set_ylim(0, 5)
    pos = {S.RUN: (1.3, 3.8), S.STOP_HAND: (5, 3.8), S.STOP_WAIT: (5, 1.2), S.RESUME: (1.3, 1.2)}
    for s, (x, y) in pos.items():
        ax.add_patch(FancyBboxPatch((x - 1.15, y - .45), 2.3, .9, boxstyle='round,pad=.05', fc='#e8f0fa' if s in (S.RUN, S.RESUME) else '#fdecea', ec='#333'))
        ax.text(x, y, S.NAMES[s], ha='center', va='center', fontsize=8.5, weight='bold')
    def arr(p, q, txt, rad=0.0, dx=0, dy=0):
        ax.add_patch(FancyArrowPatch(p, q, arrowstyle='-|>', mutation_scale=11, connectionstyle=f'arc3,rad={rad}', color='#333'))
        ax.text((p[0] + q[0]) / 2 + dx, (p[1] + q[1]) / 2 + dy, txt, fontsize=7.5, ha='center', va='center', bbox=dict(fc='white', ec='none', pad=.5))
    arr((2.5, 3.95), (3.85, 3.95), 'T1: hand', 0, 0, .22)
    arr((5, 3.3), (5, 1.7), 'T2: ¬hand', 0, .8, 0)
    arr((3.85, 1.2), (2.5, 1.2), 'T5: ¬hand ∧ rearm', 0, 0, -.28)
    arr((1.3, 1.7), (1.3, 3.3), 'T8: resume_done', 0, -.85, 0)
    arr((2.3, 1.65), (3.9, 3.4), 'T7: hand (corrigido)', -.25, -.3, .55)
    arr((4.6, 1.7), (3.0, 3.4), 'T4: hand', .25, .8, -.15)
    ax.text(5, 0.25, 'T0,T3,T6,T9: permanência (sem alteração de estado)', ha='center', fontsize=7.5, color='#555')
    ax.text(0.1, 4.8, 'Torque habilitado: RUN, RESUME | Torque zero: STOP_HAND, STOP_WAIT_REARM', fontsize=8)
    fig.tight_layout(); fig.savefig(os.path.join(FIG, 'automato.png')); plt.close(fig)
    # 6) erro modelo x codigo
    ref = sil.stimulus(); inputs = ref['inp']; g = Ctrl(); u64 = np.array([g.step(*a) for a in inputs]); u32, _ = sil.mirror_f32(inputs)
    fig, ax = plt.subplots(figsize=(5.6, 3)); ax.semilogy(np.arange(len(u64)) * TS, np.abs(u32 - u64) + 1e-12, color='#1f5fa8')
    ax.axhline(sil.EPS32 * np.max(np.abs(u64)), color='#c0392b', ls='--', label='ε·|u|max'); ax.set(xlabel='t [s]', ylabel='|u_float32 − u_float64| [V]'); ax.legend()
    fig.tight_layout(); fig.savefig(os.path.join(FIG, 'equivalencia.png')); plt.close(fig)


if __name__ == '__main__':
    item_a(); item_b(); item_c(); item_d(); item_e(); figures()
    json.dump(M, open(os.path.join(ROOT, 'results', 'memorial.json'), 'w'), indent=1, default=float)
    print(json.dumps(M, indent=1, default=float)[:6000])
