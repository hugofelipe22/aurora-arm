"""Suite de testes AuroraArm: desempenho, seguranca, baseados em modelo (percurso do grafo do supervisor),
mutacao e equivalencia. Executar: python tests/test_aurora.py  -> results/tests.json (+ cobertura)."""
import json, os, sys, itertools
from collections import deque
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
import supervisor as S
from params import *
from controller import Ctrl, run_closed_loop
from design import metrics
import sim, modelcheck, sil

RESULTS = []


def record(tid, req, desc, passed, detail):
    RESULTS.append(dict(id=tid, req=req, desc=desc, passed=bool(passed), detail=detail))
    print(('PASS ' if passed else 'FAIL ') + tid, detail)


def m_of(l, rf):
    return metrics({'th': l['th'], 't': l['t'], 'iint': l['ii'], 'u': l['u']}, rf)


# ---------------- desempenho ----------------
def test_perf():
    ms = {rf: m_of(run_closed_loop(rf), rf) for rf in (2.0, 10.0)}
    record('TC-CTRL-001', 'REQ-CTRL-AA-001', 'erro de regime nulo (degrau 2 e 10 rad, com T_carga=0,25 N.m)',
           all(m['sse'] < 1e-6 for m in ms.values()), {k: float(v['sse']) for k, v in ms.items()})
    record('TC-CTRL-002', 'REQ-CTRL-AA-002', 'sobressinal <= 8%', all(m['overshoot_pct'] <= 8 for m in ms.values()),
           {k: round(float(v['overshoot_pct']), 2) for k, v in ms.items()})
    record('TC-CTRL-003', 'REQ-CTRL-AA-003', 'acomodacao (2%) <= 300 ms', all(m['settling_s'] <= 0.3 for m in ms.values()),
           {k: round(float(v['settling_s']) * 1000, 1) for k, v in ms.items()})
    worst_u = worst_i = 0.0
    for rf in (0.5, 2.0, 10.0, 50.0, -10.0):
        l = run_closed_loop(rf, T=1.0)
        worst_u = max(worst_u, float(np.abs(l['u']).max())); worst_i = max(worst_i, float(l['ii'].max()), float(-l['ii'].min()))
    record('TC-CTRL-004', 'REQ-CTRL-AA-004', '|V|<=48 V e |i|<=8 A (corrente continua intra-amostra, 5 degraus)',
           worst_u <= VMAX and worst_i <= IMAX, dict(Vmax=worst_u, Imax=worst_i))
    aw = m_of(run_closed_loop(2.0, antiwindup=True), 2.0); naw = m_of(run_closed_loop(2.0, antiwindup=False), 2.0)
    record('TC-CTRL-005', 'REQ-CTRL-AA-002', 'anti-windup: sem AW estoura 8%, com AW nao',
           naw['overshoot_pct'] > 8 and aw['overshoot_pct'] <= 8,
           dict(com_AW=float(aw['overshoot_pct']), sem_AW=float(naw['overshoot_pct'])))


# ---------------- seguranca em malha fechada ----------------
def worst_case_stop():
    """Varre atraso do sensor (0..20 ms, passo 1 ms), instante de intrusao dentro do ciclo (4 fases de 0,5 ms),
    e ciclos perdidos (nenhum / 1 ciclo logo apos a deteccao / 2 ciclos). Parte do ponto critico (450 rad/s)."""
    t_hand = 0.2; worst = 0.0; worst_cfg = None; n = 0
    for d_ms in range(0, 21):
        for ph in (0.0, 0.0005, 0.001, 0.0015):
            th = t_hand + ph
            for nskip in (0, 1, 2):
                delay = d_ms / 1000
                k_det = int(np.ceil((th + delay) / TS))        # primeiro ciclo que ve a mao
                skip = tuple(range(k_det, k_det + nskip))
                c = Ctrl()
                l = sim.scenario(c, 0.5, lambda t: 450.0 * t + 50, lambda t: 1 if t >= th else 0, delay=delay, skip=skip,
                                 x0=[3.735, 450.0, 0.0])
                tz = sim.torque_zero_after(l, th)
                n += 1
                if tz is None:
                    return 1e9, dict(delay_ms=d_ms, phase=ph, skips=nskip), n
                if tz - th > worst:
                    worst, worst_cfg = tz - th, dict(delay_ms=d_ms, phase_ms=ph * 1000, skips=nskip)
    return worst, worst_cfg, n


def test_safety():
    w, cfg, n = worst_case_stop()
    record('TC-SAFE-001', 'REQ-SAFE-AA-001', f'torque zero <= 100 ms, malha fechada, {n} combinacoes (atraso x fase x jitter)',
           w <= T_DEADLINE, dict(pior_caso_ms=round(w * 1000, 2), config=cfg))
    mc = modelcheck.check(J=1, Z=2)
    record('TC-SAFE-001-MC', 'REQ-SAFE-AA-001', 'model checking exaustivo P1 (supervisor+sensor+jitter J=1)', mc['p1_ok'],
           dict(estados=mc['states'], pior_caso_ms=mc['wcrt_ms']))
    # 002: mao some, sem rearme => permanece parado
    c = Ctrl()
    l = sim.scenario(c, 1.5, lambda t: 10.0, lambda t: 1 if 0.5 <= t < 0.7 else 0, lambda t: 0, delay=0.02)
    mask = l['t'] > 0.8
    ok = np.all(l['sup'][mask] == S.STOP_WAIT) and np.all(np.abs(l['i'][mask]) <= sim.I_ZERO)
    record('TC-SAFE-002', 'REQ-SAFE-AA-002', 'sem rearme nao retoma (0,7..1,5 s) e torque permanece zero', ok,
           dict(estado_final=S.NAMES[int(l['sup'][-1])]))
    mc2 = modelcheck.check(J=1, Z=2)
    record('TC-SAFE-002-MC', 'REQ-SAFE-AA-002', 'model checking P2: RESUME so via T5 (rearme consumido, sem mao)', mc2['p2_ok'], {})
    # 003: rearme => retoma trajetoria original
    c = Ctrl()
    l = sim.scenario(c, 2.5, lambda t: 10.0, lambda t: 1 if 0.4 <= t < 0.6 else 0, lambda t: 1 if 1.0 <= t < 1.1 else 0, delay=0.02)
    ok = l['sup'][-1] == S.RUN and abs(l['th'][-1] - 10.0) < 1e-2
    record('TC-SAFE-003', 'REQ-SAFE-AA-003', 'apos rearme o braco retorna a trajetoria original (theta->10 rad) e volta a RUN', ok,
           dict(theta_final=float(l['th'][-1]), estado=S.NAMES[int(l['sup'][-1])]))
    lv = modelcheck.check_liveness(1, 2)
    record('TC-SAFE-003-MC', 'REQ-SAFE-AA-003', 'model checking P3 (CTL AF): rearme & mao ausente => RUN inevitavel', lv['live_ok'], lv)
    # 004: velocidade colaborativa
    peak = 0.0
    for rf, tl in ((30.0, T_LOAD), (-30.0, T_LOAD), (100.0, 0.0), (30.0, -0.25)):
        l = run_closed_loop(rf, T=3.0, collab=True, tload=tl)
        peak = max(peak, float(np.abs(l['w']).max()))
    v = peak / N * R_EFF * 1000
    record('TC-SAFE-004', 'REQ-SAFE-AA-004', 'v_efetuador <= 250 mm/s na zona colaborativa (4 cenarios)', v <= 250,
           dict(pico_w=peak, v_mm_s=round(v, 1)))


# ---------------- testes baseados em modelo ----------------
INPUTS = list(itertools.product((0, 1), (0, 1), (0, 1)))   # hand, rearm, resume_done


def generate_paths():
    """BFS sobre o grafo (estado do supervisor): para cada transicao T0..T9 gera o menor prefixo de
    entradas que a exercita. Retorna {tid: [(hand,rearm,rd),...]} e o conjunto de transicoes alcancaveis."""
    seen = {S.RUN: []}; q = deque([S.RUN]); paths = {}
    while q:
        s = q.popleft()
        for hand, rearm, rd in INPUTS:
            ns, cnt, en, cons, tid = S.step(s, hand, rearm, rd, 0)
            if tid not in paths:
                paths[tid] = seen[s] + [(hand, rearm, rd)]
            if ns not in seen:
                seen[ns] = seen[s] + [(hand, rearm, rd)]; q.append(ns)
    return paths


def run_path(path, buggy=False, dtype=np.float64):
    c = Ctrl(dtype=dtype, buggy=buggy); tids = []; states = []
    for hand, rearm, rd in path:
        # resume_done realizado pela referencia: perto => rd=1, longe => rd=0
        r_t = float(c.r) if rd else float(c.r) + 100.0
        c.step(0.0, 0.0, 0.0, r_t, hand, rearm, 0)
        tids.append(c.last_tid); states.append(int(c.sup))
    return tids, states


def test_mbt():
    paths = generate_paths()
    cov_t, cov_s, fails = set(), {S.RUN}, []
    for tid, p in sorted(paths.items()):
        tids, st = run_path(p)
        cov_t |= set(tids); cov_s |= set(st)
        if tids[-1] != tid:
            fails.append(tid)
    # timeout de RESUME (T8 forcado por cnt) - sequencia direta
    seq = [(1, 0, 0), (0, 0, 0), (0, 1, 0)] + [(0, 0, 0)] * (S.RESUME_MAX + 1)
    tids, _ = run_path(seq)
    forced = 'T8' in tids and tids.count('T9') == S.RESUME_MAX - 1
    all_t = set(S.TRANSITIONS)
    record('TC-MBT-001', 'REQ-SAFE-AA-001..003', 'testes gerados por percurso do grafo: cobertura de estados e transicoes',
           not fails and cov_t == all_t and len(cov_s) == 4 and forced,
           dict(estados=f'{len(cov_s)}/4', transicoes=f'{len(cov_t)}/{len(all_t)}', caminhos=len(paths),
                T8_forcado_por_timeout=forced, falhas=fails))
    rows = ['# Relatorio de cobertura do supervisor (gerado por tests/test_aurora.py)', '',
            f'Estados cobertos: {len(cov_s)}/4 ({", ".join(S.NAMES[s] for s in sorted(cov_s))})',
            f'Transicoes cobertas: {len(cov_t)}/{len(all_t)}', '',
            '| Transicao | Origem | Guarda | Destino | Sequencia (hand,rearm,resume_done) | Coberta |', '|---|---|---|---|---|---|']
    for tid, (o, g, d) in sorted(S.TRANSITIONS.items(), key=lambda kv: int(kv[0][1:])):
        rows.append(f'| {tid} | {S.NAMES[o]} | {g} | {S.NAMES[d]} | {paths.get(tid)} | {"sim" if tid in cov_t else "NAO"} |')
    rows += ['', f'T8 por timeout (RESUME_MAX={S.RESUME_MAX}): {"coberto" if forced else "NAO coberto"}']
    open(os.path.join(os.path.dirname(__file__), '..', 'results', 'coverage_report.md'), 'w', encoding='utf8').write(chr(10).join(rows) + chr(10))
    # equivalencia dos caminhos float32 x float64
    mism = [t for t, p in paths.items() if run_path(p)[0] != run_path(p, dtype=np.float32)[0]]
    record('TC-MBT-002', 'REQ-SAFE-AA-001..003', 'mesmos caminhos no espelho float32 do codigo C: mesmas transicoes', not mism, dict(divergencias=mism))
    # mutacao: versao preliminar (bug do quase-acidente) deve ser reprovada pela suite
    killed = [t for t, p in paths.items() if run_path(p, buggy=True)[0] != run_path(p)[0]]
    record('TC-MBT-003', 'REQ-SAFE-AA-001', 'mutante (hand ignorado em RESUME) e detectado pela suite gerada', 'T7' in killed or len(killed) > 0,
           dict(testes_que_matam_o_mutante=killed))
    return paths


def test_equivalence():
    r = sil.equivalence()
    ok = r['max_abs_err_V'] < 1e-3 and r['err_over_eps_umax'] < 1e3 and r['supervisor_state_mismatch'] == 0
    record('TC-EQV-001', 'REQ-CTRL-AA-004', 'equivalencia modelo float64 x codigo float32 (SIL, 1000 amostras)', ok, r)


def write_reports(paths):
    os.makedirs(os.path.join(os.path.dirname(__file__), '..', 'results'), exist_ok=True)
    out = os.path.join(os.path.dirname(__file__), '..', 'results', 'tests.json')
    json.dump(dict(tests=RESULTS, paths={k: v for k, v in paths.items()}), open(out, 'w'), indent=1)
    np_ = sum(r['passed'] for r in RESULTS)
    print(f'\n{np_}/{len(RESULTS)} testes passaram')
    return np_ == len(RESULTS)


if __name__ == '__main__':
    test_perf(); test_safety(); p = test_mbt(); test_equivalence()
    sys.exit(0 if write_reports(p) else 1)
