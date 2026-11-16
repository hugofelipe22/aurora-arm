"""Gera docs/traceability.md (requisito -> modelo -> codigo -> teste) a partir de results/tests.json."""
import json, os
ROOT = os.path.join(os.path.dirname(__file__), '..')
T = {t['id']: t for t in json.load(open(os.path.join(ROOT, 'results', 'tests.json')))['tests']}
ROWS = [
 ('REQ-CTRL-AA-001', 'src/plant.py (A,B,C,D); src/design.py (acao integral z)', 'generated/aurora_joint_ctrl.c: aa_joint_step() estado c->z', ['TC-CTRL-001']),
 ('REQ-CTRL-AA-002', 'src/design.py (LQR, Q/R); anti-windup back-calculation', 'aurora_joint_ctrl.c: AA_K_*, AA_KAW, c->z', ['TC-CTRL-002', 'TC-CTRL-005']),
 ('REQ-CTRL-AA-003', 'src/design.py (LQR, polos de MF)', 'aurora_joint_ctrl.c: AA_K_*', ['TC-CTRL-003']),
 ('REQ-CTRL-AA-004', 'src/design.py limits() (limitador preditivo V e i)', 'aurora_joint_ctrl.c: lo/hi, AA_VMAX, AA_IMAX_EFF', ['TC-CTRL-004', 'TC-EQV-001']),
 ('REQ-SAFE-AA-001', 'src/supervisor.py T1,T4,T7; src/modelcheck.py P1', 'aurora_joint_ctrl.c: case AA_RUN/AA_RESUME -> AA_STOP_HAND; corrente-zero', ['TC-SAFE-001', 'TC-SAFE-001-MC', 'TC-MBT-001', 'TC-MBT-003']),
 ('REQ-SAFE-AA-002', 'src/supervisor.py T2,T3,T5,T6; modelcheck P2', 'aurora_joint_ctrl.c: case AA_STOP_WAIT_REARM (exige rearm)', ['TC-SAFE-002', 'TC-SAFE-002-MC', 'TC-MBT-001']),
 ('REQ-SAFE-AA-003', 'src/supervisor.py T5,T8,T9; modelcheck P3 (CTL AF)', 'aurora_joint_ctrl.c: AA_RESUME, r_cmd rampeado', ['TC-SAFE-003', 'TC-SAFE-003-MC', 'TC-MBT-001']),
 ('REQ-SAFE-AA-004', 'src/params.py W_COLLAB; limitador preditivo de velocidade', 'aurora_joint_ctrl.c: AA_W_COLLAB, wl/lim', ['TC-SAFE-004']),
]
L = ['# Matriz de rastreabilidade (gerada por src/traceability.py)', '',
     '| Requisito | Modelo | Código gerado | Teste(s) | Resultado |', '|---|---|---|---|---|']
for req, m, c, ts in ROWS:
    res = ', '.join(f"{t}: {'PASS' if T[t]['passed'] else 'FAIL'}" for t in ts)
    L.append(f'| {req} | {m} | {c} | {", ".join(ts)} | {res} |')
open(os.path.join(ROOT, 'docs', 'traceability.md'), 'w', encoding='utf8').write('\n'.join(L) + '\n')
print('\n'.join(L))
