"""Software-in-the-loop: equivalencia numerica modelo de referencia (float64) x codigo C gerado (float32).
Se houver compilador C no PATH, o .c gerado e compilado e executado; caso contrario usa-se o espelho float32
(controller.Ctrl com dtype=float32 e constantes lidas do proprio .c gerado) e o resultado e marcado como
'C NAO EXECUTADO'."""
import os, shutil, subprocess, sys, json
import numpy as np
from params import *
from controller import Ctrl
from design import design
import codegen, sim

HERE = os.path.dirname(__file__); GEN = os.path.join(HERE, '..', 'generated')
EPS32 = float(np.finfo(np.float32).eps)


def stimulus():
    """Cenario de teste do memorial (e): degrau 10 rad, intrusao de mao (atraso 20 ms), rearme, retomada,
    trecho colaborativo e degrau reverso. Entradas gravadas da malha fechada float64."""
    r = lambda t: 10.0 if t < 1.3 else (-4.0)
    hand = lambda t: 1 if 0.45 <= t < 0.80 else 0
    rearm = lambda t: 1 if 0.90 <= t < 1.0 else 0
    collab = lambda t: 1 if t >= 1.1 else 0
    log = sim.scenario(Ctrl(), 2.0, r, hand, rearm, collab, delay=T_SENSOR_MAX)
    return log


def mirror_f32(inputs):
    c = Ctrl(dtype=np.float32)
    K = codegen.parse_constants()           # constantes lidas do .c gerado
    c.K = [K['AA_K_I'], K['AA_K_W'], K['AA_K_E'], K['AA_K_Z']]
    c.a0, c.a1, c.b0 = K['AA_A_I_I'], K['AA_A_I_W'], K['AA_B_I']
    c.c0, c.c1, c.e0 = K['AA_A_W_I'], K['AA_A_W_W'], K['AA_B_W']
    c.kaw = K['AA_KAW']
    out = [(c.step(*a), c.sup) for a in inputs]
    return np.array([o[0] for o in out]), np.array([o[1] for o in out])


def find_cc():
    for cc in ('gcc', 'cc', 'clang', 'tcc'):
        if shutil.which(cc):
            return cc
    return None


def run_c(inputs):
    cc = find_cc()
    if cc is None:
        return None
    exe = os.path.join(GEN, 'sil_main.exe')
    subprocess.run([cc, '-O0', '-ffp-contract=off', '-o', exe, os.path.join(GEN, 'sil_main.c'),
                    os.path.join(GEN, 'aurora_joint_ctrl.c')], check=True)
    txt = '\n'.join(' '.join(f'{v:.17g}' if k < 4 else str(int(v)) for k, v in enumerate(a)) for a in inputs)
    res = subprocess.run([exe], input=txt, capture_output=True, text=True, check=True).stdout.split('\n')
    return np.array([float(l.split()[0]) for l in res if l.strip()])


def equivalence():
    codegen.generate()
    ref = stimulus(); inputs = ref['inp']
    golden = Ctrl(); u64 = np.array([golden.step(*a) for a in inputs])      # modelo de referencia (float64)
    u32, sup32 = mirror_f32(inputs)
    uc = run_c(inputs)
    err = np.abs(u32.astype(np.float64) - u64)
    scale = float(np.max(np.abs(u64)))
    res = dict(n_samples=len(inputs), max_abs_err_V=float(err.max()), max_abs_u=scale,
               eps_float32=EPS32, err_over_eps_umax=float(err.max() / (EPS32 * scale)),
               err_rel=float(err.max() / scale), c_executed=uc is not None,
               supervisor_state_mismatch=int(np.sum(sup32 != ref['sup'][:len(sup32)]) if len(sup32) == len(ref['sup']) else -1))
    if uc is not None:
        res['c_vs_mirror_max_abs'] = float(np.max(np.abs(uc - u32)))
        res['c_vs_model_max_abs'] = float(np.max(np.abs(uc - u64)))
    return res


if __name__ == '__main__':
    r = equivalence(); print(json.dumps(r, indent=1))
