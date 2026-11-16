"""Modelo em espaco de estados da junta (leis de Kirchhoff e Newton) e discretizacao ZOH.
Derivacao: L di/dt = V - R i - Ke w ;  J dw/dt = Kt i - b w - Tc ;  dtheta/dt = w."""
import numpy as np
from params import *


def expm(M, terms=40):
    """Exponencial de matriz por scaling-and-squaring + Taylor (somente numpy)."""
    nrm = np.linalg.norm(M, 1)
    s = max(0, int(np.ceil(np.log2(max(nrm, 1e-16)))) + 1)
    X = M / 2.0 ** s
    E = np.eye(M.shape[0]); T = np.eye(M.shape[0])
    for k in range(1, terms):
        T = T @ X / k
        E = E + T
    for _ in range(s):
        E = E @ E
    return E


def continuous2():
    """Modelo do memorial (a): x=[i, w], y=w."""
    A = np.array([[-R / L, -KE / L], [KT / J, -B / J]])
    Bm = np.array([[1 / L], [0.0]])
    C = np.array([[0.0, 1.0]]); D = np.zeros((1, 1))
    return A, Bm, C, D


def continuous3():
    """Modelo de projeto da malha de posicao: x=[i, w, theta], y=theta; Ed = entrada de perturbacao Tc."""
    A = np.array([[-R / L, -KE / L, 0], [KT / J, -B / J, 0], [0, 1, 0]])
    Bm = np.array([[1 / L], [0], [0]])
    Ed = np.array([[0], [-1 / J], [0]])
    C = np.array([[0.0, 0, 1]]); D = np.zeros((1, 1))
    return A, Bm, Ed, C, D


def c2d(A, Bm, Ts):
    """ZOH exato (Van Loan): expm([[A,B],[0,0]]*Ts)."""
    n, m = Bm.shape
    M = np.zeros((n + m, n + m)); M[:n, :n] = A; M[:n, n:] = Bm
    E = expm(M * Ts)
    return E[:n, :n], E[:n, n:]


def ctrb(A, Bm):
    n = A.shape[0]
    return np.hstack([np.linalg.matrix_power(A, k) @ Bm for k in range(n)])


def obsv(A, C):
    n = A.shape[0]
    return np.vstack([C @ np.linalg.matrix_power(A, k) for k in range(n)])
