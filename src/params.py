"""Parametros fisicos da junta do cotovelo do AuroraArm (enunciado, secao 2 'Onde?')."""
R = 0.8            # ohm      resistencia de armadura
L = 1.2e-3         # H        indutancia de armadura
KT = 0.085         # N.m/A    constante de torque
KE = 0.085         # V.s/rad  constante de f.c.e.m.
J = 4.2e-4         # kg.m^2   inercia refletida no eixo do motor
B = 1.5e-4         # N.m.s/rad atrito viscoso refletido
N = 100            # reducao harmonica
VMAX = 48.0        # V        barramento
IMAX = 8.0         # A        limite do driver
TS = 2e-3          # s        periodo do laco de posicao
W_SS = 450.0       # rad/s    ponto critico (eixo do motor)
T_LOAD = 0.25      # N.m      torque de carga refletido no ponto critico
R_EFF = 0.35       # m        raio do payload / efetuador
V_COLLAB = 0.250   # m/s      limite REQ-SAFE-AA-004
# Limite de velocidade no eixo do motor na zona colaborativa, com 5% de margem (a perturbacao de carga move
# w em Tc*Ts/J = 1,19 rad/s por ciclo, que o limitador preditivo nao enxerga)
W_COLLAB = 0.95 * V_COLLAB / R_EFF * N   # rad/s (eixo do motor)
IMAX_EFF = 7.5     # A  limite efetivo de projeto (margem de 0,5 A p/ ripple intra-amostra)
D_SENSE = 0.500    # m  distancia de deteccao REQ-SAFE-AA-001
T_DEADLINE = 0.100 # s
T_SENSOR_MAX = 0.020  # s
