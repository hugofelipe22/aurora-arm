# Relatorio de cobertura do supervisor (gerado por tests/test_aurora.py)

Estados cobertos: 4/4 (RUN, STOP_HAND, STOP_WAIT_REARM, RESUME)
Transicoes cobertas: 10/10

| Transicao | Origem | Guarda | Destino | Sequencia (hand,rearm,resume_done) | Coberta |
|---|---|---|---|---|---|
| T0 | RUN | !hand | RUN | [(0, 0, 0)] | sim |
| T1 | RUN | hand | STOP_HAND | [(1, 0, 0)] | sim |
| T2 | STOP_HAND | !hand | STOP_WAIT_REARM | [(1, 0, 0), (0, 0, 0)] | sim |
| T3 | STOP_HAND | hand | STOP_HAND | [(1, 0, 0), (1, 0, 0)] | sim |
| T4 | STOP_WAIT_REARM | hand | STOP_HAND | [(1, 0, 0), (0, 0, 0), (1, 0, 0)] | sim |
| T5 | STOP_WAIT_REARM | !hand & rearm | RESUME | [(1, 0, 0), (0, 0, 0), (0, 1, 0)] | sim |
| T6 | STOP_WAIT_REARM | !hand & !rearm | STOP_WAIT_REARM | [(1, 0, 0), (0, 0, 0), (0, 0, 0)] | sim |
| T7 | RESUME | hand | STOP_HAND | [(1, 0, 0), (0, 0, 0), (0, 1, 0), (1, 0, 0)] | sim |
| T8 | RESUME | !hand & (resume_done | cnt>=MAX) | RUN | [(1, 0, 0), (0, 0, 0), (0, 1, 0), (0, 0, 1)] | sim |
| T9 | RESUME | !hand & !resume_done | RESUME | [(1, 0, 0), (0, 0, 0), (0, 1, 0), (0, 0, 0)] | sim |

T8 por timeout (RESUME_MAX=50): coberto
