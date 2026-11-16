"""Supervisor de seguranca (maquina de estados). Fonte unica de verdade: usada pelo model checker,
pelos testes gerados e pelo gerador de codigo C (codegen.py).

Estados: RUN -> STOP_HAND -> STOP_WAIT_REARM -> RESUME -> RUN
Entradas (por ciclo de 2 ms): hand (scanner, ja com atraso), rearm (comando explicito), resume_done.
REQ-SAFE-AA-001: hand em QUALQUER estado => STOP_HAND (torque zero).
REQ-SAFE-AA-002: so se sai de STOP_* para RESUME via rearm explicito (T5).
REQ-SAFE-AA-003: STOP_WAIT_REARM --(rearm & !hand)--> RESUME --(resume_done)--> RUN.
"""
RUN, STOP_HAND, STOP_WAIT, RESUME = 0, 1, 2, 3
NAMES = ['RUN', 'STOP_HAND', 'STOP_WAIT_REARM', 'RESUME']
RESUME_MAX = 50      # ciclos (100 ms): ao fim o supervisor forca RUN (limite que garante vivacidade)


def step(s, hand, rearm, resume_done, cnt=0, buggy=False):
    """Retorna (proximo_estado, novo_cnt, torque_enable, rearm_consumido, id_transicao)."""
    if s == RUN:
        if hand:
            return STOP_HAND, 0, 0, 0, 'T1'
        return RUN, 0, 1, 0, 'T0'
    if s == STOP_HAND:
        if not hand:
            return STOP_WAIT, 0, 0, 0, 'T2'
        return STOP_HAND, 0, 0, 0, 'T3'
    if s == STOP_WAIT:
        if hand:
            return STOP_HAND, 0, 0, 0, 'T4'
        if rearm:
            return RESUME, 0, 1, 1, 'T5'
        return STOP_WAIT, 0, 0, 0, 'T6'
    if s == RESUME:
        # BUG da versao preliminar (reproduz o quase-acidente: condicao de borda entre movimento e
        # parada): 'hand' era ignorado enquanto o braco retornava a trajetoria.
        if hand and not buggy:
            return STOP_HAND, 0, 0, 0, 'T7'
        if resume_done or cnt + 1 >= RESUME_MAX:
            return RUN, 0, 1, 0, 'T8'
        return RESUME, cnt + 1, 1, 0, 'T9'
    raise ValueError(s)


TRANSITIONS = {  # id -> (origem, guarda, destino), para documentacao e cobertura
    'T0': (RUN, '!hand', RUN), 'T1': (RUN, 'hand', STOP_HAND),
    'T2': (STOP_HAND, '!hand', STOP_WAIT), 'T3': (STOP_HAND, 'hand', STOP_HAND),
    'T4': (STOP_WAIT, 'hand', STOP_HAND), 'T5': (STOP_WAIT, '!hand & rearm', RESUME),
    'T6': (STOP_WAIT, '!hand & !rearm', STOP_WAIT), 'T7': (RESUME, 'hand', STOP_HAND),
    'T8': (RESUME, '!hand & (resume_done | cnt>=MAX)', RUN), 'T9': (RESUME, '!hand & !resume_done', RESUME),
}
