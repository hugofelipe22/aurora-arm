# Roteiro do vídeo (≤ 5 min) — auditoria de segurança AuroraArm

1. **0:00–0:40 Problema.** Quase-acidente do protótipo: torque não zerado quando a mão chegou durante a retomada; sem evidência reprodutível. Mostrar `figs/automato.png`.
2. **0:40–1:50 Modelo e controlador.** A,B,C,D; τe = 1,55 ms, τm = 44,2 ms; 41,24 V / 3,74 A em 450 rad/s (folga 6,76 V); Ts = 2 ms = 22 amostras por τm; LQR + integral + anti-windup (`figs/antiwindup.png`: 1026% → 0%).
3. **1:50–3:00 Prazo de 100 ms.** Mostrar o contraexemplo (105 ticks) da versão preliminar → correção T7 → P1 passa; pior caso 25 ms; tolera 39 ciclos perdidos (`figs/prazo_vs_atraso.png`). Rodar `python tests/test_aurora.py`.
4. **3:00–4:00 Código e rastreabilidade.** `generated/aurora_joint_ctrl.c` (cabeçalho com REQs); equivalência: erro 3,4e-4 V ≈ 2,6 ε·Σ|termos|; abrir `docs/traceability.md`.
5. **4:00–5:00 Honestidade.** Sustenta: evidência reprodutível no modelo. Não sustenta: C real ainda não executado (SIL com espelho), checker não qualificado, sem HIL, sem análise de força ISO/TS 15066, e para ISO 26262 faltam qualificação de ferramentas, independência e cobertura MC/DC.
