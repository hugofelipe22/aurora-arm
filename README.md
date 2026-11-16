# AuroraArm — junta do cotovelo (Operação Linha Viva)

Pacote de evidências model-based: planta → controlador → verificação formal → código C gerado → rastreabilidade.

| Caminho | Conteúdo |
|---|---|
| `src/params.py`, `src/plant.py` | parâmetros e matrizes A,B,C,D (2 e 3 estados), ZOH exato, controlabilidade/observabilidade |
| `src/design.py` | LQR discreto + ação integral, limitador preditivo de V/i, anti-windup (back-calculation) |
| `src/supervisor.py` | autômato de segurança (fonte única: checker, testes, gerador) |
| `src/modelcheck.py` | model checker de estados explícitos temporizado (tick 1 ms): P1 prazo 100 ms, P2 rearme, P3 vivacidade CTL AF |
| `src/controller.py`, `src/sim.py` | modelo de referência (float64/float32) e co-simulação com planta |
| `src/codegen.py` | gera `generated/aurora_joint_ctrl.{h,c}` e `sil_main.c` |
| `src/sil.py` | equivalência modelo × código (compila o C se houver compilador; senão usa espelho float32) |
| `src/memorial.py` | itens (a)–(e) do memorial, figuras, `results/memorial.json` |
| `tests/test_aurora.py` | 16 testes (desempenho, segurança, baseados em modelo, mutação, equivalência) |
| `docs/` | matriz de rastreabilidade e relatório técnico |

Reproduzir: `python run_all.py` (requer apenas numpy e matplotlib). Para fechar o SIL com o C real:
`gcc -O0 -ffp-contract=off -o sil generated/sil_main.c generated/aurora_joint_ctrl.c` (ou apenas instalar gcc e rodar `python src/sil.py`).
