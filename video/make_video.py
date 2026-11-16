"""Gera video/AuroraArm_pitch.avi (MJPEG + PCM): simulacao de gravacao de tela para a auditoria.
Narracao: SAPI 'Microsoft Maria Desktop' (pt-BR). Cenas usam saidas reais (results/*.json, figs/*.png, generated/*.c).
Uso: python video/make_video.py"""
import json, os, struct, subprocess, wave, io, sys
from PIL import Image, ImageDraw, ImageFont

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
OUT = os.path.join(ROOT, 'video'); TMP = os.path.join(OUT, '_tmp'); os.makedirs(TMP, exist_ok=True)
W, H, FPS, SR = 1280, 720, 2, 22050
FD = 'C:/Windows/Fonts/'
F = lambda n, s: ImageFont.truetype(FD + n, s)
f_t, f_b, f_h, f_m, f_s = F('segoeuib.ttf', 34), F('segoeui.ttf', 22), F('segoeuib.ttf', 22), F('consola.ttf', 16), F('segoeui.ttf', 15)
M = json.load(open(os.path.join(ROOT, 'results', 'memorial.json')))
T = {t['id']: t for t in json.load(open(os.path.join(ROOT, 'results', 'tests.json')))['tests']}
img = lambda n, w, h: _fit(Image.open(os.path.join(ROOT, 'figs', n)).convert('RGB'), w, h)


def _fit(im, w, h):
    r = min(w / im.width, h / im.height); return im.resize((int(im.width * r), int(im.height * r)), Image.LANCZOS)


# ---------------- narracao ----------------
NARR = [
 "Boa tarde. Sou o engenheiro de design baseado em modelos responsável pela junta do cotovelo do AuroraArm, e esta apresentação é para a auditoria de segurança que libera a operação colaborativa sem cerca. O contexto: o protótipo anterior teve um quase-acidente. O torque não foi zerado a tempo quando um operador aproximou a mão, porque o firmware manual não tratava a condição de borda entre movimento e parada de segurança. Não houve lesão, mas também não havia nenhuma evidência reprodutível de que os requisitos eram satisfeitos. O pacote que apresento substitui essa lacuna: modelo, controle, verificação formal, código gerado e rastreabilidade.",
 "Primeiro, o modelo. A partir das leis de Kirchhoff e de Newton, os estados são corrente e velocidade, com a posição acrescentada para o laço de posição. Os autovalores são menos 644 e menos 22,6, o que dá constante de tempo elétrica de 1,55 milissegundos e mecânica de 44,2 milissegundos, uma separação de 28 vezes. A junta é controlável e observável, com posto 2. No ponto crítico, 450 radianos por segundo com 0,25 newton metro, são necessários 3,74 ampères e 41,2 volts. A folga é de 6,8 volts, 14 por cento. Em regime cabe, mas na partida a corrente de rotor bloqueado seria 60 ampères contra o limite de 8, então o atuador satura, e o anti-windup é obrigatório. O período de 2 milissegundos dá 22 amostras por constante de tempo mecânica, o que é adequado.",
 "O controlador é uma realimentação de estados com integrador, projetada por LQR em tempo discreto. Um limitador preditivo mantém a tensão abaixo de 48 volts e a corrente abaixo de 8 ampères, e o anti-windup por back-calculation evita o acúmulo do integrador. No degrau de 2 radianos, o sobressinal é zero e a acomodação é de 104 milissegundos. Em 10 radianos, 6 por cento e 208 milissegundos. Sem o anti-windup, o sobressinal passa de mil por cento. Na zona colaborativa, só limitar a referência dava 430 milímetros por segundo; acrescentei um limitador de velocidade, e o pico caiu para 241, abaixo dos 250 exigidos.",
 "Agora, a verificação do prazo de 100 milissegundos. O supervisor tem quatro estados. O sistema verificado inclui o sensor com atraso não determinístico de até 20 milissegundos, o escalonador que pode perder ciclos, e a atuação. A busca é exaustiva. Na versão preliminar, o model checker encontrou um contraexemplo de 105 passos: a mão reaparece durante a retomada, o supervisor a ignora, e o torque fica ativo por mais de 100 milissegundos. É exatamente o quase-acidente. Na versão corrigida, a mão tem prioridade em todos os estados, e a propriedade passa. O pior caso é de 25 milissegundos com um ciclo perdido, e em malha fechada, 25,5. O sistema tolera até 39 ciclos perdidos antes de violar o prazo. O rearme explícito e a retomada da trajetória também foram verificados, esta última como propriedade de vivacidade em CTL.",
 "O código C é gerado automaticamente do mesmo modelo, com o bloco de rastreabilidade no cabeçalho. Os testes também são gerados: percorrendo o grafo do supervisor, cobrimos 4 de 4 estados e 10 de 10 transições, e o mutante com o bug original é detectado. Na equivalência, em mil amostras, o erro máximo entre o modelo em dupla precisão e o código em precisão simples é de 3,4 décimos de milivolt, cerca de 2,6 vezes o arredondamento esperado, sem evidência de defeito na geração. A matriz de rastreabilidade liga cada um dos oito requisitos ao modelo, ao código e ao teste, e os 16 testes passam.",
 "Com honestidade técnica: o pacote sustenta que, no modelo, os requisitos de desempenho e de segurança são satisfeitos, de forma reprodutível. Ele não sustenta o resto. O código C ainda não foi compilado e executado; usei um espelho em precisão simples, e o harness está pronto. O verificador de modelos é próprio e não qualificado. Faltam teste hardware-in-the-loop, tempo de execução medido no Cortex-M4 e análise de força de contato da ISO TS 15066. E, para ISO 26262 ou norma equivalente, seriam exigidas qualificação de ferramentas, independência da verificação e cobertura MC/DC. Obrigado.",
]


def tts():
    wavs = []
    for i, txt in enumerate(NARR):
        tp, wp = os.path.join(TMP, f's{i}.txt'), os.path.join(TMP, f's{i}.wav')
        open(tp, 'w', encoding='utf8').write(txt)
        ps = ("Add-Type -AssemblyName System.Speech; $s=New-Object System.Speech.Synthesis.SpeechSynthesizer;"
              "$s.SelectVoice('Microsoft Maria Desktop'); $s.Rate=4;"
              f"$f=New-Object System.Speech.AudioFormat.SpeechAudioFormatInfo({SR},[System.Speech.AudioFormat.AudioBitsPerSample]::Sixteen,[System.Speech.AudioFormat.AudioChannel]::Mono);"
              f"$s.SetOutputToWaveFile('{wp}',$f); $s.Speak([IO.File]::ReadAllText('{tp}',[Text.Encoding]::UTF8)); $s.Dispose()")
        subprocess.run(['powershell', '-NoProfile', '-Command', ps], check=True)
        wavs.append(wp)
    return wavs


# ---------------- visual ----------------
def chrome(d, title):
    d.rectangle([0, 0, W, 34], fill=(32, 34, 40))
    for k, c in enumerate(((255, 95, 86), (255, 189, 46), (39, 201, 63))):
        d.ellipse([14 + k * 22, 11, 26 + k * 22, 23], fill=c)
    d.text((W // 2 - 200, 7), title, font=f_s, fill=(190, 195, 205))


def terminal(im, d, box, lines, n, prompt):
    x0, y0, x1, y1 = box
    d.rectangle(box, fill=(18, 20, 26), outline=(60, 64, 74))
    d.text((x0 + 10, y0 + 8), prompt, font=f_m, fill=(110, 200, 140))
    y = y0 + 32
    for ln in lines[:n]:
        col = (120, 220, 140) if ln.startswith('PASS') else (240, 110, 110) if ln.startswith('FAIL') or 'VIOLADA' in ln else (210, 215, 225)
        d.text((x0 + 10, y), ln[:int((x1 - x0 - 20) / 8.8)], font=f_m, fill=col); y += 21
    if n < len(lines) or True:
        d.rectangle([x0 + 10, y + 2, x0 + 19, y + 16], fill=(210, 215, 225))


def pan(im, d, box, pic, title):
    x0, y0, x1, y1 = box
    d.rectangle(box, fill=(250, 250, 250), outline=(60, 64, 74))
    p = pic if isinstance(pic, Image.Image) else img(pic, x1 - x0 - 10, y1 - y0 - 10)
    im.paste(p, (x0 + (x1 - x0 - p.width) // 2, y0 + (y1 - y0 - p.height) // 2))


def cursor(d, x, y):
    d.polygon([(x, y), (x, y + 20), (x + 5, y + 15), (x + 10, y + 25), (x + 14, y + 23), (x + 9, y + 14), (x + 16, y + 14)], fill=(255, 255, 255), outline=(0, 0, 0))


def bullets(d, x, y, items, n, f=f_b, col=(235, 238, 245), gap=44):
    for k, it in enumerate(items[:n]):
        d.text((x, y + k * gap), it, font=f, fill=col)


def base(title):
    im = Image.new('RGB', (W, H), (24, 26, 32)); d = ImageDraw.Draw(im); chrome(d, title); return im, d


# linhas de terminal a partir de resultados reais
def test_lines():
    out = []
    for t in T.values():
        out.append(f"{'PASS' if t['passed'] else 'FAIL'} {t['id']:<15} {t['desc'][:62]}")
    out.append(f"{sum(t['passed'] for t in T.values())}/{len(T)} testes passaram")
    return out


MD = M['d']
MC_LINES = ["Z (ms) = %d   # tempo p/ corrente zerar (corrente-zero deadbeat)" % MD['Z_ms'],
            "-- versao preliminar (hand ignorado em RESUME), J=1 --",
            "P1 prazo 100 ms: VIOLADA   estados=%d  contraexemplo=%d passos" % (MD['bug']['states'], MD['bug']['len_cex']),
            "   t=0 mao entra -> STOP_HAND -> mao sai -> STOP_WAIT -> rearme -> RESUME",
            "   mao REAPARECE em RESUME (ignorada)... torque ativo, relogio = %d ms" % MD['bug']['final']['mon'],
            "-- versao corrigida (T7: hand tem prioridade em todos os estados) --"]
for r in MD['fixed']:
    MC_LINES.append("J=%d  estados=%-7d trans=%-8d P1 ok=%s  P2 ok=%s  pior caso=%d ms" % (r['J'], r['states'], r['transitions'], r['p1_ok'], r['p2_ok'], r['wcrt_ms']))
MC_LINES += ["P3 (CTL AF, vivacidade): ok=%s  violacoes=%d" % (MD['live']['live_ok'], MD['live']['violations']),
             "malha fechada (252 comb.): pior caso 25,5 ms | tolera ate %d ciclos perdidos" % MD['max_lost_cycles']]

A = M['a']; B_ = M['b']; C_ = M['c']
MEM_LINES = ["autovalores = [%.2f, %.2f] s^-1" % tuple(A['eig']), "tau_e = %.2f ms   tau_m = %.1f ms   (x%.1f)" % (A['tau_fast_ms'], A['tau_slow_ms'], A['separation']),
             "posto(ctrb)=%d  posto(obsv)=%d  -> controlavel e observavel" % (A['rank_ctrb'], A['rank_obsv']),
             "i_ss = (b*450+0,25)/Kt = %.3f A" % B_['iss'], "V_ss = R*i_ss + Ke*450 = %.2f V   folga = %.2f V (%.1f%%)" % (B_['vss'], B_['folga_V'], B_['folga_pct']),
             "i_stall = 48/0,8 = %.0f A  >>  8 A  -> satura -> anti-windup" % B_['i_stall'],
             "tau_m/Ts = %.1f amostras  -> Ts = 2 ms adequado" % C_['tm_over_Ts'],
             "K = [%.3f %.3f %.2f %.1f]" % tuple(C_['K'])]
C_HEAD = open(os.path.join(ROOT, 'generated', 'aurora_joint_ctrl.h'), encoding='utf8').read().split('\n')[:22]
E = M['e']
EQ_LINES = ["amostras=%d  erro max = %.2e V  |u|max = %.2f V" % (E['n_samples'], E['max_abs_err_V'], E['max_abs_u']),
            "eps(float32) = %.2e   erro/(eps*|u|max) = %.0f" % (E['eps_float32'], E['err_over_eps_umax']),
            "erro/(eps*soma|termos|) = %.2f   -> equivalencia aceitavel" % E['err_over_eps_terms'],
            "estados do supervisor divergentes = %d de %d" % (E['supervisor_state_mismatch'], E['n_samples'])]
TRACE = [l for l in open(os.path.join(ROOT, 'docs', 'traceability.md'), encoding='utf8').read().split('\n') if l.startswith('| REQ')]
TR_LINES = []
for l in TRACE:
    c = [x.strip() for x in l.strip('|').split('|')]
    TR_LINES.append(f"{c[0]:<16} -> {c[3]:<44} {'PASS' if 'FAIL' not in c[4] else 'FAIL'}")


def scene(i, f):
    """f = fracao 0..1 do tempo da cena."""
    p = min(1.0, f / 0.85)
    if i == 0:
        im, d = base('AuroraArm — relatorio_AuroraArm.pdf')
        d.text((40, 60), 'Operação Linha Viva — AuroraArm', font=f_t, fill=(255, 255, 255))
        d.text((40, 105), 'Apresentação à auditoria de segurança — junta do cotovelo', font=f_b, fill=(150, 190, 240))
        items = ['• Protótipo anterior: quase-acidente em teste interno', '• Torque não zerado quando a mão se aproximou do efetuador',
                 '• Causa: condição de borda movimento × parada no firmware manual', '• Sem lesão, mas SEM evidência reprodutível de segurança',
                 '• Resposta: modelo → controle → verificação → código → rastreio']
        bullets(d, 40, 170, items, 1 + int(p * len(items)) if p < 1 else len(items), gap=48)
        pan(im, d, (40, 430, 720, 705), 'automato.png', '')
        d.text((750, 440), 'Autômato do supervisor', font=f_h, fill=(255, 255, 255))
        d.text((750, 475), 'RUN → STOP_HAND → STOP_WAIT_REARM → RESUME', font=f_s, fill=(190, 195, 205))
        cx, cy = 300 + 400 * f, 500 + 100 * f
    elif i == 1:
        im, d = base('PowerShell — python src/memorial.py')
        terminal(im, d, (20, 50, 640, 700), MEM_LINES, 1 + int(p * len(MEM_LINES)), 'PS aurora_arm> python src/memorial.py')
        pan(im, d, (660, 50, 1262, 700), 'degraus.png', '')
        cx, cy = 300, 120 + 400 * f
    elif i == 2:
        im, d = base('figs — antiwindup.png | velocidade_colab.png')
        pan(im, d, (20, 50, 700, 330), 'antiwindup.png', ''); pan(im, d, (20, 345, 700, 700), 'velocidade_colab.png', '')
        L = [l for l in test_lines() if 'CTRL' in l or 'SAFE-004' in l]
        terminal(im, d, (715, 50, 1262, 700), L, 1 + int(p * len(L)), 'PS aurora_arm> python tests/test_aurora.py')
        cx, cy = 400 + 600 * f, 200 + 300 * f
    elif i == 3:
        im, d = base('PowerShell — python src/modelcheck.py')
        terminal(im, d, (20, 50, 760, 480), MC_LINES, 1 + int(p * len(MC_LINES)), 'PS aurora_arm> python src/modelcheck.py')
        pan(im, d, (780, 50, 1262, 330), 'prazo_vs_atraso.png', ''); pan(im, d, (780, 345, 1262, 700), 'supervisor_cenario.png', '')
        d.rectangle([20, 495, 760, 700], fill=(18, 20, 26), outline=(60, 64, 74))
        d.text((32, 505), 'Propriedades (src/modelcheck.py)', font=f_h, fill=(255, 255, 255))
        for k, s in enumerate(['P1  AG ¬(mão presente > 100 ms & torque ativo)   [segurança]', 'P2  RESUME só via rearme explícito, sem mão   [segurança]',
                               'P3  AG((STOP_WAIT & rearme & ¬mão) → AF RUN)   [vivacidade]', 'Atraso do sensor 0–20 ms × ciclos perdidos J × fase']):
            d.text((32, 545 + k * 36), s, font=f_s, fill=(200, 205, 215))
        cx, cy = 100 + 500 * f, 150 + 250 * f
    elif i == 4:
        im, d = base('VS Code — generated/aurora_joint_ctrl.h | docs/traceability.md')
        terminal(im, d, (20, 50, 760, 360), C_HEAD, 1 + int(p * len(C_HEAD)), '// generated/aurora_joint_ctrl.h (gerado por src/codegen.py)')
        terminal(im, d, (20, 375, 760, 700), TR_LINES + [''] + EQ_LINES, 1 + int(p * (len(TR_LINES) + 1 + len(EQ_LINES))), 'matriz de rastreabilidade | equivalencia (SIL)')
        pan(im, d, (780, 50, 1262, 330), 'equivalencia.png', '')
        L = [l for l in test_lines() if 'MBT' in l or 'EQV' in l] + [test_lines()[-1]]
        terminal(im, d, (780, 345, 1262, 700), L, 1 + int(p * len(L)), 'python tests/test_aurora.py')
        cx, cy = 200 + 500 * f, 120 + 500 * f
    else:
        im, d = base('AuroraArm — o que o pacote sustenta')
        d.text((40, 60), 'Honestidade técnica', font=f_t, fill=(255, 255, 255))
        d.text((40, 125), 'Sustenta (no modelo, reprodutível por run_all.py)', font=f_h, fill=(120, 220, 140))
        a = ['[OK] REQ-CTRL-001..004 nos cenários declarados', '[OK] REQ-SAFE-001..004: P1–P3 exaustivos + 16/16 testes', '[OK] Código e testes gerados do mesmo modelo, com rastreio']
        b = ['[--] C real ainda não compilado/executado (espelho float32; harness pronto)', '[--] Checker próprio, não qualificado (repetir em UPPAAL/NuSMV)', '[--] Sem HIL, sem medição de tempo/memória no Cortex-M4',
             '[--] Sem análise de força de contato (ISO/TS 15066); freio é premissa', '[--] ISO 26262: qualificação de ferramentas, independência, MC/DC']
        bullets(d, 60, 165, a, 1 + int(p * 3), f=f_b, col=(200, 235, 205), gap=38)
        d.text((40, 300), 'Ainda exigiria processo formal de certificação', font=f_h, fill=(240, 130, 120))
        bullets(d, 60, 345, b, int(max(0, (p - .35) / .65 * 5)) + (1 if p > .35 else 0), f=f_b, col=(245, 205, 200), gap=42)
        cx, cy = 900, 500
    cursor(d, int(cx), int(cy))
    return im


# ---------------- AVI ----------------
def write_avi(path, frames_jpg, pcm):
    n = len(frames_jpg); spf = SR // FPS  # amostras de audio por quadro
    chunks = []; idx = []; off = 4
    for k, jp in enumerate(frames_jpg):
        for tag, data in ((b'00dc', jp), (b'01wb', pcm[k * spf * 2:(k + 1) * spf * 2])):
            pad = len(data) % 2
            chunks.append(tag + struct.pack('<I', len(data)) + data + (b'\0' * pad)); idx.append((tag, 0x10, off, len(data)))
            off += 8 + len(data) + pad
    movi = b'LIST' + struct.pack('<I', 4 + sum(len(c) for c in chunks)) + b'movi' + b''.join(chunks)
    idx1 = b'idx1' + struct.pack('<I', 16 * len(idx)) + b''.join(t + struct.pack('<III', fl, o, s) for t, fl, o, s in idx)
    avih = struct.pack('<IIIIIIIIIIIIII', 1000000 // FPS, 0, 0, 0x10, n, 0, 2, 1 << 20, W, H, 0, 0, 0, 0)
    strh_v = b'vidsMJPG' + struct.pack('<IHHIIIIIIIIhhhh', 0, 0, 0, 0, 1, FPS, 0, n, 1 << 20, 0xFFFFFFFF, 0, 0, 0, W, H)
    strf_v = struct.pack('<IiiHHIIiiII', 40, W, H, 1, 24, 0x47504A4D, W * H * 3, 0, 0, 0, 0)
    na = len(frames_jpg) * spf
    strh_a = b'audsPCM\0' [:8] + struct.pack('<IHHIIIIIIIIhhhh', 0, 0, 0, 0, 2, SR * 2, 0, na, 1 << 16, 0xFFFFFFFF, 2, 0, 0, 0, 0)
    strf_a = struct.pack('<HHIIHHH', 1, 1, SR, SR * 2, 2, 16, 0)
    ck = lambda t, d: t + struct.pack('<I', len(d)) + d + (b'\0' * (len(d) % 2))
    lst = lambda t, d: b'LIST' + struct.pack('<I', 4 + len(d)) + t + d
    hdrl = lst(b'hdrl', ck(b'avih', avih) + lst(b'strl', ck(b'strh', strh_v) + ck(b'strf', strf_v)) + lst(b'strl', ck(b'strh', strh_a) + ck(b'strf', strf_a)))
    body = b'AVI ' + hdrl + movi + idx1
    open(path, 'wb').write(b'RIFF' + struct.pack('<I', len(body)) + body)


def main():
    wavs = tts(); pcm = b''; durs = []
    for w in wavs:
        with wave.open(w) as wf:
            assert wf.getframerate() == SR and wf.getnchannels() == 1
            fr = wf.readframes(wf.getnframes()); pad = int(0.6 * SR) * 2  # 0,6 s de respiro entre cenas
            pcm += fr + b'\0' * pad; durs.append((len(fr) + pad) / 2 / SR)
    print('duracoes por cena (s):', [round(x, 1) for x in durs], 'total', round(sum(durs), 1))
    if sum(durs) > 300:
        sys.exit('video excede 5 minutos')
    frames = []
    for i, dur in enumerate(durs):
        nf = int(round(dur * FPS))
        for k in range(nf):
            buf = io.BytesIO(); scene(i, k / max(1, nf - 1)).save(buf, 'JPEG', quality=82); frames.append(buf.getvalue())
    pcm = pcm[:len(frames) * (SR // FPS) * 2].ljust(len(frames) * (SR // FPS) * 2, b'\0')
    out = os.path.join(OUT, 'AuroraArm_pitch.avi'); write_avi(out, frames, pcm)
    print('gerado', out, os.path.getsize(out) // 1024, 'KB', len(frames), 'quadros')


if __name__ == '__main__':
    main()
