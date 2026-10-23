#%%
import os
import sys
import time
import csv
import json
import faulthandler
import clr
import ctypes
import math
from collections import deque
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

kinesis_path = r"C:\Program Files\Thorlabs\Kinesis"
if hasattr(os, 'add_dll_directory'):
    os.add_dll_directory(kinesis_path)
sys.path.append(kinesis_path)

clr.AddReference("Thorlabs.MotionControl.DeviceManagerCLI")
clr.AddReference("Thorlabs.MotionControl.GenericPiezoCLI")
clr.AddReference("Thorlabs.MotionControl.KCube.PiezoStrainGaugeCLI")
clr.AddReference("Thorlabs.MotionControl.KCube.PositionAlignerCLI")

from System import Decimal
from System import Exception as DotNetException
from Thorlabs.MotionControl.DeviceManagerCLI import DeviceManagerCLI
from Thorlabs.MotionControl.GenericPiezoCLI.Piezo import PiezoControlModeTypes
from Thorlabs.MotionControl.KCube.PiezoStrainGaugeCLI import KCubePiezoStrainGauge
from Thorlabs.MotionControl.KCube.PositionAlignerCLI import KCubePositionAligner

# =============================================================================
# CONFIGURAÇÃO
# =============================================================================
SERIAL_X = "113252111"
SERIAL_Y = "113251965"
SERIAL_DQ = "69254247"

LIMIAR_SOMA = 0.25          # soma mínima do detector para a posição ser válida
V_MIN, V_MAX = 0.0, 150.0   # faixa do piezo
V_BASE = 75.0               # ponto de partida do alinhamento

# --- Pré-centralização (malha aberta, antes de ligar o controle) -------------
CAL_DELTA_V = 20.0          # excursão ±V usada para medir a sensibilidade
CAL_T_SETTLE = 0.6          # s de espera após cada degrau (creep do piezo)
CAL_T_MEDIA = 2.0           # s de média por ponto (o detector oscila)
CENTRO_T_SETTLE = 1.0
CENTRO_T_MEDIA = 2.0
CENTRO_TOL = 0.03           # raio (unid. normalizada) para considerar centralizado
CENTRO_MAX_ITER = 10
CENTRO_PASSO = 0.8          # fração da correção aplicada por iteração (evita overshoot)
SENSIB_MIN = 8e-4           # sensibilidade mínima (posição/V); abaixo disso aborta
COND_MAX = 8.0              # condição máxima da matriz de sensibilidade
ACOPLAMENTO_MIN = 0.25      # termo cruzado < 25% do diagonal é tratado como ruído de medição

# --- Validação (também em malha aberta) --------------------------------------
VAL_DURACAO = 15.0          # s de observação
VAL_RAIO = 0.20             # o feixe deve ficar dentro deste raio...
VAL_FRACAO_MIN = 0.70       # ...durante pelo menos esta fração do tempo
VAL_TOL_MEDIANA = 0.05      # |mediana| máxima de (x, y)
VAL_TOL_DERIVA = 0.05       # deriva máxima entre 1ª e 2ª metade da janela
VAL_FRACAO_VALIDA_MIN = 0.95
V_MARGEM = 15.0             # folga mínima até os limites 0/150 V (para acomodar deriva)
VAL_MAX_TENTATIVAS = 3
FORCAR_INICIO = True       # True = inicia o controle mesmo se a validação falhar (Cenário muito turbulendo falha validação)

# --- Comunicação USB (reduz carga na fila de mensagens dos cubos) -----------
POLL_DQ_MS = 15             # polling do KPA101: define a taxa real de amostragem do detector
POLL_PIEZO_MS = 100         # os KPC101 só recebem comandos; não precisam de polling rápido
CMD_PERIODO_S = 0.033       # no máximo 1 comando de tensão por cubo a cada 33 ms (~30 Hz); 0 = sem limite
CMD_DELTA_MIN_V = 0.05      # não reenvia se a tensão mudou menos que isto
MAX_ERROS_USB = 20          # erros consecutivos de comando antes de abortar com segurança
MAX_FALHAS_LEITURA_S = 3.0  # s sem ler o detector antes de abortar com segurança
WATCHDOG_S = 20.0           # s sem progresso até gravar a pilha no crash_log.txt

# --- Controle ----------------------------------------------------------------
GANHOS = {
    "Calmo":      dict(kp=0.30, ki=5.0,  alpha=0.60),
    "Turbulento": dict(kp=0.45, ki=35.0, alpha=0.50),
}
SIGMA_LIMIAR_TURB = 0.045   # com histerese de ±15 % para não ficar alternando
MAX_STEP_V = 8.0            # limite de variação de tensão por amostra


#%%
# =============================================================================
# DIAGNÓSTICO DE TRAVAMENTOS / CRASHES NATIVOS
# Se o processo morrer dentro da DLL do Kinesis (sem traceback Python), o
# faulthandler grava em crash_log.txt a linha Python que estava em execução.
# Se o código ficar PRESO numa chamada .NET, o watchdog grava a pilha de todas
# as threads a cada WATCHDOG_S segundos sem progresso.
# =============================================================================
_LOG_CRASH = open("crash_log.txt", "a", buffering=1)
_LOG_CRASH.write(f"\n--- execução iniciada em {time.ctime()} ---\n")
faulthandler.enable(file=_LOG_CRASH, all_threads=True)
_ultimo_armado = 0.0


def armar_watchdog():
    """Reinicia a contagem do watchdog (no máximo 1x por segundo)."""
    global _ultimo_armado
    agora = time.time()
    if agora - _ultimo_armado >= 1.0:
        faulthandler.cancel_dump_traceback_later()
        faulthandler.dump_traceback_later(WATCHDOG_S, repeat=True, file=_LOG_CRASH)
        _ultimo_armado = agora

def manter_windows_ativo(ativo=True):
    ES_CONTINUOUS = 0x80000000
    ES_SYSTEM_REQUIRED = 0x00000001
    try:
        if ativo:
            ctypes.windll.kernel32.SetThreadExecutionState(ES_CONTINUOUS | ES_SYSTEM_REQUIRED)
        else:
            ctypes.windll.kernel32.SetThreadExecutionState(ES_CONTINUOUS)
    except Exception:
        pass
    
# CÓDIGO PARA ZEROING (descarga suave dos atuadores)
def zeroing_suave(serial_x, serial_y):
    DeviceManagerCLI.BuildDeviceList()
    time.sleep(1.0)

    print("Conectando aos cubos para o Zeroing...")
    cubo_x = KCubePiezoStrainGauge.CreateKCubePiezoStrainGauge(serial_x)
    cubo_y = KCubePiezoStrainGauge.CreateKCubePiezoStrainGauge(serial_y)

    try:
        if cubo_x.IsConnected: cubo_x.Disconnect()
        if cubo_y.IsConnected: cubo_y.Disconnect()
    except:
        pass

    DeviceManagerCLI.BuildDeviceList()
    time.sleep(2)
    lista = DeviceManagerCLI.GetDeviceList()
    print("Dispositivos encontrados:")
    for d in lista: print(d)

    cubo_x.Connect(serial_x)
    cubo_y.Connect(serial_y)
    cubo_x.StartPolling(20)
    cubo_y.StartPolling(20)
    time.sleep(0.2)

    cubo_x.EnableDevice()
    cubo_y.EnableDevice()
    time.sleep(0.2)

    if not cubo_x.IsSettingsInitialized(): cubo_x.WaitForSettingsInitialized(10000)
    if not cubo_y.IsSettingsInitialized(): cubo_y.WaitForSettingsInitialized(10000)

    cubo_x.SetPositionControlMode(PiezoControlModeTypes.OpenLoop)
    cubo_y.SetPositionControlMode(PiezoControlModeTypes.OpenLoop)
    time.sleep(0.1)

    print("Zerando os cubos...")
    cubo_x.SetOutputVoltage(Decimal(0.0))
    cubo_y.SetOutputVoltage(Decimal(0.0))
    time.sleep(1.0)

    cubo_x.StopPolling()
    cubo_y.StopPolling()
    cubo_x.Disconnect()
    cubo_y.Disconnect()
    print("Zeroing concluído.\n")


#%%
# FUNÇÃO DE PLOTAGEM DOS RESULTADOS
def plotar_resultados_csv(file_path, tempo_loop_aberto=None):
    if not os.path.exists(file_path):
        print(f"Arquivo '{file_path}' não encontrado para plotagem.")
        return

    print(f"\nGerando gráficos a partir de '{file_path}'...")
    df = pd.read_csv(file_path)

    if df.empty:
        print("O arquivo CSV está vazio. Nenhum gráfico será gerado.")
        return

    fig, axs = plt.subplots(3, 1, figsize=(10, 12), sharex=True)

    axs[0].plot(df['tempo_s'], df['diff_x'], label='diff_x', color='blue', alpha=0.7)
    axs[0].plot(df['tempo_s'], df['diff_y'], label='diff_y', color='red', alpha=0.7)
    axs[0].axhline(0.0, color='black', linestyle=':', alpha=0.6)
    axs[0].set_ylabel('Diferença (diff)')
    axs[0].set_title('Diferença (X e Y) ao longo do tempo')
    axs[0].grid(True)

    axs[1].plot(df['tempo_s'], df['x_pos_real'], label='x_pos_real', color='blue', alpha=0.7)
    axs[1].plot(df['tempo_s'], df['y_pos_real'], label='y_pos_real', color='red', alpha=0.7)
    axs[1].axhline(0.0, color='black', linestyle=':', alpha=0.6)
    axs[1].set_ylabel('Posição Real')
    axs[1].set_title('Posição Real (X e Y) ao longo do tempo')
    axs[1].grid(True)

    axs[2].plot(df['tempo_s'], df['tensao_x'], label='tensao_x', color='blue', alpha=0.7)
    axs[2].plot(df['tempo_s'], df['tensao_y'], label='tensao_y', color='red', alpha=0.7)
    axs[2].axhline(V_MIN, color='black', linestyle=':', alpha=0.4)
    axs[2].axhline(V_MAX, color='black', linestyle=':', alpha=0.4)
    axs[2].set_xlabel('Tempo (s)')
    axs[2].set_ylabel('Tensão (V)')
    axs[2].set_title('Tensão de Controle (X e Y) ao longo do tempo')
    axs[2].grid(True)

    if tempo_loop_aberto is not None:
        for ax in axs:
            ax.axvline(
                x=tempo_loop_aberto, color='green', linestyle='--', alpha=0.8,
                label='Início Malha Fechada'
            )

    for ax in axs:
        ax.legend(loc='upper right')

    plt.tight_layout()
    plt.show()


#%%
# =============================================================================
# FUNÇÕES AUXILIARES DE MEDIÇÃO E ATUAÇÃO
# =============================================================================
def _ler_dq(cubo_dq):
    st = cubo_dq.Status
    dx = float(str(st.PositionDifference.X).replace(',', '.'))
    dy = float(str(st.PositionDifference.Y).replace(',', '.'))
    s = float(str(st.Sum).replace(',', '.'))
    return dx, dy, s


def amostrar_posicao(cubo_dq, duracao, delay=0.010):
    """Lê o detector por 'duracao' s. Devolve (x, y, soma, n_tentativas)."""
    xs, ys, ss, total = [], [], [], 0
    t0 = time.time()
    while time.time() - t0 < duracao:
        armar_watchdog()
        total += 1
        try:
            dx, dy, s = _ler_dq(cubo_dq)
            if s > LIMIAR_SOMA:
                xs.append(dx / s)
                ys.append(dy / s)
                ss.append(s)
        except Exception:
            pass
        time.sleep(delay)
    return np.array(xs), np.array(ys), np.array(ss), total


def posicao_media(cubo_dq, duracao):
    xs, ys, ss, _ = amostrar_posicao(cubo_dq, duracao)
    if len(xs) < 10:
        raise RuntimeError(
            "Sinal do detector insuficiente (soma abaixo do limiar). "
            "Verifique se o feixe está incidindo no detector."
        )
    return float(np.mean(xs)), float(np.mean(ys)), float(np.mean(ss))


def aplicar_tensoes(cubo_x, cubo_y, vx, vy):
    cubo_x.SetOutputVoltage(Decimal(round(float(vx), 2)))
    cubo_y.SetOutputVoltage(Decimal(round(float(vy), 2)))


#%%
# =============================================================================
# PRÉ-CENTRALIZAÇÃO E VALIDAÇÃO (malha aberta, antes de ligar o controle)
# =============================================================================
def calibrar_sensibilidade(cubo_x, cubo_y, cubo_dq, v0):
    """
    Mede a matriz J (posição por volt):
        [dpos_x]   [Jxx Jxy] [dVx]
        [dpos_y] = [Jyx Jyy] [dVy]
    Já captura sinal, sensibilidade de cada eixo, acoplamento e eixos trocados.
    """
    for vi in v0:
        if vi - CAL_DELTA_V < V_MIN or vi + CAL_DELTA_V > V_MAX:
            raise RuntimeError(
                f"Ponto de operação {v0} V sem espaço para a calibração de ±{CAL_DELTA_V:.0f} V."
            )

    print(f"Medindo sensibilidade dos eixos (±{CAL_DELTA_V:.0f} V, {CAL_T_MEDIA:.1f}s por ponto)...")
    J = np.zeros((2, 2))
    for col in (0, 1):
        pos = {}
        for lado, dv in (("b", -CAL_DELTA_V), ("a", +CAL_DELTA_V)):
            v = list(v0)
            v[col] += dv
            aplicar_tensoes(cubo_x, cubo_y, *v)
            time.sleep(CAL_T_SETTLE)
            px, py, _ = posicao_media(cubo_dq, CAL_T_MEDIA)
            pos[lado] = (px, py)
        J[0, col] = (pos["a"][0] - pos["b"][0]) / (2 * CAL_DELTA_V)
        J[1, col] = (pos["a"][1] - pos["b"][1]) / (2 * CAL_DELTA_V)
    aplicar_tensoes(cubo_x, cubo_y, *v0)
    time.sleep(CAL_T_SETTLE)

    # Com turbulência forte os termos cruzados saem ruidosos: se forem pequenos
    # frente ao termo diagonal da mesma linha, são zerados (eixos desacoplados).
    for lin, col_cruz in ((0, 1), (1, 0)):
        if abs(J[lin, col_cruz]) < ACOPLAMENTO_MIN * abs(J[lin, lin]):
            J[lin, col_cruz] = 0.0

    sv = np.linalg.svd(J, compute_uv=False)
    cond = sv[0] / sv[1] if sv[1] > 0 else float('inf')
    print(f"  J (posição/V) = [[{J[0,0]:+.5f}, {J[0,1]:+.5f}], [{J[1,0]:+.5f}, {J[1,1]:+.5f}]]")
    print(f"  Cubo_x ({SERIAL_X}) -> dX={J[0,0]*1000:+.2f}  dY={J[1,0]*1000:+.2f}  (mpos/V)")
    print(f"  Cubo_y ({SERIAL_Y}) -> dX={J[0,1]*1000:+.2f}  dY={J[1,1]*1000:+.2f}  (mpos/V)")

    if abs(J[0, 1] * J[1, 0]) > abs(J[0, 0] * J[1, 1]):
        print("  [!] Cubos X e Y parecem cruzados (resposta predominante fora da diagonal). "
              "O controle compensa isso automaticamente.")
    if sv[1] < SENSIB_MIN:
        raise RuntimeError(
            f"Sensibilidade muito baixa (menor valor singular = {sv[1]:.2e}/V). "
            "Sinal ruidoso, feixe fora do detector ou um dos eixos não responde."
        )
    if cond > COND_MAX:
        print(f"  [!] Matriz mal condicionada (cond={cond:.1f}): os dois cubos movem o feixe "
              "em direções quase iguais; o controle será sensível a ruído.")
    return J


def centralizar(cubo_x, cubo_y, cubo_dq, J, v_ini):
    """Newton amortecido em malha aberta: leva a posição média a (0, 0)."""
    Jinv = np.linalg.inv(J)
    v = np.array(v_ini, dtype=float)
    saturou_antes = False
    convergiu = False
    print("Pré-centralizando o feixe (malha aberta)...")
    for it in range(1, CENTRO_MAX_ITER + 1):
        aplicar_tensoes(cubo_x, cubo_y, *v)
        time.sleep(CENTRO_T_SETTLE)
        px, py, soma = posicao_media(cubo_dq, CENTRO_T_MEDIA)
        r = math.hypot(px, py)
        print(f"  it {it:2d}: pos=({px:+.3f}, {py:+.3f}) r={r:.3f} | V=({v[0]:6.2f}, {v[1]:6.2f}) | soma={soma:.2f}")
        if r < CENTRO_TOL:
            convergiu = True
            break
        v_novo = v - CENTRO_PASSO * (Jinv @ np.array([px, py]))
        v_lim = np.clip(v_novo, V_MIN, V_MAX)
        saturou = bool(np.any(v_lim != v_novo))
        if saturou:
            print("  [!] A correção necessária ultrapassa a faixa do piezo (0–150 V).")
        v = v_lim
        if saturou and saturou_antes:
            print("  [!] Curso insuficiente: é preciso melhorar o alinhamento mecânico.")
            aplicar_tensoes(cubo_x, cubo_y, *v)
            time.sleep(CENTRO_T_SETTLE)
            break
        saturou_antes = saturou
    return v, convergiu


def validar_centralizacao(cubo_dq, v, J):
    """
    Observa o feixe com tensões fixas. Como o detector oscila (turbulência),
    o critério é estatístico: fração do tempo dentro de um raio, mediana
    próxima de zero, ausência de deriva e folga de curso do piezo.
    """
    print(f"Validando centralização ({VAL_DURACAO:.0f}s)...")
    xs, ys, _, total = amostrar_posicao(cubo_dq, VAL_DURACAO)
    n = len(xs)
    res = {"ok": False, "motivos": [], "n_amostras": int(n), "v_x": float(v[0]), "v_y": float(v[1])}
    if n < 30:
        res["motivos"].append("sinal do detector insuficiente durante a validação")
        return res

    r = np.hypot(xs, ys)
    meio = n // 2
    res.update({
        "frac_valida": n / total if total else 0.0,
        "frac_dentro_raio": float(np.mean(r <= VAL_RAIO)),
        "mediana_x": float(np.median(xs)),
        "mediana_y": float(np.median(ys)),
        "std_x": float(np.std(xs)),
        "std_y": float(np.std(ys)),
        "sigma_turb": float(math.hypot(np.std(xs), np.std(ys))),
        "deriva": float(math.hypot(
            np.median(xs[meio:]) - np.median(xs[:meio]),
            np.median(ys[meio:]) - np.median(ys[:meio]))),
        "folga_v": float(min(np.min(v) - V_MIN, V_MAX - np.max(v))),
    })
    # Excursão de tensão (2,5 sigma) necessária para cancelar a turbulência observada.
    # Se for maior que a folga, o controle vai encostar nos limites e a rejeição será parcial.
    Jinv = np.linalg.inv(J)
    sigma_v = np.sqrt((Jinv ** 2) @ np.array([np.std(xs) ** 2, np.std(ys) ** 2]))
    res["excursao_v_2p5sigma"] = [float(2.5 * sigma_v[0]), float(2.5 * sigma_v[1])]
    res["aviso_excursao"] = bool(np.any(
        2.5 * sigma_v > np.minimum(v - V_MIN, V_MAX - v)))

    if res["frac_valida"] < VAL_FRACAO_VALIDA_MIN:
        res["motivos"].append(f"sinal válido em apenas {res['frac_valida']*100:.0f}% das leituras")
    if math.hypot(res["mediana_x"], res["mediana_y"]) > VAL_TOL_MEDIANA:
        res["motivos"].append(
            f"mediana fora do centro ({res['mediana_x']:+.3f}, {res['mediana_y']:+.3f})")
    if res["frac_dentro_raio"] < VAL_FRACAO_MIN:
        res["motivos"].append(
            f"apenas {res['frac_dentro_raio']*100:.0f}% do tempo dentro do raio {VAL_RAIO:.2f}")
    if res["deriva"] > VAL_TOL_DERIVA:
        res["motivos"].append(f"deriva de {res['deriva']:.3f} dentro da janela")
    if res["folga_v"] < V_MARGEM:
        res["motivos"].append(
            f"piezo próximo do limite (folga {res['folga_v']:.1f} V < {V_MARGEM:.0f} V): "
            "sem curso para acompanhar deriva; melhore o alinhamento mecânico")
    res["ok"] = len(res["motivos"]) == 0
    return res


def _imprimir_validacao(res):
    if "frac_dentro_raio" in res:
        print(f"  dentro do raio {VAL_RAIO:.2f}: {res['frac_dentro_raio']*100:5.1f}% | "
              f"mediana=({res['mediana_x']:+.3f}, {res['mediana_y']:+.3f}) | "
              f"std=({res['std_x']:.3f}, {res['std_y']:.3f}) | deriva={res['deriva']:.3f} | "
              f"folga={res['folga_v']:.1f} V")
    if "excursao_v_2p5sigma" in res:
        ex = res["excursao_v_2p5sigma"]
        print(f"  excursão de tensão p/ cancelar a turbulência (2,5σ): ±({ex[0]:.0f}, {ex[1]:.0f}) V")
        if res["aviso_excursao"]:
            print("  [aviso] excursão maior que a folga até os limites: o controle vai saturar "
                  "ocasionalmente e a rejeição será parcial.")
    if res["ok"]:
        print("  >>> VALIDAÇÃO APROVADA")
    else:
        print("  >>> VALIDAÇÃO REPROVADA:")
        for m in res["motivos"]:
            print(f"      - {m}")


def preparar_alinhamento(cubo_x, cubo_y, cubo_dq):
    """Calibra, centraliza e valida. Devolve (J, tensões, resultado) ou levanta RuntimeError."""
    v = [V_BASE, V_BASE]
    aplicar_tensoes(cubo_x, cubo_y, *v)
    print(f"Aguardando estabilização mecânica em {V_BASE:.0f} V (3.0s)...")
    time.sleep(3.0)

    J = calibrar_sensibilidade(cubo_x, cubo_y, cubo_dq, v)
    res = None
    for tentativa in range(1, VAL_MAX_TENTATIVAS + 1):
        print(f"\n--- Tentativa de alinhamento {tentativa}/{VAL_MAX_TENTATIVAS} ---")
        if tentativa > 1 and all(CAL_DELTA_V <= vi <= V_MAX - CAL_DELTA_V for vi in v):
            J = calibrar_sensibilidade(cubo_x, cubo_y, cubo_dq, v)  # re-mede no novo ponto
        v, _ = centralizar(cubo_x, cubo_y, cubo_dq, J, v)
        time.sleep(2.0)  # deixa o creep do piezo assentar antes de validar
        res = validar_centralizacao(cubo_dq, v, J)
        _imprimir_validacao(res)
        if res["ok"]:
            return J, v, res

    if FORCAR_INICIO:
        print("\n[!] Validação não aprovada, mas FORCAR_INICIO=True: iniciando assim mesmo.")
        return J, v, res
    raise RuntimeError(
        "Validação de centralização reprovada após "
        f"{VAL_MAX_TENTATIVAS} tentativas: " + "; ".join(res["motivos"])
    )


#%%
# =============================================================================
# FUNÇÃO PRINCIPAL DE CONTROLE EM MALHA FECHADA
# =============================================================================
def executar_malha_fechada_dq_software(
    serial_x, serial_y, serial_dq, tempo_total_segundos,
    tempo_loop_aberto_segundos=100.0, nome_arquivo="PolarisTest/Test1.csv"
):
    manter_windows_ativo(True)
    DeviceManagerCLI.BuildDeviceList()
    time.sleep(1.0)

    print("Iniciando a conexão principal...")
    cubo_x = KCubePiezoStrainGauge.CreateKCubePiezoStrainGauge(serial_x)
    cubo_y = KCubePiezoStrainGauge.CreateKCubePiezoStrainGauge(serial_y)
    cubo_dq = KCubePositionAligner.CreateKCubePositionAligner(serial_dq)

    try:
        if cubo_x.IsConnected: cubo_x.Disconnect()
        if cubo_y.IsConnected: cubo_y.Disconnect()
        if cubo_dq.IsConnected: cubo_dq.Disconnect()
    except:
        pass

    cubo_x.Connect(serial_x)
    cubo_y.Connect(serial_y)
    cubo_dq.Connect(serial_dq)

    cubo_x.StartPolling(POLL_PIEZO_MS)
    cubo_y.StartPolling(POLL_PIEZO_MS)
    cubo_dq.StartPolling(POLL_DQ_MS)
    time.sleep(0.25)

    cubo_x.EnableDevice()
    cubo_y.EnableDevice()
    cubo_dq.EnableDevice()
    time.sleep(0.25)

    f_csv = None
    arquivo_ok = None
    try:
        if not cubo_x.IsSettingsInitialized(): cubo_x.WaitForSettingsInitialized(10000)
        if not cubo_y.IsSettingsInitialized(): cubo_y.WaitForSettingsInitialized(10000)
        if not cubo_dq.IsSettingsInitialized(): cubo_dq.WaitForSettingsInitialized(10000)

        print("Dispositivos prontos! Configurando travas de segurança...")
        max_voltage = Decimal(V_MAX)
        cubo_x.SetMaxOutputVoltage(max_voltage)
        cubo_y.SetMaxOutputVoltage(max_voltage)
        cubo_x.SetPositionControlMode(PiezoControlModeTypes.OpenLoop)
        cubo_y.SetPositionControlMode(PiezoControlModeTypes.OpenLoop)

        # ---------------------------------------------------------------------
        # CALIBRAÇÃO + PRÉ-CENTRALIZAÇÃO + VALIDAÇÃO (aborta se reprovar)
        # ---------------------------------------------------------------------
        J, v_centro, res_val = preparar_alinhamento(cubo_x, cubo_y, cubo_dq)
        Jinv = np.linalg.inv(J)

        pasta_saida = os.path.dirname(nome_arquivo)
        if pasta_saida:
            os.makedirs(pasta_saida, exist_ok=True)

        with open(os.path.splitext(nome_arquivo)[0] + "_validacao.json", "w") as fj:
            json.dump({"jacobiana_pos_por_volt": J.tolist(), "validacao": res_val}, fj, indent=2)

        f_csv = open(nome_arquivo, 'w', newline='')
        escritor_csv = csv.writer(f_csv)
        escritor_csv.writerow([
            'tempo_s', 'modo', 'regime_turb', 'diff_x', 'diff_y', 'intensidade',
            'x_pos_real', 'y_pos_real', 'sigma_turb', 'tensao_x', 'tensao_y', 'saturado'
        ])

        v_alvo = np.array(v_centro, dtype=float)
        v_enviada = v_alvo.copy()
        e_ant = np.zeros(2)
        e_filt = np.zeros(2)
        regime = "Calmo"
        saturado = False
        seg_saturado = 0.0
        ultimo_cmd = 0.0
        n_erros_usb = 0
        total_erros_usb = 0
        falhas_leitura = 0

        janela_x = deque(maxlen=30)
        janela_y = deque(maxlen=30)

        tempo_inicio_total = time.time()
        ultimo_tempo = tempo_inicio_total
        ultimo_print_tempo = -1.0
        contador_leituras = 0
        malha_fechada_ativada = False

        print(f"\nIniciando aquisição em LOOP ABERTO por {tempo_loop_aberto_segundos:.1f}s "
              f"(a partir do ponto centralizado)...")

        while True:
            agora = time.time()
            tempo_decorrido = agora - tempo_inicio_total
            if tempo_decorrido >= tempo_total_segundos:
                break

            dt = agora - ultimo_tempo
            ultimo_tempo = agora
            if dt <= 0.0 or dt > 0.5:
                dt = 0.016

            armar_watchdog()
            try:
                diff_x, diff_y, intensidade = _ler_dq(cubo_dq)
                falhas_leitura = 0
            except (DotNetException, Exception) as e_leitura:
                falhas_leitura += 1
                if falhas_leitura == 1:
                    print(f"\n[usb] falha ao ler o detector: {e_leitura}")
                if falhas_leitura * 0.01 >= MAX_FALHAS_LEITURA_S:
                    raise RuntimeError(
                        f"Sem leitura do detector por {MAX_FALHAS_LEITURA_S:.0f}s: comunicação USB perdida.")
                time.sleep(0.01)
                continue

            if intensidade > LIMIAR_SOMA:
                x_pos_real = diff_x / intensidade
                y_pos_real = diff_y / intensidade
                sinal_valido = True
                janela_x.append(x_pos_real)
                janela_y.append(y_pos_real)
            else:
                x_pos_real = 0.0
                y_pos_real = 0.0
                sinal_valido = False

            if len(janela_x) >= 10:
                media_x = sum(janela_x) / len(janela_x)
                media_y = sum(janela_y) / len(janela_y)
                var_x = sum((x - media_x) ** 2 for x in janela_x) / len(janela_x)
                var_y = sum((y - media_y) ** 2 for y in janela_y) / len(janela_y)
                sigma_turb = math.sqrt(var_x + var_y)
            else:
                sigma_turb = 0.0

            # Regime com histerese (evita alternar a cada amostra)
            if regime == "Calmo" and sigma_turb > SIGMA_LIMIAR_TURB * 1.15:
                regime = "Turbulento"
            elif regime == "Turbulento" and sigma_turb < SIGMA_LIMIAR_TURB * 0.85:
                regime = "Calmo"

            if tempo_decorrido >= tempo_loop_aberto_segundos and not malha_fechada_ativada:
                print("\n\n>>> ATIVANDO MALHA FECHADA (PI incremental em espaço de posição) <<<")
                malha_fechada_ativada = True
                e_ant = -np.array([x_pos_real, y_pos_real])
                e_filt = e_ant.copy()

            modo_log = "Fechado" if malha_fechada_ativada else "Aberto"
            saturado = False

            if malha_fechada_ativada and sinal_valido:
                g = GANHOS[regime]
                e_bruto = -np.array([x_pos_real, y_pos_real])
                e_filt = g["alpha"] * e_bruto + (1.0 - g["alpha"]) * e_filt

                raio_pos = math.hypot(x_pos_real, y_pos_real)
                if raio_pos < 0.025:
                    fator_geom = 0.5
                elif raio_pos > 0.65:
                    fator_geom = 1.35
                else:
                    fator_geom = 1.0

                # PI incremental na variável de posição desejada...
                dpos = g["kp"] * (e_filt - e_ant) + g["ki"] * fator_geom * e_filt * dt
                e_ant = e_filt.copy()

                # ...convertido em volts pela sensibilidade medida (sinal, ganho e
                # acoplamento de cada eixo já embutidos em J)
                dv = np.clip(Jinv @ dpos, -MAX_STEP_V, MAX_STEP_V)
                v_novo = v_alvo + dv
                v_alvo = np.clip(v_novo, V_MIN, V_MAX)
                saturado = bool(np.any(v_alvo != v_novo))

                # Comandos limitados em taxa: o v_alvo continua sendo atualizado a cada
                # amostra, mas só é enviado ao cubo a cada CMD_PERIODO_S.
                if agora - ultimo_cmd >= CMD_PERIODO_S:
                    vx_cmd, vy_cmd = round(v_alvo[0], 2), round(v_alvo[1], 2)
                    try:
                        if abs(vx_cmd - v_enviada[0]) >= CMD_DELTA_MIN_V:
                            cubo_x.SetOutputVoltage(Decimal(vx_cmd))
                            v_enviada[0] = vx_cmd
                        if abs(vy_cmd - v_enviada[1]) >= CMD_DELTA_MIN_V:
                            cubo_y.SetOutputVoltage(Decimal(vy_cmd))
                            v_enviada[1] = vy_cmd
                        ultimo_cmd = agora
                        n_erros_usb = 0
                    except (DotNetException, Exception) as e_cmd:
                        n_erros_usb += 1
                        total_erros_usb += 1
                        if n_erros_usb == 1:
                            print(f"\n[usb] erro ao enviar tensão: {e_cmd}")
                            _LOG_CRASH.write(f"{time.ctime()} erro SetOutputVoltage: {e_cmd}\n")
                        if n_erros_usb >= MAX_ERROS_USB:
                            raise RuntimeError(
                                f"{MAX_ERROS_USB} erros consecutivos ao enviar tensão: comunicação USB com os cubos falhando.")

                seg_saturado = seg_saturado + dt if saturado else 0.0
                if seg_saturado > 2.0 and int(seg_saturado) != int(seg_saturado - dt):
                    print(f"\n[!] Atuador saturado há {seg_saturado:.0f}s "
                          f"(V=({v_alvo[0]:.1f}, {v_alvo[1]:.1f})): falta curso, realinhe o espelho.")

            escritor_csv.writerow([
                f"{tempo_decorrido:.3f}", modo_log, regime,
                diff_x, diff_y, intensidade,
                x_pos_real, y_pos_real,
                round(sigma_turb, 5),
                round(v_alvo[0], 2), round(v_alvo[1], 2),
                int(saturado)
            ])

            if tempo_decorrido - ultimo_print_tempo >= 1.0:
                ultimo_print_tempo = tempo_decorrido
                tempo_restante = max(0.0, tempo_total_segundos - tempo_decorrido)
                mins, segs = divmod(int(tempo_restante), 60)
                progresso = (tempo_decorrido / tempo_total_segundos) * 100.0
                print(
                    f"\r[Status: {modo_log:7s} | {regime:10s}] "
                    f"Restante: {mins:02d}m {segs:02d}s ({progresso:5.1f}%) | "
                    f"Pos: ({x_pos_real:+.3f}, {y_pos_real:+.3f}) | "
                    f"V: ({v_alvo[0]:5.1f}V, {v_alvo[1]:5.1f}V)",
                    end="", flush=True
                )

            if contador_leituras % 50 == 0:
                f_csv.flush()

            contador_leituras += 1
            time.sleep(0.005)

        tempo_total_decorrido = time.time() - tempo_inicio_total
        frequencia_aquisicao = (contador_leituras / tempo_total_decorrido) if tempo_total_decorrido > 0 else 0.0
        print(f"\n\nEnsaio finalizado! {contador_leituras} amostras ({frequencia_aquisicao:.2f} Hz). "
              f"Erros USB de comando: {total_erros_usb}.")
        arquivo_ok = nome_arquivo

    except KeyboardInterrupt:
        print("\n\nControle interrompido pelo usuário.")
        arquivo_ok = nome_arquivo if f_csv is not None else None
    except (DotNetException, Exception) as e:
        print(f"\n\nERRO CAPTURADO: {e}")
        _LOG_CRASH.write(f"{time.ctime()} ERRO CAPTURADO: {e}\n")
        arquivo_ok = nome_arquivo if f_csv is not None else None  # preserva os dados parciais
    finally:
        faulthandler.cancel_dump_traceback_later()
        manter_windows_ativo(False)
        print("\nLiberando hardware com segurança...")
        if f_csv is not None:
            f_csv.close()
        try:
            cubo_x.SetOutputVoltage(Decimal(75.0))
            cubo_y.SetOutputVoltage(Decimal(75.0))
            time.sleep(0.5)
            cubo_x.StopPolling()
            cubo_y.StopPolling()
            cubo_dq.StopPolling()
            cubo_x.Disconnect()
            cubo_y.Disconnect()
            cubo_dq.Disconnect()
            print("Portas USB liberadas.")
        except Exception as e_end:
            print(f"Aviso ao desconectar: {e_end}")

    return arquivo_ok


#%%
# EXECUÇÃO E PLOTAGEM AUTOMÁTICA
if __name__ == "__main__":
    zeroing_suave(SERIAL_X, SERIAL_Y)
    time.sleep(2.0)

    tempo_total_experimento = 1300.0
    tempo_livre_antes_fechar = 100.0
    caminho_csv = "PolarisTest/Test1.csv"

    arquivo_gerado = executar_malha_fechada_dq_software(
        SERIAL_X,
        SERIAL_Y,
        SERIAL_DQ,
        tempo_total_segundos=tempo_total_experimento,
        tempo_loop_aberto_segundos=tempo_livre_antes_fechar,
        nome_arquivo=caminho_csv
    )

    if arquivo_gerado:
        plotar_resultados_csv(arquivo_gerado, tempo_loop_aberto=tempo_livre_antes_fechar)
    else:
        print("Ensaio não executado (validação reprovada ou erro). Nenhum gráfico gerado.")