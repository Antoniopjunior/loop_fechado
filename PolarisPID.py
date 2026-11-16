#%%
import os
import sys
import gc
import time
import csv
import json
import faulthandler
import subprocess
import socket
import ctypes
import math
from collections import deque
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# =============================================================================
# 1. CAMINHOS E IMPORTAÇÃO DAS DLLs DO THORLABS KINESIS (.NET)
# =============================================================================
KINESIS_PATH = r"C:\Program Files\Thorlabs\Kinesis"
if hasattr(os, 'add_dll_directory'):
    os.add_dll_directory(KINESIS_PATH)
sys.path.append(KINESIS_PATH)

# -----------------------------------------------------------------------------
# CHAVES QUE DECIDEM O PAPEL DESTE PROCESSO (precisam vir ANTES dos imports do Kinesis)
# -----------------------------------------------------------------------------
# Escolha o modo do ensaio:
#   "controle"   -> carrega/calibra J, pré-centraliza, (valida) e liga a malha fechada
#   "referencia" -> mantém os dois cubos fixos em V_REFERENCIA (75 V), sem controle (linha de base)
#   "latencia"   -> NÃO controla: mede tempo de leitura/comando USB e o atraso fim a fim
#                   (rode com a TURBULÊNCIA DESLIGADA)
#   "api"        -> NÃO controla: grava em arquivo a lista de métodos/propriedades que o Kinesis
#                   expõe para o KPA101 e os KPC101 (para descobrir o modo de malha fechada em hardware)
MODO_ENSAIO   = "controle"
# True  -> este script vira um SUPERVISOR: a aquisição/controle roda num processo filho. Se o filho
#          morrer por crash nativo (0xc0000374) ou travar, o supervisor o reinicia e o ensaio RETOMA
#          do ponto em que parou (mesmo CSV, mesmas tensões). False -> tudo num processo só (antigo).
SUPERVISIONAR = True
EH_WORKER     = "--worker" in sys.argv            # True no processo filho (acesso ao hardware)
_USA_KINESIS  = EH_WORKER or (not SUPERVISIONAR) or MODO_ENSAIO in ("latencia", "api")

if _USA_KINESIS:
    import clr
    clr.AddReference("Thorlabs.MotionControl.DeviceManagerCLI")
    clr.AddReference("Thorlabs.MotionControl.GenericPiezoCLI")
    clr.AddReference("Thorlabs.MotionControl.KCube.PiezoStrainGaugeCLI")
    clr.AddReference("Thorlabs.MotionControl.KCube.PositionAlignerCLI")

    import System
    from System import Decimal
    from System import Exception as DotNetException
    from Thorlabs.MotionControl.DeviceManagerCLI import DeviceManagerCLI
    from Thorlabs.MotionControl.GenericPiezoCLI.Piezo import PiezoControlModeTypes
    from Thorlabs.MotionControl.KCube.PiezoStrainGaugeCLI import KCubePiezoStrainGauge
    from Thorlabs.MotionControl.KCube.PositionAlignerCLI import KCubePositionAligner
else:
    # O processo supervisor NÃO carrega o Kinesis: não pode ser derrubado pela DLL.
    clr = System = DeviceManagerCLI = PiezoControlModeTypes = None
    KCubePiezoStrainGauge = KCubePositionAligner = None
    Decimal = float
    DotNetException = Exception


#%%
# =============================================================================
# 2. PAINEL DE CONFIGURAÇÃO GLOBAL (TODOS OS PARÂMETROS ALTERÁVEIS)
# =============================================================================

# --- 2.1 Execução Principal, Modos e Caminhos de Arquivos --------------------
# Escolha o modo do ensaio:
#   "controle"   -> calibra/carrega J, pré-centraliza, (valida) e liga a malha fechada
#   "referencia" -> mantém os dois cubos fixos em V_REFERENCIA (75 V), sem controle (linha de base)
#   "latencia"   -> NÃO controla: mede o tempo de leitura USB, de comando USB e o atraso
#                   fim a fim (comando -> detector). Rode com a TURBULÊNCIA DESLIGADA.
# (MODO_ENSAIO e SUPERVISIONAR são definidos no topo do arquivo, antes dos imports.)
FILENAME                  = "Teste1"       # Nome-base para os arquivos CSV e PNG gerados
PASTA_SAIDA               = "Polaris"     # Pasta onde os resultados serão salvos
TEMPO_TOTAL_EXPERIMENTO_S = 3600.0            # Duração total do ensaio (s)
TEMPO_LOOP_ABERTO_S       = 1800.0            # Tempo inicial em malha aberta antes de fechar a malha (s)

CAMINHO_REF_CSV           = f"{PASTA_SAIDA}/{FILENAME}_referencia.csv"
CAMINHO_CTL_CSV           = f"{PASTA_SAIDA}/{FILENAME}_controle.csv"
CAMINHO_LAT_JSON          = f"{PASTA_SAIDA}/{FILENAME}_latencia.json"
CAMINHO_PNG_COMPARACAO    = f"{PASTA_SAIDA}/comparacao_centroides_{FILENAME}.png"
ARQUIVO_LOG_CRASH         = "crash_log.txt"   # Log de diagnóstico do faulthandler

# Parâmetros padrão para a função comparar_centroides():
DESCARTAR_REF_INICIAL_S   = 5.0               # Segundos iniciais descartados no CSV de referência
DESCARTAR_CTL_INICIAL_S   = 30.0              # Segundos de transiente descartados ao fechar a malha
RAIOS_COMPARACAO          = (0.10, 0.20, 0.30)# Raios (unid. normalizada) avaliados na comparação

# --- 2.2 Identificação do Hardware e Limites Físicos -------------------------
SERIAL_X     = "113252111"  # Serial do KCube Piezo (KPC101) do eixo X
SERIAL_Y     = "113251965"  # Serial do KCube Piezo (KPC101) do eixo Y
SERIAL_DQ    = "69254247"   # Serial do KCube Position Aligner (KPA101 - Detector Quadrante)

LIMIAR_SOMA  = 0.25         # Soma mínima (V) do detector para considerar a posição válida
V_MIN, V_MAX = 0.0, 150.0   # Limites físicos de tensão dos atuadores piezoelétricos (V)
V_BASE       = 75.0         # Ponto de partida (V) do alinhamento e tensão de repouso final
V_REFERENCIA = 75.0         # Tensão fixa (V) aplicada aos dois cubos no modo "referencia"
T_ESTABILIZACAO_MEC_S = 3.0 # Espera inicial (s) após aplicar V_BASE ou V_REFERENCIA

# --- 2.3 Pré-Centralização e Calibração Ciente do Ruído (Malha Aberta) -------
# J = sensibilidade posição/V. Medida sob turbulência ela varia até ±50 % (J_y de
# 0,0019 a 0,0045 nos seus ensaios); em ar calmo ela é estável (X≈0,0089, Y≈0,0040).
# Por isso a J salva em ar calmo passa a ser a PREFERIDA quando existir.
J_ARQUIVO          = "jacobiana_polaris.json"  # J calibrada em ar calmo (salva automaticamente)
J_SALVAR           = True   # Salva J automaticamente quando medida em regime calmo
J_PREFERIR_SALVA   = True   # True = usa a J salva (ar calmo) e pula a medição ruidosa
J_FORCAR_RECALIBRAR = False # True = ignora a J salva e mede de novo (use após mexer no arranjo)

CAL_DELTA_V       = 20.0    # Excursão ±V usada para medir a sensibilidade (regime calmo)
CAL_DELTA_V_FORTE = 35.0    # Excursão ±V maior (mais relação sinal/ruído) sob turbulência forte
CAL_T_SETTLE      = 0.6     # Tempo de espera (s) após cada degrau de tensão (creep do piezo)
CAL_T_MEDIA       = 2.0     # Tempo mínimo de média (s) por ponto na calibração
SE_ALVO           = 0.03    # Erro-padrão alvo para dimensionar automaticamente a janela de média
T_MEDIA_MAX       = 100.0   # Teto máximo (s) para a janela de média sob turbulência severa
QUALIDADE_J_MIN   = 4.0     # Razão mínima |J_ii| / incerteza para aceitar a calibração medida
J_SALVAR_SIGMA_MAX = 0.045  # Só salva J se σ do feixe na calibração for menor que isto (ar calmo)
SENSIB_MIN        = 8e-4    # Sensibilidade mínima aceitável (menor valor singular em pos/V)
COND_MAX          = 8.0     # Número de condicionamento máximo aceitável da matriz J
ACOPLAMENTO_MIN   = 0.25    # Termo cruzado < 25% do diagonal é zerado como ruído de medição

CENTRO_T_SETTLE   = 1.0     # Espera (s) após cada passo de tensão durante a pré-centralização
CENTRO_T_MEDIA    = 2.0     # Tempo base de média (s) na pré-centralização
CENTRO_TOL        = 0.03    # Raio alvo (unid. normalizada) para considerar o feixe centralizado
CENTRO_MAX_ITER   = 10      # Número máximo de iterações do método de Newton na centralização
CENTRO_PASSO      = 0.8     # Fração da correção aplicada por iteração (evita overshoot)

# --- 2.4 Validação Estatística (Malha Aberta) --------------------------------
VALIDAR               = False # False = centraliza e inicia direto sem passar pelo portão de validação
VAL_DURACAO           = 15.0  # Janela de observação (s) da validação (use >= 30 s sob turbulência forte)
VAL_TOL_MEDIANA       = 0.05  # |Mediana| máxima permitida para (x, y) em torno da origem
VAL_TOL_DERIVA        = 0.05  # Deriva máxima aceitável entre a 1ª e a 2ª metade da janela
VAL_TAU_CORR          = 0.3   # Tempo de correlação (s) assumido para a turbulência atmosférica
VAL_RAIO              = 0.20  # Raio de dispersão avaliado (obrigatório apenas em regime calmo)
VAL_FRACAO_MIN        = 0.70  # Fração mínima do tempo dentro de VAL_RAIO (em regime calmo)
VAL_FRACAO_VALIDA_MIN = 0.95  # Fração mínima de amostras com soma acima de LIMIAR_SOMA
V_MARGEM              = 15.0  # Folga mínima (V) até os limites 0/150 V para acomodar deriva
VAL_MAX_TENTATIVAS    = 3     # Número máximo de tentativas de centralização + validação
FORCAR_INICIO         = True  # True = inicia o controle mesmo se a validação reprovar

# --- 2.5 Temporização e Comunicação USB (REDUÇÃO DE ATRASO) ------------------
# DIAGNÓSTICO nos seus CSVs: 100 % dos intervalos entre amostras eram múltiplos de
# 15,625 ms (tick padrão do Windows): 77 % de 16 ms e 22 % de 31 ms. O time.sleep()
# só acordava nesses degraus => jitter e atraso extra. As opções abaixo corrigem isso.
ALTA_RESOLUCAO_TIMER  = True   # timeBeginPeriod(1): tick do Windows de 15,6 ms -> 1 ms
PRIORIDADE_PROCESSO   = "acima_do_normal"  # "normal" | "acima_do_normal" | "alta"
ESPERA_ATIVA_S        = 0.002  # Últimos ms de cada espera são feitos em espera ativa (precisão)

# MODO DE LEITURA DO KPA101
#  True  (ALTA TAXA): StartPolling(POLL_DQ_MS) + leitura direta de .Status, como no seu código de
#        ~300 Hz. O dado só é NOVO a cada POLL_DQ_MS: o laço detecta amostras novas, roda o PI e
#        envia comandos só nelas e mede a taxa ÚTIL (veja resumir_temporizacao_csv).
#        Risco: a thread de polling nativa concorre com a leitura (origem provável do crash
#        0xc0000374). O supervisor (SUPERVISIONAR=True) absorve as quedas.
#  False (SEGURO): sem polling; RequestStatus() síncrono a cada leitura (~16 ms, ~62 Hz).
USAR_POLLING_ASSINCRONO_DQ = True

POLL_DQ_MS            = 2     # Polling do KPA101 (ms) = taxa máxima de dado NOVO (2 ms -> 500 Hz).
                              # Teste 1, 2, 3 e 5 ms e compare a taxa útil e o nº de reinícios.
POLL_PIEZO_MS         = 100   # Período de polling dos KPC101 (usado apenas na rotina de Zeroing)
PERIODO_LOOP_S        = 0.003 # Período de LEITURA do laço (s). Com 0,001 o laço lê a cada 1 ms e pega o
                              # dado novo com <=1 ms de atraso (mais leituras de .Status e CPU em 100 %).
                              # Se a leitura demorar mais que isso, o laço roda no ritmo possível.
CMD_PERIODO_S         = 0.0   # Intervalo mínimo entre comandos de tensão (s). 0 = a cada amostra.
                              # (antes 0,033: o limite de taxa somava ~16-33 ms de atraso ao laço)
CMD_DELTA_MIN_V       = 0.02  # Variação mínima de tensão (V) para enviar comando USB aos cubos
MAX_ERROS_USB         = 20    # Erros consecutivos de comando USB antes de abortar com segurança
MAX_FALHAS_LEITURA_S  = 3.0   # Tempo máximo (s) sem conseguir ler o KPA101 antes de abortar
WATCHDOG_S            = 20.0  # Tempo (s) sem resposta até gravar a pilha de threads no crash_log.txt
INTERVALO_FLUSH_CSV   = 500   # Salva o buffer do CSV no disco a cada N leituras do laço
INTERVALO_GC_COLLECT  = 3000  # Ciclo controlado de limpeza do Garbage Collector a cada N leituras do laço
LOG_A_CADA_N_AMOSTRAS = 1     # Grava no CSV 1 a cada N amostras NOVAS (o PI usa todas). Use 2-4 p/ CSV menor.

# --- 2.6 Controlador PI Incremental e Adaptação de Regime --------------------
# Ganhos reotimizados por simulação sobre os seus dados (modelo validado nas 4 réplicas
# com atraso ~64 ms): reduzem a amplificação em 3-31 Hz e a atividade do piezo (~24 V ->
# ~16 V rms) sem perder redução de σ. Valem para X e Y.
GANHOS = {
    # kp: fração do erro corrigida por amostra (adimensional)
    # ki: ganho integral em 1/s (banda integral aproximada em rad/s)
    # alpha: filtro passa-baixas exponencial (1.0 = sem filtro, que só acrescenta atraso)
    "Calmo":      dict(kp=0.30, ki=4.0, alpha=1.0),
    "Turbulento": dict(kp=0.20, ki=2.0, alpha=1.0),
}
JANELA_TURB_S         = 0.5   # Janela móvel (s) para calcular sigma_turb em tempo real
PERIODO_AMOSTRA_S     = max(PERIODO_LOOP_S, POLL_DQ_MS / 1000.0) if USAR_POLLING_ASSINCRONO_DQ else PERIODO_LOOP_S
JANELA_TURB_AMOSTRAS  = max(10, int(round(JANELA_TURB_S / PERIODO_AMOSTRA_S)))
SIGMA_LIMIAR_TURB     = 0.045 # Limiar de desvio-padrão para chavear entre "Calmo" e "Turbulento"
HISTERESE_TURB        = 0.15  # Banda de histerese (±15%) para evitar alternância rápida de ganhos
MAX_STEP_V            = 8.0   # Variação máxima de tensão (V) permitida por iteração

# Escalonamento geométrico do ganho integral conforme a distância radial ao centro:
RAIO_ZONA_CENTRO      = 0.025 # Abaixo deste raio, reduz o ganho integral para não amplificar ruído
FATOR_GEOM_CENTRO     = 0.50  # Multiplicador de ki quando raio < RAIO_ZONA_CENTRO
RAIO_ZONA_BORDA       = 0.65  # Acima deste raio, acelera o ganho integral para evitar perda do feixe
FATOR_GEOM_BORDA      = 1.35  # Multiplicador de ki quando raio > RAIO_ZONA_BORDA

# Recuo automático de ganho (backoff) por saturação do piezo: com turbulência forte a parte
# rápida consome o curso (principalmente do Y) que a deriva lenta precisa. Se o piezo
# saturar demais, os ganhos são reduzidos; quando normaliza, voltam aos poucos.
BACKOFF_ATIVO         = True
BACKOFF_JANELA_S      = 60.0  # Janela (s) em que a fração de saturação é medida
BACKOFF_SAT_ALTA      = 0.02  # Acima de 2 % de amostras saturadas -> reduz os ganhos
BACKOFF_SAT_BAIXA     = 0.005 # Abaixo de 0,5 % -> recupera os ganhos
BACKOFF_FATOR_MIN     = 0.25  # Os ganhos nunca caem abaixo de 25 % do nominal
BACKOFF_INTERVALO_S   = 10.0  # Intervalo mínimo (s) entre ajustes

# --- 2.7 Teste de Latência (MODO_ENSAIO = "latencia") ------------------------
LAT_N_LEITURAS        = 400   # Leituras consecutivas do KPA101 para medir o tempo de uma leitura
LAT_N_COMANDOS        = 60    # Comandos SetOutputVoltage para medir o tempo de uma chamada
LAT_DELTA_V           = 20.0  # Degrau de tensão (V) do teste fim a fim
LAT_N_REP             = 40    # Repetições do degrau por eixo (a média alinhada reduz o ruído)
LAT_T_REPOUSO_S       = 0.8   # Repouso (s) antes de cada degrau
LAT_JANELA_S          = 0.6   # Janela (s) de observação após cada degrau

# --- 2.8 Supervisão: sobreviver a crashes nativos (0xc0000374) ---------------
MAX_REINICIOS         = 30    # Máximo de reinícios automáticos por ensaio
HANG_S                = 45.0  # s sem sinal de vida (heartbeat) => considera travado e mata o processo
PAUSA_REINICIO_S      = 6.0   # s de espera antes de reiniciar (deixa o USB/Kinesis liberar as portas)
RETOMAR_ENSAIO_ANTERIOR = False  # True: continua um ensaio interrompido (usa o arquivo de estado existente)
USAR_MONITOR_LOCK     = True  # False: lê o Status sem Monitor.Enter/Exit (para testar se o lock influencia)

# --- 2.9 Interface gráfica: telemetria ao vivo e parada ----------------------
LIVE_PORTA            = 0     # porta UDP local para enviar a posição em tempo real à interface (0 = desligado)
LIVE_INTERVALO_S      = 0.05  # um pacote a cada 50 ms (20 Hz)
LIVE_MAX_PONTOS       = 150   # máximo de pontos por pacote (decima se o laço for mais rápido)
PARAR_FLAG            = ""    # caminho de um arquivo-sinal: se existir, o ensaio para com segurança


#%%
# =============================================================================
# 3. DIAGNÓSTICO, ENERGIA, PRIORIDADE E TEMPORIZAÇÃO PRECISA (WINDOWS)
# =============================================================================
_LOG_CRASH = open(ARQUIVO_LOG_CRASH, "a", buffering=1)
_LOG_CRASH.write(f"\n--- execução iniciada em {time.ctime()} ---\n")
faulthandler.enable(file=_LOG_CRASH, all_threads=True)
_ultimo_armado = 0.0


_HB_PATH = None


def _tocar_heartbeat():
    """
    O QUE EXECUTA:
        Grava um "sinal de vida" em arquivo (lido pelo supervisor). Se o processo de
        aquisição ficar mais de `HANG_S` segundos sem atualizá-lo, o supervisor o encerra
        e reinicia.
    """
    if _HB_PATH:
        try:
            with open(_HB_PATH, "w") as f:
                f.write(str(time.time()))
        except Exception:
            pass


class _LiveSender:
    """
    O QUE EXECUTA:
        Envia a posição do feixe em tempo real para a interface gráfica por UDP local
        (best-effort: se ninguém estiver ouvindo, nada acontece). Desligado se LIVE_PORTA = 0.
    """

    def __init__(self):
        self.sock = None
        self.addr = None
        self.v = (None, None)          # últimas tensões aplicadas (atualizadas por aplicar_tensoes)
        self._buf = {k: [] for k in ("t", "x", "y", "vx", "vy", "s")}
        self._meta = {}
        self._ultimo = 0.0

    def abrir(self, porta):
        if not porta:
            return
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setblocking(False)
            self.addr = ("127.0.0.1", int(porta))
        except Exception:
            self.sock = None

    def add(self, t, x, y, soma, modo, regime="", v=None, sat=0, fator=1.0, taxa=0.0, te=None, tt=None):
        if self.sock is None:
            return
        vx, vy = v if v is not None else self.v
        b = self._buf
        b["t"].append(t); b["x"].append(x); b["y"].append(y)
        b["vx"].append(vx); b["vy"].append(vy); b["s"].append(soma)
        self._meta = dict(m=modo, rg=regime, sat=sat, fg=fator, tx=taxa, te=te, tt=tt)

    def flush(self, forcar=False):
        if self.sock is None or not self._buf["t"]:
            return
        agora = time.perf_counter()
        if not forcar and (agora - self._ultimo) < LIVE_INTERVALO_S:
            return
        self._ultimo = agora
        n = len(self._buf["t"])
        passo = max(1, n // max(1, LIVE_MAX_PONTOS))
        pacote = {"k": "d"}
        for k, lista in self._buf.items():
            pacote[k] = [None if a is None else round(a, 5) for a in lista[::passo]] if k != "t" \
                else [round(a, 4) for a in lista[::passo]]
        pacote.update(self._meta)
        try:
            self.sock.sendto(json.dumps(pacote).encode("utf-8"), self.addr)
        except Exception:
            pass
        for lista in self._buf.values():
            lista.clear()


_LIVE = _LiveSender()


def armar_watchdog():
    """
    O QUE EXECUTA:
        Reinicia a contagem do watchdog nativo (`faulthandler`) no máximo 1 vez
        por segundo. Caso o interpretador Python trave dentro de uma chamada da
        DLL do Kinesis por mais de `WATCHDOG_S` segundos, grava o traceback de
        todas as threads em `ARQUIVO_LOG_CRASH`.
    """
    global _ultimo_armado
    agora = time.time()
    if agora - _ultimo_armado >= 1.0:
        faulthandler.cancel_dump_traceback_later()
        faulthandler.dump_traceback_later(WATCHDOG_S, repeat=True, file=_LOG_CRASH)
        _ultimo_armado = agora
        _tocar_heartbeat()
        if PARAR_FLAG and os.path.exists(PARAR_FLAG):   # pedido de parada vindo da interface
            raise KeyboardInterrupt("parada solicitada pela interface")


def manter_windows_ativo(ativo=True):
    """
    O QUE EXECUTA:
        Impede que o Windows suspenda o sistema operacional ou desligue o
        controlador USB por economia de energia durante experimentos longos.

    PARÂMETROS:
        - ativo (bool): True mantém o sistema acordado; False restaura o padrão.
    """
    ES_CONTINUOUS = 0x80000000
    ES_SYSTEM_REQUIRED = 0x00000001
    try:
        if ativo:
            ctypes.windll.kernel32.SetThreadExecutionState(ES_CONTINUOUS | ES_SYSTEM_REQUIRED)
        else:
            ctypes.windll.kernel32.SetThreadExecutionState(ES_CONTINUOUS)
    except Exception:
        pass


_PRIORIDADES_WIN = {"normal": 0x00000020, "acima_do_normal": 0x00008000, "alta": 0x00000080}


def configurar_desempenho_windows(ativo=True):
    """
    O QUE EXECUTA:
        Reduz o jitter de temporização do Windows durante o ensaio:
        - `timeBeginPeriod(1)`: o tick do sistema cai de 15,6 ms para 1 ms, de modo que
          `time.sleep()` deixa de acordar só em múltiplos de 15,6 ms (a causa dos
          intervalos de 16/31 ms vistos nos CSVs);
        - eleva a prioridade do processo (`PRIORIDADE_PROCESSO`).
        Chame com `ativo=True` no início e `ativo=False` no final (balanceado).
    """
    if os.name != "nt":
        return
    try:
        if ALTA_RESOLUCAO_TIMER:
            if ativo:
                ctypes.windll.winmm.timeBeginPeriod(1)
            else:
                ctypes.windll.winmm.timeEndPeriod(1)
        classe = _PRIORIDADES_WIN.get(PRIORIDADE_PROCESSO, _PRIORIDADES_WIN["normal"])
        if not ativo:
            classe = _PRIORIDADES_WIN["normal"]
        ctypes.windll.kernel32.SetPriorityClass(ctypes.windll.kernel32.GetCurrentProcess(), classe)
    except Exception:
        pass


def esperar_ate(t_alvo):
    """
    O QUE EXECUTA:
        Espera até `time.perf_counter() >= t_alvo` com precisão de ~100 µs: dorme a
        maior parte do tempo e faz espera ativa (cedendo a CPU com sleep(0)) nos
        últimos `ESPERA_ATIVA_S` segundos.
    """
    while True:
        falta = t_alvo - time.perf_counter()
        if falta <= 0.0:
            return
        if falta > ESPERA_ATIVA_S:
            time.sleep(falta - ESPERA_ATIVA_S)
        else:
            time.sleep(0)


def zeroing_suave(serial_x, serial_y):
    """
    O QUE EXECUTA:
        Conecta aos dois cubos piezoelétricos (KPC101), configura ambos em malha
        aberta (`OpenLoop`), aplica 0.0 V para descarregar os atuadores de forma
        segura e desconecta as portas USB antes do início do ensaio.

    PARÂMETROS:
        - serial_x, serial_y (str): Números de série dos cubos X e Y.
    """
    DeviceManagerCLI.BuildDeviceList()
    time.sleep(1.0)

    print("Conectando aos cubos para o Zeroing...")
    cubo_x = KCubePiezoStrainGauge.CreateKCubePiezoStrainGauge(serial_x)
    cubo_y = KCubePiezoStrainGauge.CreateKCubePiezoStrainGauge(serial_y)

    try:
        if cubo_x.IsConnected: cubo_x.Disconnect()
        if cubo_y.IsConnected: cubo_y.Disconnect()
    except Exception:
        pass

    DeviceManagerCLI.BuildDeviceList()
    time.sleep(2.0)
    lista = DeviceManagerCLI.GetDeviceList()
    print("Dispositivos encontrados:")
    for d in lista:
        print(d)

    cubo_x.Connect(serial_x)
    cubo_y.Connect(serial_y)
    cubo_x.StartPolling(POLL_PIEZO_MS)
    cubo_y.StartPolling(POLL_PIEZO_MS)
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
# =============================================================================
# 4. ANÁLISE ESTATÍSTICA, COMPARAÇÃO DE CENTROIDES E PLOTAGEM
# =============================================================================
def _carregar_posicoes(csv_path, modo_csv, descartar_s):
    """
    O QUE EXECUTA:
        Filtra um arquivo CSV de ensaio buscando apenas as linhas pertencentes ao
        `modo_csv` especificado ("Referencia" ou "Fechado") cuja soma óptica supere
        `LIMIAR_SOMA`, descartando os primeiros `descartar_s` segundos de acomodação.

    RETORNA:
        - (tempos, x_pos, y_pos): Três arrays NumPy com os dados filtrados.
    """
    df = pd.read_csv(csv_path)
    d = df[(df["modo"] == modo_csv) & (df["intensidade"] > LIMIAR_SOMA)].copy()
    if d.empty:
        raise ValueError(f"'{csv_path}' não tem amostras válidas no modo '{modo_csv}'.")
    d = d[d["tempo_s"] >= d["tempo_s"].min() + descartar_s]
    return d["tempo_s"].values, d["x_pos_real"].values, d["y_pos_real"].values


def _metricas(t, x, y, raios):
    """
    O QUE EXECUTA:
        Calcula métricas estatísticas de apontamento sobre as séries temporais (x, y):
        posição do centroide, distância à origem, incerteza padrão do centroide
        corrigida pelo tempo de correlação `VAL_TAU_CORR`, desvios-padrão, raio RMS
        e fração do tempo dentro de cada raio de `raios`.
    """
    dur = t.max() - t.min()
    r = np.hypot(x, y)
    n_eff = max(5.0, dur / VAL_TAU_CORR)
    return {
        "duracao_s": float(dur), "n": int(len(x)),
        "centroide_x": float(np.mean(x)), "centroide_y": float(np.mean(y)),
        "dist_centroide_origem": float(math.hypot(np.mean(x), np.mean(y))),
        "incerteza_centroide": float(math.hypot(np.std(x), np.std(y)) / math.sqrt(n_eff)),
        "std_x": float(np.std(x)), "std_y": float(np.std(y)),
        "rms_raio": float(np.sqrt(np.mean(r ** 2))),
        **{f"frac_r<{rr:.2f}": float(np.mean(r <= rr)) for rr in raios},
    }


def comparar_centroides(
    csv_referencia, csv_controle,
    descartar_ref_s=None, descartar_ctl_s=None,
    raios=None, salvar_png=None, mostrar=True
):
    """
    O QUE EXECUTA:
        Compara estatística e visualmente o comportamento do feixe SEM controle
        (arquivo CSV gerado em modo `'referencia'`, tensão fixa em 75 V) contra o
        feixe COM controle (trecho `'Fechado'` do CSV de controle). Trunca ambos
        os conjuntos para a mesma duração temporal, imprime tabela comparativa e
        gera painel duplo (nuvem de dispersão + função distribuição acumulada radial).

        ATENÇÃO: só compare ensaios feitos na MESMA sessão, com o mesmo alinhamento
        mecânico. Se o centroide da referência estiver muito longe de (0, 0) (por
        exemplo |y| ~ 0,8), o detector está fora da zona linear e a comparação de
        dispersão não é válida.
    """
    # padrões lidos na hora da chamada (a interface pode ter alterado as constantes)
    descartar_ref_s = DESCARTAR_REF_INICIAL_S if descartar_ref_s is None else descartar_ref_s
    descartar_ctl_s = DESCARTAR_CTL_INICIAL_S if descartar_ctl_s is None else descartar_ctl_s
    raios = RAIOS_COMPARACAO if raios is None else raios
    tr, xr, yr = _carregar_posicoes(csv_referencia, "Referencia", descartar_ref_s)
    tc, xc, yc = _carregar_posicoes(csv_controle, "Fechado", descartar_ctl_s)
    dur = min(tr.max() - tr.min(), tc.max() - tc.min())
    mr = (tr - tr.min()) <= dur
    mc = (tc - tc.min()) <= dur
    tr, xr, yr = tr[mr], xr[mr], yr[mr]
    tc, xc, yc = tc[mc], xc[mc], yc[mc]
    ref = _metricas(tr, xr, yr, raios)
    ctl = _metricas(tc, xc, yc, raios)

    print(f"\nComparação sobre {dur:.0f}s de dados em cada ensaio")
    print(f"{'':28s}{'SEM controle':>16s}{'COM controle':>16s}")
    linhas = [
        ("centroide X", "centroide_x", "{:+.4f}"), ("centroide Y", "centroide_y", "{:+.4f}"),
        ("|centroide| (dist. à origem)", "dist_centroide_origem", "{:.4f}"),
        ("incerteza do centroide (±)", "incerteza_centroide", "{:.4f}"),
        ("desvio-padrão X", "std_x", "{:.4f}"), ("desvio-padrão Y", "std_y", "{:.4f}"),
        ("raio rms (em torno de 0,0)", "rms_raio", "{:.4f}"),
    ] + [(f"fração dentro do raio {rr:.2f}", f"frac_r<{rr:.2f}", "{:.1%}") for rr in raios]
    for nome, chave, fmt in linhas:
        print(f"{nome:28s}{fmt.format(ref[chave]):>16s}{fmt.format(ctl[chave]):>16s}")

    reducao = (
        1.0 - ctl["dist_centroide_origem"] / ref["dist_centroide_origem"]
        if ref["dist_centroide_origem"] > 0 else float('nan')
    )
    print(f"\nO controle moveu o centroide de ({ref['centroide_x']:+.3f}, {ref['centroide_y']:+.3f}) "
          f"para ({ctl['centroide_x']:+.3f}, {ctl['centroide_y']:+.3f}) "
          f"(distância à origem {abs(reducao)*100:.0f}% {'menor' if reducao >= 0 else 'MAIOR'}).")
    print("Obs.: só compare ensaios feitos com a mesma turbulência e o mesmo arranjo óptico.")

    fig, axs = plt.subplots(1, 2, figsize=(13, 6))
    passo = max(1, len(xr) // 15000)
    axs[0].scatter(xr[::passo], yr[::passo], s=3, alpha=0.25, color="tab:red", label="sem controle (75 V)")
    passo = max(1, len(xc) // 15000)
    axs[0].scatter(xc[::passo], yc[::passo], s=3, alpha=0.25, color="tab:blue", label="com controle")
    axs[0].plot(ref["centroide_x"], ref["centroide_y"], "X", color="darkred", ms=14, mec="k", label="centroide sem controle")
    axs[0].plot(ctl["centroide_x"], ctl["centroide_y"], "X", color="navy", ms=14, mec="k", label="centroide com controle")
    axs[0].plot(0, 0, "+", color="k", ms=18, mew=2)
    for rr in raios:
        axs[0].add_patch(plt.Circle((0, 0), rr, fill=False, ls="--", color="k", lw=1.0, alpha=0.8, zorder=5))
    axs[0].set_aspect("equal"); axs[0].grid(True, alpha=0.4)
    axs[0].set_xlabel("x (normalizado)"); axs[0].set_ylabel("y (normalizado)")
    axs[0].set_title("Nuvem de pontos e centroides"); axs[0].legend(loc="upper right", fontsize=8)
    rmax = max(np.percentile(np.hypot(xr, yr), 99.5), np.percentile(np.hypot(xc, yc), 99.5))
    grade = np.linspace(0, rmax, 200)
    axs[1].plot(grade, [np.mean(np.hypot(xr, yr) <= g) for g in grade], color="tab:red", label="sem controle")
    axs[1].plot(grade, [np.mean(np.hypot(xc, yc) <= g) for g in grade], color="tab:blue", label="com controle")
    axs[1].set_xlabel("raio à origem"); axs[1].set_ylabel("fração do tempo dentro do raio")
    axs[1].set_title("Distribuição acumulada da distância ao centro"); axs[1].grid(True, alpha=0.4); axs[1].legend()
    plt.tight_layout()
    if salvar_png:
        plt.savefig(salvar_png, dpi=130)
    if mostrar:
        plt.show()
    return {"sem_controle": ref, "com_controle": ctl}


def resumir_temporizacao_csv(file_path):
    """
    O QUE EXECUTA:
        Lê as colunas de temporização do CSV e imprime a taxa do laço (leituras de Status),
        a taxa ÚTIL (amostras NOVAS, as únicas que trazem informação), a fração de leituras
        repetidas e os tempos de leitura/comando USB.
    """
    try:
        df = pd.read_csv(file_path)
        if "periodo_ms" not in df.columns:
            return
    except Exception:
        return
    print("\nTemporização (a partir do CSV):")
    if "n_laco" in df.columns and "dt_util_ms" in df.columns:
        t = df["tempo_s"].values
        n = df["n_laco"].values
        seg = np.concatenate([[0], np.cumsum(np.diff(n) < 0)])   # n_laco reinicia a cada reinício do worker
        iters = secs = 0.0
        for k in np.unique(seg):
            m = seg == k
            iters += n[m].max() - n[m].min() + 1
            secs += t[m].max() - t[m].min()
        du = df["dt_util_ms"].values
        du = du[(du > 0) & (du < 500)]
        taxa_laco = iters / secs if secs > 0 else float('nan')
        taxa_util = 1000.0 / np.mean(du)
        print(f"  taxa do laço (leituras de Status): {taxa_laco:.0f} Hz | taxa ÚTIL (amostras novas): {taxa_util:.0f} Hz | "
              f"leituras repetidas: {max(0.0, 1.0 - taxa_util / taxa_laco) * 100:.0f}%")
        print(f"  intervalo entre amostras novas: mediana={np.median(du):.2f} ms | p5={np.percentile(du, 5):.2f} ms | "
              f"p95={np.percentile(du, 95):.2f} ms | p99={np.percentile(du, 99):.2f} ms | "
              f"> 2x mediana: {np.mean(du > 2.0 * np.median(du)) * 100:.1f}%")
    else:
        p = df["periodo_ms"].values[1:]
        print(f"  período entre amostras: mediana={np.median(p):.1f} ms | p95={np.percentile(p, 95):.1f} ms | "
              f"p99={np.percentile(p, 99):.1f} ms | máx={p.max():.1f} ms | taxa efetiva={1000.0/np.mean(p):.1f} Hz")
    lr = df["lat_leitura_ms"].values
    print(f"  tempo de uma leitura de Status: mediana={np.median(lr):.2f} ms | p95={np.percentile(lr, 95):.2f} ms")
    lc = df.loc[df["lat_cmd_ms"] > 0, "lat_cmd_ms"].values
    if len(lc):
        print(f"  tempo de uma chamada SetOutputVoltage (X+Y): mediana={np.median(lc):.2f} ms | p95={np.percentile(lc, 95):.2f} ms "
              f"({len(lc)} comandos)")


def plotar_resultados_csv(file_path, tempo_loop_aberto=None):
    """
    O QUE EXECUTA:
        Plota a evolução temporal de um único ensaio em 3 subplots:
        1. Diferenças brutas (diff_x, diff_y);
        2. Posições reais normalizadas (x_pos_real, y_pos_real);
        3. Tensões enviadas aos piezos (tensao_x, tensao_y).
    """
    if not os.path.exists(file_path):
        print(f"Arquivo '{file_path}' não encontrado para plotagem.")
        return

    print(f"\nGerando gráficos a partir de '{file_path}'...")
    df = pd.read_csv(file_path)

    if df.empty:
        print("O arquivo CSV está vazio. Nenhum gráfico será gerado.")
        return
    passo_plot = max(1, len(df) // 150000)   # com taxas altas o CSV tem milhões de pontos
    if passo_plot > 1:
        print(f"  (plotando 1 a cada {passo_plot} pontos)")
        df = df.iloc[::passo_plot]

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

    try:  # marca os reinícios automáticos (crashes) no gráfico
        with open(os.path.splitext(file_path)[0] + "_eventos.json") as f:
            eventos = json.load(f)
        for k, ev in enumerate(eventos):
            if ev.get("t_rel") is not None:
                for ax in axs:
                    ax.axvline(ev["t_rel"], color='red', linestyle=':', alpha=0.9,
                               label='Reinício (crash)' if k == 0 else None)
    except Exception:
        pass

    for ax in axs:
        ax.legend(loc='upper right')

    plt.tight_layout()
    plt.show()


#%%
# =============================================================================
# 5. FUNÇÕES AUXILIARES DE MEDIÇÃO E ATUAÇÃO (PROTEGIDAS CONTRA 0xc0000374)
# =============================================================================
def _ler_dq(cubo_dq):
    """
    O QUE EXECUTA:
        Lê os sinais diferenciais X, Y e a Soma do detector KPA101 prevenindo o
        erro fatal do Windows `0xc0000374` (Heap Corruption na DLL do Kinesis):
        - Quando `USAR_POLLING_ASSINCRONO_DQ = False`, chama `cubo_dq.RequestStatus()`
          de forma síncrona na mesma thread Python, evitando que uma thread C++ em
          background realoque a struct `Status` enquanto o Python a lê.
        - Protege o acesso com `System.Threading.Monitor.Enter/Exit` e converte os
          números via `System.Convert.ToDouble` (sem criar strings intermediárias).
    """
    if not USAR_POLLING_ASSINCRONO_DQ:
        cubo_dq.RequestStatus()

    lock_ok = False
    try:
        if USAR_MONITOR_LOCK:
            System.Threading.Monitor.Enter(cubo_dq)
            lock_ok = True

        st = cubo_dq.Status
        if st is None:
            raise RuntimeError("Status do KPA101 retornou None.")

        pos_diff = st.PositionDifference
        dx = float(System.Convert.ToDouble(pos_diff.X))
        dy = float(System.Convert.ToDouble(pos_diff.Y))
        s = float(System.Convert.ToDouble(st.Sum))

        del pos_diff
        del st
        return dx, dy, s
    finally:
        if lock_ok:
            System.Threading.Monitor.Exit(cubo_dq)


def amostrar_posicao(cubo_dq, duracao, delay=None):
    """
    O QUE EXECUTA:
        Amostra o detector quadrante durante `duracao` segundos com período alvo
        `delay` (temporização precisa por `esperar_ate`). Filtra apenas amostras
        com `s > LIMIAR_SOMA`.

    RETORNA:
        - (xs, ys, ss, total): Arrays NumPy de X, Y, Soma e contagem de leituras.
    """
    delay = PERIODO_LOOP_S if delay is None else delay
    xs, ys, ss, total = [], [], [], 0
    t_fim = time.perf_counter() + duracao
    t_prox = time.perf_counter()
    chave_ant = None
    while time.perf_counter() < t_fim:
        armar_watchdog()
        try:
            dx, dy, s = _ler_dq(cubo_dq)
            chave = (dx, dy, s)
            if chave != chave_ant:   # só amostras NOVAS (com polling rápido o Status se repete)
                chave_ant = chave
                total += 1
                if s > LIMIAR_SOMA:
                    xs.append(dx / s)
                    ys.append(dy / s)
                    ss.append(s)
                    _LIVE.add(time.time(), dx / s, dy / s, s, "Preparação")
                else:
                    _LIVE.add(time.time(), 0.0, 0.0, s, "Preparação")
                _LIVE.flush()
        except Exception:
            total += 1
        t_prox += delay
        agora = time.perf_counter()
        if t_prox < agora:
            t_prox = agora
        esperar_ate(t_prox)
    return np.array(xs), np.array(ys), np.array(ss), total


def posicao_media(cubo_dq, duracao):
    """
    O QUE EXECUTA:
        Mede o feixe durante `duracao` segundos e retorna as médias e desvios-padrão.

    RETORNA:
        - (média_x, média_y, média_soma, std_x, std_y)
    """
    xs, ys, ss, _ = amostrar_posicao(cubo_dq, duracao)
    if len(xs) < 10:
        raise RuntimeError(
            "Sinal do detector insuficiente (soma abaixo do limiar). "
            "Verifique se o feixe está incidindo no detector."
        )
    return (float(np.mean(xs)), float(np.mean(ys)), float(np.mean(ss)),
            float(np.std(xs)), float(np.std(ys)))


def tempo_de_media(sigma):
    """
    O QUE EXECUTA:
        Calcula dinamicamente a duração da janela de média (s) necessária para que
        o erro-padrão da média fique próximo de `SE_ALVO`, limitada entre
        `CAL_T_MEDIA` e `T_MEDIA_MAX`.
    """
    return float(min(T_MEDIA_MAX, max(CAL_T_MEDIA, VAL_TAU_CORR * (sigma / SE_ALVO) ** 2)))


def aplicar_tensoes(cubo_x, cubo_y, vx, vy):
    """
    O QUE EXECUTA:
        Envia tensões `vx` e `vy` (arredondadas em 2 casas decimais) aos cubos X e Y.
    """
    _LIVE.v = (float(vx), float(vy))
    cubo_x.SetOutputVoltage(Decimal(round(float(vx), 2)))
    cubo_y.SetOutputVoltage(Decimal(round(float(vy), 2)))


def conectar_cubos(serial_x, serial_y, serial_dq):
    """
    O QUE EXECUTA:
        Cria os três objetos de dispositivo, garante que estejam desconectados,
        conecta e habilita (sem iniciar polling de nenhum deles).

    RETORNA:
        - (cubo_x, cubo_y, cubo_dq)
    """
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
    except Exception:
        pass

    cubo_x.Connect(serial_x)
    cubo_y.Connect(serial_y)
    cubo_dq.Connect(serial_dq)

    cubo_x.EnableDevice()
    cubo_y.EnableDevice()
    cubo_dq.EnableDevice()
    time.sleep(0.25)
    return cubo_x, cubo_y, cubo_dq


def inicializar_cubos(cubo_x, cubo_y, cubo_dq):
    """
    O QUE EXECUTA:
        Espera a inicialização das configurações, inicia polling do KPA101 apenas
        se `USAR_POLLING_ASSINCRONO_DQ=True`, aplica o limite de tensão e coloca os
        KPC101 em malha aberta.
    """
    if not cubo_x.IsSettingsInitialized(): cubo_x.WaitForSettingsInitialized(10000)
    if not cubo_y.IsSettingsInitialized(): cubo_y.WaitForSettingsInitialized(10000)
    if not cubo_dq.IsSettingsInitialized(): cubo_dq.WaitForSettingsInitialized(10000)

    # Nos KPC101 nunca iniciamos polling no ensaio principal: só recebem SetOutputVoltage.
    if USAR_POLLING_ASSINCRONO_DQ:
        cubo_dq.StartPolling(POLL_DQ_MS)
    time.sleep(0.25)

    print("Dispositivos prontos! Configurando travas de segurança...")
    max_voltage = Decimal(V_MAX)
    cubo_x.SetMaxOutputVoltage(max_voltage)
    cubo_y.SetMaxOutputVoltage(max_voltage)
    cubo_x.SetPositionControlMode(PiezoControlModeTypes.OpenLoop)
    cubo_y.SetPositionControlMode(PiezoControlModeTypes.OpenLoop)


def liberar_cubos(cubo_x, cubo_y, cubo_dq):
    """
    O QUE EXECUTA:
        Devolve os piezos para `V_BASE`, para o polling (se ativo) e desconecta as
        portas USB, sem propagar exceções.
    """
    try:
        cubo_x.SetOutputVoltage(Decimal(V_BASE))
        cubo_y.SetOutputVoltage(Decimal(V_BASE))
        time.sleep(0.5)
        if USAR_POLLING_ASSINCRONO_DQ:
            cubo_dq.StopPolling()
        cubo_x.Disconnect()
        cubo_y.Disconnect()
        cubo_dq.Disconnect()
        print("Portas USB liberadas.")
    except Exception as e_end:
        print(f"Aviso ao desconectar: {e_end}")


#%%
# =============================================================================
# 6. PRÉ-CENTRALIZAÇÃO, GESTÃO DA JACOBIANA E VALIDAÇÃO (MALHA ABERTA)
# =============================================================================
def calibrar_sensibilidade(cubo_x, cubo_y, cubo_dq, v0, t_media, delta_v):
    """
    O QUE EXECUTA:
        Mede a matriz Jacobiana `J` (2x2, posição por Volt) aplicando excursões
        simétricas `±delta_v` em torno de `v0` e estima a matriz de incerteza `SE`
        (1 desvio-padrão de cada elemento de `J`). Zera termos fora da diagonal
        que sejam menores que `ACOPLAMENTO_MIN * |J_ii|` ou que `2 * SE`.

    RETORNA:
        - (J, SE): Matriz Jacobiana 2x2 e matriz de erro-padrão 2x2.
    """
    delta_v = min(delta_v, min(v0) - V_MIN, V_MAX - max(v0))
    if delta_v < 10.0:
        raise RuntimeError(f"Ponto de operação {v0} V sem espaço para a calibração.")

    print(f"Medindo sensibilidade dos eixos (±{delta_v:.0f} V, {t_media:.1f}s por ponto)...")
    J = np.zeros((2, 2))
    SE = np.zeros((2, 2))
    n_eff = max(5.0, t_media / VAL_TAU_CORR)
    for col in (0, 1):
        pos = {}
        for lado, dv in (("b", -delta_v), ("a", +delta_v)):
            v = list(v0)
            v[col] += dv
            aplicar_tensoes(cubo_x, cubo_y, *v)
            time.sleep(CAL_T_SETTLE)
            px, py, _, sx, sy = posicao_media(cubo_dq, t_media)
            pos[lado] = (px, py, sx, sy)
        for lin in (0, 1):
            J[lin, col] = (pos["a"][lin] - pos["b"][lin]) / (2.0 * delta_v)
            SE[lin, col] = math.hypot(pos["a"][2 + lin], pos["b"][2 + lin]) / math.sqrt(n_eff) / (2.0 * delta_v)
    aplicar_tensoes(cubo_x, cubo_y, *v0)
    time.sleep(CAL_T_SETTLE)

    for lin, col_cruz in ((0, 1), (1, 0)):
        if abs(J[lin, col_cruz]) < max(ACOPLAMENTO_MIN * abs(J[lin, lin]), 2.0 * SE[lin, col_cruz]):
            J[lin, col_cruz] = 0.0
    return J, SE


def qualidade_de_J(J, SE):
    """
    O QUE EXECUTA:
        Calcula a relação sinal-ruído da matriz Jacobiana (`|J_ii| / SE_ii`) no pior eixo.
    """
    return float(min(abs(J[0, 0]) / max(SE[0, 0], 1e-12), abs(J[1, 1]) / max(SE[1, 1], 1e-12)))


def _verificar_J(J):
    """
    O QUE EXECUTA:
        Inspeciona os valores singulares e o condicionamento da matriz `J`, avisando
        se os cabos X/Y estiverem invertidos ou se a sensibilidade for insuficiente.
    """
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
            "Sinal ruidoso, feixe fora do detector ou um dos eixos não responde.")
    if cond > COND_MAX:
        print(f"  [!] Matriz mal condicionada (cond={cond:.1f}): os dois cubos movem o feixe "
              "em direções quase iguais; o controle será sensível a ruído.")


def salvar_J(J, SE, sigma):
    """
    O QUE EXECUTA:
        Salva a matriz Jacobiana recém-medida em `J_ARQUIVO` (JSON) para que possa
        ser reutilizada nos ensaios seguintes (principalmente sob turbulência).
    """
    try:
        with open(J_ARQUIVO, "w") as f:
            json.dump({"serial_x": SERIAL_X, "serial_y": SERIAL_Y, "serial_dq": SERIAL_DQ,
                       "quando": time.ctime(), "sigma_na_calibracao": sigma,
                       "J": J.tolist(), "incerteza": SE.tolist()}, f, indent=2)
        print(f"  J salva em '{J_ARQUIVO}' para reutilização.")
    except Exception as e:
        print(f"  Aviso: não foi possível salvar J ({e}).")


def carregar_J_salva():
    """
    O QUE EXECUTA:
        Tenta carregar a matriz Jacobiana de `J_ARQUIVO`, verificando se os números
        de série atuais coincidem com os do arquivo salvo.
    """
    try:
        with open(J_ARQUIVO) as f:
            d = json.load(f)
        if (d["serial_x"], d["serial_y"], d["serial_dq"]) != (SERIAL_X, SERIAL_Y, SERIAL_DQ):
            print("  J salva ignorada: seriais diferentes.")
            return None
        return np.array(d["J"], dtype=float), d.get("quando", "?")
    except Exception:
        return None


def obter_J(cubo_x, cubo_y, cubo_dq, v0, sigma_base):
    """
    O QUE EXECUTA:
        Obtém a matriz `J` na seguinte ordem de preferência:
        1. J salva em ar calmo (`J_PREFERIR_SALVA=True` e `J_FORCAR_RECALIBRAR=False`):
           estável e sem o ruído da turbulência. Se a geometria mudou, a centralização
           detecta a divergência e aborta pedindo recalibração.
        2. J medida agora (janela de média e degrau adaptados a `sigma_base`). Só é
           salva se `sigma_base <= J_SALVAR_SIGMA_MAX` (ar calmo).
        3. Se a medição ficar ruidosa demais, recorre à J salva (ou aborta se não houver).
    """
    if J_PREFERIR_SALVA and not J_FORCAR_RECALIBRAR:
        salva = carregar_J_salva()
        if salva is not None:
            Jsalva, quando = salva
            print(f"  Usando J salva (calibrada em ar calmo em {quando}); medição ruidosa dispensada.")
            print("  Se o arranjo óptico mudou, defina J_FORCAR_RECALIBRAR=True e rode sem turbulência.")
            _verificar_J(Jsalva)
            return Jsalva

    t_media = tempo_de_media(sigma_base)
    delta_v = CAL_DELTA_V_FORTE if sigma_base > SIGMA_LIMIAR_TURB else CAL_DELTA_V
    J, SE = calibrar_sensibilidade(cubo_x, cubo_y, cubo_dq, v0, t_media, delta_v)
    q = qualidade_de_J(J, SE)
    print(f"  Qualidade da calibração (|J|/incerteza, pior eixo) = {q:.1f} (mínimo {QUALIDADE_J_MIN:.0f})")
    if q >= QUALIDADE_J_MIN:
        _verificar_J(J)
        if J_SALVAR and sigma_base <= J_SALVAR_SIGMA_MAX:
            salvar_J(J, SE, sigma_base)
        elif sigma_base > J_SALVAR_SIGMA_MAX:
            print("  (J medida sob turbulência: não foi salva; use uma calibração em ar calmo.)")
        return J
    salva = carregar_J_salva()
    if salva is not None:
        Jsalva, quando = salva
        print(f"  [!] Medição ruidosa demais (turbulência forte). Usando J salva em {quando}.")
        print("      A centralização confirmará se ela ainda vale; se divergir, recalibre sem turbulência.")
        _verificar_J(Jsalva)
        return Jsalva
    raise RuntimeError(
        "Calibração pouco confiável com esta turbulência e não há J salva. Rode uma vez com a "
        f"turbulência desligada para gerar '{J_ARQUIVO}', ou aumente T_MEDIA_MAX.")


def centralizar(cubo_x, cubo_y, cubo_dq, J, v_ini, t_media):
    """
    O QUE EXECUTA:
        Aplica o método de Newton amortecido em malha aberta para levar a posição
        média do feixe até `(0, 0)` dentro da tolerância `max(CENTRO_TOL, 0.5 * SE_ALVO)`.
        Detecta divergência caso a matriz `J` não corresponda mais à geometria óptica.
    """
    Jinv = np.linalg.inv(J)
    v = np.array(v_ini, dtype=float)
    saturou_antes = False
    convergiu = False
    r_melhor = float('inf')
    tol = max(CENTRO_TOL, 0.5 * SE_ALVO)
    print(f"Pré-centralizando o feixe (malha aberta, {t_media:.1f}s por medida)...")
    for it in range(1, CENTRO_MAX_ITER + 1):
        aplicar_tensoes(cubo_x, cubo_y, *v)
        time.sleep(CENTRO_T_SETTLE)
        px, py, soma, _, _ = posicao_media(cubo_dq, t_media)
        r = math.hypot(px, py)
        print(f"  it {it:2d}: pos=({px:+.3f}, {py:+.3f}) r={r:.3f} | V=({v[0]:6.2f}, {v[1]:6.2f}) | soma={soma:.2f}")
        if r < tol:
            convergiu = True
            break
        if r > 2.0 * r_melhor and it >= 3:
            raise RuntimeError(
                "A centralização está divergindo: a J usada provavelmente não vale mais (sinal ou "
                f"geometria mudou). Apague '{J_ARQUIVO}' (ou use J_FORCAR_RECALIBRAR=True) e "
                "recalibre com a turbulência desligada.")
        r_melhor = min(r_melhor, r)
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
    O QUE EXECUTA:
        Observa o feixe por `VAL_DURACAO` segundos com tensões fixas e verifica se
        mediana, deriva e folga de tensão estão dentro dos limites escalonados pelo
        erro-padrão da turbulência.
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
    Jinv = np.linalg.inv(J)
    sigma_v = np.sqrt((Jinv ** 2) @ np.array([np.std(xs) ** 2, np.std(ys) ** 2]))
    res["excursao_v_2p5sigma"] = [float(2.5 * sigma_v[0]), float(2.5 * sigma_v[1])]
    res["aviso_excursao"] = bool(np.any(
        2.5 * sigma_v > np.minimum(v - V_MIN, V_MAX - v)))

    n_eff = max(5.0, VAL_DURACAO / VAL_TAU_CORR)
    se = math.hypot(np.std(xs), np.std(ys)) / math.sqrt(n_eff)
    tol_med = max(VAL_TOL_MEDIANA, 3.0 * se)
    tol_der = max(VAL_TOL_DERIVA, 4.0 * se)
    turbulento = res["sigma_turb"] > SIGMA_LIMIAR_TURB
    res.update({"turbulento": bool(turbulento), "tol_mediana": float(tol_med), "tol_deriva": float(tol_der)})

    if res["frac_valida"] < VAL_FRACAO_VALIDA_MIN:
        res["motivos"].append(f"sinal válido em apenas {res['frac_valida']*100:.0f}% das leituras")
    if math.hypot(res["mediana_x"], res["mediana_y"]) > tol_med:
        res["motivos"].append(
            f"mediana fora do centro ({res['mediana_x']:+.3f}, {res['mediana_y']:+.3f})")
    if (not turbulento) and res["frac_dentro_raio"] < VAL_FRACAO_MIN:
        res["motivos"].append(
            f"apenas {res['frac_dentro_raio']*100:.0f}% do tempo dentro do raio {VAL_RAIO:.2f}")
    if res["deriva"] > tol_der:
        res["motivos"].append(f"deriva de {res['deriva']:.3f} dentro da janela")
    if res["folga_v"] < V_MARGEM:
        res["motivos"].append(
            f"piezo próximo do limite (folga {res['folga_v']:.1f} V < {V_MARGEM:.0f} V): "
            "sem curso para acompanhar deriva; melhore o alinhamento mecânico")
    res["ok"] = len(res["motivos"]) == 0
    return res


def _imprimir_validacao(res):
    """
    O QUE EXECUTA:
        Exibe no terminal o resumo formatado das métricas de validação.
    """
    if "frac_dentro_raio" in res:
        regime_txt = "turbulento (dispersão só informativa)" if res["turbulento"] else "calmo (dispersão validada)"
        print(f"  regime: {regime_txt} | σ={res['sigma_turb']:.3f}")
        print(f"  mediana=({res['mediana_x']:+.3f}, {res['mediana_y']:+.3f}) "
              f"[tol {res['tol_mediana']:.3f}] | deriva={res['deriva']:.3f} [tol {res['tol_deriva']:.3f}] | "
              f"folga={res['folga_v']:.1f} V")
        print(f"  dentro do raio {VAL_RAIO:.2f}: {res['frac_dentro_raio']*100:5.1f}% | "
              f"std=({res['std_x']:.3f}, {res['std_y']:.3f})")
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
    """
    O QUE EXECUTA:
        Mede o nível de turbulência inicial em `V_BASE`, obtém a matriz `J`,
        executa a pré-centralização e (se `VALIDAR=True`) valida a estabilidade.
    """
    v = [V_BASE, V_BASE]
    aplicar_tensoes(cubo_x, cubo_y, *v)
    print(f"Aguardando estabilização mecânica em {V_BASE:.0f} V ({T_ESTABILIZACAO_MEC_S:.1f}s)...")
    time.sleep(T_ESTABILIZACAO_MEC_S)

    _, _, _, sx, sy = posicao_media(cubo_dq, 3.0)
    sigma_base = math.hypot(sx, sy)
    t_media = tempo_de_media(sigma_base)
    print(f"Dispersão do feixe em malha aberta: σ={sigma_base:.3f} "
          f"({'turbulento' if sigma_base > SIGMA_LIMIAR_TURB else 'calmo'}); janela de média = {t_media:.1f}s")

    J = obter_J(cubo_x, cubo_y, cubo_dq, v, sigma_base)
    res = None
    for tentativa in range(1, VAL_MAX_TENTATIVAS + 1):
        print(f"\n--- Tentativa de alinhamento {tentativa}/{VAL_MAX_TENTATIVAS} ---")
        v, _ = centralizar(cubo_x, cubo_y, cubo_dq, J, v, t_media)
        time.sleep(2.0)  # deixa o creep do piezo assentar antes de validar
        if not VALIDAR:
            print("  Validação desativada (VALIDAR=False): iniciando após a centralização.")
            return J, v, {"ok": None, "validacao": "desativada", "v_x": float(v[0]), "v_y": float(v[1])}
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
# 6B. TESTE DE LATÊNCIA (MODO_ENSAIO = "latencia")
# =============================================================================
def _percentis(v):
    v = np.asarray(v, dtype=float)
    return {"mediana_ms": float(np.median(v)), "p95_ms": float(np.percentile(v, 95)),
            "max_ms": float(np.max(v)), "n": int(len(v))}


def medir_latencia_usb(cubo_x, cubo_y, cubo_dq):
    """
    O QUE EXECUTA:
        Quantifica de onde vem o atraso do laço, em três etapas (RODE SEM TURBULÊNCIA):
        1. Tempo de UMA leitura do KPA101 (RequestStatus + Status), leituras seguidas.
        2. Tempo de UMA chamada SetOutputVoltage.
        3. Atraso FIM A FIM: aplica um degrau de `LAT_DELTA_V` volts (alternando subida e
           descida, `LAT_N_REP` vezes por eixo), registra a resposta do detector alinhada
           ao instante do comando e faz a média (isso tira o ruído). Reporta o tempo até a
           resposta atingir 10 % e 63 % do valor final.

    RETORNA:
        - dict com os resultados (também gravado em `CAMINHO_LAT_JSON`).
    """
    pc = time.perf_counter
    resultado = {"quando": time.ctime(), "periodo_loop_alvo_s": PERIODO_LOOP_S}
    aplicar_tensoes(cubo_x, cubo_y, V_BASE, V_BASE)
    print(f"Aguardando estabilização mecânica ({T_ESTABILIZACAO_MEC_S:.1f}s)...")
    time.sleep(T_ESTABILIZACAO_MEC_S)

    # ---- 1) tempo de leitura ----
    print(f"\n[1/3] Tempo de leitura USB ({LAT_N_LEITURAS} leituras seguidas)...")
    t_leit = []
    for _ in range(LAT_N_LEITURAS):
        armar_watchdog()
        t0 = pc()
        try:
            _ler_dq(cubo_dq)
        except Exception:
            continue
        t_leit.append((pc() - t0) * 1000.0)
    if len(t_leit) < 10:
        raise RuntimeError("Leituras do KPA101 falharam durante o teste de latência.")
    resultado["leitura"] = _percentis(t_leit)
    print(f"  leitura: mediana={resultado['leitura']['mediana_ms']:.2f} ms | "
          f"p95={resultado['leitura']['p95_ms']:.2f} ms | máx={resultado['leitura']['max_ms']:.2f} ms")

    # ---- 2) tempo de comando ----
    print(f"\n[2/3] Tempo de chamada SetOutputVoltage ({LAT_N_COMANDOS} comandos)...")
    t_cmd = []
    for i in range(LAT_N_COMANDOS):
        armar_watchdog()
        v = V_BASE + (0.2 if i % 2 else -0.2)
        t0 = pc()
        cubo_x.SetOutputVoltage(Decimal(round(v, 2)))
        t_cmd.append((pc() - t0) * 1000.0)
        time.sleep(0.02)
    cubo_x.SetOutputVoltage(Decimal(V_BASE))
    resultado["comando"] = _percentis(t_cmd)
    print(f"  comando: mediana={resultado['comando']['mediana_ms']:.2f} ms | "
          f"p95={resultado['comando']['p95_ms']:.2f} ms | máx={resultado['comando']['max_ms']:.2f} ms")
    time.sleep(1.0)

    # ---- 3) degrau fim a fim ----
    print(f"\n[3/3] Resposta ao degrau de {LAT_DELTA_V:.0f} V ({LAT_N_REP} repetições por eixo)...")
    grade = np.arange(-0.15, LAT_JANELA_S, 0.004)
    v_lo, v_hi = V_BASE - LAT_DELTA_V / 2.0, V_BASE + LAT_DELTA_V / 2.0
    resultado["degrau"] = {}
    for nome, cubo, idx in (("X", cubo_x, 0), ("Y", cubo_y, 1)):
        cubo.SetOutputVoltage(Decimal(round(v_lo, 2)))
        time.sleep(1.5)
        acc = np.zeros(len(grade))
        n_ok = 0
        nivel_alto = False
        for rep in range(LAT_N_REP):
            armar_watchdog()
            sinal = -1.0 if nivel_alto else +1.0
            novo = v_lo if nivel_alto else v_hi
            amostras = []  # (t_abs, valor)
            t_fim_rep = pc() + LAT_T_REPOUSO_S
            while pc() < t_fim_rep:  # repouso: coleta a linha de base
                try:
                    dx, dy, s = _ler_dq(cubo_dq)
                except Exception:
                    continue
                if s > LIMIAR_SOMA:
                    amostras.append((pc(), (dx / s, dy / s)[idx]))
            t0 = pc()
            cubo.SetOutputVoltage(Decimal(round(novo, 2)))
            t_fim_rep = pc() + LAT_JANELA_S
            while pc() < t_fim_rep:
                try:
                    dx, dy, s = _ler_dq(cubo_dq)
                except Exception:
                    continue
                if s > LIMIAR_SOMA:
                    amostras.append((pc(), (dx / s, dy / s)[idx]))
            nivel_alto = not nivel_alto
            if len(amostras) < 20:
                continue
            ta = np.array([a[0] for a in amostras]) - t0
            va = np.array([a[1] for a in amostras])
            base = va[(ta > -0.3) & (ta < -0.02)]
            if len(base) < 5:
                continue
            acc += np.interp(grade, ta, sinal * (va - base.mean()))
            n_ok += 1
        cubo.SetOutputVoltage(Decimal(V_BASE))
        time.sleep(0.5)
        if n_ok < 5:
            print(f"  eixo {nome}: dados insuficientes.")
            resultado["degrau"][nome] = {"conclusivo": False}
            continue
        media = acc / n_ok
        sigma_b = float(np.std(media[grade < -0.02]))
        final = float(np.mean(media[grade >= 0.4]))
        r = {"n_ok": n_ok, "ganho_final_pos_por_V": final / LAT_DELTA_V, "ruido_base": sigma_b,
             "conclusivo": bool(final > 6.0 * max(sigma_b, 1e-6))}
        if r["conclusivo"]:
            def primeiro_cruzamento(frac):
                lim = max(frac * final, 3.0 * sigma_b)
                ok = media >= lim
                for i in range(len(grade) - 3):
                    if grade[i] >= 0 and ok[i] and ok[i + 1] and ok[i + 2]:
                        return float(grade[i])
                return None
            t10, t63 = primeiro_cruzamento(0.10), primeiro_cruzamento(0.63)
            r.update({"t10_ms": None if t10 is None else t10 * 1000.0,
                      "t63_ms": None if t63 is None else t63 * 1000.0})
            print(f"  eixo {nome}: ganho={r['ganho_final_pos_por_V']*1000:.2f} mpos/V | "
                  f"1ª resposta (10 %) = {r['t10_ms']:.0f} ms | 63 % = {r['t63_ms']:.0f} ms | n={n_ok}")
        else:
            print(f"  eixo {nome}: degrau pequeno demais frente ao ruído (turbulência ligada?). "
                  "Desligue a turbulência ou aumente LAT_DELTA_V / LAT_N_REP.")
        resultado["degrau"][nome] = r

    print("\nInterpretação:")
    print("  - 'leitura' + 'comando' = custo USB por ciclo do laço; o laço não pode ser mais rápido que isso.")
    print("  - '1ª resposta' é o atraso comando -> detector (USB + amplificador + piezo + espelho),")
    print("    com resolução limitada pelo tempo de leitura. O atraso total do controle é ele somado a")
    print("    ~metade do período de amostragem e ao atraso do filtro/limite de taxa de comandos.")
    print("  - Meta: atraso total <= 32 ms (teto teórico sobe de ~21 % para ~42 % de redução no X).")
    try:
        os.makedirs(os.path.dirname(CAMINHO_LAT_JSON) or ".", exist_ok=True)
        with open(CAMINHO_LAT_JSON, "w") as f:
            json.dump(resultado, f, indent=2)
        print(f"  Resultados gravados em '{CAMINHO_LAT_JSON}'.")
    except Exception as e:
        print(f"  Aviso: não foi possível gravar o JSON ({e}).")
    return resultado


def executar_teste_latencia(serial_x, serial_y, serial_dq):
    """
    O QUE EXECUTA:
        Conecta aos três cubos, executa `medir_latencia_usb` e libera o hardware com
        segurança. Não liga nenhum controle.
    """
    manter_windows_ativo(True)
    configurar_desempenho_windows(True)
    cubo_x, cubo_y, cubo_dq = conectar_cubos(serial_x, serial_y, serial_dq)
    resultado = None
    try:
        inicializar_cubos(cubo_x, cubo_y, cubo_dq)
        resultado = medir_latencia_usb(cubo_x, cubo_y, cubo_dq)
    except KeyboardInterrupt:
        print("\n\nTeste interrompido pelo usuário.")
    except (DotNetException, Exception) as e:
        print(f"\n\nERRO CAPTURADO: {e}")
        _LOG_CRASH.write(f"{time.ctime()} ERRO no teste de latência: {e}\n")
    finally:
        faulthandler.cancel_dump_traceback_later()
        configurar_desempenho_windows(False)
        manter_windows_ativo(False)
        print("\nLiberando hardware com segurança...")
        liberar_cubos(cubo_x, cubo_y, cubo_dq)
    return resultado


#%%
# =============================================================================
# 7. FUNÇÃO PRINCIPAL DE AQUISIÇÃO E CONTROLE EM MALHA FECHADA
# =============================================================================
def executar_malha_fechada_dq_software(
    serial_x, serial_y, serial_dq, tempo_total_segundos,
    tempo_loop_aberto_segundos=TEMPO_LOOP_ABERTO_S,
    nome_arquivo=f"{PASTA_SAIDA}/{FILENAME}.csv",
    modo=MODO_ENSAIO,
    retomar=False
):
    """
    O QUE EXECUTA:
        Executa o ensaio principal nos modos:
        - `modo="controle"`: obtém J (preferindo a salva em ar calmo), pré-centraliza,
          registra `tempo_loop_aberto_segundos` em malha aberta e depois ativa o PI
          incremental com recuo automático de ganho por saturação.
        - `modo="referencia"`: Mantém os dois cubos cravados em `V_REFERENCIA` (75 V)
          durante todo o tempo para medir a linha de base sem atuação.

        Temporização: o laço usa `perf_counter` com espera precisa (`esperar_ate`),
        tick de 1 ms (`timeBeginPeriod`) e comandos a cada amostra (`CMD_PERIODO_S=0`).
        O CSV registra `periodo_ms`, `lat_leitura_ms`, `lat_cmd_ms` e `fator_ganho`.
    """
    if modo not in ("controle", "referencia"):
        raise ValueError("modo deve ser 'controle' ou 'referencia'")
    estado_path = _caminho_estado(nome_arquivo)
    estado = _carregar_estado(estado_path) if retomar else None
    if retomar and estado is None:
        raise RuntimeError("Não há arquivo de estado para retomar o ensaio.")
    if retomar:
        modo = estado["modo"]
    _gravar_estado = None
    manter_windows_ativo(True)
    configurar_desempenho_windows(True)
    cubo_x, cubo_y, cubo_dq = conectar_cubos(serial_x, serial_y, serial_dq)

    f_csv = None
    arquivo_ok = None
    gc_estava_ativo = gc.isenabled()
    try:
        inicializar_cubos(cubo_x, cubo_y, cubo_dq)

        # ---------------------------------------------------------------------
        # CALIBRAÇÃO + PRÉ-CENTRALIZAÇÃO + VALIDAÇÃO (ou MODO REFERÊNCIA)
        # ---------------------------------------------------------------------
        if retomar:
            J = None if estado.get("J") is None else np.array(estado["J"], dtype=float)
            Jinv = None if J is None else np.linalg.inv(J)
            v_centro = [float(estado["v_alvo"][0]), float(estado["v_alvo"][1])]
            print(f"\n[retomada] reaplicando V=({v_centro[0]:.2f}, {v_centro[1]:.2f}) V e continuando o ensaio...")
            aplicar_tensoes(cubo_x, cubo_y, *v_centro)
            time.sleep(1.0)
            res_val = {"retomado": True}
            tempo_loop_aberto_segundos = (float('inf') if estado.get("tempo_aberto") is None
                                          else float(estado["tempo_aberto"]))
        elif modo == "referencia":
            v_centro = [V_REFERENCIA, V_REFERENCIA]
            aplicar_tensoes(cubo_x, cubo_y, *v_centro)
            print(f"MODO REFERÊNCIA: cubos fixos em {V_REFERENCIA:.0f} V (sem controle). "
                  f"Aguardando estabilização mecânica ({T_ESTABILIZACAO_MEC_S:.1f}s)...")
            time.sleep(T_ESTABILIZACAO_MEC_S)
            px0, py0, soma0, sx0, sy0 = posicao_media(cubo_dq, 3.0)
            print(f"  Centroide inicial: ({px0:+.3f}, {py0:+.3f}) | σ=({sx0:.3f}, {sy0:.3f}) | soma={soma0:.2f}")
            if math.hypot(px0, py0) > 0.4:
                print("  [!] Centroide da referência longe de (0, 0): o detector está fora da zona linear e a "
                      "comparação com o ensaio controlado não será válida (verifique o alinhamento).")
            J, Jinv = None, None
            res_val = {"ok": None, "modo": "referencia", "v_x": V_REFERENCIA, "v_y": V_REFERENCIA,
                       "centroide_inicial": [px0, py0]}
            tempo_loop_aberto_segundos = float('inf')  # a malha nunca é ligada
        else:
            J, v_centro, res_val = preparar_alinhamento(cubo_x, cubo_y, cubo_dq)
            Jinv = np.linalg.inv(J)

        pasta_saida = os.path.dirname(nome_arquivo)
        if pasta_saida:
            os.makedirs(pasta_saida, exist_ok=True)

        if not retomar:
            with open(os.path.splitext(nome_arquivo)[0] + "_validacao.json", "w") as fj:
                json.dump({"modo": modo, "jacobiana_pos_por_volt": None if J is None else J.tolist(),
                           "validacao": res_val}, fj, indent=2)

        f_csv = open(nome_arquivo, 'a' if retomar else 'w', newline='')
        escritor_csv = csv.writer(f_csv)
        if not retomar:
            escritor_csv.writerow([
                'tempo_s', 'modo', 'regime_turb', 'diff_x', 'diff_y', 'intensidade',
                'x_pos_real', 'y_pos_real', 'sigma_turb', 'tensao_x', 'tensao_y', 'saturado',
                'lat_leitura_ms', 'lat_cmd_ms', 'periodo_ms', 'fator_ganho',
                'amostra_nova', 'dt_util_ms', 'n_laco'
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

        # Recuo automático de ganho por saturação
        n_janela_sat = max(10, int(round(BACKOFF_JANELA_S / PERIODO_AMOSTRA_S)))
        janela_sat = deque(maxlen=n_janela_sat)
        soma_sat = 0
        fator_ganho = 1.0
        t_ultimo_backoff = 0.0

        janela_x = deque(maxlen=JANELA_TURB_AMOSTRAS)
        janela_y = deque(maxlen=JANELA_TURB_AMOSTRAS)

        tempo_inicio_total = float(estado["t_inicio_epoch"]) if retomar else time.time()
        if retomar and estado.get("interrompido"):
            # parada MANUAL: o tempo parado não conta na duração do ensaio (queda por crash: conta, é curto)
            tempo_inicio_total += max(0.0, time.time() - float(estado.get("atualizado", time.time())))
        ultimo_tempo = time.time()
        ultimo_print_tempo = -1.0
        contador_leituras = 0
        malha_fechada_ativada = bool(estado["malha_fechada"]) if retomar else False
        reinit_pid = bool(retomar and malha_fechada_ativada)
        if retomar:
            fator_ganho = float(estado.get("fator_ganho", 1.0))

        def _gravar_estado(concluido=False, interrompido=False):
            """Grava o estado que permite RETOMAR o ensaio se o processo for derrubado."""
            _salvar_estado(estado_path, dict(
                modo=modo, t_inicio_epoch=tempo_inicio_total, malha_fechada=bool(malha_fechada_ativada),
                v_alvo=[float(v_alvo[0]), float(v_alvo[1])], J=None if J is None else J.tolist(),
                tempo_total=tempo_total_segundos,
                tempo_aberto=None if math.isinf(tempo_loop_aberto_segundos) else tempo_loop_aberto_segundos,
                fator_ganho=float(fator_ganho), concluido=concluido, interrompido=interrompido,
                atualizado=time.time()))

        _gravar_estado()

        t_iter_anterior = time.perf_counter()
        t_proximo = t_iter_anterior

        n_novas = 0
        n_repetidas = 0
        chave_anterior = None
        t_ultima_nova = time.time()
        t_loop_inicio = time.perf_counter()
        taxa_util = 0.0

        def _aguardar_proximo():
            """Espera precisa até o próximo instante da grade de leitura (sem recuperar atrasos)."""
            nonlocal t_proximo
            t_proximo += PERIODO_LOOP_S
            t_agora_pc = time.perf_counter()
            if t_proximo < t_agora_pc:
                t_proximo = t_agora_pc
            esperar_ate(t_proximo)

        if modo == "referencia":
            print(f"\nIniciando aquisição de REFERÊNCIA por {tempo_total_segundos:.1f}s (tensão fixa)...")
        else:
            print(f"\nIniciando aquisição em LOOP ABERTO por {tempo_loop_aberto_segundos:.1f}s "
                  f"(a partir do ponto centralizado)...")

        # Desativa coletas automáticas assíncronas do GC durante o loop crítico;
        # a coleta será feita de forma síncrona a cada INTERVALO_GC_COLLECT amostras.
        gc.disable()

        while True:
            t_iter = time.perf_counter()
            periodo_ms = (t_iter - t_iter_anterior) * 1000.0
            t_iter_anterior = t_iter

            agora = time.time()
            tempo_decorrido = agora - tempo_inicio_total
            if tempo_decorrido >= tempo_total_segundos:
                break

            armar_watchdog()
            try:
                t_l0 = time.perf_counter()
                diff_x, diff_y, intensidade = _ler_dq(cubo_dq)
                lat_leitura_ms = (time.perf_counter() - t_l0) * 1000.0
                falhas_leitura = 0
            except (DotNetException, Exception) as e_leitura:
                falhas_leitura += 1
                if falhas_leitura == 1:
                    print(f"\n[usb] falha ao ler o detector: {e_leitura}")
                if falhas_leitura * PERIODO_LOOP_S >= MAX_FALHAS_LEITURA_S:
                    raise RuntimeError(
                        f"Sem leitura do detector por {MAX_FALHAS_LEITURA_S:.0f}s: comunicação USB perdida.")
                time.sleep(PERIODO_LOOP_S)
                continue

            # --- é uma amostra NOVA? (com polling rápido o Status se repete entre as atualizações) ---
            chave = (diff_x, diff_y, intensidade)
            if chave == chave_anterior:
                n_repetidas += 1
                if (agora - t_ultima_nova) > MAX_FALHAS_LEITURA_S:
                    raise RuntimeError(
                        f"O KPA101 não entrega amostras novas há {MAX_FALHAS_LEITURA_S:.0f}s "
                        "(polling parado ou comunicação perdida).")
                contador_leituras += 1
                _aguardar_proximo()
                continue
            chave_anterior = chave
            n_novas += 1
            dt = agora - t_ultima_nova          # tempo desde a amostra nova anterior (usado pelo PI)
            t_ultima_nova = agora
            if dt <= 0.0 or dt > 0.5:
                dt = PERIODO_AMOSTRA_S
            taxa_util = n_novas / max(time.perf_counter() - t_loop_inicio, 1e-9)

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
            if regime == "Calmo" and sigma_turb > SIGMA_LIMIAR_TURB * (1.0 + HISTERESE_TURB):
                regime = "Turbulento"
            elif regime == "Turbulento" and sigma_turb < SIGMA_LIMIAR_TURB * (1.0 - HISTERESE_TURB):
                regime = "Calmo"

            if reinit_pid and sinal_valido:   # após retomada: reinicia o estado interno do PI
                e_ant = -np.array([x_pos_real, y_pos_real])
                e_filt = e_ant.copy()
                t_ultimo_backoff = tempo_decorrido
                reinit_pid = False

            if tempo_decorrido >= tempo_loop_aberto_segundos and not malha_fechada_ativada:
                print("\n\n>>> ATIVANDO MALHA FECHADA (PI incremental em espaço de posição) <<<")
                malha_fechada_ativada = True
                e_ant = -np.array([x_pos_real, y_pos_real])
                e_filt = e_ant.copy()
                t_ultimo_backoff = tempo_decorrido

            modo_log = "Fechado" if malha_fechada_ativada else ("Referencia" if modo == "referencia" else "Aberto")
            saturado = False
            lat_cmd_ms = 0.0

            if malha_fechada_ativada and sinal_valido:
                g = GANHOS[regime]
                kp_ef = g["kp"] * fator_ganho
                ki_ef = g["ki"] * fator_ganho
                e_bruto = -np.array([x_pos_real, y_pos_real])
                e_filt = g["alpha"] * e_bruto + (1.0 - g["alpha"]) * e_filt

                raio_pos = math.hypot(x_pos_real, y_pos_real)
                if raio_pos < RAIO_ZONA_CENTRO:
                    fator_geom = FATOR_GEOM_CENTRO
                elif raio_pos > RAIO_ZONA_BORDA:
                    fator_geom = FATOR_GEOM_BORDA
                else:
                    fator_geom = 1.0

                # PI incremental na variável de posição desejada...
                dpos = kp_ef * (e_filt - e_ant) + ki_ef * fator_geom * e_filt * dt
                e_ant = e_filt.copy()

                # ...convertido em volts pela sensibilidade medida (Jinv)
                dv = np.clip(Jinv @ dpos, -MAX_STEP_V, MAX_STEP_V)
                v_novo = v_alvo + dv
                v_alvo = np.clip(v_novo, V_MIN, V_MAX)
                saturado = bool(np.any(v_alvo != v_novo))

                # Comandos (a cada amostra se CMD_PERIODO_S=0; senão limitados em taxa)
                if CMD_PERIODO_S <= 0.0 or (agora - ultimo_cmd) >= CMD_PERIODO_S:
                    vx_cmd, vy_cmd = round(v_alvo[0], 2), round(v_alvo[1], 2)
                    enviar_x = abs(vx_cmd - v_enviada[0]) >= CMD_DELTA_MIN_V
                    enviar_y = abs(vy_cmd - v_enviada[1]) >= CMD_DELTA_MIN_V
                    if enviar_x or enviar_y:
                        try:
                            t_c0 = time.perf_counter()
                            if enviar_x:
                                cubo_x.SetOutputVoltage(Decimal(vx_cmd))
                                v_enviada[0] = vx_cmd
                            if enviar_y:
                                cubo_y.SetOutputVoltage(Decimal(vy_cmd))
                                v_enviada[1] = vy_cmd
                            lat_cmd_ms = (time.perf_counter() - t_c0) * 1000.0
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

                # Recuo/recuperação automática de ganho conforme a fração de saturação
                if len(janela_sat) == janela_sat.maxlen:
                    soma_sat -= janela_sat[0]
                janela_sat.append(int(saturado))
                soma_sat += int(saturado)
                if BACKOFF_ATIVO and (tempo_decorrido - t_ultimo_backoff) >= BACKOFF_INTERVALO_S \
                        and len(janela_sat) >= janela_sat.maxlen // 2:
                    frac_sat = soma_sat / len(janela_sat)
                    if frac_sat > BACKOFF_SAT_ALTA and fator_ganho > BACKOFF_FATOR_MIN:
                        fator_ganho = max(BACKOFF_FATOR_MIN, fator_ganho * 0.5)
                        t_ultimo_backoff = tempo_decorrido
                        print(f"\n[backoff] saturação de {frac_sat*100:.1f}% em {BACKOFF_JANELA_S:.0f}s: "
                              f"ganhos reduzidos para {fator_ganho*100:.0f}% do nominal.")
                    elif frac_sat < BACKOFF_SAT_BAIXA and fator_ganho < 1.0:
                        fator_ganho = min(1.0, fator_ganho * 1.25)
                        t_ultimo_backoff = tempo_decorrido
                        print(f"\n[backoff] saturação normalizada ({frac_sat*100:.2f}%): "
                              f"ganhos restaurados para {fator_ganho*100:.0f}% do nominal.")

            _LIVE.add(agora, x_pos_real, y_pos_real, intensidade, modo_log, regime,
                      (float(v_alvo[0]), float(v_alvo[1])), int(saturado), float(fator_ganho), taxa_util,
                      tempo_decorrido, tempo_total_segundos)
            _LIVE.flush()

            if n_novas % LOG_A_CADA_N_AMOSTRAS == 0:
                escritor_csv.writerow([
                    f"{tempo_decorrido:.3f}", modo_log, regime,
                    diff_x, diff_y, intensidade,
                    x_pos_real, y_pos_real,
                    round(sigma_turb, 5),
                    round(v_alvo[0], 2), round(v_alvo[1], 2),
                    int(saturado),
                    round(lat_leitura_ms, 3), round(lat_cmd_ms, 3), round(periodo_ms, 3), round(fator_ganho, 3),
                    1, round(dt * 1000.0, 3), contador_leituras
                ])

            if tempo_decorrido - ultimo_print_tempo >= 1.0:
                ultimo_print_tempo = tempo_decorrido
                _gravar_estado()
                tempo_restante = max(0.0, tempo_total_segundos - tempo_decorrido)
                mins, segs = divmod(int(tempo_restante), 60)
                progresso = (tempo_decorrido / tempo_total_segundos) * 100.0
                print(
                    f"\r[Status: {modo_log:10s} | {regime:10s}] "
                    f"Restante: {mins:02d}m {segs:02d}s ({progresso:5.1f}%) | "
                    f"Pos: ({x_pos_real:+.3f}, {y_pos_real:+.3f}) | "
                    f"V: ({v_alvo[0]:5.1f}V, {v_alvo[1]:5.1f}V) | {taxa_util:5.0f} Hz úteis",
                    end="", flush=True
                )

            if contador_leituras % INTERVALO_FLUSH_CSV == 0:
                f_csv.flush()

            if contador_leituras % INTERVALO_GC_COLLECT == 0:
                if not USAR_POLLING_ASSINCRONO_DQ:   # não mexe na fila enquanto a thread de polling roda
                    try:
                        cubo_dq.ClearMessageQueue()
                    except Exception:
                        pass
                gc.collect()

            contador_leituras += 1

            _aguardar_proximo()

        tempo_total_decorrido = time.time() - tempo_inicio_total
        frequencia_aquisicao = (contador_leituras / tempo_total_decorrido) if tempo_total_decorrido > 0 else 0.0
        print(f"\n\nEnsaio finalizado! {contador_leituras} leituras ({frequencia_aquisicao:.1f} Hz) | "
              f"amostras NOVAS: {n_novas} ({n_novas / max(tempo_total_decorrido, 1e-9):.1f} Hz úteis, "
              f"{n_repetidas} leituras repetidas). Erros USB de comando: {total_erros_usb}.")
        arquivo_ok = nome_arquivo
        _gravar_estado(concluido=True)

    except KeyboardInterrupt:
        print("\n\nControle interrompido pelo usuário.")
        arquivo_ok = nome_arquivo if f_csv is not None else None
        try:
            _gravar_estado(interrompido=True)
        except Exception:
            pass
    except (DotNetException, Exception) as e:
        print(f"\n\nERRO CAPTURADO: {e}")
        _LOG_CRASH.write(f"{time.ctime()} ERRO CAPTURADO: {e}\n")
        arquivo_ok = nome_arquivo if f_csv is not None else None  # preserva os dados parciais
    finally:
        if gc_estava_ativo:
            gc.enable()
        faulthandler.cancel_dump_traceback_later()
        configurar_desempenho_windows(False)
        manter_windows_ativo(False)
        print("\nLiberando hardware com segurança...")
        if f_csv is not None:
            f_csv.close()
        liberar_cubos(cubo_x, cubo_y, cubo_dq)

    return arquivo_ok


#%%
# =============================================================================
# 7B. ESTADO, SUPERVISÃO E RETOMADA (SOBREVIVER A CRASHES NATIVOS)
# =============================================================================
# O erro 0xc0000374 (corrupção de heap nativo, na leitura `.Status` do KPA101) derruba o
# PROCESSO inteiro: try/except e finally não executam. A única proteção possível é isolar o
# acesso ao hardware num processo filho (worker). O supervisor (processo pai) não carrega o
# Kinesis, então não cai junto: ele detecta a queda (ou um travamento), espera o USB liberar,
# reinicia o worker com `--retomar` e o ensaio continua no MESMO CSV, com as mesmas tensões.
EXIT_OK, EXIT_ABORTAR, EXIT_REINICIAVEL, EXIT_INTERROMPIDO = 0, 10, 20, 130


def _caminho_estado(nome_arquivo):
    return os.path.splitext(nome_arquivo)[0] + "_estado.json"


def _caminho_hb(nome_arquivo):
    return os.path.splitext(nome_arquivo)[0] + "_hb.txt"


def _caminho_eventos(nome_arquivo):
    return os.path.splitext(nome_arquivo)[0] + "_eventos.json"


def _salvar_estado(path, dados):
    """Grava o estado de forma atômica (arquivo temporário + os.replace)."""
    try:
        tmp = path + ".tmp"
        with open(tmp, "w") as f:
            json.dump(dados, f)
        os.replace(tmp, path)
    except Exception:
        pass


def _carregar_estado(path):
    """Lê o arquivo de estado; devolve None se não existir ou estiver ilegível."""
    for _ in range(3):
        try:
            with open(path) as f:
                return json.load(f)
        except FileNotFoundError:
            return None
        except Exception:
            time.sleep(0.2)
    return None


def _mtime(path):
    try:
        return os.path.getmtime(path)
    except Exception:
        return None


_PROTEGIDAS = {"SUPERVISIONAR", "EH_WORKER", "KINESIS_PATH"}


def recalcular_derivados():
    """Recalcula as constantes derivadas de outras (caminhos, períodos e janelas)."""
    global CAMINHO_REF_CSV, CAMINHO_CTL_CSV, CAMINHO_LAT_JSON, CAMINHO_PNG_COMPARACAO
    global PERIODO_AMOSTRA_S, JANELA_TURB_AMOSTRAS
    CAMINHO_REF_CSV = f"{PASTA_SAIDA}/{FILENAME}_referencia.csv"
    CAMINHO_CTL_CSV = f"{PASTA_SAIDA}/{FILENAME}_controle.csv"
    CAMINHO_LAT_JSON = f"{PASTA_SAIDA}/{FILENAME}_latencia.json"
    CAMINHO_PNG_COMPARACAO = f"{PASTA_SAIDA}/comparacao_centroides_{FILENAME}.png"
    PERIODO_AMOSTRA_S = max(PERIODO_LOOP_S, POLL_DQ_MS / 1000.0) if USAR_POLLING_ASSINCRONO_DQ else PERIODO_LOOP_S
    JANELA_TURB_AMOSTRAS = max(10, int(round(JANELA_TURB_S / PERIODO_AMOSTRA_S)))


def aplicar_config_dict(cfg):
    """
    O QUE EXECUTA:
        Sobrescreve constantes do módulo com os valores de `cfg` (dict nome -> valor), convertendo
        para o tipo da constante original, e recalcula os derivados. É assim que a interface
        gráfica configura o processo de aquisição (via arquivo JSON).

    RETORNA:
        - Lista de chaves ignoradas (desconhecidas, protegidas ou de tipo não suportado).
    """
    g = globals()
    ignoradas = []
    for k, v in cfg.items():
        if k in _PROTEGIDAS or k not in g or not k.isupper():
            ignoradas.append(k)
            continue
        atual = g[k]
        if k == "GANHOS":
            g[k] = {reg: {p: float(val) for p, val in d.items()} for reg, d in v.items()}
        elif isinstance(atual, bool):
            g[k] = bool(v)
        elif isinstance(atual, int):
            g[k] = int(v)
        elif isinstance(atual, float):
            g[k] = float(v)
        elif isinstance(atual, str):
            g[k] = str(v)
        elif isinstance(atual, tuple):
            g[k] = tuple(float(a) for a in v)
        else:
            ignoradas.append(k)
    recalcular_derivados()
    return ignoradas


def worker_main(retomar):
    """
    O QUE EXECUTA:
        Corpo do processo que acessa o hardware. Em ensaio novo faz o zeroing e roda o
        ensaio; com `retomar=True` pula o zeroing (que derrubaria os piezos a 0 V) e
        continua a partir do arquivo de estado.

    RETORNA:
        Código de saída: 0 concluído | 130 interrompido pelo usuário | 20 erro no meio do
        ensaio (reiniciável) | 10 falha antes de iniciar (não adianta reiniciar).
    """
    global _HB_PATH
    caminho_csv = CAMINHO_REF_CSV if MODO_ENSAIO == "referencia" else CAMINHO_CTL_CSV
    pasta = os.path.dirname(caminho_csv)
    if pasta:
        os.makedirs(pasta, exist_ok=True)
    _HB_PATH = _caminho_hb(caminho_csv)
    _tocar_heartbeat()
    if not retomar:   # ensaio novo: descarta estado de execuções anteriores
        try:
            os.remove(_caminho_estado(caminho_csv))
        except OSError:
            pass
    try:
        if not retomar:
            zeroing_suave(SERIAL_X, SERIAL_Y)
            time.sleep(2.0)
        executar_malha_fechada_dq_software(
            SERIAL_X, SERIAL_Y, SERIAL_DQ,
            tempo_total_segundos=TEMPO_TOTAL_EXPERIMENTO_S,
            tempo_loop_aberto_segundos=TEMPO_LOOP_ABERTO_S,
            nome_arquivo=caminho_csv, modo=MODO_ENSAIO, retomar=retomar)
    except KeyboardInterrupt:
        return EXIT_INTERROMPIDO
    except (DotNetException, Exception) as e:
        print(f"\n[worker] erro: {e}")
        _LOG_CRASH.write(f"{time.ctime()} [worker] erro: {e}\n")
    if PARAR_FLAG and os.path.exists(PARAR_FLAG):
        return EXIT_INTERROMPIDO
    est = _carregar_estado(_caminho_estado(caminho_csv))
    if est and est.get("concluido"):
        return EXIT_OK
    if est and est.get("interrompido"):
        return EXIT_INTERROMPIDO
    return EXIT_REINICIAVEL if est else EXIT_ABORTAR


def supervisionar(montar_cmd=None, caminho_csv=None, hang_s=None, max_reinicios=None, pausa_s=None,
                  popen_kwargs=None, ao_iniciar=None):
    """
    O QUE EXECUTA:
        Lança o worker, monitora o código de saída e o heartbeat e, se ele cair ou travar,
        reinicia com `--retomar` (até `MAX_REINICIOS`). Cada reinício é registrado em
        `<csv>_eventos.json` e marcado nos gráficos.

    RETORNA:
        - Caminho do CSV (completo ou parcial) ou None se nada foi gravado.
    """
    caminho_csv = caminho_csv or (CAMINHO_REF_CSV if MODO_ENSAIO == "referencia" else CAMINHO_CTL_CSV)
    hang_s = HANG_S if hang_s is None else hang_s
    max_reinicios = MAX_REINICIOS if max_reinicios is None else max_reinicios
    pausa_s = PAUSA_REINICIO_S if pausa_s is None else pausa_s
    est_p, hb_p, ev_p = _caminho_estado(caminho_csv), _caminho_hb(caminho_csv), _caminho_eventos(caminho_csv)
    pasta = os.path.dirname(caminho_csv)
    if pasta:
        os.makedirs(pasta, exist_ok=True)

    eventos, retomar = [], False
    if RETOMAR_ENSAIO_ANTERIOR and _carregar_estado(est_p) is not None:
        retomar = True
        try:
            with open(ev_p) as f:
                eventos = json.load(f)
        except Exception:
            eventos = []
        print("[supervisor] retomando o ensaio anterior a partir do arquivo de estado.")
    else:
        for pth in (est_p, hb_p, ev_p):
            try:
                os.remove(pth)
            except OSError:
                pass

    if montar_cmd is None:
        def montar_cmd(retomar_):
            arq = globals().get("__file__")
            if not arq:
                raise RuntimeError(
                    "SUPERVISIONAR=True precisa que o arquivo seja executado como script "
                    "(python arquivo.py), não por células interativas. Rode pelo terminal "
                    "ou defina SUPERVISIONAR = False.")
            return [sys.executable, os.path.abspath(arq), "--worker"] + (["--retomar"] if retomar_ else [])

    reinicios = 0
    while True:
        print(f"[supervisor] {'retomando' if retomar else 'iniciando'} o processo de aquisição "
              f"(reinícios até agora: {reinicios}).")
        proc = subprocess.Popen(montar_cmd(retomar), **(popen_kwargs or {}))
        if ao_iniciar:
            ao_iniciar(proc)
        t_lanc = time.time()
        rc, motivo = None, "crash"
        try:
            while rc is None:
                try:
                    rc = proc.wait(timeout=1.0)
                except subprocess.TimeoutExpired:
                    ultimo_sinal = max(_mtime(hb_p) or 0.0, t_lanc)
                    if time.time() - ultimo_sinal > hang_s:
                        print(f"\n[supervisor] sem sinal de vida há {hang_s:.0f}s: encerrando o processo travado.")
                        proc.kill()
                        proc.wait()
                        rc, motivo = -999, "travou (sem sinal de vida)"
        except KeyboardInterrupt:
            print("\n[supervisor] Ctrl+C: aguardando o processo de aquisição liberar o hardware...")
            try:
                proc.wait(timeout=20)
            except Exception:
                proc.kill()
            return caminho_csv if os.path.exists(caminho_csv) else None

        est = _carregar_estado(est_p)
        if PARAR_FLAG and os.path.exists(PARAR_FLAG):
            print("\n[supervisor] parada solicitada: sem reinício.")
            return caminho_csv if os.path.exists(caminho_csv) else None
        if rc in (EXIT_OK, EXIT_INTERROMPIDO):
            return caminho_csv if os.path.exists(caminho_csv) else None
        if rc == EXIT_ABORTAR:
            print("\n[supervisor] o ensaio abortou antes de iniciar (veja a mensagem acima); sem reinício.")
            return None
        if est and est.get("concluido"):
            return caminho_csv
        if est and (time.time() - est["t_inicio_epoch"]) >= est.get("tempo_total", float('inf')):
            print("\n[supervisor] o tempo total do ensaio já foi atingido.")
            return caminho_csv if os.path.exists(caminho_csv) else None

        reinicios += 1
        rc_hex = hex(rc & 0xFFFFFFFF) if isinstance(rc, int) else str(rc)
        ev = {"quando": time.ctime(), "rc": rc, "rc_hex": rc_hex, "motivo": motivo,
              "t_rel": (time.time() - est["t_inicio_epoch"]) if est else None}
        eventos.append(ev)
        try:
            with open(ev_p, "w") as f:
                json.dump(eventos, f, indent=2)
        except Exception:
            pass
        _LOG_CRASH.write(f"{time.ctime()} [supervisor] worker terminou: rc={rc_hex} ({motivo}); reinício {reinicios}\n")
        nome = " (corrupção de heap nativo)" if (rc & 0xFFFFFFFF) == 0xC0000374 else ""
        print(f"\n[supervisor] processo de aquisição terminou com código {rc_hex}{nome} ({motivo}).")
        if reinicios > max_reinicios:
            print(f"[supervisor] limite de {max_reinicios} reinícios atingido: encerrando.")
            return caminho_csv if os.path.exists(caminho_csv) else None
        retomar = est is not None and not est.get("concluido")
        print(f"[supervisor] reinício {reinicios}/{max_reinicios} em {pausa_s:.0f}s "
              f"({'retomando do estado salvo' if retomar else 'recomeçando do zero'})...")
        time.sleep(pausa_s)


#%%
# =============================================================================
# 7C. DESCOBERTA DA API DO KINESIS (MODO_ENSAIO = "api")
# =============================================================================
def listar_api():
    """
    O QUE EXECUTA:
        Conecta aos cubos e grava em `PASTA_SAIDA/api_kinesis.txt` todos os métodos e
        propriedades públicos do KPA101 (cubo_dq), do seu `Status` e de um KPC101. Serve para
        descobrir, na SUA versão do Kinesis, quais chamadas existem para o modo de malha
        fechada em hardware (procure por Mode, Loop, Pid, Corr, Demand, Gain).
    """
    manter_windows_ativo(True)
    cubo_x, cubo_y, cubo_dq = conectar_cubos(SERIAL_X, SERIAL_Y, SERIAL_DQ)
    try:
        inicializar_cubos(cubo_x, cubo_y, cubo_dq)
        linhas = []
        alvos = [("KPA101 (cubo_dq)", cubo_dq), ("KPC101 (cubo_x)", cubo_x)]
        try:
            alvos.insert(1, ("KPA101 .Status", cubo_dq.Status))
        except Exception as e:
            linhas.append(f"(não foi possível ler .Status: {e})")
        for nome, obj in alvos:
            linhas.append(f"\n===== {nome}: {type(obj)} =====")
            linhas += [m for m in sorted(dir(obj)) if not m.startswith("_")]
        pasta = PASTA_SAIDA
        os.makedirs(pasta, exist_ok=True)
        caminho = os.path.join(pasta, "api_kinesis.txt")
        with open(caminho, "w", encoding="utf-8") as f:
            f.write("\n".join(linhas))
        chaves = ("mode", "loop", "pid", "corr", "demand", "gain", "closed", "aligner")
        cand = [m for m in linhas if any(k in m.lower() for k in chaves) and not m.startswith("=")]
        print(f"\nAPI gravada em '{caminho}' ({len(linhas)} linhas). Itens que parecem relevantes:")
        for m in cand[:60]:
            print("  ", m)
        print("\nEnvie o arquivo para análise do modo de malha fechada em hardware.")
    except (DotNetException, Exception) as e:
        print(f"ERRO: {e}")
    finally:
        manter_windows_ativo(False)
        liberar_cubos(cubo_x, cubo_y, cubo_dq)


def pos_processamento(arquivo_gerado, caminho_csv):
    """
    O QUE EXECUTA:
        Resumo de temporização, gráficos do ensaio e (se existirem os dois CSVs)
        comparação de centroides com/sem controle.
    """
    if not arquivo_gerado:
        print("Ensaio não executado (validação reprovada ou erro). Nenhum gráfico gerado.")
        return
    try:
        with open(_caminho_eventos(caminho_csv)) as f:
            ev = json.load(f)
        print(f"\nReinícios automáticos durante o ensaio: {len(ev)}")
        for e in ev:
            print(f"  - {e['quando']}: código {e['rc_hex']} ({e['motivo']}) em t≈{e['t_rel'] if e['t_rel'] is None else round(e['t_rel'])} s")
    except Exception:
        print("\nReinícios automáticos durante o ensaio: 0")
    resumir_temporizacao_csv(arquivo_gerado)
    plotar_resultados_csv(
        arquivo_gerado,
        tempo_loop_aberto=TEMPO_LOOP_ABERTO_S if MODO_ENSAIO == "controle" else None
    )
    if os.path.exists(CAMINHO_REF_CSV) and os.path.exists(CAMINHO_CTL_CSV):
        comparar_centroides(CAMINHO_REF_CSV, CAMINHO_CTL_CSV, salvar_png=CAMINHO_PNG_COMPARACAO)


#%%
# =============================================================================
# 8. EXECUÇÃO (CONFIGURADA NA SEÇÃO 2.1 E NAS CHAVES DO TOPO DO ARQUIVO)
# =============================================================================
if __name__ == "__main__":
    caminho_csv = CAMINHO_REF_CSV if MODO_ENSAIO == "referencia" else CAMINHO_CTL_CSV
    if EH_WORKER:
        if "--config" in sys.argv:                               # configuração vinda da interface gráfica
            with open(sys.argv[sys.argv.index("--config") + 1], encoding="utf-8") as _f:
                _ign = aplicar_config_dict(json.load(_f))
            if _ign:
                print(f"[config] chaves ignoradas: {_ign}")
            _LIVE.abrir(LIVE_PORTA)
        if MODO_ENSAIO == "api":
            listar_api()
            sys.exit(0)
        if MODO_ENSAIO == "latencia":
            zeroing_suave(SERIAL_X, SERIAL_Y)
            time.sleep(2.0)
            executar_teste_latencia(SERIAL_X, SERIAL_Y, SERIAL_DQ)
            sys.exit(0)
        sys.exit(worker_main("--retomar" in sys.argv))          # processo filho: acessa o hardware
    elif MODO_ENSAIO == "api":
        listar_api()
    elif MODO_ENSAIO == "latencia":
        zeroing_suave(SERIAL_X, SERIAL_Y)
        time.sleep(2.0)
        executar_teste_latencia(SERIAL_X, SERIAL_Y, SERIAL_DQ)
    elif SUPERVISIONAR:
        arquivo = supervisionar(caminho_csv=caminho_csv)        # processo pai: não carrega o Kinesis
        pos_processamento(arquivo, caminho_csv)
    else:
        rc = worker_main(False)                                 # tudo num processo só (modo antigo)
        pos_processamento(caminho_csv if os.path.exists(caminho_csv) and rc != EXIT_ABORTAR else None, caminho_csv)
