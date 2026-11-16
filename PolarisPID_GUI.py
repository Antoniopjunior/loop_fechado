#%%
"""
PolarisPID_GUI.py - Interface gráfica do controle de feixe (Polaris + KPA101 + KPC101).

Coloque este arquivo na MESMA PASTA do PolarisPID_v3.py e execute:  python PolarisPID_GUI.py

Arquitetura (por que não trava a janela e sobrevive a crashes do Kinesis):
  - Esta janela NÃO carrega o Kinesis. Ela só configura, lança e acompanha o processo de aquisição.
  - O processo de aquisição (PolarisPID_v3.py --worker) acessa o hardware. Se ele cair com o erro
    0xc0000374 ou travar, o supervisor do v3 o reinicia e o ensaio RETOMA do ponto em que parou.
  - A posição do feixe chega à janela em tempo real por UDP local (porta livre escolhida sozinha).
  - Toda a configuração vai para o worker por um arquivo JSON, salvo junto com os dados do ensaio.
"""
import os
import sys
import re
import io
import json
import copy
import math
import queue
import socket
import codecs
import threading
import subprocess
import contextlib
from collections import deque

import numpy as np
import matplotlib
matplotlib.use("Agg")      # o pyplot usado pelo v3 não abre janelas; os gráficos desta janela usam Figure
from matplotlib.figure import Figure
from matplotlib.patches import Circle

import tkinter as tk
from tkinter import ttk, messagebox, filedialog
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk

PASTA_SCRIPT = os.path.dirname(os.path.abspath(__file__))
ARQ_V3 = os.path.join(PASTA_SCRIPT, "PolarisPID_v3.py")
CONFIG_AUTO = os.path.join(PASTA_SCRIPT, "polaris_gui_config.json")
sys.path.insert(0, PASTA_SCRIPT)
import PolarisPID as v3          # só para ler os padrões e usar o supervisor (não carrega o Kinesis)


#%%
# =============================================================================
# DEFINIÇÃO DOS PARÂMETROS (chave no v3, rótulo, dica). O tipo vem do valor padrão no v3.
# =============================================================================
MODOS = {
    "Controle (malha fechada)": "controle",
    "Referência (tensão fixa, sem controle)": "referencia",
    "Teste de latência (sem turbulência)": "latencia",
    "Listar API do Kinesis": "api",
}
OPCOES = {"PRIORIDADE_PROCESSO": ["normal", "acima_do_normal", "alta"]}

CONTROLE_PRINCIPAL = [
    ("MAX_STEP_V", "Passo máx. de tensão (V/amostra)", "limita a variação de tensão a cada correção"),
    ("SIGMA_LIMIAR_TURB", "Limiar de turbulência (σ)", "σ do feixe acima do qual o regime vira 'Turbulento'"),
    ("HISTERESE_TURB", "Histerese do regime", "banda (fração) para não alternar de regime toda hora"),
    ("JANELA_TURB_S", "Janela do σ (s)", "janela móvel que mede a turbulência em tempo real"),
]
CONTROLE_GEOMETRICO = [
    ("RAIO_ZONA_CENTRO", "Raio da zona central", "abaixo dele o ganho integral é reduzido"),
    ("FATOR_GEOM_CENTRO", "Fator de ki no centro", "multiplicador de ki com o feixe no centro"),
    ("RAIO_ZONA_BORDA", "Raio da zona de borda", "acima dele o ganho integral é aumentado"),
    ("FATOR_GEOM_BORDA", "Fator de ki na borda", "multiplicador de ki com o feixe longe do centro"),
]
CONTROLE_BACKOFF = [
    ("BACKOFF_ATIVO", "Recuo de ganho por saturação", "reduz os ganhos se o piezo saturar demais"),
    ("BACKOFF_SAT_ALTA", "Saturação alta (fração)", "acima disto os ganhos são reduzidos"),
    ("BACKOFF_SAT_BAIXA", "Saturação baixa (fração)", "abaixo disto os ganhos são restaurados"),
    ("BACKOFF_FATOR_MIN", "Fator mínimo de ganho", "os ganhos nunca caem abaixo desta fração"),
    ("BACKOFF_JANELA_S", "Janela de saturação (s)", "janela para medir a fração de saturação"),
    ("BACKOFF_INTERVALO_S", "Intervalo entre ajustes (s)", "tempo mínimo entre dois ajustes de ganho"),
]

AVANCADO = [
    ("Tempos do ensaio", [
        ("TEMPO_TOTAL_EXPERIMENTO_S", "Duração total (s)", "tempo de aquisição do ensaio"),
        ("TEMPO_LOOP_ABERTO_S", "Tempo em malha aberta (s)", "só grava, sem controle, antes de fechar a malha (0 = fecha já)"),
        ("T_ESTABILIZACAO_MEC_S", "Estabilização mecânica (s)", "espera após aplicar a tensão inicial"),
        ("DESCARTAR_REF_INICIAL_S", "Descartar início da referência (s)", "usado na comparação de centroides"),
        ("DESCARTAR_CTL_INICIAL_S", "Descartar transiente do controle (s)", "usado na comparação de centroides"),
        ("RAIOS_COMPARACAO", "Raios da comparação", "separe por ponto e vírgula, ex.: 0.1; 0.2; 0.3"),
    ]),
    ("Hardware e limites físicos", [
        ("SERIAL_X", "Serial do KPC101 (eixo X)", ""),
        ("SERIAL_Y", "Serial do KPC101 (eixo Y)", ""),
        ("SERIAL_DQ", "Serial do KPA101 (detector)", ""),
        ("LIMIAR_SOMA", "Soma mínima do detector", "abaixo disto a posição é inválida e o controle congela"),
        ("V_MIN", "Tensão mínima (V)", "limite inferior do piezo"),
        ("V_MAX", "Tensão máxima (V)", "limite superior do piezo"),
        ("V_BASE", "Tensão inicial/de repouso (V)", "ponto de partida e tensão ao terminar"),
        ("V_REFERENCIA", "Tensão do modo referência (V)", "tensão fixa dos dois cubos sem controle"),
    ]),
    ("Leitura do detector e temporização (USB)", [
        ("USAR_POLLING_ASSINCRONO_DQ", "Polling assíncrono do KPA101", "ligado = alta taxa; desligado = RequestStatus (16 ms, mais seguro)"),
        ("POLL_DQ_MS", "Polling do KPA101 (ms)", "taxa máxima de dado novo (2 ms = 500 Hz)"),
        ("PERIODO_LOOP_S", "Período de leitura do laço (s)", "intervalo entre leituras de Status"),
        ("CMD_PERIODO_S", "Intervalo mínimo entre comandos (s)", "0 = envia a cada amostra"),
        ("CMD_DELTA_MIN_V", "Variação mínima p/ enviar (V)", "não reenvia se a tensão mudou menos que isto"),
        ("ALTA_RESOLUCAO_TIMER", "Timer do Windows de 1 ms", "evita o jitter do tick de 15,6 ms"),
        ("PRIORIDADE_PROCESSO", "Prioridade do processo", "normal, acima_do_normal ou alta"),
        ("ESPERA_ATIVA_S", "Espera ativa final (s)", "precisão da temporização do laço"),
        ("USAR_MONITOR_LOCK", "Usar Monitor.Enter/Exit na leitura", "teste de estabilidade do Kinesis"),
        ("POLL_PIEZO_MS", "Polling dos KPC101 no zeroing (ms)", ""),
        ("INTERVALO_FLUSH_CSV", "Gravar no disco a cada N leituras", ""),
        ("INTERVALO_GC_COLLECT", "Coleta de lixo a cada N leituras", ""),
        ("LOG_A_CADA_N_AMOSTRAS", "Gravar 1 a cada N amostras no CSV", "o PI usa todas; 1 = grava todas"),
    ]),
    ("Timeouts e segurança", [
        ("MAX_FALHAS_LEITURA_S", "Timeout sem amostra nova (s)", "aborta se o detector parar de atualizar"),
        ("MAX_ERROS_USB", "Erros USB seguidos p/ abortar", ""),
        ("WATCHDOG_S", "Watchdog de travamento (s)", "grava a pilha no crash_log.txt"),
        ("HANG_S", "Timeout do supervisor (s)", "sem sinal de vida por este tempo = mata e reinicia"),
        ("PAUSA_REINICIO_S", "Pausa antes de reiniciar (s)", "deixa o USB liberar as portas"),
        ("MAX_REINICIOS", "Máximo de reinícios automáticos", ""),
    ]),
    ("Calibração da sensibilidade J", [
        ("J_ARQUIVO", "Arquivo da J", "calibrada em ar calmo"),
        ("J_PREFERIR_SALVA", "Preferir a J salva", "pula a medição ruidosa sob turbulência"),
        ("J_FORCAR_RECALIBRAR", "Forçar recalibração", "use após mexer no arranjo óptico (sem turbulência)"),
        ("J_SALVAR", "Salvar J medida em ar calmo", ""),
        ("J_SALVAR_SIGMA_MAX", "σ máx. para salvar J", ""),
        ("CAL_DELTA_V", "Degrau de calibração (V)", "excursão ± em regime calmo"),
        ("CAL_DELTA_V_FORTE", "Degrau sob turbulência (V)", ""),
        ("CAL_T_SETTLE", "Espera após o degrau (s)", ""),
        ("CAL_T_MEDIA", "Média mínima por ponto (s)", ""),
        ("SE_ALVO", "Erro-padrão alvo da média", ""),
        ("T_MEDIA_MAX", "Janela máx. de média (s)", ""),
        ("QUALIDADE_J_MIN", "Qualidade mínima da J", "|J| / incerteza"),
        ("SENSIB_MIN", "Sensibilidade mínima (pos/V)", ""),
        ("COND_MAX", "Condicionamento máximo", ""),
        ("ACOPLAMENTO_MIN", "Acoplamento mínimo", "termos cruzados menores são zerados"),
    ]),
    ("Pré-centralização", [
        ("CENTRO_T_SETTLE", "Espera por passo (s)", ""),
        ("CENTRO_T_MEDIA", "Média por passo (s)", ""),
        ("CENTRO_TOL", "Tolerância de centralização", "raio alvo"),
        ("CENTRO_MAX_ITER", "Iterações máximas", ""),
        ("CENTRO_PASSO", "Fração da correção por iteração", ""),
    ]),
    ("Validação da centralização", [
        ("VALIDAR", "Validar antes de iniciar", ""),
        ("FORCAR_INICIO", "Iniciar mesmo se reprovar", ""),
        ("VAL_DURACAO", "Duração da validação (s)", ""),
        ("VAL_TOL_MEDIANA", "Tolerância da mediana", ""),
        ("VAL_TOL_DERIVA", "Tolerância de deriva", ""),
        ("VAL_TAU_CORR", "Tempo de correlação (s)", ""),
        ("VAL_RAIO", "Raio de dispersão", ""),
        ("VAL_FRACAO_MIN", "Fração mínima dentro do raio", ""),
        ("VAL_FRACAO_VALIDA_MIN", "Fração mínima de amostras válidas", ""),
        ("V_MARGEM", "Folga mínima até 0/150 V (V)", ""),
        ("VAL_MAX_TENTATIVAS", "Tentativas máximas", ""),
    ]),
    ("Teste de latência", [
        ("LAT_N_LEITURAS", "Leituras seguidas", ""),
        ("LAT_N_COMANDOS", "Comandos", ""),
        ("LAT_DELTA_V", "Degrau (V)", ""),
        ("LAT_N_REP", "Repetições por eixo", ""),
        ("LAT_T_REPOUSO_S", "Repouso antes do degrau (s)", ""),
        ("LAT_JANELA_S", "Janela de observação (s)", ""),
    ]),
    ("Telemetria ao vivo", [
        ("LIVE_INTERVALO_S", "Intervalo entre pacotes (s)", "0,05 = 20 atualizações por segundo"),
        ("LIVE_MAX_PONTOS", "Máx. de pontos por pacote", ""),
    ]),
]

# limites (mín, máx) para validação básica
LIMITES = {
    "MAX_STEP_V": (0.01, 150), "SIGMA_LIMIAR_TURB": (0, 10), "HISTERESE_TURB": (0, 0.95),
    "JANELA_TURB_S": (0.05, 120), "BACKOFF_SAT_ALTA": (0, 1), "BACKOFF_SAT_BAIXA": (0, 1),
    "BACKOFF_FATOR_MIN": (0.01, 1), "TEMPO_TOTAL_EXPERIMENTO_S": (1, 1e7), "TEMPO_LOOP_ABERTO_S": (0, 1e7),
    "V_MIN": (-10, 200), "V_MAX": (0, 200), "V_BASE": (0, 200), "V_REFERENCIA": (0, 200),
    "POLL_DQ_MS": (1, 1000), "PERIODO_LOOP_S": (0.0005, 1), "LIVE_INTERVALO_S": (0.01, 5),
    "LIVE_MAX_PONTOS": (1, 5000), "LOG_A_CADA_N_AMOSTRAS": (1, 10000), "LIMIAR_SOMA": (0, 100),
    "MAX_REINICIOS": (0, 1000), "HANG_S": (1, 1e5), "MAX_FALHAS_LEITURA_S": (0.1, 1e5),
}

# snapshot dos padrões ANTES de qualquer mudança (o v3 é alterado quando uma sessão começa)
PADROES = {}
for _g in [CONTROLE_PRINCIPAL, CONTROLE_GEOMETRICO, CONTROLE_BACKOFF] + [x[1] for x in AVANCADO]:
    for _k, *_ in _g:
        PADROES[_k] = copy.deepcopy(getattr(v3, _k))
PADROES["GANHOS"] = copy.deepcopy(v3.GANHOS)
PADROES["MODO_ENSAIO"] = "controle"
PADROES["FILENAME"] = v3.FILENAME
PADROES["PASTA_SAIDA"] = v3.PASTA_SAIDA


#%%
# =============================================================================
# SESSÃO (sem Tk): lança o supervisor/worker, recebe a telemetria UDP e o log
# =============================================================================
class _EscritorFila:
    """Substitui sys.stdout para mandar o que o supervisor imprime para o log da janela."""

    def __init__(self, fila):
        self.fila = fila
        self._resto = ""

    def write(self, texto):
        self._resto += texto
        partes = re.split(r"[\r\n]", self._resto)
        self._resto = partes.pop()
        for p in partes:
            if p.strip():
                self.fila.put(p)
        return len(texto)

    def flush(self):
        pass


class Sessao:
    """
    Um ensaio em andamento. Não usa Tk: pode ser testada sozinha.
      fila_pacotes : pacotes de posição vindos do worker (dict)
      fila_log     : linhas de log (str)
      status_linha : última linha de status do worker
    """
    TIMEOUT_PARADA_S = 25.0

    def __init__(self, cfg, retomar=False, extra_env=None):
        self.cfg = dict(cfg)
        self.retomar = retomar
        self.extra_env = extra_env or {}
        self.fila_pacotes = queue.Queue()
        self.fila_log = queue.Queue()
        self.status_linha = ""
        self.proc = None
        self.thread = None
        self.terminou = False
        self.resultado = None
        self.rc = None
        self.parada_pedida = False
        self.flag = ""
        self.cfg_path = ""
        self.porta = 0
        self._sock = None
        self._udp_ativo = False

    # ---------- UDP ----------
    def _abrir_udp(self):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind(("127.0.0.1", 0))
        self._sock.settimeout(0.2)
        self.porta = self._sock.getsockname()[1]
        self._udp_ativo = True
        threading.Thread(target=self._ouvir_udp, daemon=True).start()

    def _ouvir_udp(self):
        while self._udp_ativo:
            try:
                dados, _ = self._sock.recvfrom(65535)
                self.fila_pacotes.put(json.loads(dados.decode("utf-8")))
            except socket.timeout:
                continue
            except Exception:
                if not self._udp_ativo:
                    break

    # ---------- processo ----------
    def _exe(self):
        exe = sys.executable
        if exe.lower().endswith("pythonw.exe"):
            alt = exe[:-len("pythonw.exe")] + "python.exe"
            if os.path.exists(alt):
                exe = alt
        return exe

    def _cmd(self, retomar):
        return [self._exe(), ARQ_V3, "--worker", "--config", self.cfg_path] + (["--retomar"] if retomar else [])

    def _popen_kwargs(self):
        env = dict(os.environ, PYTHONIOENCODING="utf-8", PYTHONUNBUFFERED="1")
        env.update(self.extra_env)
        kw = dict(stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, bufsize=0, env=env)
        if os.name == "nt":
            kw["creationflags"] = 0x08000000      # CREATE_NO_WINDOW
        return kw

    def _ao_iniciar(self, proc):
        self.proc = proc
        threading.Thread(target=self._ler_saida, args=(proc,), daemon=True).start()

    def _ler_saida(self, proc):
        """Lê a saída do worker por blocos (as linhas de status usam \\r, sem \\n)."""
        dec = codecs.getincrementaldecoder("utf-8")(errors="replace")
        resto = ""
        fd = proc.stdout.fileno()
        while True:
            try:
                bloco = os.read(fd, 4096)
            except Exception:
                break
            if not bloco:
                break
            resto += dec.decode(bloco)
            partes = re.split(r"[\r\n]", resto)
            resto = partes.pop()
            for linha in partes:
                if not linha.strip():
                    continue
                if linha.lstrip().startswith("[Status:"):
                    self.status_linha = linha.strip()
                else:
                    self.fila_log.put(linha)
        if resto.strip():
            self.fila_log.put(resto)

    def _caminho_csv(self):
        return v3.CAMINHO_REF_CSV if self.cfg["MODO_ENSAIO"] == "referencia" else v3.CAMINHO_CTL_CSV

    def _rodar(self):
        modo = self.cfg["MODO_ENSAIO"]
        escritor = _EscritorFila(self.fila_log)
        try:
            if modo in ("controle", "referencia"):
                with contextlib.redirect_stdout(escritor), contextlib.redirect_stderr(escritor):
                    self.resultado = v3.supervisionar(
                        montar_cmd=self._cmd, caminho_csv=self._caminho_csv(),
                        popen_kwargs=self._popen_kwargs(), ao_iniciar=self._ao_iniciar)
            else:   # latência e API: um único processo, sem reinício automático
                proc = subprocess.Popen(self._cmd(False), **self._popen_kwargs())
                self._ao_iniciar(proc)
                self.rc = proc.wait()
                self.resultado = None
        except Exception as e:
            self.fila_log.put(f"[GUI] erro ao executar: {e}")
        finally:
            self.terminou = True

    def iniciar(self):
        pasta, nome = self.cfg["PASTA_SAIDA"], self.cfg["FILENAME"]
        os.makedirs(pasta, exist_ok=True)
        self.flag = os.path.join(pasta, f"{nome}_parar.flag")
        if os.path.exists(self.flag):
            os.remove(self.flag)
        self._abrir_udp()
        self.cfg["LIVE_PORTA"] = self.porta
        self.cfg["PARAR_FLAG"] = self.flag
        self.cfg["RETOMAR_ENSAIO_ANTERIOR"] = bool(self.retomar)
        self.cfg_path = os.path.join(pasta, f"{nome}_config.json")
        with open(self.cfg_path, "w", encoding="utf-8") as f:
            json.dump(self.cfg, f, indent=2, ensure_ascii=False)
        v3.aplicar_config_dict(self.cfg)      # o supervisor (neste processo) usa HANG_S, MAX_REINICIOS, etc.
        self.thread = threading.Thread(target=self._rodar, daemon=True)
        self.thread.start()

    @property
    def rodando(self):
        return self.thread is not None and not self.terminou

    def parar(self):
        """Pede parada segura (arquivo-sinal). Se o worker não sair a tempo, é encerrado à força."""
        if not self.rodando or self.parada_pedida:
            return
        self.parada_pedida = True
        try:
            open(self.flag, "w").write("parar")
        except Exception:
            pass
        threading.Timer(self.TIMEOUT_PARADA_S, self._matar).start()

    def _matar(self):
        p = self.proc
        if p is not None and p.poll() is None:
            self.fila_log.put("[GUI] o processo não parou a tempo: encerrando à força.")
            try:
                p.kill()
            except Exception:
                pass

    def fechar(self):
        self._udp_ativo = False
        try:
            if self._sock:
                self._sock.close()
        except Exception:
            pass
        try:
            if self.flag and os.path.exists(self.flag):
                os.remove(self.flag)
        except Exception:
            pass


#%%
# =============================================================================
# JANELA
# =============================================================================
class FrameRolavel(ttk.Frame):
    """Frame com barra de rolagem vertical (para as muitas configurações)."""

    def __init__(self, master, largura=None):
        super().__init__(master)
        kw = {"highlightthickness": 0}
        if largura:
            kw["width"] = largura
        self.canvas = tk.Canvas(self, **kw)
        self.vsb = ttk.Scrollbar(self, orient="vertical", command=self.canvas.yview)
        self.interior = ttk.Frame(self.canvas)
        self.interior.bind("<Configure>", lambda e: self.canvas.configure(scrollregion=self.canvas.bbox("all")))
        self._janela = self.canvas.create_window((0, 0), window=self.interior, anchor="nw")
        self.canvas.configure(yscrollcommand=self.vsb.set)
        self.canvas.pack(side="left", fill="both", expand=True)
        self.vsb.pack(side="right", fill="y")
        self.canvas.bind("<Configure>", lambda e: self.canvas.itemconfigure(self._janela, width=e.width))
        self.canvas.bind("<Enter>", lambda e: self.canvas.bind_all("<MouseWheel>", self._roda))
        self.canvas.bind("<Leave>", lambda e: self.canvas.unbind_all("<MouseWheel>"))

    def _roda(self, e):
        self.canvas.yview_scroll(int(-e.delta / 120), "units")


def _fmt(valor):
    if isinstance(valor, bool):
        return valor
    if isinstance(valor, float):
        return f"{valor:g}"
    if isinstance(valor, tuple):
        return "; ".join(f"{a:g}" for a in valor)
    return str(valor)


class App:
    JANELAS_S = {"10 s": 10, "30 s": 30, "60 s": 60, "120 s": 120}
    ESCALAS = {"±0,25": 0.25, "±0,5": 0.5, "±1,0": 1.0, "±2,0": 2.0, "Auto": None}
    EXTRA_ENV = {}      # usado nos testes (ex.: simulador do Kinesis)

    def __init__(self, root):
        self.root = root
        root.title("Polaris - Controle de Feixe")
        root.geometry("1360x860")
        try:
            style = ttk.Style()
            style.theme_use("vista" if os.name == "nt" else "clam")
        except Exception:
            pass

        self.sessao = None
        self._finalizada = True
        self.vars = {}            # chave -> StringVar/BooleanVar
        self.vars_ganhos = {}     # (regime, param) -> StringVar
        self.chunks = deque()     # blocos (t, x, y, vx, vy, s) dos últimos ~150 s
        self.meta = {}
        self.t_ref = None
        self.reinicios = 0
        self._n_log = 0
        self.ultimo_csv = None

        self.var_modo = tk.StringVar(value=list(MODOS)[0])
        self.var_estado = tk.StringVar(value="Pronto")
        self.var_status_worker = tk.StringVar(value="")
        self.var_janela = tk.StringVar(value="30 s")
        self.var_escala = tk.StringVar(value="±0,5")
        self.var_pausar = tk.BooleanVar(value=False)
        self.info = {k: tk.StringVar(value="-") for k in
                     ("modo", "pos", "raio", "tensao", "soma", "taxa", "tempo", "ganho", "reinicios")}

        self._montar_barra()          # primeiro, para a barra de status ficar sempre visível embaixo
        self._montar_abas()
        self._montar_ensaio()
        self._montar_avancado()
        self._montar_log()
        self._criar_graficos()
        self._carregar_auto()
        self._atualizar_botoes()
        root.protocol("WM_DELETE_WINDOW", self._sair)
        root.after(100, self._tick)

    # ------------------------------------------------------------------ layout
    def _montar_abas(self):
        self.nb = ttk.Notebook(self.root)
        self.nb.pack(fill="both", expand=True)
        self.aba_ensaio = ttk.Frame(self.nb)
        self.aba_avancado = ttk.Frame(self.nb)
        self.aba_log = ttk.Frame(self.nb)
        self.nb.add(self.aba_ensaio, text="  Ensaio  ")
        self.nb.add(self.aba_avancado, text="  Configurações avançadas  ")
        self.nb.add(self.aba_log, text="  Log  ")

    def _campo(self, pai, linha, chave, rotulo, dica, largura=14):
        padrao = PADROES[chave]
        ttk.Label(pai, text=rotulo).grid(row=linha, column=0, sticky="w", padx=(4, 8), pady=1)
        if isinstance(padrao, bool):
            var = tk.BooleanVar(value=padrao)
            ttk.Checkbutton(pai, variable=var).grid(row=linha, column=1, sticky="w")
        elif chave in OPCOES:
            var = tk.StringVar(value=str(padrao))
            ttk.Combobox(pai, textvariable=var, values=OPCOES[chave], width=largura - 2,
                         state="readonly").grid(row=linha, column=1, sticky="w")
        else:
            var = tk.StringVar(value=_fmt(padrao))
            ttk.Entry(pai, textvariable=var, width=largura).grid(row=linha, column=1, sticky="w")
        if dica:
            ttk.Label(pai, text=dica, foreground="#666666").grid(row=linha, column=2, sticky="w", padx=8)
        self.vars[chave] = var

    def _grupo(self, pai, titulo, itens, coluna_dica=True):
        quadro = ttk.LabelFrame(pai, text=titulo)
        quadro.pack(fill="x", padx=6, pady=4)
        for i, (chave, rotulo, dica) in enumerate(itens):
            self._campo(quadro, i, chave, rotulo, dica if coluna_dica else "")
        return quadro

    def _montar_ensaio(self):
        aba = self.aba_ensaio
        aba.columnconfigure(1, weight=1)
        aba.rowconfigure(0, weight=1)
        esq = FrameRolavel(aba, largura=430)
        esq.grid(row=0, column=0, sticky="ns")
        dir_ = ttk.Frame(aba)
        dir_.grid(row=0, column=1, sticky="nsew")
        self.dir_ = dir_
        L = esq.interior

        # --- ensaio
        q = ttk.LabelFrame(L, text="Ensaio")
        q.pack(fill="x", padx=6, pady=4)
        q.columnconfigure(1, weight=1)
        ttk.Label(q, text="Modo de operação").grid(row=0, column=0, sticky="w", padx=4, pady=2)
        ttk.Combobox(q, textvariable=self.var_modo, values=list(MODOS), state="readonly",
                     width=38).grid(row=0, column=1, columnspan=2, sticky="ew", padx=4)
        ttk.Label(q, text="Nome do arquivo").grid(row=1, column=0, sticky="w", padx=4, pady=2)
        self.vars["FILENAME"] = tk.StringVar(value=PADROES["FILENAME"])
        ttk.Entry(q, textvariable=self.vars["FILENAME"]).grid(row=1, column=1, columnspan=2, sticky="ew", padx=4)
        ttk.Label(q, text="Pasta de saída").grid(row=2, column=0, sticky="w", padx=4, pady=2)
        self.vars["PASTA_SAIDA"] = tk.StringVar(value=PADROES["PASTA_SAIDA"])
        ttk.Entry(q, textvariable=self.vars["PASTA_SAIDA"]).grid(row=2, column=1, sticky="ew", padx=4)
        ttk.Button(q, text="...", width=3, command=self._escolher_pasta).grid(row=2, column=2, padx=(0, 4))
        ttk.Label(q, text="Saída: <nome>_controle.csv (ou _referencia.csv), além de _config.json e _validacao.json",
                  foreground="#666666", wraplength=400).grid(row=3, column=0, columnspan=3, sticky="w", padx=4, pady=2)
        bt = ttk.Frame(q)
        bt.grid(row=4, column=0, columnspan=3, pady=6)
        self.bt_iniciar = ttk.Button(bt, text="Iniciar", command=lambda: self._iniciar(False))
        self.bt_iniciar.pack(side="left", padx=3)
        self.bt_parar = ttk.Button(bt, text="Parar", command=self._parar)
        self.bt_parar.pack(side="left", padx=3)
        self.bt_retomar = ttk.Button(bt, text="Retomar interrompido", command=lambda: self._iniciar(True))
        self.bt_retomar.pack(side="left", padx=3)

        # --- ganhos
        g = ttk.LabelFrame(L, text="Ganhos do PI (iguais para X e Y)")
        g.pack(fill="x", padx=6, pady=4)
        for j, nome in enumerate(("", "kp", "ki (1/s)", "alpha")):
            ttk.Label(g, text=nome).grid(row=0, column=j, padx=4)
        for i, reg in enumerate(("Calmo", "Turbulento"), start=1):
            ttk.Label(g, text=reg).grid(row=i, column=0, sticky="w", padx=4, pady=2)
            for j, par in enumerate(("kp", "ki", "alpha"), start=1):
                var = tk.StringVar(value=_fmt(float(PADROES["GANHOS"][reg][par])))
                ttk.Entry(g, textvariable=var, width=9).grid(row=i, column=j, padx=3)
                self.vars_ganhos[(reg, par)] = var
        ttk.Label(g, text="kp: fração do erro corrigida por amostra | ki: ganho integral | "
                          "alpha: filtro (1 = sem filtro)", foreground="#666666",
                  wraplength=400).grid(row=3, column=0, columnspan=4, sticky="w", padx=4, pady=2)

        # --- demais variáveis de controle
        self._grupo(L, "Regime e limites", CONTROLE_PRINCIPAL)
        self._grupo(L, "Escalonamento geométrico do ganho integral", CONTROLE_GEOMETRICO)
        self._grupo(L, "Recuo automático por saturação", CONTROLE_BACKOFF)

        c = ttk.LabelFrame(L, text="Configuração")
        c.pack(fill="x", padx=6, pady=4)
        ttk.Button(c, text="Salvar...", command=self._salvar_config).pack(side="left", padx=4, pady=4)
        ttk.Button(c, text="Carregar...", command=self._carregar_config).pack(side="left", padx=4)
        ttk.Button(c, text="Restaurar padrões", command=self._restaurar).pack(side="left", padx=4)

        # --- lado direito: informações + gráficos + análise
        info = ttk.LabelFrame(dir_, text="Ao vivo")
        info.pack(fill="x", padx=6, pady=4)
        rot = [("modo", "Fase / regime"), ("pos", "Posição (x, y)"), ("raio", "Distância ao centro"),
               ("tensao", "Tensões (V)"), ("soma", "Soma do detector"), ("taxa", "Taxa útil"),
               ("tempo", "Tempo"), ("ganho", "Fator de ganho"), ("reinicios", "Reinícios")]
        for i, (k, nome) in enumerate(rot):
            col = (i % 3) * 2
            ttk.Label(info, text=nome + ":", foreground="#666666").grid(row=i // 3, column=col, sticky="e", padx=(8, 2))
            ttk.Label(info, textvariable=self.info[k], font=("Arial", 11, "bold")).grid(
                row=i // 3, column=col + 1, sticky="w", padx=(0, 12))
        self.quadro_graf = ttk.Frame(dir_)
        self.quadro_graf.pack(fill="both", expand=True, padx=6)
        ctr = ttk.Frame(dir_)
        ctr.pack(fill="x", padx=6, pady=4)
        ttk.Label(ctr, text="Janela do tempo:").pack(side="left")
        ttk.Combobox(ctr, textvariable=self.var_janela, values=list(self.JANELAS_S), width=7,
                     state="readonly").pack(side="left", padx=4)
        ttk.Label(ctr, text="Escala XY:").pack(side="left", padx=(12, 0))
        ttk.Combobox(ctr, textvariable=self.var_escala, values=list(self.ESCALAS), width=7,
                     state="readonly").pack(side="left", padx=4)
        ttk.Checkbutton(ctr, text="Pausar gráficos", variable=self.var_pausar).pack(side="left", padx=12)
        ttk.Button(ctr, text="Ver resultado", command=self._ver_resultado).pack(side="right", padx=3)
        ttk.Button(ctr, text="Comparar referência x controle", command=self._comparar).pack(side="right", padx=3)
        ttk.Button(ctr, text="Abrir pasta", command=self._abrir_pasta).pack(side="right", padx=3)

    def _montar_avancado(self):
        rol = FrameRolavel(self.aba_avancado)
        rol.pack(fill="both", expand=True)
        ttk.Label(rol.interior, text="Estas configurações valem para o próximo ensaio. "
                  "Os valores padrão vêm do PolarisPID_v3.py.", foreground="#666666").pack(anchor="w", padx=8, pady=4)
        for titulo, itens in AVANCADO:
            self._grupo(rol.interior, titulo, itens)

    def _montar_log(self):
        self.txt_log = tk.Text(self.aba_log, wrap="none", height=20, state="normal")
        sb = ttk.Scrollbar(self.aba_log, orient="vertical", command=self.txt_log.yview)
        self.txt_log.configure(yscrollcommand=sb.set)
        self.txt_log.pack(side="left", fill="both", expand=True)
        sb.pack(side="right", fill="y")

    def _montar_barra(self):
        barra = ttk.Frame(self.root)
        barra.pack(fill="x", side="bottom")
        ttk.Label(barra, textvariable=self.var_estado, font=("Arial", 10, "bold")).pack(side="left", padx=8)
        ttk.Label(barra, textvariable=self.var_status_worker, foreground="#555555").pack(side="left", padx=8)

    # ---------------------------------------------------------------- gráficos
    def _criar_graficos(self):
        self.fig = Figure(figsize=(9, 6), dpi=100, constrained_layout=True)
        gs = self.fig.add_gridspec(2, 2, width_ratios=[1, 1.25])
        self.ax_xy = self.fig.add_subplot(gs[:, 0])
        self.ax_pos = self.fig.add_subplot(gs[0, 1])
        self.ax_v = self.fig.add_subplot(gs[1, 1], sharex=self.ax_pos)

        ax = self.ax_xy
        ax.set_title("Posição do feixe no detector")
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        ax.set_aspect("equal", adjustable="box")
        ax.grid(True, alpha=0.3)
        for r in (0.1, 0.2, 0.3):
            ax.add_patch(Circle((0, 0), r, fill=False, ls="--", color="gray", lw=0.8))
        ax.axhline(0, color="k", lw=0.6)
        ax.axvline(0, color="k", lw=0.6)
        self.ln_trilha, = ax.plot([], [], "-", color="tab:blue", alpha=0.35, lw=0.8)
        self.pt_atual, = ax.plot([], [], "o", color="tab:red", ms=10, mec="k")
        self.txt_aviso = ax.text(0.5, 0.95, "", transform=ax.transAxes, ha="center", va="top",
                                 color="tab:red", fontweight="bold")
        ax.set_xlim(-0.5, 0.5)
        ax.set_ylim(-0.5, 0.5)

        self.ax_pos.set_title("Posição x, y")
        self.ax_pos.grid(True, alpha=0.3)
        self.ax_pos.axhline(0, color="k", lw=0.6, ls=":")
        self.ln_x, = self.ax_pos.plot([], [], color="tab:blue", lw=0.9, label="x")
        self.ln_y, = self.ax_pos.plot([], [], color="tab:red", lw=0.9, label="y")
        self.ax_pos.legend(loc="upper right")

        self.ax_v.set_title("Tensões enviadas aos espelhos")
        self.ax_v.set_xlabel("Tempo desde o início (s)")
        self.ax_v.set_ylabel("V")
        self.ax_v.set_ylim(-5, 155)
        self.ax_v.grid(True, alpha=0.3)
        self.ax_v.axhline(0, color="k", lw=0.6, ls=":")
        self.ax_v.axhline(150, color="k", lw=0.6, ls=":")
        self.ln_vx, = self.ax_v.plot([], [], color="tab:blue", lw=0.9, label="Vx")
        self.ln_vy, = self.ax_v.plot([], [], color="tab:red", lw=0.9, label="Vy")
        self.ax_v.legend(loc="upper right")

        self.canvas = FigureCanvasTkAgg(self.fig, master=self.quadro_graf)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

    def _janela_dados(self, segundos):
        """Concatena os blocos dos últimos `segundos` segundos. Devolve arrays (t, x, y, vx, vy, s)."""
        if not self.chunks:
            return None
        t_fim = self.chunks[-1][0][-1]
        sel = []
        for ch in reversed(self.chunks):
            sel.append(ch)
            if ch[0][0] < t_fim - segundos:
                break
        sel.reverse()
        arr = [np.concatenate([c[i] for c in sel]) for i in range(6)]
        m = arr[0] >= t_fim - segundos
        return [a[m] for a in arr]

    @staticmethod
    def _decimar(*arrays, maximo=2500):
        n = len(arrays[0])
        passo = max(1, n // maximo)
        return [a[::passo] for a in arrays]

    def _atualizar_graficos(self):
        janela = self.JANELAS_S.get(self.var_janela.get(), 30)
        dados = self._janela_dados(janela)
        if dados is None or len(dados[0]) == 0:
            return
        t, x, y, vx, vy, s = dados
        tr = t - self.t_ref
        td, xd, yd, vxd, vyd = self._decimar(tr, x, y, vx, vy)
        self.ln_x.set_data(td, xd)
        self.ln_y.set_data(td, yd)
        self.ln_vx.set_data(td, vxd)
        self.ln_vy.set_data(td, vyd)
        self.ax_pos.set_xlim(td[-1] - janela, td[-1] + 0.01)
        amp = max(0.1, 1.15 * float(np.nanmax(np.abs(np.concatenate([xd, yd])))))
        self.ax_pos.set_ylim(-amp, amp)

        # trilha XY dos últimos 2 s e ponto atual
        m = t >= t[-1] - 2.0
        xt, yt = self._decimar(x[m], y[m], maximo=1200)
        self.ln_trilha.set_data(xt, yt)
        self.pt_atual.set_data([x[-1]], [y[-1]])
        esc = self.ESCALAS.get(self.var_escala.get(), 0.5)
        if esc is None:
            esc = max(0.2, 1.15 * float(np.nanmax(np.abs(np.concatenate([xd, yd])))))
        self.ax_xy.set_xlim(-esc, esc)
        self.ax_xy.set_ylim(-esc, esc)
        limiar = float(self.meta.get("limiar", PADROES["LIMIAR_SOMA"]))
        self.txt_aviso.set_text("SINAL FRACO" if s[-1] <= limiar else "")
        self.canvas.draw_idle()

    # ----------------------------------------------------------- configuração
    def _parse(self, chave, texto):
        padrao = PADROES[chave]
        if isinstance(padrao, bool):
            return bool(texto)
        if isinstance(padrao, str):
            return str(texto).strip()
        if isinstance(padrao, tuple):
            partes = [p for p in re.split(r"[;\s]+", str(texto).strip()) if p]
            return tuple(float(p.replace(",", ".")) for p in partes)
        valor = float(str(texto).strip().replace(",", "."))
        if not math.isfinite(valor):
            raise ValueError("valor não finito")
        if isinstance(padrao, int):
            if abs(valor - round(valor)) > 1e-9:
                raise ValueError("precisa ser inteiro")
            return int(round(valor))
        return valor

    def ler_config(self):
        """Lê todos os campos. Devolve (cfg, erros) com erros como lista de textos."""
        cfg, erros = {}, []
        cfg["MODO_ENSAIO"] = MODOS.get(self.var_modo.get(), "controle")
        for chave, var in self.vars.items():
            try:
                cfg[chave] = self._parse(chave, var.get())
            except Exception as e:
                erros.append(f"{chave}: valor inválido ({e})")
        ganhos = {}
        for (reg, par), var in self.vars_ganhos.items():
            try:
                ganhos.setdefault(reg, {})[par] = float(str(var.get()).strip().replace(",", "."))
            except Exception:
                erros.append(f"Ganho {reg}/{par}: valor inválido")
        cfg["GANHOS"] = ganhos
        # limites por campo
        for chave, (lo, hi) in LIMITES.items():
            v = cfg.get(chave)
            if isinstance(v, (int, float)) and not isinstance(v, bool) and not (lo <= v <= hi):
                erros.append(f"{chave}: fora do intervalo [{lo:g}, {hi:g}]")
        for reg, d in ganhos.items():
            if d.get("kp", 0) < 0 or d.get("ki", 0) < 0 or not (0 < d.get("alpha", 1) <= 1):
                erros.append(f"Ganhos {reg}: kp e ki >= 0 e 0 < alpha <= 1")
        nome = cfg.get("FILENAME", "")
        if not nome or re.search(r'[<>:"/\\|?*]', nome):
            erros.append('Nome do arquivo vazio ou com caracteres inválidos (< > : " / \\ | ? *)')
        if not cfg.get("PASTA_SAIDA"):
            erros.append("Pasta de saída vazia")
        if not erros:
            if cfg["V_MIN"] >= cfg["V_MAX"]:
                erros.append("V_MIN precisa ser menor que V_MAX")
            if not (cfg["V_MIN"] <= cfg["V_BASE"] <= cfg["V_MAX"]):
                erros.append("V_BASE precisa estar entre V_MIN e V_MAX")
            if cfg["MODO_ENSAIO"] == "controle" and cfg["TEMPO_LOOP_ABERTO_S"] >= cfg["TEMPO_TOTAL_EXPERIMENTO_S"]:
                erros.append("O tempo em malha aberta precisa ser menor que a duração total (senão o controle nunca liga)")
        return cfg, erros

    def aplicar_na_tela(self, cfg):
        """Preenche os campos com um dicionário de configuração (os ausentes ficam como estão)."""
        modo = cfg.get("MODO_ENSAIO")
        for nome, cod in MODOS.items():
            if cod == modo:
                self.var_modo.set(nome)
        for chave, var in self.vars.items():
            if chave in cfg:
                v = cfg[chave]
                if isinstance(var, tk.BooleanVar):
                    var.set(bool(v))
                else:
                    var.set(_fmt(tuple(v) if isinstance(PADROES[chave], tuple) else v))
        for reg, d in cfg.get("GANHOS", {}).items():
            for par, val in d.items():
                if (reg, par) in self.vars_ganhos:
                    self.vars_ganhos[(reg, par)].set(_fmt(float(val)))

    def _salvar_auto(self):
        cfg, _ = self.ler_config()
        try:
            with open(CONFIG_AUTO, "w", encoding="utf-8") as f:
                json.dump(cfg, f, indent=2, ensure_ascii=False)
        except Exception:
            pass

    def _carregar_auto(self):
        try:
            with open(CONFIG_AUTO, encoding="utf-8") as f:
                self.aplicar_na_tela(json.load(f))
        except Exception:
            pass

    def _salvar_config(self):
        cfg, erros = self.ler_config()
        if erros:
            messagebox.showerror("Configuração inválida", "\n".join(erros))
            return
        caminho = filedialog.asksaveasfilename(defaultextension=".json", filetypes=[("JSON", "*.json")],
                                               initialfile="config_polaris.json")
        if caminho:
            with open(caminho, "w", encoding="utf-8") as f:
                json.dump(cfg, f, indent=2, ensure_ascii=False)

    def _carregar_config(self):
        caminho = filedialog.askopenfilename(filetypes=[("JSON", "*.json")])
        if caminho:
            try:
                with open(caminho, encoding="utf-8") as f:
                    self.aplicar_na_tela(json.load(f))
            except Exception as e:
                messagebox.showerror("Erro ao carregar", str(e))

    def _restaurar(self):
        if messagebox.askyesno("Restaurar padrões", "Voltar todos os campos aos valores padrão do v3?"):
            self.aplicar_na_tela(PADROES)

    def _escolher_pasta(self):
        pasta = filedialog.askdirectory()
        if pasta:
            self.vars["PASTA_SAIDA"].set(pasta)

    # ----------------------------------------------------------------- ensaio
    def _csv_do_modo(self, cfg):
        sufixo = "referencia" if cfg["MODO_ENSAIO"] == "referencia" else "controle"
        return os.path.join(cfg["PASTA_SAIDA"], f"{cfg['FILENAME']}_{sufixo}.csv")

    def _iniciar(self, retomar=False):
        if self.sessao is not None and self.sessao.rodando:
            return
        cfg, erros = self.ler_config()
        if erros:
            messagebox.showerror("Configuração inválida", "\n".join(erros))
            return
        modo = cfg["MODO_ENSAIO"]
        csv_ = self._csv_do_modo(cfg)
        if modo in ("controle", "referencia"):
            estado = os.path.splitext(csv_)[0] + "_estado.json"
            if retomar and not os.path.exists(estado):
                messagebox.showerror("Nada para retomar", f"Não existe o arquivo de estado:\n{estado}")
                return
            if not retomar and os.path.exists(csv_):
                if not messagebox.askyesno("Arquivo existente", f"'{csv_}' já existe e será SOBRESCRITO.\nContinuar?"):
                    return
        elif retomar:
            return
        self._salvar_auto()
        self.chunks.clear()
        self.meta = {"limiar": cfg["LIMIAR_SOMA"]}
        self.t_ref = None
        self.reinicios = 0
        self.info["reinicios"].set("0")
        self.txt_log.delete("1.0", "end")
        self._n_log = 0
        self.sessao = Sessao(cfg, retomar=retomar, extra_env=self.EXTRA_ENV)
        self._finalizada = False
        try:
            self.sessao.iniciar()
        except Exception as e:
            messagebox.showerror("Erro ao iniciar", str(e))
            self.sessao = None
            self._finalizada = True
            return
        self.ultimo_csv = csv_ if modo in ("controle", "referencia") else None
        self.var_estado.set("Executando (preparando)..." if modo in ("controle", "referencia") else "Executando...")
        self._atualizar_botoes()

    def _parar(self):
        if self.sessao is not None and self.sessao.rodando:
            self.sessao.parar()
            self.var_estado.set("Parando com segurança...")
            self._atualizar_botoes()

    def _atualizar_botoes(self):
        rodando = self.sessao is not None and self.sessao.rodando
        self.bt_iniciar.configure(state="disabled" if rodando else "normal")
        self.bt_retomar.configure(state="disabled" if rodando else "normal")
        self.bt_parar.configure(state="normal" if rodando else "disabled")

    def _log(self, linha):
        self.txt_log.insert("end", linha + "\n")
        self.txt_log.see("end")
        self._n_log += 1
        if self._n_log > 6000:                       # mantém o log com tamanho limitado
            self.txt_log.delete("1.0", "2000.0")
            self._n_log -= 2000
        if "terminou com código" in linha:
            self.reinicios += 1
            self.info["reinicios"].set(str(self.reinicios))

    # ------------------------------------------------------------------- tick
    def _tick(self):
        try:
            self._processar()
        except Exception as e:                       # a janela nunca deve morrer por um erro de exibição
            self.var_status_worker.set(f"[GUI] erro de exibição: {e}")
        self.root.after(100, self._tick)

    def _processar(self):
        s = self.sessao
        if s is None:
            return
        # log
        for _ in range(500):
            try:
                self._log(s.fila_log.get_nowait())
            except queue.Empty:
                break
        self.var_status_worker.set(s.status_linha)
        # pacotes de posição
        novos = 0
        for _ in range(400):
            try:
                p = s.fila_pacotes.get_nowait()
            except queue.Empty:
                break
            if p.get("k") != "d" or not p.get("t"):
                continue
            arr = [np.array(p[k], dtype=float) for k in ("t", "x", "y", "vx", "vy", "s")]
            if self.t_ref is None:
                self.t_ref = float(arr[0][0])
            self.chunks.append(tuple(arr))
            self.meta.update({k: p.get(k) for k in ("m", "rg", "sat", "fg", "tx", "te", "tt")})
            novos += 1
        if self.chunks:
            t_fim = self.chunks[-1][0][-1]
            while len(self.chunks) > 1 and self.chunks[0][0][-1] < t_fim - 150:
                self.chunks.popleft()
        if novos:
            self._atualizar_info()
            if not self.var_pausar.get():
                self._atualizar_graficos()
        # fim da sessão
        if s.terminou and not self._finalizada:
            self._finalizada = True
            self._ao_terminar()

    def _atualizar_info(self):
        d = self.chunks[-1]
        x, y, vx, vy, so = d[1][-1], d[2][-1], d[3][-1], d[4][-1], d[5][-1]
        m = self.meta
        self.info["modo"].set(f"{m.get('m', '-')} / {m.get('rg') or '-'}")
        self.info["pos"].set(f"({x:+.3f}, {y:+.3f})")
        self.info["raio"].set(f"{math.hypot(x, y):.3f}")
        self.info["tensao"].set(f"({vx:.1f}, {vy:.1f})" if not (math.isnan(vx) or math.isnan(vy)) else "-")
        self.info["soma"].set(f"{so:.2f}")
        self.info["taxa"].set(f"{m['tx']:.0f} Hz" if m.get("tx") else "-")
        if m.get("te") is not None and m.get("tt"):
            self.info["tempo"].set(f"{m['te']:.0f} / {m['tt']:.0f} s")
        self.info["ganho"].set(f"{m['fg']*100:.0f}%" if m.get("fg") is not None else "-")
        if self.sessao is not None and self.sessao.rodando and not self.sessao.parada_pedida:
            self.var_estado.set(f"Executando - {m.get('m', '')}")

    def _ao_terminar(self):
        s = self.sessao
        self._atualizar_botoes()
        estado = {}
        if self.ultimo_csv:
            estado = v3._carregar_estado(v3._caminho_estado(self.ultimo_csv)) or {}
        if estado.get("concluido"):
            msg = "Concluído"
        elif estado.get("interrompido") or s.parada_pedida:
            msg = "Interrompido pelo usuário"
        elif s.resultado is None and s.cfg["MODO_ENSAIO"] in ("controle", "referencia"):
            msg = "Abortado (veja o Log)"
        else:
            msg = "Finalizado"
        if s.cfg["MODO_ENSAIO"] in ("latencia", "api"):
            msg = f"Finalizado (código {s.rc})"
        self.var_estado.set(f"{msg}. Reinícios automáticos: {self.reinicios}")
        s.fechar()

    # --------------------------------------------------------------- análises
    def _pasta_atual(self):
        return self.vars["PASTA_SAIDA"].get().strip() or "."

    def _valor(self, chave):
        """Valor atual de um campo da tela (ou o padrão, se o texto estiver inválido)."""
        try:
            return self._parse(chave, self.vars[chave].get())
        except Exception:
            return PADROES[chave]

    def _abrir_pasta(self):
        pasta = os.path.abspath(self._pasta_atual())
        os.makedirs(pasta, exist_ok=True)
        try:
            if os.name == "nt":
                os.startfile(pasta)
            else:
                subprocess.Popen(["xdg-open", pasta])
        except Exception as e:
            messagebox.showerror("Erro", str(e))

    def _ver_resultado(self):
        caminho = self.ultimo_csv if self.ultimo_csv and os.path.exists(self.ultimo_csv) else None
        if caminho is None:
            caminho = filedialog.askopenfilename(initialdir=self._pasta_atual(), filetypes=[("CSV", "*.csv")])
        if not caminho:
            return
        try:
            import pandas as pd
            df = pd.read_csv(caminho)
        except Exception as e:
            messagebox.showerror("Erro ao ler o CSV", str(e))
            return
        jan = tk.Toplevel(self.root)
        jan.title(f"Resultado - {os.path.basename(caminho)}")
        jan.geometry("1100x850")
        fig = Figure(figsize=(10, 8), dpi=100, constrained_layout=True)
        passo = max(1, len(df) // 120000)
        d = df.iloc[::passo]
        ax1, ax2, ax3 = fig.subplots(3, 1, sharex=True)
        ax1.plot(d["tempo_s"], d["x_pos_real"], color="tab:blue", lw=0.6, label="x")
        ax1.plot(d["tempo_s"], d["y_pos_real"], color="tab:red", lw=0.6, label="y")
        ax1.axhline(0, color="k", ls=":", lw=0.6)
        ax1.set_ylabel("Posição")
        ax1.legend(loc="upper right")
        ax2.plot(d["tempo_s"], d["tensao_x"], color="tab:blue", lw=0.6, label="Vx")
        ax2.plot(d["tempo_s"], d["tensao_y"], color="tab:red", lw=0.6, label="Vy")
        ax2.axhline(0, color="k", ls=":", lw=0.6)
        ax2.axhline(150, color="k", ls=":", lw=0.6)
        ax2.set_ylabel("Tensão (V)")
        ax2.legend(loc="upper right")
        ax3.plot(d["tempo_s"], d["intensidade"], color="tab:green", lw=0.6)
        ax3.set_ylabel("Soma")
        ax3.set_xlabel("Tempo (s)")
        for ax in (ax1, ax2, ax3):
            ax.grid(True, alpha=0.3)
            if "modo" in df.columns and (df["modo"] == "Fechado").any():
                ax.axvline(df.loc[df["modo"] == "Fechado", "tempo_s"].min(), color="green", ls="--", lw=1)
        resumo = []
        for modo in ("Aberto", "Referencia", "Fechado"):
            if "modo" not in df.columns:
                break
            sub = df[(df["modo"] == modo) & (df["intensidade"] > self._valor("LIMIAR_SOMA"))]
            if modo == "Fechado" and len(sub):
                sub = sub[sub["tempo_s"] > sub["tempo_s"].min() + self._valor("DESCARTAR_CTL_INICIAL_S")]
            if len(sub):
                r = np.hypot(sub["x_pos_real"], sub["y_pos_real"])
                resumo.append(f"{modo}: centroide=({sub['x_pos_real'].mean():+.3f}, {sub['y_pos_real'].mean():+.3f}) "
                              f"σ=({sub['x_pos_real'].std():.3f}, {sub['y_pos_real'].std():.3f}) "
                              f"rms r={np.sqrt((r**2).mean()):.3f} | r<0,2: {(r < 0.2).mean()*100:.0f}%")
        ttk.Label(jan, text="\n".join(resumo) or "(sem resumo)", justify="left").pack(anchor="w", padx=8, pady=4)
        canvas = FigureCanvasTkAgg(fig, master=jan)
        NavigationToolbar2Tk(canvas, jan)
        canvas.get_tk_widget().pack(fill="both", expand=True)
        canvas.draw()

    def _comparar(self):
        pasta, nome = self._pasta_atual(), self.vars["FILENAME"].get().strip()
        ref = os.path.join(pasta, f"{nome}_referencia.csv")
        ctl = os.path.join(pasta, f"{nome}_controle.csv")
        if not os.path.exists(ref):
            ref = filedialog.askopenfilename(title="CSV de referência (sem controle)", initialdir=pasta,
                                             filetypes=[("CSV", "*.csv")])
        if not ref:
            return
        if not os.path.exists(ctl):
            ctl = filedialog.askopenfilename(title="CSV com controle", initialdir=pasta, filetypes=[("CSV", "*.csv")])
        if not ctl:
            return
        png = os.path.join(pasta, f"comparacao_centroides_{nome}.png")
        buf = io.StringIO()
        try:
            with contextlib.redirect_stdout(buf):
                v3.comparar_centroides(
                    ref, ctl, salvar_png=png, mostrar=False,
                    descartar_ref_s=self._valor("DESCARTAR_REF_INICIAL_S"),
                    descartar_ctl_s=self._valor("DESCARTAR_CTL_INICIAL_S"),
                    raios=self._valor("RAIOS_COMPARACAO"))
        except Exception as e:
            messagebox.showerror("Erro na comparação", str(e))
            return
        jan = tk.Toplevel(self.root)
        jan.title("Comparação: referência x controle")
        ttk.Label(jan, text=buf.getvalue(), justify="left", font=("Courier", 9)).pack(anchor="w", padx=8, pady=4)
        try:
            self._img = tk.PhotoImage(file=png).subsample(2, 2)
            ttk.Label(jan, image=self._img).pack(padx=8, pady=4)
        except Exception:
            ttk.Label(jan, text=f"Figura salva em {png}").pack()

    # ------------------------------------------------------------------ saída
    def _sair(self):
        if self.sessao is not None and self.sessao.rodando:
            if not messagebox.askyesno("Ensaio em andamento", "Parar o ensaio com segurança e sair?"):
                return
            self.sessao.parar()
            self._aguardar_e_sair(0)
            return
        self._salvar_auto()
        self.root.destroy()

    def _aguardar_e_sair(self, n):
        if self.sessao is not None and self.sessao.rodando and n < 300:
            self.root.after(100, lambda: self._aguardar_e_sair(n + 1))
            return
        self._salvar_auto()
        self.root.destroy()


def main():
    try:      # nitidez em telas de alta resolução no Windows
        import ctypes
        ctypes.windll.shcore.SetProcessDpiAwareness(1)
    except Exception:
        pass
    root = tk.Tk()
    App(root)
    root.mainloop()


if __name__ == "__main__":
    main()
