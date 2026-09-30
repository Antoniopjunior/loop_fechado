#%%
"""
Controle PID em malha fechada para alinhamento com Thorlabs Polaris.

Estrutura:
    - PID: lógica de controle pura, sem dependência de hardware (testável).
    - PolarisRig: encapsula conexão/desconexão segura dos dispositivos
      (context manager garante liberação das portas USB mesmo em erro).
    - ExperimentConfig / PIDGains: parâmetros centralizados (nada hardcoded
      dentro das funções de execução).
    - zeroing_suave / rodar_experimento: igual ao
      original, mas usando as peças acima.
"""

import os
import sys
import time
import csv
from dataclasses import dataclass, field
from datetime import datetime

KINESIS_PATH = r"C:\Program Files\Thorlabs\Kinesis" # Verificar aqui antonio o caminho correto do Kinesis, pois pode variar de acordo com a versão instalada
if hasattr(os, "add_dll_directory"):
    os.add_dll_directory(KINESIS_PATH)
sys.path.append(KINESIS_PATH)

import clr  # noqa: E402

clr.AddReference("Thorlabs.MotionControl.DeviceManagerCLI")
clr.AddReference("Thorlabs.MotionControl.GenericPiezoCLI")
clr.AddReference("Thorlabs.MotionControl.KCube.PiezoStrainGaugeCLI")
clr.AddReference("Thorlabs.MotionControl.KCube.PositionAlignerCLI")

from Thorlabs.MotionControl.DeviceManagerCLI import DeviceManagerCLI  # noqa: E402
from Thorlabs.MotionControl.GenericPiezoCLI.Piezo import PiezoControlModeTypes  # noqa: E402
from Thorlabs.MotionControl.KCube.PiezoStrainGaugeCLI import KCubePiezoStrainGauge  # noqa: E402
from Thorlabs.MotionControl.KCube.PositionAlignerCLI import KCubePositionAligner  # noqa: E402
from System import Decimal  # noqa: E402
from System.Globalization import CultureInfo  # noqa: E402
from System.Threading import Thread  # noqa: E402

# OTIMIZAÇÃO 1: setar a cultura da thread para invariant UMA VEZ.
# Evita o `str(...).replace(',', '.')` a cada leitura de posição (era chamado
# 2x por iteração, ~500x/s no loop de aquisição).
Thread.CurrentThread.CurrentCulture = CultureInfo.InvariantCulture


#%% --------------------------------------------------------------- Config

@dataclass
class PIDGains:
    kp: float = 250.0
    ki: float = 70.0
    kd: float = 0.0
    deriv_filter: float = 0.9  # peso do valor filtrado anterior (0-1)


@dataclass
class ExperimentConfig:
    serial_x: str = "113252111"
    serial_y: str = "113251965"
    serial_dq: str = "69254212"

    tensao_min: float = 0.0
    tensao_max: float = 150.0
    tensao_base: float = 0.0
    tensao_repouso: float = 75.0  # tensão de "estacionamento" ao final

    sinal_eixo_x: float = -1.0
    sinal_eixo_y: float = -1.0

    poll_ms: int = 5
    tempo_total_s: float = 80.0
    tempo_loop_aberto_s: float = 20.0

    csv_flush_every: int = 50
    output_dir: str = "dados"

    gains: PIDGains = field(default_factory=PIDGains)

    def caminho_csv(self) -> str:
        os.makedirs(self.output_dir, exist_ok=True)
        carimbo = datetime.now().strftime("%Y%m%d_%H%M%S")
        return os.path.join(
            self.output_dir,
            f"pid_kp{self.gains.kp}_ki{self.gains.ki}_{carimbo}.csv",
        )


#%% --------------------------------------------------------------- PID

class PID:
    """PID discreto, independente de hardware.

    Anti-windup por saturação condicional: só acumula erro na integral
    quando a saída NÃO estiver saturada (evita windup sem back-calculation
    explícito). Derivada passa por filtro passa-baixa simples.
    """

    def __init__(self, kp, ki, kd, out_min, out_max, base=0.0, deriv_filter=0.9):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_min, self.out_max = out_min, out_max
        self.base = base
        self.deriv_filter = deriv_filter
        self.reset()

    def reset(self, erro_inicial: float = 0.0) -> None:
        self.erro_anterior = erro_inicial
        self.erro_acumulado = 0.0
        self.deriv_filtrada = 0.0

    def step(self, erro: float, dt: float) -> float:
        derivada = (erro - self.erro_anterior) / dt if dt > 1e-6 else 0.0
        self.deriv_filtrada = (
            self.deriv_filter * self.deriv_filtrada
            + (1 - self.deriv_filter) * derivada
        )
        self.erro_anterior = erro

        saida_bruta = (
            self.base
            + self.kp * erro
            + self.ki * self.erro_acumulado
            + self.kd * self.deriv_filtrada
        )
        saida = min(max(saida_bruta, self.out_min), self.out_max)

        if saida == saida_bruta:  # não saturou - pode integrar
            self.erro_acumulado += erro * dt

        return saida


#%% --------------------------------------------------------------- Hardware

class PolarisRig:
    """Gerencia conexão/desconexão dos 3 KCubes como um único recurso.

    Uso:
        with PolarisRig(cfg) as rig:
            rig.cubo_x.SetOutputVoltage(...)

    Garante StopPolling + Disconnect mesmo se algo falhar no meio,
    e evita o NameError que o script original podia gerar no `finally`
    caso a conexão de um dos cubos falhasse antes dos outros serem criados.
    """

    def __init__(self, cfg: ExperimentConfig, incluir_dq: bool = True):
        self.cfg = cfg
        self.incluir_dq = incluir_dq
        self.cubo_x = None
        self.cubo_y = None
        self.cubo_dq = None

    def __enter__(self):
        DeviceManagerCLI.BuildDeviceList()
        time.sleep(1.0)

        self.cubo_x = KCubePiezoStrainGauge.CreateKCubePiezoStrainGauge(self.cfg.serial_x)
        self.cubo_y = KCubePiezoStrainGauge.CreateKCubePiezoStrainGauge(self.cfg.serial_y)
        if self.incluir_dq:
            self.cubo_dq = KCubePositionAligner.CreateKCubePositionAligner(self.cfg.serial_dq)

        for cubo in self._dispositivos():
            try:
                if cubo.IsConnected:
                    cubo.Disconnect()
            except Exception:
                pass

        self.cubo_x.Connect(self.cfg.serial_x)
        self.cubo_y.Connect(self.cfg.serial_y)
        if self.incluir_dq:
            self.cubo_dq.Connect(self.cfg.serial_dq)

        for cubo in self._dispositivos():
            cubo.StartPolling(self.cfg.poll_ms)
        time.sleep(0.25)

        for cubo in self._dispositivos():
            cubo.EnableDevice()
        time.sleep(0.25)

        for cubo in self._dispositivos():
            if not cubo.IsSettingsInitialized():
                cubo.WaitForSettingsInitialized(10000)

        max_v = Decimal(self.cfg.tensao_max)
        self.cubo_x.SetMaxOutputVoltage(max_v)
        self.cubo_y.SetMaxOutputVoltage(max_v)
        self.cubo_x.SetPositionControlMode(PiezoControlModeTypes.OpenLoop)
        self.cubo_y.SetPositionControlMode(PiezoControlModeTypes.OpenLoop)

        return self

    def _dispositivos(self):
        return [d for d in (self.cubo_x, self.cubo_y, self.cubo_dq) if d is not None]

    def ler_posicao_dq(self):
        status = self.cubo_dq.Status
        # Com CultureInfo.InvariantCulture já setado no import, não precisamos
        # mais de str(...).replace(',', '.') aqui.
        x = float(status.PositionDifference.X)
        y = float(status.PositionDifference.Y)
        return x, y

    def __exit__(self, exc_type, exc_val, exc_tb):
        print("Liberando hardware com segurança...")
        try:
            if self.cubo_x is not None:
                self.cubo_x.SetOutputVoltage(Decimal(self.cfg.tensao_repouso))
            if self.cubo_y is not None:
                self.cubo_y.SetOutputVoltage(Decimal(self.cfg.tensao_repouso))
            time.sleep(0.5)
        except Exception as e:
            print(f"Aviso: falha ao levar atuadores à tensão de repouso: {e}")

        for cubo in self._dispositivos():
            try:
                cubo.StopPolling()
            except Exception:
                pass
            try:
                cubo.Disconnect()
            except Exception:
                pass

        print("Portas USB liberadas.")
        return False  # não suprime exceções


#%% --------------------------------------------------------------- Zeroing

def zeroing_suave(cfg: ExperimentConfig) -> None:
    """Descarga suave dos atuadores (tensão -> 0V) antes do experimento."""
    print("Conectando aos cubos para o Zeroing...")
    with PolarisRig(cfg, incluir_dq=False) as rig:
        rig.cubo_x.SetPositionControlMode(PiezoControlModeTypes.OpenLoop)
        rig.cubo_y.SetPositionControlMode(PiezoControlModeTypes.OpenLoop)
        time.sleep(0.1)

        print("Zerando os cubos...")
        rig.cubo_x.SetOutputVoltage(Decimal(0.0))
        rig.cubo_y.SetOutputVoltage(Decimal(0.0))
        time.sleep(1.0)

    print("Zeroing concluído.\n")


#%% --------------------------------------------------------------- Loop principal

def rodar_experimento(cfg: ExperimentConfig) -> None:
    g = cfg.gains
    pid_x = PID(g.kp, g.ki, g.kd, cfg.tensao_min, cfg.tensao_max, cfg.tensao_base, g.deriv_filter)
    pid_y = PID(g.kp, g.ki, g.kd, cfg.tensao_min, cfg.tensao_max, cfg.tensao_base, g.deriv_filter)

    caminho_csv = cfg.caminho_csv()
    # OTIMIZAÇÃO 2: buffer em memória, grava em lotes menos chamadas de I/O
    # no loop de alta frequência. Ainda assim faz flush periódico em disco
    # para não perder tudo em caso de queda de energia/erro.
    buffer_linhas = []

    with PolarisRig(cfg, incluir_dq=True) as rig, \
         open(caminho_csv, "w", newline="") as f_csv:

        escritor_csv = csv.writer(f_csv)
        escritor_csv.writerow(["tempo_s", "modo", "x_pos", "y_pos", "v_x", "v_y"])

        rig.cubo_x.SetOutputVoltage(Decimal(cfg.tensao_base))
        rig.cubo_y.SetOutputVoltage(Decimal(cfg.tensao_base))

        print("Iniciando aquisição em LOOP ABERTO...")
        print(f"O controle PID assumirá após {cfg.tempo_loop_aberto_s:.0f} s.")

        malha_fechada_ativada = False
        contador = 0

        # OTIMIZAÇÃO 3: usar perf_counter (monotônico, alta resolução) em vez
        # de time.time() para medir intervalos evita saltos por ajuste de
        # relógio do sistema e é mais preciso para dt pequenos.
        t_inicio = time.perf_counter()
        t_anterior = t_inicio

        try:
            while True:
                agora = time.perf_counter()
                tempo_decorrido = agora - t_inicio
                if tempo_decorrido >= cfg.tempo_total_s:
                    break

                dt = agora - t_anterior
                t_anterior = agora

                x_pos, y_pos = rig.ler_posicao_dq()

                if not malha_fechada_ativada and tempo_decorrido >= cfg.tempo_loop_aberto_s:
                    print("\n>>> TEMPO ATINGIDO: ATIVANDO MALHA FECHADA! <<<")
                    malha_fechada_ativada = True
                    pid_x.reset(erro_inicial=(0.0 - x_pos) * cfg.sinal_eixo_x)
                    pid_y.reset(erro_inicial=(0.0 - y_pos) * cfg.sinal_eixo_y)

                v_x = cfg.tensao_base
                v_y = cfg.tensao_base

                if malha_fechada_ativada:
                    erro_x = (0.0 - x_pos) * cfg.sinal_eixo_x
                    erro_y = (0.0 - y_pos) * cfg.sinal_eixo_y

                    v_x = round(pid_x.step(erro_x, dt), 2)
                    v_y = round(pid_y.step(erro_y, dt), 2)

                    rig.cubo_x.SetOutputVoltage(Decimal(v_x))
                    rig.cubo_y.SetOutputVoltage(Decimal(v_y))

                buffer_linhas.append((
                    f"{tempo_decorrido:.3f}",
                    "Fechado" if malha_fechada_ativada else "Aberto",
                    x_pos, y_pos, v_x, v_y,
                ))
                contador += 1

                if contador % cfg.csv_flush_every == 0:
                    escritor_csv.writerows(buffer_linhas)
                    buffer_linhas.clear()
                    f_csv.flush()

                # Sem sleep artificial: o loop já é limitado pela latência
                # real de comunicação USB com o DeviceManager. Se necessário
                # limitar a taxa, prefira comparar contra um período alvo
                # (agora - t_anterior) em vez de um sleep fixo, para não
                # somar atraso ao tempo já gasto no corpo do loop.

        except KeyboardInterrupt:
            print("\nControle interrompido pelo usuário.")

        finally:
            if buffer_linhas:
                escritor_csv.writerows(buffer_linhas)
                f_csv.flush()

            tempo_total_decorrido = time.perf_counter() - t_inicio
            freq = contador / tempo_total_decorrido if tempo_total_decorrido > 0 else 0.0
            print(f"\nEnsaio finalizado. Gravadas {contador} amostras.")
            print(f"Frequência de aquisição medida: {freq:.2f} Hz")
            print(f"Dados salvos em: {caminho_csv}")


#%% --------------------------------------------------------------- Main

if __name__ == "__main__":
    cfg = ExperimentConfig(
        tempo_total_s=80.0,
        tempo_loop_aberto_s=20.0,
        gains=PIDGains(kp=250.0, ki=70.0, kd=0.0),
    )

    zeroing_suave(cfg)
    time.sleep(2.0)
    rodar_experimento(cfg)

# %%
