#%%
"""
Calibração da planta (atuadores X e Y) — malha ABERTA, sem PID.

Aplica uma sequência de degraus de tensão (subindo e descendo, para revelar
histerese) em um eixo por vez, mantendo o outro na tensão base, e grava
tempo/posição em CSV. Use analisar_calibracao.py (sem dependência de
hardware) para ajustar K e tau a partir do arquivo gerado aqui.

Reaproveita PolarisRig/ExperimentConfig do polaris_pid_control.py — mantenha
os dois arquivos na mesma pasta.
"""

import os
import time
import csv
from datetime import datetime

from polaris_pid_control_paulo import PolarisRig, ExperimentConfig
from System import Decimal


def calibrar_eixo(rig, cfg, eixo, degraus, tempo_por_degrau, escritor_csv, f_csv):
    """Aplica a sequência de degraus em um eixo, mantendo o outro na base."""
    set_v = rig.cubo_x.SetOutputVoltage if eixo == "X" else rig.cubo_y.SetOutputVoltage
    outro_cubo = rig.cubo_y if eixo == "X" else rig.cubo_x

    outro_cubo.SetOutputVoltage(Decimal(cfg.tensao_base))

    for tensao in degraus:
        print(f"  [{eixo}] aplicando {tensao:.1f} V por {tempo_por_degrau:.1f} s...")
        set_v(Decimal(tensao))
        t_inicio_degrau = time.perf_counter()

        while True:
            t = time.perf_counter() - t_inicio_degrau
            if t >= tempo_por_degrau:
                break

            x_pos, y_pos = rig.ler_posicao_dq()
            escritor_csv.writerow([
                f"{time.perf_counter():.4f}",
                eixo,
                f"{tensao:.2f}",
                f"{t:.4f}",
                x_pos,
                y_pos,
            ])
            time.sleep(0.005)  # ~200 Hz de amostragem para a calibração

        f_csv.flush()

    # devolve o eixo calibrado à base antes de seguir
    set_v(Decimal(cfg.tensao_base))
    time.sleep(1.0)


def calibrar_planta(
    cfg: ExperimentConfig,
    degraus_x=(0, 20, 40, 60, 80, 100, 120, 150, 100, 60, 20, 0),
    degraus_y=(0, 20, 40, 60, 80, 100, 120, 150, 100, 60, 20, 0),
    tempo_por_degrau: float = 8.0,
) -> str:
    os.makedirs(cfg.output_dir, exist_ok=True)
    carimbo = datetime.now().strftime("%Y%m%d_%H%M%S")
    caminho_csv = os.path.join(cfg.output_dir, f"calibracao_{carimbo}.csv")

    with PolarisRig(cfg, incluir_dq=True) as rig, \
         open(caminho_csv, "w", newline="") as f_csv:

        escritor_csv = csv.writer(f_csv)
        escritor_csv.writerow(["t_abs", "eixo", "tensao_aplicada", "t_no_degrau", "x_pos", "y_pos"])

        print("Calibrando eixo X...")
        calibrar_eixo(rig, cfg, "X", degraus_x, tempo_por_degrau, escritor_csv, f_csv)

        print("Calibrando eixo Y...")
        calibrar_eixo(rig, cfg, "Y", degraus_y, tempo_por_degrau, escritor_csv, f_csv)

    print(f"\nCalibração concluída. Dados em: {caminho_csv}")
    print("Rode analisar_calibracao.py apontando para esse arquivo.")
    return caminho_csv


if __name__ == "__main__":
    cfg = ExperimentConfig()

    # Comece pequeno para checar que está tudo bem antes de ir a tensões altas.
    # Depois de validar, troque para os degraus completos (0 a 150 V).
    degraus_teste = (0, 150, 0)

    calibrar_planta(
        cfg,
        degraus_x=degraus_teste,
        degraus_y=degraus_teste,
        tempo_por_degrau=20.0,
    )

# %%
