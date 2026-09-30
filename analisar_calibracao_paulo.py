"""
Análise offline da calibração --- não depende de Kinesis/clr, roda em
qualquer máquina com pandas/scipy/matplotlib.

Uso:
    python analisar_calibracao.py caminho/para/calibracao_YYYYMMDD_HHMMSS.csv

Para cada degrau de tensão, ajusta o modelo de 1ª ordem:
    pos(t) = pos0 + (pos_final - pos0) * (1 - exp(-t / tau))
via mínimos quadrados, e estima:
    K   = (pos_final - pos0) / (tensao_degrau - tensao_anterior)
    tau = constante de tempo ajustada

Ao final, reporta K/tau por eixo e por sentido (subindo vs. descendo, para
flagar histerese), além do ruído de posição medido no primeiro trecho em
tensão constante.
"""

import sys
import numpy as np
import pandas as pd
from scipy.optimize import curve_fit
import matplotlib.pyplot as plt


def modelo_primeira_ordem(t, pos0, pos_final, tau):
    tau = max(tau, 1e-6)
    return pos0 + (pos_final - pos0) * (1 - np.exp(-t / tau))


def ajustar_degrau(t, pos):
    """Ajusta um único segmento de degrau. Retorna (pos0, pos_final, tau, r2)."""
    pos0_guess = pos[0]
    pos_final_guess = pos[-1]
    tau_guess = max(t[-1] / 3.0, 0.05)

    try:
        popt, _ = curve_fit(
            modelo_primeira_ordem, t, pos,
            p0=[pos0_guess, pos_final_guess, tau_guess],
            maxfev=5000,
        )
        pos0, pos_final, tau = popt
        pred = modelo_primeira_ordem(t, *popt)
        ss_res = np.sum((pos - pred) ** 2)
        ss_tot = np.sum((pos - np.mean(pos)) ** 2)
        r2 = 1 - ss_res / ss_tot if ss_tot > 0 else float("nan")
        return pos0, pos_final, abs(tau), r2
    except RuntimeError:
        return pos0_guess, pos_final_guess, float("nan"), float("nan")


def analisar(caminho_csv: str):
    df = pd.read_csv(caminho_csv)

    resultados = []

    for eixo in df["eixo"].unique():
        sub = df[df["eixo"] == eixo].reset_index(drop=True)
        coluna_pos = "x_pos" if eixo == "X" else "y_pos"

        # Detecta blocos de tensão constante (cada degrau)
        tensoes = sub["tensao_aplicada"].values
        mudancas = np.where(np.diff(tensoes) != 0)[0] + 1
        limites = [0] + list(mudancas) + [len(sub)]

        tensao_anterior = None
        for i in range(len(limites) - 1):
            ini, fim = limites[i], limites[i + 1]
            bloco = sub.iloc[ini:fim]
            if len(bloco) < 10:
                continue

            t = bloco["t_no_degrau"].values
            pos = bloco[coluna_pos].values
            tensao_degrau = bloco["tensao_aplicada"].values[0]

            pos0, pos_final, tau, r2 = ajustar_degrau(t, pos)

            if tensao_anterior is not None and tensao_degrau != tensao_anterior:
                delta_v = tensao_degrau - tensao_anterior
                k = (pos_final - pos0) / delta_v if abs(delta_v) > 1e-9 else float("nan")
                sentido = "subindo" if delta_v > 0 else "descendo"
                resultados.append({
                    "eixo": eixo,
                    "tensao_de": tensao_anterior,
                    "tensao_para": tensao_degrau,
                    "sentido": sentido,
                    "K": k,
                    "tau_s": tau,
                    "r2": r2,
                })

            tensao_anterior = tensao_degrau

    res_df = pd.DataFrame(resultados)
    if res_df.empty:
        print("Não foi possível identificar degraus suficientes no CSV.")
        return res_df

    pd.set_option("display.float_format", lambda x: f"{x:.4f}")
    print("\n=== Ajuste por degrau ===")
    print(res_df.to_string(index=False))

    print("\n=== Resumo por eixo e sentido (mediana, robusta a outliers) ===")
    resumo = res_df.groupby(["eixo", "sentido"])[["K", "tau_s", "r2"]].median()
    print(resumo.to_string())

    # Ruído: usa o primeiro degrau de cada eixo (geralmente 0V, mais estável)
    print("\n=== Ruído de posição (desvio padrão, primeiro degrau de cada eixo) ===")
    for eixo in df["eixo"].unique():
        sub = df[df["eixo"] == eixo]
        coluna_pos = "x_pos" if eixo == "X" else "y_pos"
        primeiro_v = sub["tensao_aplicada"].iloc[0]
        primeiro_bloco = sub[sub["tensao_aplicada"] == primeiro_v]
        ruido = primeiro_bloco[coluna_pos].std()
        print(f"  Eixo {eixo} (V={primeiro_v}): desvio padrão = {ruido:.5f}")

    aviso_histerese = res_df.groupby(["eixo", "sentido"])["K"].median()
    for eixo in df["eixo"].unique():
        try:
            k_sobe = aviso_histerese[(eixo, "subindo")]
            k_desce = aviso_histerese[(eixo, "descendo")]
            diff_pct = abs(k_sobe - k_desce) / max(abs(k_sobe), 1e-9) * 100
            if diff_pct > 15:
                print(
                    f"\nAVISO: eixo {eixo} mostra ~{diff_pct:.0f}% de diferença de K "
                    "entre subida e descida — possível histerese relevante. "
                    "O modelo de 1ª ordem simples é uma aproximação nesse caso."
                )
        except KeyError:
            pass

    _plotar(df, res_df)
    return res_df


def _plotar(df, ç):
    fig, axes = plt.subplots(2, 1, figsize=(9, 6), sharex=False)
    for ax, eixo in zip(axes, df["eixo"].unique()):
        sub = df[df["eixo"] == eixo]
        coluna_pos = "x_pos" if eixo == "X" else "y_pos"
        ax.plot(sub["t_abs"], sub[coluna_pos], linewidth=0.8, label=f"posição {eixo}")
        ax2 = ax.twinx()
        ax2.plot(sub["t_abs"], sub["tensao_aplicada"], color="tab:orange", linewidth=0.8, alpha=0.6, label="tensão")
        ax.set_title(f"Eixo {eixo}")
        ax.set_ylabel("posição")
        ax2.set_ylabel("tensão (V)")
    plt.tight_layout()
    saida_png = "calibracao_resultado.png"
    plt.savefig(saida_png, dpi=140)
    print(f"\nGráfico salvo em: {saida_png}")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Uso: python analisar_calibracao.py caminho/para/calibracao.csv")
        sys.exit(1)

    analisar(sys.argv[1])
