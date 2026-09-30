"""
Análise offline da calibração — não depende de Kinesis/clr, roda em
qualquer máquina com pandas/scipy/matplotlib.

Uso:
    python analisar_calibracao.py caminho/para/calibracao_YYYYMMDD_HHMMSS.csv

MÉTODO PRINCIPAL (trajetória completa, robusto):
Ajusta o modelo de 1ª ordem
    dpos/dt = (baseline + K*v(t) - pos) / tau
usando a sequência REAL de tensão aplicada ao longo de todo o ensaio, em vez
de tratar cada degrau isoladamente. Isso é necessário quando o tempo de
espera por degrau é curto demais para o sistema alcançar o regime — nesse
caso, um ajuste por segmento (que assume um degrau isolado que se acomoda)
falha e devolve tau sem sentido físico.

A chave é que, para um tau FIXO, a resposta é LINEAR em (baseline, K) — dá
pra resolver os dois por mínimos quadrados lineares (exato) e fazer só uma
busca 1D em tau (log-espaçada + refino), o que é muito mais estável que
ajustar os 3 parâmetros juntos com um otimizador não-linear genérico.

MÉTODO SECUNDÁRIO (por segmento, mantido para diagnóstico):
Ajusta cada degrau isoladamente como antes. Útil só para comparação e para
detectar visualmente que "cada degrau não estabilizou" quando os dois
métodos discordam muito.
"""

import sys
import numpy as np
import pandas as pd
from scipy.optimize import curve_fit, minimize_scalar
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


def ajustar_trajetoria_completa(t_abs, tensao, pos_medida, decimacao=4,
                                 tau_min=0.1, tau_max=600.0, n_grid=40):
    """Ajusta (K, tau, baseline) usando a série temporal inteira.

    Retorna dict com K, tau, baseline, r2, e as séries (t, pos_medida,
    pos_simulada) já decimadas, para plot.
    """
    t = np.asarray(t_abs, dtype=float)
    v = np.asarray(tensao, dtype=float)
    pos = np.asarray(pos_medida, dtype=float)

    t_d = t[::decimacao] - t[0]
    v_d = v[::decimacao]
    pos_d = pos[::decimacao]
    dt_arr = np.diff(t_d)
    n = len(t_d)
    pos_inicial = pos_d[0]

    def componentes(tau):
        pos_h = np.empty(n)
        pos_h[0] = pos_inicial
        phi1 = np.zeros(n)
        phi2 = np.zeros(n)
        for i in range(1, n):
            dt = dt_arr[i - 1]
            pos_h[i] = pos_h[i - 1] + (-pos_h[i - 1]) / tau * dt
            phi1[i] = phi1[i - 1] + (1 - phi1[i - 1]) / tau * dt
            phi2[i] = phi2[i - 1] + (v_d[i - 1] - phi2[i - 1]) / tau * dt
        return pos_h, phi1, phi2

    def sse_para_log_tau(log_tau):
        tau = 10 ** log_tau
        pos_h, phi1, phi2 = componentes(tau)
        A = np.column_stack([phi1, phi2])
        b = pos_d - pos_h
        coef, *_ = np.linalg.lstsq(A, b, rcond=None)
        resid = b - A @ coef
        return np.sum(resid ** 2)

    grid = np.linspace(np.log10(tau_min), np.log10(tau_max), n_grid)
    sse_grid = [sse_para_log_tau(g) for g in grid]
    melhor_idx = int(np.argmin(sse_grid))
    lo = grid[max(0, melhor_idx - 1)]
    hi = grid[min(len(grid) - 1, melhor_idx + 1)]
    res = minimize_scalar(sse_para_log_tau, bounds=(lo, hi), method="bounded")
    tau = 10 ** res.x

    pos_h, phi1, phi2 = componentes(tau)
    A = np.column_stack([phi1, phi2])
    b = pos_d - pos_h
    coef, *_ = np.linalg.lstsq(A, b, rcond=None)
    baseline, K = coef
    pos_sim = pos_h + A @ coef

    ss_res = np.sum((pos_d - pos_sim) ** 2)
    ss_tot = np.sum((pos_d - pos_d.mean()) ** 2)
    r2 = 1 - ss_res / ss_tot if ss_tot > 0 else float("nan")

    return {
        "K": K, "tau_s": tau, "baseline": baseline, "r2": r2,
        "t": t_d, "pos_medida": pos_d, "pos_simulada": pos_sim,
    }


def analisar(caminho_csv: str):
    df = pd.read_csv(caminho_csv)

    print("MÉTODO PRINCIPAL: ajuste de trajetória completa (use este antonio)")
    resultados_completos = {}
    for eixo in sorted(df["eixo"].unique()):
        sub = df[df["eixo"] == eixo].reset_index(drop=True)
        coluna_pos = "x_pos" if eixo == "X" else "y_pos"
        r = ajustar_trajetoria_completa(
            sub["t_abs"].values, sub["tensao_aplicada"].values, sub[coluna_pos].values
        )
        resultados_completos[eixo] = r
        print(
            f"  Eixo {eixo}: K={r['K']:.5f}  tau={r['tau_s']:.2f} s  "
            f"baseline={r['baseline']:.4f}  R²={r['r2']:.4f}"
        )
        if r["r2"] < 0.7:
            print(
                f"    AVISO: R² baixo para o eixo {eixo} mesmo com este método — "
                "a dinâmica pode não ser bem descrita por um sistema de 1ª ordem "
                "simples (ex.: creep, ruído de processo/deriva, ou o sensor não "
                "está enxergando o atuador de forma limpa nesse ensaio)."
            )

    _plotar_trajetoria_completa(df, resultados_completos)

    print("\nMÉTODO SECUNDÁRIO: ajuste por segmento (só para diagnóstico)")
    print(
        "Se os R² aqui forem muito piores que os do método principal, é sinal de\n"
        "que o tempo de espera por degrau é curto demais para o sistema "
        "estabilizar\ndentro de um único degrau — confie no método principal "
        "acima nesse caso.\n"
    )

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


def _plotar_trajetoria_completa(df, resultados_completos):
    eixos = sorted(df["eixo"].unique())
    fig, axes = plt.subplots(len(eixos), 1, figsize=(9, 3.2 * len(eixos)), sharex=False)
    if len(eixos) == 1:
        axes = [axes]

    for ax, eixo in zip(axes, eixos):
        r = resultados_completos[eixo]
        ax.plot(r["t"], r["pos_medida"], linewidth=0.8, alpha=0.6, label="medido")
        ax.plot(r["t"], r["pos_simulada"], linewidth=1.5, color="tab:red",
                label=f"modelo (K={r['K']:.4f}, τ={r['tau_s']:.1f}s, R²={r['r2']:.3f})")
        ax.set_title(f"Eixo {eixo} — trajetória completa: medido vs. modelo ajustado")
        ax.set_xlabel("tempo (s)")
        ax.set_ylabel("posição")
        ax.legend(fontsize=8, loc="best")

    plt.tight_layout()
    saida_png = "calibracao_trajetoria_completa.png"
    plt.savefig(saida_png, dpi=140)
    print(f"\nGráfico (medido vs. modelo) salvo em: {saida_png}")
    print("Confira visualmente: se a curva vermelha não acompanhar a azul, não")
    print("confie nos números de K/tau — pode ser preciso um modelo mais rico")
    print("(dois tempos de resposta, por exemplo).")


def _plotar(df, res_df):
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
