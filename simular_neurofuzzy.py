"""
Simulação comparativa: PID fixo vs PID neuro-fuzzy, sem hardware.

Planta: 1ª ordem  tau * dpos/dt = baseline + K*v - pos   (K < 0, como medido)
Cenários (o ganho real da planta difere do usado para sintonizar o nominal):
    A) K real = K nominal
    B) K real = 0,6 x nominal  (planta "mais fraca" -> PID fixo fica lento)
    C) K muda durante o ensaio (x0,6 em t=60 s, x1,6 em t=120 s)
Ruído de medição gaussiano no sensor.

Uso: python simular_neurofuzzy.py   (gera sim_neurofuzzy.png e imprime métricas)
"""

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from neurofuzzy_pid import NeuroFuzzyPID

# PID idêntico ao polaris_pid_control.PID (copiado para rodar sem Kinesis/clr)
class PID:
    def __init__(self, kp, ki, kd, out_min, out_max, base=0.0, deriv_filter=0.9):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_min, self.out_max, self.base = out_min, out_max, base
        self.deriv_filter = deriv_filter
        self.reset()

    def reset(self, erro_inicial=0.0):
        self.erro_anterior = erro_inicial
        self.erro_acumulado = 0.0
        self.deriv_filtrada = 0.0

    def step(self, erro, dt):
        d = (erro - self.erro_anterior) / dt if dt > 1e-6 else 0.0
        self.deriv_filtrada = self.deriv_filter * self.deriv_filtrada + (1 - self.deriv_filter) * d
        self.erro_anterior = erro
        bruta = self.base + self.kp * erro + self.ki * self.erro_acumulado + self.kd * self.deriv_filtrada
        saida = min(max(bruta, self.out_min), self.out_max)
        if saida == bruta:
            self.erro_acumulado += erro * dt
        return saida


K_NOM, TAU, BASELINE = -0.0025, 20.0, 0.20
KP0, KI0 = 250.0, 70.0
V_MIN, V_MAX = 0.0, 150.0
DT = 0.02
T_TOTAL = 180.0
T_ABERTO = 20.0
RUIDO = 0.012


def k_real_cenario(cenario, t):
    if cenario == "A":
        return K_NOM
    if cenario == "B":
        return 0.6 * K_NOM
    if cenario == "C":
        if t < 60:
            return K_NOM
        if t < 120:
            return 0.6 * K_NOM
        return 1.6 * K_NOM
    raise ValueError(cenario)


def simular(ctrl, cenario, seed=0):
    rng = np.random.default_rng(seed)
    n = int(T_TOTAL / DT)
    pos = BASELINE
    v = 0.0
    fechada = False
    T, POS, V, KP, KI = [], [], [], [], []
    for i in range(n):
        t = i * DT
        pos_med = pos + rng.normal(0, RUIDO)
        if not fechada and t >= T_ABERTO:
            fechada = True
            ctrl.reset(erro_inicial=pos_med)
        if fechada:
            v = ctrl.step(pos_med, DT)  # erro = pos (alvo 0), como no script
        else:
            v = 0.0
        K = k_real_cenario(cenario, t)
        pos += (BASELINE + K * v - pos) / TAU * DT
        T.append(t); POS.append(pos); V.append(v)
        KP.append(getattr(ctrl, "kp_atual", getattr(ctrl, "kp", 0)))
        KI.append(getattr(ctrl, "ki_atual", getattr(ctrl, "ki", 0)))
    return np.array(T), np.array(POS), np.array(V), np.array(KP), np.array(KI)


def metricas(T, POS):
    m = T >= T_ABERTO
    t, e = T[m] - T_ABERTO, np.abs(POS[m])
    iae = np.trapezoid(e, t)
    itae = np.trapezoid(t * e, t)
    return iae, itae, e[-int(10 / DT):].mean()


if __name__ == "__main__":
    cenarios = ["A", "B", "C"]
    fig, axes = plt.subplots(len(cenarios), 3, figsize=(15, 8), sharex=True)
    print(f"{'cenário':8} {'controlador':14} {'IAE':>8} {'ITAE':>10} {'|e| final':>10}")
    for r, c in enumerate(cenarios):
        fixo = PID(KP0, KI0, 0.0, V_MIN, V_MAX)
        nf = NeuroFuzzyPID(KP0, KI0, 0.0, V_MIN, V_MAX)
        res = {}
        for nome, ctrl in (("PID fixo", fixo), ("neuro-fuzzy", nf)):
            T, POS, V, KP, KI = simular(ctrl, c)
            res[nome] = (T, POS, V, KP, KI)
            iae, itae, ef = metricas(T, POS)
            print(f"{c:8} {nome:14} {iae:8.2f} {itae:10.1f} {ef:10.4f}")
        for nome, (T, POS, V, KP, KI) in res.items():
            axes[r, 0].plot(T, POS, label=nome, lw=1)
            axes[r, 1].plot(T, V, label=nome, lw=1)
            axes[r, 2].plot(T, KP, label=f"Kp {nome}", lw=1)
        axes[r, 0].set_ylabel(f"cenário {c}\nposição")
        axes[r, 1].set_ylabel("tensão (V)")
        axes[r, 2].set_ylabel("Kp efetivo")
        axes[r, 0].axvline(T_ABERTO, color="gray", ls=":")
        axes[r, 0].legend(fontsize=7)
    for ax in axes[-1]:
        ax.set_xlabel("tempo (s)")
    plt.tight_layout()
    plt.savefig("sim_neurofuzzy.png", dpi=130)
    print("\nGráfico salvo em sim_neurofuzzy.png")
