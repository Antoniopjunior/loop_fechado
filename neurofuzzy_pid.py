"""
PID neuro-fuzzy (Takagi-Sugeno) com adaptação online — sem dependência de hardware.

Ideia
-----
Em vez de Kp/Ki/Kd constantes, os ganhos são a saída de um sistema fuzzy TSK:

    entradas : erro e(t) e derivada filtrada de(t)  (normalizadas)
    regras   : 3 funções de pertinência gaussianas por entrada (N, Z, P)
               -> 3 x 3 = 9 regras
    saída    : Kp, Ki, Kd = soma ponderada (pesos normalizados) dos
               consequentes de cada regra

A parte "neural" é o aprendizado online dos consequentes (Kp_r, Ki_r, Kd_r)
por descida de gradiente, como em um ANFIS: o custo é o erro entre a resposta
real e um MODELO DE REFERÊNCIA de 1ª ordem (constante de tempo `tau_ref`, que
é a velocidade de convergência que você QUER). Assim os ganhos:
    - sobem se a malha está mais lenta que a referência;
    - descem se está mais rápida/oscilando mais que a referência;
    - estabilizam quando a malha acompanha a referência (não divergem).

Segurança (importante para piezo em hardware real):
    - cada consequente fica limitado a [gain_min_factor, gain_max_factor] x nominal;
    - "leak" puxa lentamente os ganhos de volta ao nominal;
    - não aprende quando a saída está saturada (anti-windup preservado);
    - zona morta: não aprende com erro menor que o ruído de medição;
    - `aprender=False` congela os ganhos (vira um PID escalonado fixo).

Interface idêntica à classe PID de polaris_pid_control.py:
    ctrl.reset(erro_inicial); u = ctrl.step(erro, dt)
Convenção de sinal: erro positivo => aumentar a tensão corrige o erro se
`jac_sign=+1`; no seu setup (erro = -(pos)*sinal_eixo, sinal=-1, K<0) o
aumento de tensão REDUZ o erro quando erro>0 com a convenção já usada no
script, então o padrão é jac_sign=-1 (d erro / d u < 0 no sentido de que
mais tensão diminui o erro). Se os ganhos "fugirem" para o limite máximo
logo de cara, confira esse sinal.
"""

import math


def _gauss(x: float, c: float, s: float) -> float:
    return math.exp(-0.5 * ((x - c) / s) ** 2)


class NeuroFuzzyPID:
    CENTROS = (-1.0, 0.0, 1.0)  # N, Z, P (em unidades normalizadas)
    SIGMA = 0.7

    def __init__(
        self,
        kp0: float,
        ki0: float,
        kd0: float,
        out_min: float,
        out_max: float,
        base: float = 0.0,
        deriv_filter: float = 0.9,
        # escalas de normalização das entradas fuzzy
        e_scale: float = 0.3,      # erro "grande" típico (mesma unidade da posição)
        de_scale: float = 0.05,    # derivada "grande" típica (unid/s)
        # modelo de referência (velocidade de convergência desejada)
        tau_ref: float = 8.0,      # s
        # aprendizado
        lr: float = 0.5,   # conservador; ~2 adapta mais forte (ver simular_neurofuzzy.py)
        leak: float = 0.01,        # 1/s: retorno lento ao nominal
        deadband: float = 0.01,    # não aprende com |erro| < deadband
        gain_min_factor: float = 0.25,
        gain_max_factor: float = 4.0,
        kd_max: float = 0.0,       # kd só aprende se kd_max > 0
        jac_sign: float = -1.0,
        aprender: bool = True,
    ):
        self.kp0, self.ki0, self.kd0 = kp0, ki0, kd0
        self.out_min, self.out_max = out_min, out_max
        self.base = base
        self.deriv_filter = deriv_filter
        self.e_scale, self.de_scale = e_scale, de_scale
        self.tau_ref = tau_ref
        self.lr, self.leak, self.deadband = lr, leak, deadband
        self.gmin, self.gmax = gain_min_factor, gain_max_factor
        self.kd_max = kd_max
        self.jac_sign = jac_sign
        self.aprender = aprender

        self.n_regras = len(self.CENTROS) ** 2
        self._init_consequentes()
        self.reset()

    def _init_consequentes(self):
        self.kp_r = [self.kp0] * self.n_regras
        self.ki_r = [self.ki0] * self.n_regras
        self.kd_r = [self.kd0] * self.n_regras

    def reset(self, erro_inicial: float = 0.0) -> None:
        """Reinicia estados do controlador. NÃO apaga o que foi aprendido
        (use reset_aprendizado() para voltar ao nominal)."""
        self.erro_anterior = erro_inicial
        self.erro_acumulado = 0.0
        self.deriv_filtrada = 0.0
        self.x_ref = erro_inicial  # estado do modelo de referência
        self.kp_atual, self.ki_atual, self.kd_atual = self.kp0, self.ki0, self.kd0

    def reset_aprendizado(self) -> None:
        self._init_consequentes()

    # ------------------------------------------------------------------ fuzzy
    def _pesos(self, erro: float, deriv: float):
        en = erro / self.e_scale
        dn = deriv / self.de_scale
        mu_e = [_gauss(en, c, self.SIGMA) for c in self.CENTROS]
        mu_d = [_gauss(dn, c, self.SIGMA) for c in self.CENTROS]
        w = [a * b for a in mu_e for b in mu_d]
        s = sum(w)
        if s < 1e-12:
            return [1.0 / self.n_regras] * self.n_regras
        return [x / s for x in w]

    # ------------------------------------------------------------------- step
    def step(self, erro: float, dt: float) -> float:
        derivada = (erro - self.erro_anterior) / dt if dt > 1e-6 else 0.0
        self.deriv_filtrada = (
            self.deriv_filter * self.deriv_filtrada
            + (1 - self.deriv_filter) * derivada
        )
        self.erro_anterior = erro

        wn = self._pesos(erro, self.deriv_filtrada)
        kp = sum(w * k for w, k in zip(wn, self.kp_r))
        ki = sum(w * k for w, k in zip(wn, self.ki_r))
        kd = sum(w * k for w, k in zip(wn, self.kd_r))
        self.kp_atual, self.ki_atual, self.kd_atual = kp, ki, kd

        saida_bruta = (
            self.base + kp * erro + ki * self.erro_acumulado + kd * self.deriv_filtrada
        )
        saida = min(max(saida_bruta, self.out_min), self.out_max)
        saturou = saida != saida_bruta

        # modelo de referência: erro desejado decai exponencialmente a zero
        self.x_ref += (0.0 - self.x_ref) / self.tau_ref * dt
        e_m = erro - self.x_ref  # erro de rastreamento (real - referência)

        if self.aprender and not saturou and abs(erro) > self.deadband:
            self._aprender(wn, erro, e_m, dt)

        if not saturou:
            self.erro_acumulado += erro * dt

        return saida

    # --------------------------------------------------------------- aprender
    def _aprender(self, wn, erro, e_m, dt):
        """Gradiente de J = 0.5*e_m^2 em relação aos consequentes.

        dJ/dθ = e_m * (d erro/d u) * (du/dθ), usando só o SINAL de d erro/d u
        (jac_sign), prática padrão em neuro-PID quando o ganho da planta não é
        conhecido com precisão. du/dKp_r = wn_r*e, du/dKi_r = wn_r*I,
        du/dKd_r = wn_r*de. Tudo normalizado pelas escalas para o passo de
        aprendizado ser adimensional.
        """
        e_n = erro / self.e_scale
        em_n = e_m / self.e_scale
        i_n = self.erro_acumulado / (self.e_scale * self.tau_ref)
        d_n = self.deriv_filtrada / self.de_scale

        for r in range(self.n_regras):
            w = wn[r]
            # Kp
            g = em_n * self.jac_sign * w * e_n
            self.kp_r[r] -= self.lr * self.kp0 * g * dt
            # Ki
            g = em_n * self.jac_sign * w * i_n
            self.ki_r[r] -= self.lr * self.ki0 * g * dt
            # Kd (opcional)
            if self.kd_max > 0:
                g = em_n * self.jac_sign * w * d_n
                self.kd_r[r] -= self.lr * self.kd_max * g * dt

            # leak para o nominal + limites
            self.kp_r[r] += self.leak * (self.kp0 - self.kp_r[r]) * dt
            self.ki_r[r] += self.leak * (self.ki0 - self.ki_r[r]) * dt
            self.kp_r[r] = min(max(self.kp_r[r], self.gmin * self.kp0), self.gmax * self.kp0)
            self.ki_r[r] = min(max(self.ki_r[r], self.gmin * self.ki0), self.gmax * self.ki0)
            if self.kd_max > 0:
                self.kd_r[r] = min(max(self.kd_r[r], 0.0), self.kd_max)
