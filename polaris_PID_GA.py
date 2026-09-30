#%%
import os
import sys
import time
import csv
import numpy as np
import random
 
kinesis_path = r"C:\Program Files\Thorlabs\Kinesis"
if hasattr(os, 'add_dll_directory'):
    os.add_dll_directory(kinesis_path)
sys.path.append(kinesis_path)
 
import clr
clr.AddReference("Thorlabs.MotionControl.DeviceManagerCLI")
clr.AddReference("Thorlabs.MotionControl.GenericPiezoCLI")
clr.AddReference("Thorlabs.MotionControl.KCube.PiezoStrainGaugeCLI")
clr.AddReference("Thorlabs.MotionControl.KCube.PositionAlignerCLI")
 
from Thorlabs.MotionControl.DeviceManagerCLI import DeviceManagerCLI
from Thorlabs.MotionControl.GenericPiezoCLI.Piezo import PiezoControlModeTypes
from Thorlabs.MotionControl.KCube.PiezoStrainGaugeCLI import KCubePiezoStrainGauge
from Thorlabs.MotionControl.KCube.PositionAlignerCLI import KCubePositionAligner
from System import Decimal
 
#%%
# --- 1. FUNÇÃO DE ZEROING MANTIDA INTACTA ---
def zeroing_suave(serial_x, serial_y, step_volts=10.0, delay_s=0.05):
    DeviceManagerCLI.BuildDeviceList()
    time.sleep(1.0)
 
    print("Conectando aos cubos para o Zeroing...")
    cubo_x = KCubePiezoStrainGauge.CreateKCubePiezoStrainGauge(serial_x)
    cubo_y = KCubePiezoStrainGauge.CreateKCubePiezoStrainGauge(serial_y)
 
    try:
        if cubo_x.IsConnected:
            cubo_x.Disconnect()
        if cubo_y.IsConnected:
            cubo_y.Disconnect()
    except:
        pass
 
    DeviceManagerCLI.BuildDeviceList()
    time.sleep(2)
 
    lista = DeviceManagerCLI.GetDeviceList()
    print("Dispositivos encontrados:")
    for d in lista:
        print(d)
 
    cubo_x.Connect(serial_x)
    cubo_y.Connect(serial_y)
 
    cubo_x.StartPolling(10)
    cubo_y.StartPolling(10)
    time.sleep(0.1)
 
    cubo_x.EnableDevice()
    cubo_y.EnableDevice()
    time.sleep(0.1)
 
    if not cubo_x.IsSettingsInitialized():
        cubo_x.WaitForSettingsInitialized(10000)
 
    if not cubo_y.IsSettingsInitialized():
        cubo_y.WaitForSettingsInitialized(10000)
 
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
 
# --- 2. FUNÇÃO DE AVALIAÇÃO (O ANTIGO LOOP WHILE DO CÓDIGO 1) ---
def avaliar_pid(cubo_x, cubo_y, cubo_dq, kp, ki, kd, geracao, individuo, tempo_teste=3.0):
    historico_tempo = []
    erros_x = []
    erros_y = []
 
    tensao_base = 75.0
    cubo_x.SetOutputVoltage(Decimal(tensao_base))
    cubo_y.SetOutputVoltage(Decimal(tensao_base))
    time.sleep(0.5)
 
    tensao_alvo_x = tensao_base
    tensao_alvo_y = tensao_base
    sinal_eixo_x = -1.0
    sinal_eixo_y = -1.0
 
    deriv_filtrada_x = 0.0
    deriv_filtrada_y = 0.0
    erro_anterior_x = 0.0
    erro_anterior_y = 0.0
    erro_acumulado_x = 0.0
    erro_acumulado_y = 0.0
 
    tempo_inicio = time.time()
    ultimo_tempo = tempo_inicio
 
    while (time.time() - tempo_inicio) < tempo_teste:
        tempo_decorrido = time.time() - tempo_inicio
        dt = time.time() - ultimo_tempo
        ultimo_tempo = time.time()
 
        status_dq = cubo_dq.Status
        x_pos = float(str(status_dq.PositionDifference.X).replace(',', '.'))
        y_pos = float(str(status_dq.PositionDifference.Y).replace(',', '.'))
 
        erro_x = (0.0 - x_pos) * sinal_eixo_x
        erro_y = (0.0 - y_pos) * sinal_eixo_y
 
        derivada_x = ((erro_x - erro_anterior_x) / dt) if dt > 1e-6 else 0.0
        derivada_y = ((erro_y - erro_anterior_y) / dt) if dt > 1e-6 else 0.0
 
        deriv_filtrada_x = 0.3 * deriv_filtrada_x + 0.1 * derivada_x
        deriv_filtrada_y = 0.3 * deriv_filtrada_y + 0.1 * derivada_y
 
        erro_anterior_x = erro_x
        erro_anterior_y = erro_y
 
        calculo_x = tensao_base + kp * erro_x + ki * erro_acumulado_x + kd * deriv_filtrada_x
        calculo_y = tensao_base + kp * erro_y + ki * erro_acumulado_y + kd * deriv_filtrada_y
 
        if calculo_x > 150.0: tensao_alvo_x = 150.0
        elif calculo_x < 0.0: tensao_alvo_x = 0.0
        else:
            tensao_alvo_x = calculo_x
            erro_acumulado_x += erro_x * dt
 
        if calculo_y > 150.0: tensao_alvo_y = 150.0
        elif calculo_y < 0.0: tensao_alvo_y = 0.0
        else:
            tensao_alvo_y = calculo_y
            erro_acumulado_y += erro_y * dt
 
        cubo_x.SetOutputVoltage(Decimal(round(tensao_alvo_x, 2)))
        cubo_y.SetOutputVoltage(Decimal(round(tensao_alvo_y, 2)))
 
        historico_tempo.append(tempo_decorrido)
        erros_x.append(x_pos)
        erros_y.append(y_pos)
 
        time.sleep(0.002)
 
    rms_x = np.sqrt(np.mean(np.square(erros_x)))
    rms_y = np.sqrt(np.mean(np.square(erros_y)))
    score_final = rms_x + rms_y
 
    pasta_saida = "dia_19_08/otimizacao_ga"
    os.makedirs(pasta_saida, exist_ok=True)
    nome_arquivo = f"{pasta_saida}/G{geracao}_I{individuo}_kp{kp:.1f}_ki{ki:.1f}_kd{kd:.1f}.csv"
 
    with open(nome_arquivo, 'w', newline='') as f_csv:
        escritor_csv = csv.writer(f_csv)
        escritor_csv.writerow(['tempo_s', 'x_pos', 'y_pos'])
        for t, x, y in zip(historico_tempo, erros_x, erros_y):
            escritor_csv.writerow([f"{t:.3f}", x, y])
 
    return score_final

 
 
# --- 3. MOTOR GENÉTICO (MATEMÁTICA) ---
def criar_populacao_global(tamanho, limites_kp, limites_ki, limites_kd):
    populacao = []
    for _ in range(tamanho):
        ind_kp = random.uniform(limites_kp[0], limites_kp[1])
        ind_ki = random.uniform(limites_ki[0], limites_ki[1])
        ind_kd = random.uniform(limites_kd[0], limites_kd[1])
        populacao.append((ind_kp, ind_ki, ind_kd))
    return populacao
 
def criar_populacao_local(tamanho, melhor_pai, taxa_variacao=0.1):
    populacao = [melhor_pai]
    for _ in range(tamanho - 1):
        ind_kp = max(0, random.gauss(melhor_pai[0], melhor_pai[0] * taxa_variacao))
        ind_ki = max(0, random.gauss(melhor_pai[1], melhor_pai[1] * taxa_variacao))
        ind_kd = max(0, random.gauss(melhor_pai[2], melhor_pai[2] * taxa_variacao))
        populacao.append((ind_kp, ind_ki, ind_kd))
    return populacao
 
def executar_ga(cubo_x, cubo_y, cubo_dq, geracoes_globais=3, geracoes_locais=3, tamanho_pop=10):
    print("\n" + "="*40)
    print(" INICIANDO FASE 1: BUSCA GLOBAL BRUTA ")
    print("="*40)
 
    limites_kp = (55.0, 120)
    limites_ki = (70.0, 90.0)
    limites_kd = (0.0, 3.0)
 
    populacao = criar_populacao_global(tamanho_pop, limites_kp, limites_ki, limites_kd)
    melhor_global = None
 
    for geracao in range(geracoes_globais):
        print(f"\n--- Geração Global {geracao+1}/{geracoes_globais} ---")
        scores = []
        for i, (kp, ki, kd) in enumerate(populacao):
            score = avaliar_pid(cubo_x, cubo_y, cubo_dq, kp, ki, kd, f"G{geracao+1}", i, tempo_teste=3.0)
            scores.append((score, (kp, ki, kd)))
            print(f"Ind. {i}: Kp={kp:.1f}, Ki={ki:.1f}, Kd={kd:.1f} -> RMS: {score:.5f}")
 
        scores.sort(key=lambda x: x[0])
        melhor_score, melhor_pid = scores[0]
        melhor_global = melhor_pid
        print(f"Vencedor da geração global {geracao+1}: {melhor_pid} (Erro: {melhor_score:.5f})")
 
        nova_populacao = [scores[0][1], scores[1][1]]
        while len(nova_populacao) < tamanho_pop:
            pai1, pai2 = random.choice(scores[:5])[1], random.choice(scores[:5])[1]
            peso = random.random()
            filho = [
                pai1[0]*peso + pai2[0]*(1-peso),
                pai1[1]*peso + pai2[1]*(1-peso),
                pai1[2]*peso + pai2[2]*(1-peso)
            ]
            if random.random() < 0.3:
                idx = random.randint(0, 2)
                filho[idx] = max(0, filho[idx] * random.uniform(0.2, 3.0))
            nova_populacao.append(tuple(filho))
        populacao = nova_populacao
 
    print("\n" + "="*40)
    print(f" INICIANDO FASE 2: SINTONIA FINA LOCAL ")
    print(f" Centrado em: Kp={melhor_global[0]:.1f}, Ki={melhor_global[1]:.1f}, Kd={melhor_global[2]:.1f}")
    print("="*40)
 
    populacao = criar_populacao_local(tamanho_pop, melhor_global, taxa_variacao=0.1)
 
    for geracao in range(geracoes_locais):
        print(f"\n--- Geração Local {geracao+1}/{geracoes_locais} ---")
        scores = []
        for i, (kp, ki, kd) in enumerate(populacao):
            score = avaliar_pid(cubo_x, cubo_y, cubo_dq, kp, ki, kd, f"L{geracao+1}", i, tempo_teste=3.0)
            scores.append((score, (kp, ki, kd)))
            print(f"Ind. {i}: Kp={kp:.2f}, Ki={ki:.2f}, Kd={kd:.2f} -> RMS: {score:.5f}")
 
        scores.sort(key=lambda x: x[0])
        melhor_score, melhor_pid = scores[0]
        print(f"Vencedor da geração local {geracao+1}: {melhor_pid} (Erro: {melhor_score:.5f})")
 
        nova_populacao = [scores[0][1], scores[1][1]]
        while len(nova_populacao) < tamanho_pop:
            pai1, pai2 = random.choice(scores[:4])[1], random.choice(scores[:4])[1]
            filho = [
                (pai1[0] + pai2[0]) / 2.0,
                (pai1[1] + pai2[1]) / 2.0,
                (pai1[2] + pai2[2]) / 2.0
            ]
            if random.random() < 0.1:
                idx = random.randint(0, 2)
                filho[idx] = max(0, filho[idx] * random.uniform(0.9, 1.1))
            nova_populacao.append(tuple(filho))
        populacao = nova_populacao
 
    print("\nOtimização Concluída!")
    return populacao[0]
 
# --- 4. A SUA FUNÇÃO DE HARDWARE INTACTA DO CÓDIGO 1 ---
def iniciar_hardware_e_otimizar(serial_x, serial_y, serial_dq):
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
 
    cubo_x.StartPolling(5)
    cubo_y.StartPolling(5)
    cubo_dq.StartPolling(5)
 
    # ESTE BLOCO ESTAVA NO SEU CÓDIGO 1 E PROVAVELMENTE "DESTAVA" O HARDWARE
    for v in [10, 30, 50, 70, 90, 120, 150]:
        cubo_x.SetOutputVoltage(Decimal(v))
        time.sleep(0.5)
        real = float(str(cubo_x.GetOutputVoltage()).replace(",", "."))
        print(v, "->", real)
 
    cubo_x.StartPolling(5)
    cubo_y.StartPolling(5)
    time.sleep(0.25)
 
    cubo_x.EnableDevice()
    cubo_y.EnableDevice()
    cubo_dq.EnableDevice()
    time.sleep(0.25)
 
    try:
        if not cubo_x.IsSettingsInitialized(): cubo_x.WaitForSettingsInitialized(10000)
        if not cubo_y.IsSettingsInitialized(): cubo_y.WaitForSettingsInitialized(10000)
        if not cubo_dq.IsSettingsInitialized(): cubo_dq.WaitForSettingsInitialized(10000)
 
        print("Dispositivos prontos! Configurando travas de segurança...")
 
        max_voltage = Decimal(150.0)
        cubo_x.SetMaxOutputVoltage(max_voltage)
        cubo_y.SetMaxOutputVoltage(max_voltage)
 
        cubo_x.SetPositionControlMode(PiezoControlModeTypes.OpenLoop)
        cubo_y.SetPositionControlMode(PiezoControlModeTypes.OpenLoop)
 
        # AQUI CHAMAMOS O MOTOR GENÉTICO EM VEZ DO LOOP INFINITO
        melhor_pid = executar_ga(cubo_x, cubo_y, cubo_dq, geracoes_globais=3, geracoes_locais=3, tamanho_pop=10)
        print(f"\nPID Vencedor Final: Kp={melhor_pid[0]:.2f}, Ki={melhor_pid[1]:.2f}, Kd={melhor_pid[2]:.2f}")
 
    except KeyboardInterrupt:
        print("\nControle interrompido pelo usuário.")
    except Exception as e:
        print(f"\nERRO: {e}")
    finally:
        print("Liberando hardware com segurança...")
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
 
if __name__ == "__main__":
    zeroing_suave("113252111", "113251965")
    time.sleep(2.0)
    iniciar_hardware_e_otimizar("113252111", "113251965", "69254212")
#%%
 