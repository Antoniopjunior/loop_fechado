#%%
import os
import sys
import time
import csv
import numpy as np
import random
import threading
 
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
# =============================================================================
# VARIÁVEIS DE ESTADO GLOBAL (Comunicação entre o Loop Principal e o GA)
# =============================================================================
ga_state = {
    'running': True,
    'malha_fechada_ativada': False,
    'kp': 63.62,
    'ki': 75.48,
    'kd': 1.02,
    'request_eval': False,
    'eval_time': 3.0,
    'score': 0.0,
    'eval_done': False
}
 
# =============================================================================
# MOTOR GENÉTICO EM SEGUNDO PLANO (THREAD)
# =============================================================================
def motor_genetico_thread(geracoes_globais=3, geracoes_locais=1, tamanho_pop=10):
    print("\n[GA] Thread de Otimização iniciada. Aguardando ativação da malha fechada...")
    
    # Aguarda os 10s iniciais de malha aberta passarem
    while not ga_state['malha_fechada_ativada']:
        if not ga_state['running']: return
        time.sleep(0.5)
        
    print("\n" + "="*40)
    print(" [GA] INICIANDO FASE 1: BUSCA GLOBAL BRUTA ")
    print("="*40)
    
    limites_kp = (10.0, 250.0)
    limites_ki = (0.0, 90.0)
    limites_kd = (0.0, 5.0)
    
    populacao = []
    for _ in range(tamanho_pop):
        populacao.append((
            random.uniform(*limites_kp),
            random.uniform(*limites_ki),
            random.uniform(*limites_kd)
        ))
        
    melhor_global = None
 
    for geracao in range(geracoes_globais):
        if not ga_state['running']: return
        print(f"\n--- [GA] Geração Global {geracao+1}/{geracoes_globais} ---")
        scores = []
        
        for i, (kp, ki, kd) in enumerate(populacao):
            if not ga_state['running']: return
            
            # Injeta os ganhos no loop principal
            ga_state['kp'] = kp
            ga_state['ki'] = ki
            ga_state['kd'] = kd
            time.sleep(0.5) # Aguarda o feixe estabilizar com o novo ganho
            
            # Solicita uma avaliação de 3s
            ga_state['eval_done'] = False
            ga_state['request_eval'] = True
            
            # Aguarda o loop principal finalizar o teste e calcular o RMS
            while not ga_state['eval_done']:
                if not ga_state['running']: return
                time.sleep(0.01)
                
            score = ga_state['score']
            scores.append((score, (kp, ki, kd)))
            print(f"[GA] Ind. {i}: Kp={kp:.1f}, Ki={ki:.1f}, Kd={kd:.1f} -> RMS: {score:.5f}")
 
        scores.sort(key=lambda x: x[0])
        melhor_global = scores[0][1]
        print(f"[GA] Vencedor Global {geracao+1}: {melhor_global} (Erro: {scores[0][0]:.5f})")
 
        nova_pop = [scores[0][1], scores[1][1]]
        while len(nova_pop) < tamanho_pop:
            p1, p2 = random.choice(scores[:5])[1], random.choice(scores[:5])[1]
            peso = random.random()
            filho = [
                p1[0]*peso + p2[0]*(1-peso),
                p1[1]*peso + p2[1]*(1-peso),
                p1[2]*peso + p2[2]*(1-peso)
            ]
            if random.random() < 0.3:
                idx = random.randint(0, 2)
                filho[idx] = max(0, filho[idx] * random.uniform(0.2, 3.0))
            nova_pop.append(tuple(filho))
        populacao = nova_pop
 
    print("\n" + "="*40)
    print(" [GA] INICIANDO FASE 2: SINTONIA FINA LOCAL ")
    print("="*40)
    
    populacao = [melhor_global]
    for _ in range(tamanho_pop - 1):
        populacao.append((
            max(0, random.gauss(melhor_global[0], melhor_global[0] * 0.1)),
            max(0, random.gauss(melhor_global[1], melhor_global[1] * 0.1)),
            max(0, random.gauss(melhor_global[2], melhor_global[2] * 0.1))
        ))
 
    for geracao in range(geracoes_locais):
        if not ga_state['running']: return
        print(f"\n--- [GA] Geração Local {geracao+1}/{geracoes_locais} ---")
        scores = []
        
        for i, (kp, ki, kd) in enumerate(populacao):
            if not ga_state['running']: return
            
            ga_state['kp'], ga_state['ki'], ga_state['kd'] = kp, ki, kd
            time.sleep(0.5)
            
            ga_state['eval_done'] = False
            ga_state['request_eval'] = True
            
            while not ga_state['eval_done']:
                if not ga_state['running']: return
                time.sleep(0.01)
                
            score = ga_state['score']
            scores.append((score, (kp, ki, kd)))
            print(f"[GA] Ind. {i}: Kp={kp:.2f}, Ki={ki:.2f}, Kd={kd:.2f} -> RMS: {score:.5f}")
 
        scores.sort(key=lambda x: x[0])
        melhor_local = scores[0][1]
        print(f"[GA] Vencedor Local {geracao+1}: {melhor_local} (Erro: {scores[0][0]:.5f})")
 
        nova_pop = [scores[0][1], scores[1][1]]
        while len(nova_pop) < tamanho_pop:
            p1, p2 = random.choice(scores[:4])[1], random.choice(scores[:4])[1]
            filho = [
                (p1[0] + p2[0]) / 2.0,
                (p1[1] + p2[1]) / 2.0,
                (p1[2] + p2[2]) / 2.0
            ]
            if random.random() < 0.1:
                idx = random.randint(0, 2)
                filho[idx] = max(0, filho[idx] * random.uniform(0.9, 1.1))
            nova_pop.append(tuple(filho))
        populacao = nova_pop
 
    ga_state['kp'], ga_state['ki'], ga_state['kd'] = scores[0][1]
    print(f"\n[GA] >>> OTIMIZAÇÃO CONCLUÍDA <<<")
    print(f"[GA] Ganhos travados em: Kp={ga_state['kp']:.2f}, Ki={ga_state['ki']:.2f}, Kd={ga_state['kd']:.2f}")
 
 
# =============================================================================
# CÓDIGO PARA ZEROING
# =============================================================================

def zeroing_suave(serial_x, serial_y, step_volts=10.0, delay_s=0.05):
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
    cubo_x.StartPolling(10)
    cubo_y.StartPolling(10)
    time.sleep(0.1)
    cubo_x.EnableDevice()
    cubo_y.EnableDevice()
    time.sleep(0.1)
 
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
 
 
# =============================================================================
# LOOP PRINCIPAL DE HARDWARE (Execução a 200 Hz)
# =============================================================================
def executar_malha_fechada_dq_software(serial_x, serial_y, serial_dq, tempo_total_segundos, tempo_loop_aberto_segundos=30):
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
 
    f_csv = None
    modo_aberto = None
 
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
 
        #pasta_saida = "dia_09_09"
        #os.makedirs(pasta_saida, exist_ok=True)
        nome_arquivo = f"dia_11_09/GA/teste_loop_GA_3.csv"
 
        f_csv = open(nome_arquivo, 'w', newline='')
        escritor_csv = csv.writer(f_csv)
        escritor_csv.writerow(['tempo_s', 'modo', 'x_pos', 'y_pos'])
 
        tensao_base = 0.0
        cubo_x.SetOutputVoltage(Decimal(tensao_base))
        cubo_y.SetOutputVoltage(Decimal(tensao_base))
 
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
 
        tempo_inicio_total = time.time()
        ultimo_tempo = tempo_inicio_total
 
        contador_leituras = 0
        malha_fechada_ativada = False
 
        # Variáveis de Avaliação do Motor Genético
        is_evaluating = False
        eval_start_time = 0.0
        eval_erros_x = []
        eval_erros_y = []
 
        print("Iniciando aquisição em LOOP ABERTO...")
        print(f"O CONTROLE PID DO PYTHON assumirá após {tempo_loop_aberto_segundos:.0f} segundos.")
 
        # --- INICIA O MOTOR GENÉTICO EM PARALELO ---
        ga_thread = threading.Thread(target=motor_genetico_thread, args=(3, 3, 10))
        ga_thread.start()
 
        while True:
            # Encerra se a thread global pediu para parar
            if not ga_state['running']:
                break
 
            tempo_decorrido = time.time() - tempo_inicio_total
            if tempo_decorrido >= tempo_total_segundos:
                break
 
            dt = time.time() - ultimo_tempo
            ultimo_tempo = time.time()
 
            status_dq = cubo_dq.Status
            x_pos = float(str(status_dq.PositionDifference.X).replace(',', '.'))
            y_pos = float(str(status_dq.PositionDifference.Y).replace(',', '.'))
 
            if tempo_decorrido >= tempo_loop_aberto_segundos and not malha_fechada_ativada:
                print("\n>>> TEMPO ATINGIDO: ATIVANDO MALHA FECHADA PELO PYTHON! <<<")
                malha_fechada_ativada = True
                ga_state['malha_fechada_ativada'] = True # Libera a thread do GA para começar
                
                erro_anterior_x = -x_pos
                erro_anterior_y = -y_pos
                erro_acumulado_x = 0.0
                erro_acumulado_y = 0.0
                deriv_filtrada_x = 0.0
                deriv_filtrada_y = 0.0
 
            modo_log = "Fechado" if malha_fechada_ativada else "Aberto"
 
            if malha_fechada_ativada:
                # OBTÉM OS VALORES DO GA DINAMICAMENTE
                kp = ga_state['kp']
                ki = ga_state['ki']
                kd = ga_state['kd']
 
                erro_x = (0.0 - x_pos)*sinal_eixo_x
                erro_y = (0.0 - y_pos)*sinal_eixo_y
 
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
 
                # ====================================================
                # FEEDBACK PARA A THREAD DO MOTOR GENÉTICO
                # ====================================================
                if ga_state['request_eval'] and not is_evaluating:
                    ga_state['request_eval'] = False
                    is_evaluating = True
                    eval_start_time = time.time()
                    eval_erros_x = []
                    eval_erros_y = []
 
                if is_evaluating:
                    eval_erros_x.append(x_pos)
                    eval_erros_y.append(y_pos)
                    if (time.time() - eval_start_time) >= ga_state['eval_time']:
                        rms_x = np.sqrt(np.mean(np.square(eval_erros_x)))
                        rms_y = np.sqrt(np.mean(np.square(eval_erros_y)))
                        ga_state['score'] = rms_x + rms_y
                        ga_state['eval_done'] = True
                        is_evaluating = False
 
            escritor_csv.writerow([f"{tempo_decorrido:.3f}", modo_log, x_pos, y_pos])
 
            if contador_leituras % 50 == 0:
                f_csv.flush()
 
            contador_leituras += 1
            time.sleep(0.002)
 
        tempo_total_decorrido = time.time() - tempo_inicio_total
        frequencia_aquisicao = (contador_leituras / tempo_total_decorrido) if tempo_total_decorrido > 0 else 0.0
 
        print(f"\nEnsaio finalizado com sucesso! Gravadas {contador_leituras} amostras.")
        print(f"Frequência de aquisição: {frequencia_aquisicao:.2f} Hz")
 
    except KeyboardInterrupt:
        print("\nControle interrompido pelo usuário.")
    except Exception as e:
        print(f"\nERRO: {e}")
    finally:
        # Encerra a thread paralela de forma segura
        ga_state['running'] = False
        try: ga_thread.join(timeout=2.0)
        except: pass
        
        print("Liberando hardware com segurança...")
        if f_csv is not None: f_csv.close()
 
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
    
    # 120s totais de experimento / 10s de loop aberto
    executar_malha_fechada_dq_software("113252111", "113251965", "69254212",
                                       tempo_total_segundos=120.0,
                                       tempo_loop_aberto_segundos=10.0)



#%%