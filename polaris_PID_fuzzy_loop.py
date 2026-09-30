#%%
import os
import sys
import time
import csv
import numpy as np
 
kinesis_path = r"C:\Program Files\Thorlabs\Kinesis"
if hasattr(os, 'add_dll_directory'):
    os.add_dll_directory(kinesis_path)
sys.path.append(kinesis_path)
 
import clr
clr.AddReference("Thorlabs.MotionControl.DeviceManagerCLI")
clr.AddReference("Thorlabs.MotionControl.GenericPiezoCLI")
clr.AddReference("Thorlabs.MotionControl.KCube.PiezoStrainGaugeCLI")
clr.AddReference("Thorlabs.MotionControl.KCube.PositionAlignerCLI")
 
# A importação do System DEVE ficar abaixo do clr
from System import Decimal
from Thorlabs.MotionControl.DeviceManagerCLI import DeviceManagerCLI
from Thorlabs.MotionControl.GenericPiezoCLI.Piezo import PiezoControlModeTypes
from Thorlabs.MotionControl.KCube.PiezoStrainGaugeCLI import KCubePiezoStrainGauge
from Thorlabs.MotionControl.KCube.PositionAlignerCLI import KCubePositionAligner
 
#%%
def calcular_ganhos_fuzzy(erro_x, erro_y, deriv_x, deriv_y, kp_base, ki_base, kd_base):
    # 1. Escala Real do Mundo Físico adaptada para micro-desvios
    LIMITE_ERRO_MAX = 0.01  # Limite ajustado para a magnitude real do seu sensor
    LIMITE_DERIV_MAX = 1.0  
 
    erro_max = max(abs(erro_x), abs(erro_y))
    deriv_max = max(abs(deriv_x), abs(deriv_y))
   
    # 2. Fator de Peso Alpha Contínuo (Evita solavancos nos ganhos)
    razao_erro = erro_max / LIMITE_ERRO_MAX
    alpha = 1.0 + np.clip(razao_erro, 0.0, 1.0) # Varia suavemente de 1.0 a 2.0
   
    # Normalização
    e_norm = np.clip(razao_erro, 0.0, 1.0)
    de_norm = np.clip(deriv_max / LIMITE_DERIV_MAX, 0.0, 1.0)
   
    # --- Regras Baseadas na Matriz Fuzzy ---
    fator_kp = 1.0 + (1.2 * e_norm) - (0.4 * de_norm)
    fator_ki = 1.0 - (0.6 * e_norm)
    fator_kd = 1.0 - (0.5 * e_norm) + (1.5 * de_norm)
   
    # Calcula os novos ganhos com travas de segurança absolutas
    kp_novo = np.clip(kp_base * fator_kp, 10.0, 250.0)
    ki_novo = np.clip(ki_base * fator_ki, 0.0, 100.0)
    kd_novo = np.clip(kd_base * fator_kd, 0.0, 5.0)
   
    return kp_novo, ki_novo, kd_novo
 
# CÓDIGO PARA ZEROING DO HARDWARE

def zeroing_suave(serial_x, serial_y, step_volts=10.0, delay_s=0.05):
    DeviceManagerCLI.BuildDeviceList()
    time.sleep(1.0)
    print("Conectando aos cubos para o Zeroing...")
    cubo_x = KCubePiezoStrainGauge.CreateKCubePiezoStrainGauge(serial_x)
    cubo_y = KCubePiezoStrainGauge.CreateKCubePiezoStrainGauge(serial_y)
    try:
        if cubo_x.IsConnected: cubo_x.Disconnect()
        if cubo_y.IsConnected: cubo_y.Disconnect()
    except: pass
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
    except: pass
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
        # pasta_saida = "dia_16_09/Fuzzy/turbulencia"
        # os.makedirs(pasta_saida, exist_ok=True)
        nome_arquivo = f"dia_21_09/fuzzy/teste_1.csv"
        f_csv = open(nome_arquivo, 'w', newline='')
        escritor_csv = csv.writer(f_csv)
        
        # Novo cabeçalho abrangente
        escritor_csv.writerow(['tempo_s', 'modo', 'diff_x', 'diff_y', 'intensidade', 'x_pos_real', 'y_pos_real', 'tensao_x', 'tensao_y', 'kp_atual', 'ki_atual', 'kd_atual'])
        # CORREÇÃO FÍSICA: O feixe PRECISA partir de 75V para ter curso em ambas as direções
        tensao_base = 75.0
        cubo_x.SetOutputVoltage(Decimal(tensao_base))
        cubo_y.SetOutputVoltage(Decimal(tensao_base))
        tensao_alvo_x = tensao_base
        tensao_alvo_y = tensao_base
        sinal_eixo_x = -1.0
        sinal_eixo_y = -1.0
        kp_base = 63.62
        ki_base = 75.48
        kd_base = 1.02
        kp_atual, ki_atual, kd_atual = kp_base, ki_base, kd_base
        limiar_fuzzy = 0.0002
        deriv_filtrada_x = 0.0
        deriv_filtrada_y = 0.0
        erro_anterior_x = 0.0
        erro_anterior_y = 0.0
       
        # Mudança Matemática: Agora acumulamos a "Ação Integral" (Tensão) e não o "Erro puro"
        acao_integral_x = 0.0
        acao_integral_y = 0.0
        tempo_inicio_total = time.time()
        ultimo_tempo = tempo_inicio_total
        contador_leituras = 0
        malha_fechada_ativada = False
        print("Iniciando aquisição em LOOP ABERTO...")
        print(f"O CONTROLE PID FUZZY assumirá após {tempo_loop_aberto_segundos:.0f} segundos.")
        while True:
            tempo_decorrido = time.time() - tempo_inicio_total
            if tempo_decorrido >= tempo_total_segundos:
                break
            dt = time.time() - ultimo_tempo
            # Proteção contra dt zero que causaria divisão por zero
            if dt < 1e-6:
                time.sleep(0.001)
                continue
            ultimo_tempo = time.time()
            status_dq = cubo_dq.Status
            
            # 1. Lê as diferenças de tensão puras
            diff_x = float(str(status_dq.PositionDifference.X).replace(',', '.'))
            diff_y = float(str(status_dq.PositionDifference.Y).replace(',', '.'))
            
            # 2. Lê a Intensidade Total (Soma dos quadrantes)
            intensidade = float(str(status_dq.Sum).replace(',', '.'))
            
            # 3. Calcula a Posição Real Normalizada imune a cintilação
            if intensidade > 0.1:
                x_pos_real = diff_x / intensidade
                y_pos_real = diff_y / intensidade
            else:
                x_pos_real = 0.0
                y_pos_real = 0.0
                
            # O PID agora é alimentado pela POSIÇÃO REAL
            x_pos = x_pos_real
            y_pos = y_pos_real
            if tempo_decorrido >= tempo_loop_aberto_segundos and not malha_fechada_ativada:
                print("\n>>> TEMPO ATINGIDO: ATIVANDO MALHA FECHADA FUZZY! <<<")
                malha_fechada_ativada = True
                erro_anterior_x = -x_pos
                erro_anterior_y = -y_pos
                acao_integral_x = 0.0
                acao_integral_y = 0.0
                deriv_filtrada_x = 0.0
                deriv_filtrada_y = 0.0
            modo_log = "Fechado" if malha_fechada_ativada else "Aberto"
            if malha_fechada_ativada:
                erro_x = (0.0 - x_pos) * sinal_eixo_x
                erro_y = (0.0 - y_pos) * sinal_eixo_y
 
                derivada_x = np.clip((erro_x - erro_anterior_x) / dt, -2.0, 2.0)
                derivada_y = np.clip((erro_y - erro_anterior_y) / dt, -2.0, 2.0)
 
                # A EQUAÇÃO FICA AQUI DENTRO!
                deriv_filtrada_x = 0.8 * deriv_filtrada_x + 0.2 * derivada_x
                deriv_filtrada_y = 0.8 * deriv_filtrada_y + 0.2 * derivada_y
               
                erro_anterior_x = erro_x
                erro_anterior_y = erro_y
 
                if max(abs(erro_x), abs(erro_y)) > limiar_fuzzy:
                    kp_atual, ki_atual, kd_atual = calcular_ganhos_fuzzy(
                        erro_x, erro_y, deriv_filtrada_x, deriv_filtrada_y,
                        kp_base, ki_base, kd_base
                    )
                # ACÚMULO SEGURO DA AÇÃO INTEGRAL (Evita picos com mudanças de Ki)
                incremento_i_x = ki_atual * erro_x * dt
                incremento_i_y = ki_atual * erro_y * dt
                # PID: Ação Proporcional + Ação Integral Acumulada + Ação Derivativa
                calculo_x = tensao_base + (kp_atual * erro_x) + (acao_integral_x + incremento_i_x) + (kd_atual * deriv_filtrada_x)
                calculo_y = tensao_base + (kp_atual * erro_y) + (acao_integral_y + incremento_i_y) + (kd_atual * deriv_filtrada_y)
                # ANTI-WINDUP INTELIGENTE: Só acumula a integral se não estourar a tensão
                if calculo_x > 150.0:
                    tensao_alvo_x = 150.0
                elif calculo_x < 0.0:
                    tensao_alvo_x = 0.0
                else:
                    tensao_alvo_x = calculo_x
                    acao_integral_x += incremento_i_x  # Efetiva o acúmulo se estiver seguro
                if calculo_y > 150.0:
                    tensao_alvo_y = 150.0
                elif calculo_y < 0.0:
                    tensao_alvo_y = 0.0
                else:
                    tensao_alvo_y = calculo_y
                    acao_integral_y += incremento_i_y  # Efetiva o acúmulo se estiver seguro
                cubo_x.SetOutputVoltage(Decimal(round(tensao_alvo_x, 2)))
                cubo_y.SetOutputVoltage(Decimal(round(tensao_alvo_y, 2)))
            escritor_csv.writerow([
                f"{tempo_decorrido:.3f}", modo_log,
                diff_x, diff_y, intensidade, x_pos_real, y_pos_real,
                round(tensao_alvo_x, 2), round(tensao_alvo_y, 2),
                f"{kp_atual:.2f}", f"{ki_atual:.2f}", f"{kd_atual:.2f}"
            ])
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
   
    executar_malha_fechada_dq_software("113252111", "113251965", "69254212",
                                       tempo_total_segundos=120.0,
                                       tempo_loop_aberto_segundos=10.0)
#%%