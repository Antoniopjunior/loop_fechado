#%%
import os
import sys
import time
import csv
 
kinesis_path = r"C:\Program Files\Thorlabs\Kinesis"
if hasattr(os, 'add_dll_directory'):
    os.add_dll_directory(kinesis_path)
sys.path.append(kinesis_path)
 
import clr
clr.AddReference("Thorlabs.MotionControl.DeviceManagerCLI")
clr.AddReference("Thorlabs.MotionControl.GenericPiezoCLI")
clr.AddReference("Thorlabs.MotionControl.KCube.PiezoStrainGaugeCLI")
clr.AddReference("Thorlabs.MotionControl.KCube.PositionAlignerCLI")
 
from System import Decimal
from Thorlabs.MotionControl.DeviceManagerCLI import DeviceManagerCLI
from Thorlabs.MotionControl.GenericPiezoCLI.Piezo import PiezoControlModeTypes
from Thorlabs.MotionControl.KCube.PiezoStrainGaugeCLI import KCubePiezoStrainGauge
from Thorlabs.MotionControl.KCube.PositionAlignerCLI import KCubePositionAligner
 
#%%
# CÓDIGO PARA ZEROING (descarga suave dos atuadores)
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
 
# CÓDIGO DE AQUISIÇÃO E CONTROLE PID POR SOFTWARE
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
 
        pasta_saida = "dia_16_09/PID_vencedores/turbulencia"
        os.makedirs(pasta_saida, exist_ok=True)
        
        kp = 119.42
        ki = 223.55
        kd = 2.41
        
        nome_arquivo = f"dia_16_09/Teste_PID/Testes_otmizacao_kp{kp}_ki{ki}_kd{kd}.csv"
 
        f_csv = open(nome_arquivo, 'w', newline='')
        escritor_csv = csv.writer(f_csv)
        
        # Cabeçalho unificado com tempo, modo, posição real normalizada e as tensões que o PID aplicou
        escritor_csv.writerow(['tempo_s', 'modo', 'diff_x', 'diff_y', 'intensidade', 'x_pos_real', 'y_pos_real', 'tensao_x', 'tensao_y'])
 
        # INÍCIO CORRETO NO CENTRO MECÂNICO PARA EVITAR CORTE FÍSICO DO SINAL
        tensao_base = 75.0
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
 
        print("Iniciando aquisição em LOOP ABERTO...")
        print(f"O CONTROLE PID assumirá após {tempo_loop_aberto_segundos:.0f} segundos.")
 
        while True:
            tempo_decorrido = time.time() - tempo_inicio_total
            if tempo_decorrido >= tempo_total_segundos:
                break
 
            dt = time.time() - ultimo_tempo
            ultimo_tempo = time.time()
 
            status_dq = cubo_dq.Status
 
            # 1. Lê os valores brutos
            diff_x = float(str(status_dq.PositionDifference.X).replace(',', '.'))
            diff_y = float(str(status_dq.PositionDifference.Y).replace(',', '.'))
            intensidade = float(str(status_dq.Sum).replace(',', '.'))
 
            # 2. Calcula a Posição Real Normalizada (Blindagem contra cintilação)
            if intensidade > 0.1:
                x_pos_real = diff_x / intensidade
                y_pos_real = diff_y / intensidade
            else:
                x_pos_real = 0.0
                y_pos_real = 0.0
 
            # 3. O PID agora é alimentado pela POSIÇÃO REAL e não mais pelo diff cru
            x_pos = x_pos_real
            y_pos = y_pos_real
 
            if tempo_decorrido >= tempo_loop_aberto_segundos and not malha_fechada_ativada:
                print("\n>>> TEMPO ATINGIDO: ATIVANDO MALHA FECHADA PELO PYTHON! <<<")
                malha_fechada_ativada = True
                erro_anterior_x = -x_pos
                erro_anterior_y = -y_pos
                erro_acumulado_x = 0.0
                erro_acumulado_y = 0.0
                deriv_filtrada_x = 0.0
                deriv_filtrada_y = 0.0
 
            modo_log = "Fechado" if malha_fechada_ativada else "Aberto"
 
            if malha_fechada_ativada:
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
 
            # 4. Grava no CSV o cenário completo para análise posterior
            escritor_csv.writerow([
                f"{tempo_decorrido:.3f}", 
                modo_log, 
                diff_x, 
                diff_y, 
                intensidade, 
                x_pos_real, 
                y_pos_real,
                round(tensao_alvo_x, 2),
                round(tensao_alvo_y, 2)
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
 
    tempo_total_experimento = 120.0
    tempo_livre_antes_fechar = 10.0
 
    executar_malha_fechada_dq_software(
        "113252111",
        "113251965",
        "69254212",
        tempo_total_segundos=tempo_total_experimento,
        tempo_loop_aberto_segundos=tempo_livre_antes_fechar
    )
#%%
 