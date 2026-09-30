#%%
import os
import sys
import time
import csv

#%%

# CÓDIGO PARA ZEROING (descarga suave dos atuadores)

def zeroing_suave(serial_x, serial_y, step_volts=10.0, delay_s=0.05):
    kinesis_path = r"C:\Program Files\Thorlabs\Kinesis"
    if hasattr(os, 'add_dll_directory'):
        os.add_dll_directory(kinesis_path)
    sys.path.append(kinesis_path)

    import clr
    clr.AddReference("Thorlabs.MotionControl.DeviceManagerCLI")
    clr.AddReference("Thorlabs.MotionControl.GenericPiezoCLI")
    clr.AddReference("Thorlabs.MotionControl.KCube.PiezoStrainGaugeCLI")
    
    from Thorlabs.MotionControl.DeviceManagerCLI import DeviceManagerCLI
    from Thorlabs.MotionControl.GenericPiezoCLI.Piezo import PiezoControlModeTypes
    from Thorlabs.MotionControl.KCube.PiezoStrainGaugeCLI import KCubePiezoStrainGauge
    from System import Decimal

    DeviceManagerCLI.BuildDeviceList()
    
    print("Conectando aos cubos para o Zeroing...")
    cubo_x = KCubePiezoStrainGauge.CreateKCubePiezoStrainGauge(serial_x)
    cubo_y = KCubePiezoStrainGauge.CreateKCubePiezoStrainGauge(serial_y)

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

zeroing_suave("113252111", "113251965")
#%%


#%%
# CÓDIGO DE MALHA FECHADA (CLOSED-LOOP) DE ALTA VELOCIDADE

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

def executar_malha_fechada_dq(serial_x, serial_y, serial_dq, setpoint_x_mm, setpoint_y_mm, tempo_total_segundos, kp=5.0, reverse_x=False, reverse_y=False):
    DeviceManagerCLI.BuildDeviceList()
    
    print("Iniciando conexão para Malha Fechada de Alta Velocidade...")
    
    cubo_x = KCubePiezoStrainGauge.CreateKCubePiezoStrainGauge(serial_x)
    cubo_y = KCubePiezoStrainGauge.CreateKCubePiezoStrainGauge(serial_y)
    cubo_dq = KCubePositionAligner.CreateKCubePositionAligner(serial_dq)
    
    cubo_x.Connect(serial_x)
    cubo_y.Connect(serial_y)
    cubo_dq.Connect(serial_dq)
    
    # Polling agressivo de 2ms para destravar a velocidade USB
    cubo_x.StartPolling(2)
    cubo_y.StartPolling(2)
    cubo_dq.StartPolling(2) 
    
    time.sleep(0.25)
    
    cubo_x.EnableDevice()
    cubo_y.EnableDevice()
    cubo_dq.EnableDevice()
    time.sleep(0.25)
    
    f_csv = None
    
    try:
        if not cubo_x.IsSettingsInitialized():
            cubo_x.WaitForSettingsInitialized(10000)
        if not cubo_y.IsSettingsInitialized():
            cubo_y.WaitForSettingsInitialized(10000)
        if not cubo_dq.IsSettingsInitialized():
            cubo_dq.WaitForSettingsInitialized(10000)
            
        print("Dispositivos prontos! Iniciando controle em malha fechada...")

        max_voltage = Decimal(150.0)
        cubo_x.SetMaxOutputVoltage(max_voltage)
        cubo_y.SetMaxOutputVoltage(max_voltage)
        cubo_x.SetPositionControlMode(PiezoControlModeTypes.OpenLoop)
        cubo_y.SetPositionControlMode(PiezoControlModeTypes.OpenLoop)
        time.sleep(0.2)
        
        fator_polaris = 2.0 / 150.0 
        
        # Ajusta o sinal do KP se a ação for Reversa (Reverse Action)
        sinal_kp_x = -1.0 if reverse_x else 1.0
        sinal_kp_y = -1.0 if reverse_y else 1.0
        
        kp_x_aplicado = kp * sinal_kp_x
        kp_y_aplicado = kp * sinal_kp_y
        
        nome_arquivo = "dados_polaris_dq_malha_fechada.csv"
        f_csv = open(nome_arquivo, 'w', newline='')
        escritor_csv = csv.writer(f_csv)
        escritor_csv.writerow(['x_pos', 'y_pos', 'x_angle', 'y_angle'])

        # Ponto de partida inicial dos atuadores (Centro do range)
        tensao_atual_x = 75.0
        tensao_atual_y = 75.0
        
        cubo_x.SetOutputVoltage(Decimal(tensao_atual_x))
        cubo_y.SetOutputVoltage(Decimal(tensao_atual_y))

        tempo_inicio_total = time.time()
        contador_leituras = 0

        # Laço principal de malha fechada
        while (time.time() - tempo_inicio_total) < tempo_total_segundos:
            
            # CORREÇÃO: Força a atualização do sensor via USB para evitar dados em cache
            cubo_dq.RequestStatus()
            
            # 1. Lê a posição atual real do feixe no Sensor DQ
            status_dq = cubo_dq.Status
            leitura_dq_x = status_dq.PositionDifference.X
            leitura_dq_y = status_dq.PositionDifference.Y
            
            val_dq_x = float(str(leitura_dq_x).replace(',', '.'))
            val_dq_y = float(str(leitura_dq_y).replace(',', '.'))
            
            # CORREÇÃO: O sensor DQ já retorna em milímetros diretamente (removido o fator 1.5 incorreto)
            pos_x_mm = val_dq_x
            pos_y_mm = val_dq_y
            
            # 2. Calcula o Erro
            erro_x = setpoint_x_mm - pos_x_mm
            erro_y = setpoint_y_mm - pos_y_mm
            
            # 3. Lei de controle Proporcional com detecção REVERSE
            tensao_atual_x += kp_x_aplicado * erro_x
            tensao_atual_y += kp_y_aplicado * erro_y
            
            # 4. TRAVA DE SEGURANÇA ESTRITA (0.0V a 150.0V cravados)
            tensao_atual_x = max(0.0, min(150.0, tensao_atual_x))
            tensao_atual_y = max(0.0, min(150.0, tensao_atual_y))
            
            # 5. Aplica a nova voltagem
            cubo_x.SetOutputVoltage(Decimal(tensao_atual_x))
            cubo_y.SetOutputVoltage(Decimal(tensao_atual_y))
            
            # Lê a voltagem para salvar no log
            leitura_x = cubo_x.GetOutputVoltage()
            leitura_y = cubo_y.GetOutputVoltage()
            val_x = float(str(leitura_x).replace(',', '.'))
            val_y = float(str(leitura_y).replace(',', '.'))
            
            angulo_x_mrad = val_x * fator_polaris
            angulo_y_mrad = val_y * fator_polaris
            
            # 6. Salva simultaneamente os dados
            escritor_csv.writerow([pos_x_mm, pos_y_mm, angulo_x_mrad, angulo_y_mrad])
            
            # Flush apenas a cada 50 leituras para não engasgar o HD e ganhar velocidade
            if contador_leituras % 50 == 0:
                f_csv.flush()
                
            contador_leituras += 1
            
            # Pausa minúscula apenas para o barramento USB não gerar Kernel Crash (2ms)
            time.sleep(0.002)
                
        taxa_hz = contador_leituras / tempo_total_segundos
        print(f"\nControle em malha fechada finalizado!")
        print(f"Total de amostras: {contador_leituras} | Taxa de Aquisição Média: {taxa_hz:.1f} Hz")

    except KeyboardInterrupt:
        print("\nControle interrompido pelo usuário.")

    finally:
        print("Liberando hardware com segurança...")
        if f_csv is not None:
            f_csv.close()
        
        cubo_x.SetOutputVoltage(Decimal(0.0))
        cubo_y.SetOutputVoltage(Decimal(0.0))
        time.sleep(0.5)
        
        cubo_x.StopPolling()
        cubo_y.StopPolling()
        cubo_dq.StopPolling()
        
        cubo_x.Disconnect()
        cubo_y.Disconnect()
        cubo_dq.Disconnect()
        print("Portas USB liberadas.")

if __name__ == "__main__":
    # Alvo central desejado no DQ (0.0 mm = centro)
    alvo_x_mm = 0.0
    alvo_y_mm = 0.0
    
    # Ganho proporcional do PID
    ganho_kp = 2.0 
    
    # Define a direção do atuador ("Reverse")
    # Mude para True se o eixo estiver fugindo do centro em vez de buscar o centro
    eixo_x_reverso = True 
    eixo_y_reverso = True
    
    # Tempo total de estabilização
    tempo_total_do_loop = 90.0 
    
    executar_malha_fechada_dq("113252111", "113251965", "69254212", 
                              alvo_x_mm, alvo_y_mm, 
                              tempo_total_do_loop, kp=ganho_kp,
                              reverse_x=eixo_x_reverso, reverse_y=eixo_y_reverso)
# %%