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
# CÓDIGO DE VARREDURA EM MALHA FECHADA (COM GANHO CALIBRADO)

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

def executar_varredura_fechada(serial_x, serial_y, serial_dq, lista_alvos_x, lista_alvos_y, tempo_por_passo, tempo_total, kp=50.0, reverse_x=True, reverse_y=True):
    DeviceManagerCLI.BuildDeviceList()
    
    print("Iniciando conexão para Varredura Estabilizada (Malha Fechada)...")
    
    cubo_x = KCubePiezoStrainGauge.CreateKCubePiezoStrainGauge(serial_x)
    cubo_y = KCubePiezoStrainGauge.CreateKCubePiezoStrainGauge(serial_y)
    cubo_dq = KCubePositionAligner.CreateKCubePositionAligner(serial_dq)
    
    cubo_x.Connect(serial_x)
    cubo_y.Connect(serial_y)
    cubo_dq.Connect(serial_dq)
    
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
            
        print("Hardware pronto! Iniciando malha fechada com ganho de controle otimizado...\n")

        max_voltage = Decimal(150.0)
        cubo_x.SetMaxOutputVoltage(max_voltage)
        cubo_y.SetMaxOutputVoltage(max_voltage)
        cubo_x.SetPositionControlMode(PiezoControlModeTypes.OpenLoop)
        cubo_y.SetPositionControlMode(PiezoControlModeTypes.OpenLoop)
        time.sleep(0.2)
        
        fator_polaris = 2.0 / 150.0 
        
        sinal_kp_x = -1.0 if reverse_x else 1.0
        sinal_kp_y = -1.0 if reverse_y else 1.0
        kp_x_aplicado = kp * sinal_kp_x
        kp_y_aplicado = kp * sinal_kp_y
        
        nome_arquivo = "dados_varredura_fechada.csv"
        f_csv = open(nome_arquivo, 'w', newline='')
        escritor_csv = csv.writer(f_csv)
        escritor_csv.writerow(['setpoint_x', 'setpoint_y', 'x_pos', 'y_pos', 'x_angle', 'y_angle'])

        # Inicia no centro seguro (75V)
        tensao_atual_x = 75.0
        tensao_atual_y = 75.0
        cubo_x.SetOutputVoltage(Decimal(tensao_atual_x))
        cubo_y.SetOutputVoltage(Decimal(tensao_atual_y))

        tempo_inicio_total = time.time()
        contador_leituras = 0

        while (time.time() - tempo_inicio_total) < tempo_total:
            for alvo_x, alvo_y in zip(lista_alvos_x, lista_alvos_y):
                
                if (time.time() - tempo_inicio_total) >= tempo_total:
                    break
                
                print(f"--> Movendo para o Alvo no Sensor: X={alvo_x}mm, Y={alvo_y}mm")
                tempo_inicio_passo = time.time()
                
                while (time.time() - tempo_inicio_passo) < tempo_por_passo:
                    if (time.time() - tempo_inicio_total) >= tempo_total:
                        break
                    
                    cubo_dq.RequestStatus()
                    status_dq = cubo_dq.Status
                    
                    pos_x_mm = float(status_dq.PositionDifference.X)
                    pos_y_mm = float(status_dq.PositionDifference.Y)
                    
                    # Erro em milímetros
                    erro_x = alvo_x - pos_x_mm
                    erro_y = alvo_y - pos_y_mm
                    
                    # Lei de controle proporcional calibrada para a escala de Volts (0-150V)
                    tensao_atual_x += kp_x_aplicado * erro_x
                    tensao_atual_y += kp_y_aplicado * erro_y
                    
                    # Trava física estrita (0V a 150V)
                    tensao_atual_x = max(0.0, min(150.0, tensao_atual_x))
                    tensao_atual_y = max(0.0, min(150.0, tensao_atual_y))
                    
                    # Aplica voltagem no Polaris
                    cubo_x.SetOutputVoltage(Decimal(tensao_atual_x))
                    cubo_y.SetOutputVoltage(Decimal(tensao_atual_y))
                    
                    val_x = float(str(cubo_x.GetOutputVoltage()).replace(',', '.'))
                    val_y = float(str(cubo_y.GetOutputVoltage()).replace(',', '.'))
                    
                    angulo_x_mrad = val_x * fator_polaris
                    angulo_y_mrad = val_y * fator_polaris
                    
                    escritor_csv.writerow([alvo_x, alvo_y, pos_x_mm, pos_y_mm, angulo_x_mrad, angulo_y_mrad])
                    
                    if contador_leituras % 50 == 0:
                        f_csv.flush()
                        
                    contador_leituras += 1
                    time.sleep(0.005)

        print(f"\nVarredura finalizada! Total de amostras: {contador_leituras}")

    except KeyboardInterrupt:
        print("\nInterrompido pelo usuário.")

    finally:
        if f_csv is not None:
            f_csv.close()
        cubo_x.SetOutputVoltage(Decimal(75.0))
        cubo_y.SetOutputVoltage(Decimal(75.0))
        time.sleep(0.5)
        cubo_x.StopPolling()
        cubo_y.StopPolling()
        cubo_dq.StopPolling()
        cubo_x.Disconnect()
        cubo_y.Disconnect()
        cubo_dq.Disconnect()
        print("Hardware liberado com segurança.")

if __name__ == "__main__":
    
    # Grade de 9 pontos reduzida e segura para o sensor não saturar nas bordas (-0.15mm a +0.15mm)
    grade_alvos_x = [-0.15,  0.0,  0.15,  0.15,  0.0, -0.15, -0.15,  0.0,  0.15]
    grade_alvos_y = [-0.15, -0.15, -0.15,  0.0,  0.0,   0.0,  0.15,  0.15,  0.15]
    
    # Reduzimos de 50 para 10 para o controle ser suave e não "chutar" o feixe para os cantos
    ganho_kp = 10.0 
    
    eixo_x_reverso = True 
    eixo_y_reverso = True
    
    tempo_estabilizado_por_ponto = 5.0 # 5 segundos travado em cada ponto da grade
    tempo_total_do_loop = 45.0         
    
    executar_varredura_fechada("113252111", "113251965", "69254212", 
                               grade_alvos_x, grade_alvos_y, 
                               tempo_estabilizado_por_ponto, tempo_total_do_loop, 
                               kp=ganho_kp,
                               reverse_x=eixo_x_reverso, reverse_y=eixo_y_reverso)
# %%