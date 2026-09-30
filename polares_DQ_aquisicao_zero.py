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
# CÓDIGO DE VARREDURA FIXA EM 0.0V (ALTA FREQUÊNCIA)

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

def executar_varredura_com_dq_estavel(serial_x, serial_y, serial_dq, perfil_x, perfil_y, intervalo_segundos, tempo_total_segundos):
    DeviceManagerCLI.BuildDeviceList()
    
    print("Iniciando conexão estável...")
    
    cubo_x = KCubePiezoStrainGauge.CreateKCubePiezoStrainGauge(serial_x)
    cubo_y = KCubePiezoStrainGauge.CreateKCubePiezoStrainGauge(serial_y)
    cubo_dq = KCubePositionAligner.CreateKCubePositionAligner(serial_dq)
    
    cubo_x.Connect(serial_x)
    cubo_y.Connect(serial_y)
    cubo_dq.Connect(serial_dq)
    
    # Polling seguro de 5 ms (~200 Hz nativo na fila USB)
    cubo_x.StartPolling(5)
    cubo_y.StartPolling(5)
    cubo_dq.StartPolling(5) 
    
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
            
        print("Dispositivos prontos! Gravando dados com 0.0V fixo...\n")

        max_voltage = Decimal(150.0)
        cubo_x.SetMaxOutputVoltage(max_voltage)
        cubo_y.SetMaxOutputVoltage(max_voltage)
        cubo_x.SetPositionControlMode(PiezoControlModeTypes.OpenLoop)
        cubo_y.SetPositionControlMode(PiezoControlModeTypes.OpenLoop)
        time.sleep(0.2)
        
        fator_polaris = 2.0 / 150.0 
        
        nome_arquivo = "dados_polaris_dq.csv"
        f_csv = open(nome_arquivo, 'w', newline='')
        escritor_csv = csv.writer(f_csv)
        escritor_csv.writerow(['x_pos', 'y_pos', 'x_angle', 'y_angle'])

        tempo_inicio_total = time.time()
        contador_leituras = 0
        buffer_dados = [] 

        while (time.time() - tempo_inicio_total) < tempo_total_segundos:
            for alvo_x, alvo_y in zip(perfil_x, perfil_y):
                
                if (time.time() - tempo_inicio_total) >= tempo_total_segundos:
                    break
                
                safe_x = max(0.0, min(150.0, float(alvo_x)))
                safe_y = max(0.0, min(150.0, float(alvo_y)))
                
                cubo_x.SetOutputVoltage(Decimal(safe_x))
                cubo_y.SetOutputVoltage(Decimal(safe_y))
                
                tempo_inicio_passo = time.time()
                
                while (time.time() - tempo_inicio_passo) < intervalo_segundos:
                    if (time.time() - tempo_inicio_total) >= tempo_total_segundos:
                        break
                    
                    status_dq = cubo_dq.Status
                    pos_x_mm = float(status_dq.PositionDifference.X)
                    pos_y_mm = float(status_dq.PositionDifference.Y)
                    
                    val_x = float(str(cubo_x.GetOutputVoltage()).replace(',', '.'))
                    val_y = float(str(cubo_y.GetOutputVoltage()).replace(',', '.'))
                    
                    angulo_x_mrad = val_x * fator_polaris
                    angulo_y_mrad = val_y * fator_polaris
                    
                    buffer_dados.append([pos_x_mm, pos_y_mm, angulo_x_mrad, angulo_y_mrad])
                    contador_leituras += 1
                    
                    if len(buffer_dados) >= 200:
                        escritor_csv.writerows(buffer_dados)
                        f_csv.flush()
                        buffer_dados.clear()
                    
                    time.sleep(0.001)

        if buffer_dados:
            escritor_csv.writerows(buffer_dados)
            f_csv.flush()

        print(f"\nVarredura finalizada com sucesso! Total de leituras gravadas: {contador_leituras}")

    except KeyboardInterrupt:
        print("\nEnsaio interrompido pelo usuário.")

    finally:
        print("Liberando hardware...")
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
        print("Portas USB liberadas com segurança.")

if __name__ == "__main__":
    # Alterado para enviar apenas 0.0V em ambos os eixos
    lista_tensoes_x = [0.0]
    lista_tensoes_y = [0.0]
    
    tempo_por_passo = 10.0 
    tempo_total_do_loop = 90.0 # Tempo total da aquisição mantendo 0.0V
    
    executar_varredura_com_dq_estavel("113252111", "113251965", "69254212", 
                                      lista_tensoes_x, lista_tensoes_y, 
                                      tempo_por_passo, tempo_total_do_loop)
# %%