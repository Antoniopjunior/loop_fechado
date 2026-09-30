#%%
import os
import sys
import time

#%%
# 1. Configurando o caminho das DLLs do Kinesis no Windows
kinesis_path = r"C:\Program Files\Thorlabs\Kinesis"

if hasattr(os, 'add_dll_directory'):
    os.add_dll_directory(kinesis_path)
sys.path.append(kinesis_path)

import clr  # Importa a biblioteca pythonnet

# 2. Referenciando as DLLs do Thorlabs Kinesis
clr.AddReference("Thorlabs.MotionControl.DeviceManagerCLI")
clr.AddReference("Thorlabs.MotionControl.GenericPiezoCLI")
clr.AddReference("Thorlabs.MotionControl.KCube.PiezoStrainGaugeCLI")

from Thorlabs.MotionControl.DeviceManagerCLI import DeviceManagerCLI
from Thorlabs.MotionControl.GenericPiezoCLI.Piezo import PiezoControlModeTypes
from Thorlabs.MotionControl.KCube.PiezoStrainGaugeCLI import KCubePiezoStrainGauge
from System import Decimal

# A função agora recebe um sexto argumento: o tempo total do loop
def executar_varredura_polaris(serial_x, serial_y, perfil_x, perfil_y, intervalo_segundos, tempo_total_segundos):
    # Constrói a lista de dispositivos conectados no USB
    DeviceManagerCLI.BuildDeviceList()
    
    print("Iniciando conexão com os cubos...")
    
    # Cria as instâncias para X e Y
    cubo_x = KCubePiezoStrainGauge.CreateKCubePiezoStrainGauge(serial_x)
    cubo_y = KCubePiezoStrainGauge.CreateKCubePiezoStrainGauge(serial_y)
    
    # Conecta fisicamente a ambos os eixos
    cubo_x.Connect(serial_x)
    cubo_y.Connect(serial_y)
    
    # Inicia polling e habilita ambos
    cubo_x.StartPolling(250)
    cubo_y.StartPolling(250)
    time.sleep(0.25)
    
    cubo_x.EnableDevice()
    cubo_y.EnableDevice()
    time.sleep(0.25)
    
    try:
        # Aguarda configurações para X e Y
        if not cubo_x.IsSettingsInitialized():
            cubo_x.WaitForSettingsInitialized(10000)
        if not cubo_y.IsSettingsInitialized():
            cubo_y.WaitForSettingsInitialized(10000)
            
        print("Dispositivos conectados e habilitados com sucesso!\n")

        # Configura Range Máximo (150V) para ambos
        max_voltage = Decimal(150.0)
        cubo_x.SetMaxOutputVoltage(max_voltage)
        cubo_y.SetMaxOutputVoltage(max_voltage)

        # Configura o Modo Open Loop (Tensão)
        cubo_x.SetPositionControlMode(PiezoControlModeTypes.OpenLoop)
        cubo_y.SetPositionControlMode(PiezoControlModeTypes.OpenLoop)
        time.sleep(0.2)

        # Grava o momento exato em que o ensaio começa
        tempo_inicio_total = time.time()

        # O laço principal checa se o tempo total já se esgotou
        while (time.time() - tempo_inicio_total) < tempo_total_segundos:
            
            # A função zip() junta as duas listas e lê passo a passo
            for alvo_x, alvo_y in zip(perfil_x, perfil_y):
                
                # Checagem extra: se o tempo total esgotar no meio da lista, ele para imediatamente
                if (time.time() - tempo_inicio_total) >= tempo_total_segundos:
                    break
                
                # Trava de segurança contra voltagem negativa
                safe_x = max(0.0, float(alvo_x))
                safe_y = max(0.0, float(alvo_y))
                
                # Envia o comando do passo atual
                cubo_x.SetOutputVoltage(Decimal(safe_x))
                cubo_y.SetOutputVoltage(Decimal(safe_y))
                
                # Marca o tempo inicial do passo
                tempo_inicio_passo = time.time()
                
                # Lê as tensões até o intervalo do passo esgotar
                while (time.time() - tempo_inicio_passo) < intervalo_segundos:
                    
                    # Outra checagem extra: quebra o passo atual se o tempo global bater no limite
                    if (time.time() - tempo_inicio_total) >= tempo_total_segundos:
                        break
                    
                    leitura_x = cubo_x.GetOutputVoltage()
                    leitura_y = cubo_y.GetOutputVoltage()
                    
                    # Tratamento da vírgula do Windows
                    val_x = float(str(leitura_x).replace(',', '.'))
                    val_y = float(str(leitura_y).replace(',', '.'))
                    
                    # Calcula o tempo restante e imprime tudo na mesma linha
                    tempo_restante = tempo_total_segundos - (time.time() - tempo_inicio_total)
                    print(f"Restam {tempo_restante:04.1f}s | Alvo (X:{safe_x:05.1f}V Y:{safe_y:05.1f}V) | Atual -> X: {val_x:06.2f}V | Y: {val_y:06.2f}V", end="\r")
                    
                    time.sleep(0.2)
                    
        print("\n\nTempo limite atingido! Encerrando a varredura automaticamente...")

    except KeyboardInterrupt:
        print("\n\nComando de parada (Ctrl+C) recebido pelo usuário. Abortando ensaio...")

    finally:
        print("\nDesabilitando o hardware e liberando as portas USB...")
        
        # Volta para zero volts por segurança no desligamento
        cubo_x.SetOutputVoltage(Decimal(0.0))
        cubo_y.SetOutputVoltage(Decimal(0.0))
        time.sleep(1)
        
        cubo_x.StopPolling()
        cubo_y.StopPolling()
        
        cubo_x.Disconnect()
        cubo_y.Disconnect()
        print("Pronto. Portas USB liberadas com segurança.")

if __name__ == "__main__":
    # Testando com os cubos simultâneos
    lista_tensoes_x = [0.0, 5.0, 10.0, 15.0, 20.0]
    lista_tensoes_y = [20.0, 15.0, 10.0, 5.0, 0.0]
    
    # Defina quanto tempo (em segundos) o cubo deve segurar CADA passo
    tempo_por_passo = 4.0 
    
    # NOVO: Defina o tempo TOTAL (em segundos) que a máquina vai ficar rodando e repetindo a lista
    tempo_total_do_loop = 30.0 # 60 segundos (1 minuto)
    
    # Executa a função passando as listas e os tempos
    executar_varredura_polaris("113252111", "113251965", lista_tensoes_x, lista_tensoes_y, tempo_por_passo, tempo_total_do_loop)
# %%