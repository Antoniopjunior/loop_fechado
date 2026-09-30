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
# CÓDIGO DE AQUISIÇÃO ULTRARRÁPIDA (SEM ERROS DE ATRIBUTO)
 
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
from System import Decimal, Enum
 
def executar_malha_fechada_dq_hardware(serial_x, serial_y, serial_dq, tempo_total_segundos, tempo_loop_aberto_segundos=30):
    DeviceManagerCLI.BuildDeviceList()
 
    print("Iniciando a conexão...")
 
    cubo_x = KCubePiezoStrainGauge.CreateKCubePiezoStrainGauge(serial_x)
    cubo_y = KCubePiezoStrainGauge.CreateKCubePiezoStrainGauge(serial_y)
    cubo_dq = KCubePositionAligner.CreateKCubePositionAligner(serial_dq)
 
    cubo_x.Connect(serial_x)
    cubo_y.Connect(serial_y)
    cubo_dq.Connect(serial_dq)
 
    # Polling moderado para os Piezos
    cubo_x.StartPolling(250)
    cubo_y.StartPolling(250)
 
    time.sleep(0.25)
 
    cubo_x.EnableDevice()
    cubo_y.EnableDevice()
    cubo_dq.EnableDevice()
    time.sleep(0.25)
 
    f_csv = None
    modo_aberto = None
 
    try:
        if not cubo_x.IsSettingsInitialized():
            cubo_x.WaitForSettingsInitialized(10000)
        if not cubo_y.IsSettingsInitialized():
            cubo_y.WaitForSettingsInitialized(10000)
        if not cubo_dq.IsSettingsInitialized():
            cubo_dq.WaitForSettingsInitialized(10000)
 
        print("Dispositivos prontos! Iniciando aquisição em LOOP ABERTO...")
        print(f"O MODO CLOSED-LOOP DO DQ será ligado fisicamente após {tempo_loop_aberto_segundos:.0f} segundos.")
 
        max_voltage = Decimal(150.0)
        cubo_x.SetMaxOutputVoltage(max_voltage)
        cubo_y.SetMaxOutputVoltage(max_voltage)
        cubo_x.SetPositionControlMode(PiezoControlModeTypes.OpenLoop)
        cubo_y.SetPositionControlMode(PiezoControlModeTypes.OpenLoop)
 
        import System
 
        # Captura modos dinamicamente do DQ
        modo_atual_obj = cubo_dq.GetOperatingMode()
        tipo_do_modo = type(modo_atual_obj)
 
        modo_aberto = System.Enum.Parse(tipo_do_modo, "OpenLoop")
        modo_fechado = System.Enum.Parse(tipo_do_modo, "ClosedLoop")
 
        # Inicia em malha aberta
        cubo_dq.SetOperatingMode(modo_aberto, True)
        time.sleep(0.2)
 
        nome_arquivo = "dados_polaris_dq_malha_fechada_hardware.csv"
        f_csv = open(nome_arquivo, 'w', newline='')
        escritor_csv = csv.writer(f_csv)
        escritor_csv.writerow(['tempo_s', 'modo', 'x_pos', 'y_pos'])
 
        tensao_base = 75.0
        cubo_x.SetOutputVoltage(Decimal(tensao_base))
        cubo_y.SetOutputVoltage(Decimal(tensao_base))
 
        tempo_inicio_total = time.time()
        contador_leituras = 0
        malha_fechada_ativada = False
 
        while True:
            tempo_decorrido = time.time() - tempo_inicio_total
            if tempo_decorrido >= tempo_total_segundos:
                break
 
            # TRANSIÇÃO PARA MALHA FECHADA FÍSICA 
            if tempo_decorrido >= tempo_loop_aberto_segundos and not malha_fechada_ativada:
                print("\n Entrando em malha fechada!")
 
                # Ativa o PID nativo da Thorlabs no hardware
                cubo_dq.SetOperatingMode(modo_fechado, True)
                malha_fechada_ativada = True
 
            # Pede o status manualmente na frequência certa
            cubo_dq.RequestStatus()
            status_dq = cubo_dq.Status
            pos_x_mm = float(str(status_dq.PositionDifference.X).replace(',', '.'))
            pos_y_mm = float(str(status_dq.PositionDifference.Y).replace(',', '.'))
 
            modo_log = "Fechado" if malha_fechada_ativada else "Aberto"
 
            # Grava no CSV a posição do sensor em milímetros
            escritor_csv.writerow([f"{tempo_decorrido:.3f}", modo_log, pos_x_mm, pos_y_mm])
 
            # Flush periódico a cada 50 leituras
            if contador_leituras % 50 == 0:
                f_csv.flush()
 
            contador_leituras += 1
 
            # Pausa de 2 ms para estabilidade do barramento USB
            time.sleep(0.002)
 
        tempo_total_decorrido = time.time() - tempo_inicio_total
        frequencia_aquisicao = contador_leituras / tempo_total_decorrido if tempo_total_decorrido > 0 else 0.0
        print(f"\nEnsaio finalizado com sucesso! Gravadas {contador_leituras} amostras.")
        print(f"Frequência de atualização do Polaris: 4.0 Hz (Polling configurado a 250 ms)")
        print(f"Frequência de aquisição: {frequencia_aquisicao:.2f} Hz")
 
    except KeyboardInterrupt:
        print("\nControle interrompido pelo usuário.")
    except Exception as e:
        print(f"\nERRO: {e}")
 
    finally:
        print("Liberando hardware com segurança...")
        if f_csv is not None:
            f_csv.close()
 
        # Devolve o DQ pro modo seguro caso tenha sido alterado
        if modo_aberto is not None:
            try:
                cubo_dq.SetOperatingMode(modo_aberto, True)
            except:
                pass
 
        cubo_x.SetOutputVoltage(Decimal(75.0))
        cubo_y.SetOutputVoltage(Decimal(75.0))
        time.sleep(0.5)
 
        cubo_x.StopPolling()
        cubo_y.StopPolling()
 
        cubo_x.Disconnect()
        cubo_y.Disconnect()
        cubo_dq.Disconnect()
        print("Portas USB liberadas.")
 
if __name__ == "__main__":
 
    tempo_total_experimento = 20.0
    tempo_livre_antes_fechar = 10.0
 
    executar_malha_fechada_dq_hardware(
        "113252111", "113251965", "69254212",
        tempo_total_segundos=tempo_total_experimento,
        tempo_loop_aberto_segundos=tempo_livre_antes_fechar
    )
# %%