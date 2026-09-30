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
clr.AddReference("Thorlabs.MotionControl.KCube.PositionAlignerCLI")
 
from Thorlabs.MotionControl.DeviceManagerCLI import DeviceManagerCLI
from Thorlabs.MotionControl.KCube.PositionAlignerCLI import KCubePositionAligner
 
def executar_leitura_dq_isolada(serial_dq, tempo_total_segundos):
    DeviceManagerCLI.BuildDeviceList()
    time.sleep(1.0)
 
    print(f"Conectando ao detector de quadrantes (Serial: {serial_dq})...")
    cubo_dq = KCubePositionAligner.CreateKCubePositionAligner(serial_dq)
 
    try:
        if cubo_dq.IsConnected:
            cubo_dq.Disconnect()
    except:
        pass
 
    cubo_dq.Connect(serial_dq)
 
    # Polling ultrarrápido de 1 ms (1000 Hz teóricos na fila USB)
    cubo_dq.StartPolling(1)
    time.sleep(0.25)
 
    cubo_dq.EnableDevice()
    time.sleep(0.25)
 
    f_csv = None
    try:
        if not cubo_dq.IsSettingsInitialized():
            cubo_dq.WaitForSettingsInitialized(10000)
 
        print("Detector pronto! Iniciando aquisição em ALTA FREQUÊNCIA...\n")
 
        #pasta_saida = "dia_16_09"
        #os.makedirs(pasta_saida, exist_ok=True)
        nome_arquivo = f"dia_16_09/Teste_Cn2/leitura_turbulencia_dq_real_turbforte_6.csv"
 
        f_csv = open(nome_arquivo, 'w', newline='')
        escritor_csv = csv.writer(f_csv)
 
        # Cabeçalho focado apenas nos dados do sensor
        escritor_csv.writerow(['tempo_s', 'x_diff', 'y_diff', 'intensidade', 'x_pos_real', 'y_pos_real'])
 
        tempo_inicio_total = time.time()
        contador_leituras = 0
        buffer_dados = []
 
        while (time.time() - tempo_inicio_total) < tempo_total_segundos:
            tempo_decorrido = time.time() - tempo_inicio_total
 
            status_dq = cubo_dq.Status
 
            # 1. Lê as diferenças de tensão puras
            diff_x = float(str(status_dq.PositionDifference.X).replace(',', '.'))
            diff_y = float(str(status_dq.PositionDifference.Y).replace(',', '.'))
 
            # 2. Lê a Intensidade Total (Soma dos quadrantes)
            intensidade = float(str(status_dq.Sum).replace(',', '.'))
 
            # 3. Calcula a Posição Real Normalizada
            if intensidade > 0.1:
                pos_x_real = diff_x / intensidade
                pos_y_real = diff_y / intensidade
            else:
                pos_x_real = 0.0
                pos_y_real = 0.0
 
            # Atualiza o buffer
            buffer_dados.append([f"{tempo_decorrido:.3f}", diff_x, diff_y, intensidade, pos_x_real, pos_y_real])
            contador_leituras += 1
 
            # Descarrega no CSV em blocos maiores para não gargalar o HD em altas frequências
            if len(buffer_dados) >= 500:
                escritor_csv.writerows(buffer_dados)
                f_csv.flush()
                buffer_dados.clear()
 
            # Mantém o compasso ultrarrápido (1 ms) sem congelar o Kernel
            time.sleep(0.001)
 
        # Grava qualquer resíduo que sobrou no buffer
        if buffer_dados:
            escritor_csv.writerows(buffer_dados)
            f_csv.flush()
 
        frequencia_media = contador_leituras / tempo_total_segundos
        print(f"\nVarredura finalizada com sucesso! Total de leituras: {contador_leituras}")
        print(f"Frequência média de aquisição: {frequencia_media:.2f} Hz")
 
    except KeyboardInterrupt:
        print("\nAquisição interrompida pelo usuário.")
 
    finally:
        print("Liberando hardware...")
        if f_csv is not None:
            f_csv.close()
 
        cubo_dq.StopPolling()
        cubo_dq.Disconnect()
        print("Porta USB liberada com segurança.")
 
if __name__ == "__main__":
    # Apenas o serial do sensor DQ e o tempo desejado de gravação (ex: 60 segundos)
    executar_leitura_dq_isolada("69254212", tempo_total_segundos=140.0)
#%%