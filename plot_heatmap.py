#%%
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import os

#%%
# --- 1. CONFIGURAÇÕES DE ESTILO (TIMES NEW ROMAN / TAMANHO 12) ---
plt.rcParams.update({
    'font.family': 'serif',
    'font.serif': ['Times New Roman', 'Times'],
    'font.size': 12,
    'axes.labelsize': 12,
    'axes.titlesize': 12,
    'xtick.labelsize': 12,
    'ytick.labelsize': 12,
    'legend.fontsize': 12
})

diretorio_base = r"C:\Users\adm_artur.matoso\Desktop\closed_loop_DQ"
ARQUIVO_ALVO = 'dq_malha_fechada.csv'
LABEL_GRAFICO = 'Varredura polaris'

# Configurações físicas baseadas no DQ de 3 mm de diâmetro (Raio = 1.5 mm)
RAIO_DQ_REAL = 1.5     # Raio físico real do DQ em milímetros (mm)
FATOR_CONVERSAO = 1.0  # Ajuste caso precise de conversão fina de escala
UNIDADE = "mm"
SKIP_POINTS = 1        

# Configurações do Heatmap (Limitado exatamente à borda do sensor)
RAIO_ZOOM = 2          # Limitado estritamente ao raio do DQ (1.5 mm)
BINS = 80              # Resolução do heatmap

# --- 2. FUNÇÃO DE CARREGAMENTO E PROCESSAMENTO COM FILTRO ---
def carregar_e_processar_dados(diretorio, nome_arquivo):
    caminho = os.path.join(diretorio, nome_arquivo)
    
    print(f"--- PROCESSANDO ARQUIVO: {nome_arquivo} ---")
    
    if not os.path.exists(caminho):
        print(f"\nERRO CRÍTICO: Não encontrei o arquivo!")
        print(f"Procurando por: {caminho}")
        return None

    try:
        df = pd.read_csv(caminho)
        df.columns = [c.strip() for c in df.columns] 
        
        if 'x_pos' not in df.columns or 'y_pos' not in df.columns:
            print(f"ERRO: Colunas 'x_pos' ou 'y_pos' não encontradas em {nome_arquivo}")
            return None
            
        x_raw = df['x_pos'].values[::SKIP_POINTS] * FATOR_CONVERSAO
        y_raw = df['y_pos'].values[::SKIP_POINTS] * FATOR_CONVERSAO
        
        # --- CENTRALIZAÇÃO DOS DADOS ---
        x_cent = x_raw - np.mean(x_raw)
        y_cent = y_raw - np.mean(y_raw)
        
        # --- FILTRAGEM: REMOVE DADOS FORA DA ÁREA FÍSICA DO DQ ---
        distancia_origem = np.sqrt(x_cent**2 + y_cent**2)
        mascara_validos = distancia_origem <= RAIO_DQ_REAL
        
        x_filtrado = x_cent[mascara_validos]
        y_filtrado = y_cent[mascara_validos]
        
        print(f"Pontos totais no arquivo: {len(x_raw)}")
        print(f"Pontos válidos dentro do DQ (<= 1.5mm): {len(x_filtrado)}")
        
        return {'x': x_filtrado, 'y': y_filtrado}
    
    except Exception as e:
        print(f"Erro ao ler/processar arquivo: {e}")
        return None

# --- EXECUÇÃO DO CARREGAMENTO ---
dado_processado = carregar_e_processar_dados(diretorio_base, ARQUIVO_ALVO)

if dado_processado is None or len(dado_processado['x']) == 0:
    print("Execução interrompida: Nenhum ponto válido dentro do DQ.")
    exit()

# --- 3. CÁLCULO DO HISTOGRAMA 2D (HEATMAP) ---
limites_xy = (-RAIO_ZOOM, RAIO_ZOOM)

H, xedges, yedges = np.histogram2d(
    dado_processado['x'],
    dado_processado['y'],
    bins=BINS,
    range=[limites_xy, limites_xy],
    density=True
)

H = H.T
max_densidade = H.max()
if max_densidade == 0: 
    H_relativo = H
else:
    H_relativo = H / max_densidade

# --- 4. CONFIGURAÇÃO DO COLORMAP ---
cmap_base = plt.get_cmap('RdBu_r')
novas_cores = cmap_base(np.linspace(0, 1, 256)**0.5)
CMAP_ALTO_CONTRASTE = mcolors.ListedColormap(novas_cores)

# --- 5. PLOTAGEM ---
fig, ax = plt.subplots(figsize=(8, 7))

im = ax.imshow(
    H_relativo,
    interpolation='nearest',
    origin='lower',
    extent=[limites_xy[0], limites_xy[1], limites_xy[0], limites_xy[1]],
    cmap=CMAP_ALTO_CONTRASTE,
    vmin=0, vmax=1.0
)

# --- DESENHO DA BORDA REAL DO DQ (Círculo de Ouro - Ø = 3.0 mm) ---
circulo_dq = plt.Circle(
    (0, 0), RAIO_DQ_REAL,
    color='white', fill=False,
    linestyle='-', linewidth=2.5, alpha=0.95,
    label=f'Borda Real do DQ (Ø = 3.0 mm)'
)
ax.add_patch(circulo_dq)

ax.set_title(
    f"{LABEL_GRAFICO}",
    fontsize=12, fontweight='bold'
)

ax.set_xlabel(f"X ({UNIDADE})", fontsize=12)
ax.set_ylabel(f"Y ({UNIDADE})", fontsize=12)

ax.set_aspect('equal') 
ax.grid(True, alpha=0.2, color='white', linestyle=':')
ax.axhline(0, color='white', alpha=0.3, lw=1)
ax.axvline(0, color='white', alpha=0.3, lw=1)

# ax.legend(loc='upper right', framealpha=0.8)

# --- BARRA DE CORES (COLORBAR) ---
cbar = plt.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
cbar.set_label('Intensidade relativa (normalizada)', rotation=270, labelpad=15, fontsize=12)

plt.tight_layout()

# --- SALVAR E MOSTRAR ---
nome_saida_pdf = f"heatmap_dq_filtrado_{ARQUIVO_ALVO.replace('.csv', '')}.pdf"
# plt.savefig(nome_saida_pdf, format='pdf', bbox_inches='tight')
# print(f"Gráfico salvo como: {nome_saida_pdf}")

plt.show()
# %%