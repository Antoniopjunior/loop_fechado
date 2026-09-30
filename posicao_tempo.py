#%%
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# ==========================================
# Configurações
# ==========================================

arquivo_csv = r"dia_16_09\Teste_fuzzy\teste_loop_fuzzy1.csv"

# Passo temporal entre amostras
dt = 1.0/120

# ==========================================
# Leitura dos dados
# ==========================================

df = pd.read_csv(arquivo_csv)

print("Colunas disponíveis:")
print(df.columns.tolist())

print("\nPrimeiras linhas:")
print(df.head())

# ==========================================
# Vetor de tempo
# ==========================================

tempo = np.arange(len(df)) * dt

# ==========================================
# Estatísticas básicas
# ==========================================

print("\nEstatísticas de x_pos:")
print(df["y_pos_real"].describe())
print(max(df["x_pos_real"]))
print(min(df["x_pos_real"]))
print(max(df["intensidade"]))
print(min(df["intensidade"]))
# ==========================================
# Gráfico x_pos versus tempo
# ==========================================

plt.figure(figsize=(12, 6))

plt.plot(
    tempo,
    df["y_pos_real"]*1.5,
    color="blue",
    linewidth=1,
    label="x_pos"
)

plt.xlabel("Tempo")
plt.ylabel("Posição X")
plt.title("Posição X em função do tempo")
plt.grid(True)
plt.legend()
plt.ylim(-1.5, 1.5)

plt.tight_layout()

# Salvar figura
#plt.savefig("x_pos_vs_tempo.png", dpi=300)

# Mostrar figura
plt.show()

print("\nFigura salva como: x_pos_vs_tempo.png")
# %%
