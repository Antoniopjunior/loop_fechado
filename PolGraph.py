import pandas as pd
import numpy as np

filename = "Teste_simples"

df = pd.read_csv(f'new/{filename}.csv')
print("Columns:", df.columns.tolist())
print("Shape:", df.shape)
print(df.head(10))
print(df.tail(10))
print(df.describe())

print(df['modo'].value_counts())
print(df['regime_turb'].value_counts())

df['modo_change'] = df['modo'] != df['modo'].shift(1)
print(df[df['modo_change']][['tempo_s', 'modo', 'regime_turb']])

stats = df.groupby('modo')[['diff_x', 'diff_y', 'x_pos_real', 'y_pos_real', 'tensao_x', 'tensao_y', 'saturado']].agg(['count', 'mean', 'median', 'std', 'min', 'max'])
for col in ['diff_x', 'diff_y', 'x_pos_real', 'y_pos_real']:
    print(f"--- {col} ---")
    print(stats[col])

df['r_pos_real'] = np.sqrt(df['x_pos_real']**2 + df['y_pos_real']**2)
df['r_diff'] = np.sqrt(df['diff_x']**2 + df['diff_y']**2)

print("--- Radial Pos Real ---")
print(df.groupby('modo')['r_pos_real'].agg(['mean', 'median', 'std', 'min', 'max', lambda x: np.percentile(x, 90), lambda x: np.percentile(x, 95)]))


import matplotlib.pyplot as plt
import seaborn as sns

print(df.groupby(['modo', 'saturado'])[['x_pos_real', 'y_pos_real', 'r_pos_real', 'sigma_turb']].agg(['count', 'mean', 'std']))

for r_thresh in [0.1, 0.15, 0.2, 0.25, 0.3]:
    pct_ab = (df[df['modo']=='Aberto']['r_pos_real'] <= r_thresh).mean() * 100
    pct_fe = (df[df['modo']=='Fechado']['r_pos_real'] <= r_thresh).mean() * 100
    print(f"Raio <= {r_thresh}: Aberto = {pct_ab:.2f}%, Fechado = {pct_fe:.2f}%")

fig = plt.figure(figsize=(16, 14))
gs = fig.add_gridspec(3, 2, width_ratios=[1.4, 1])

t_transicao = df[df['modo'] == 'Fechado']['tempo_s'].min()

ax0 = fig.add_subplot(gs[0, 0])
ax0.plot(df['tempo_s'], df['diff_x'], label='diff_x', color='royalblue', alpha=0.6, lw=0.6)
ax0.plot(df['tempo_s'], df['diff_y'], label='diff_y', color='crimson', alpha=0.6, lw=0.6)
ax0.axvline(t_transicao, color='black', linestyle='--', lw=1.5, label=f'Início Controle ({t_transicao:.1f}s)')
ax0.set_ylabel('Diferença (diff)')
ax0.set_title('Diferença (X e Y) ao longo do tempo')
ax0.legend(loc='upper right')
ax0.grid(True, alpha=0.3)

ax1 = fig.add_subplot(gs[1, 0], sharex=ax0)
ax1.plot(df['tempo_s'], df['x_pos_real'], label='x_pos_real', color='royalblue', alpha=0.6, lw=0.6)
ax1.plot(df['tempo_s'], df['y_pos_real'], label='y_pos_real', color='crimson', alpha=0.6, lw=0.6)
ax1.axvline(t_transicao, color='black', linestyle='--', lw=1.5, label='Início Controle')
ax1.axhline(0, color='gray', linestyle=':', lw=1)
ax1.set_ylabel('Posição Real')
ax1.set_title('Posição Real (X e Y) ao longo do tempo')
ax1.legend(loc='upper right')
ax1.grid(True, alpha=0.3)

ax2 = fig.add_subplot(gs[2, 0], sharex=ax0)
ax2.plot(df['tempo_s'], df['tensao_x'], label='tensao_x', color='royalblue', alpha=0.8, lw=0.8)
ax2.plot(df['tempo_s'], df['tensao_y'], label='tensao_y', color='crimson', alpha=0.8, lw=0.8)
ax2.axvline(t_transicao, color='black', linestyle='--', lw=1.5, label='Início Controle')
ax2.axhline(150, color='darkorange', linestyle=':', lw=1.2, label='Limites Saturação (0V / 150V)')
ax2.axhline(0, color='darkorange', linestyle=':', lw=1.2)
ax2.set_xlabel('Tempo (s)')
ax2.set_ylabel('Tensão (V)')
ax2.set_title('Tensão de Controle (X e Y) ao longo do tempo')
ax2.legend(loc='lower right')
ax2.grid(True, alpha=0.3)

ax_map = fig.add_subplot(gs[0:2, 1])
df_ab = df[df['modo'] == 'Aberto']
df_fe = df[df['modo'] == 'Fechado']

df_fe_sample = df_fe.sample(n=min(10000, len(df_fe)), random_state=42)

ax_map.scatter(df_fe_sample['x_pos_real'], df_fe_sample['y_pos_real'], color='mediumseagreen', alpha=0.15, s=8, label='Pós-Controle (Fechado - Amostra)')
ax_map.scatter(df_ab['x_pos_real'], df_ab['y_pos_real'], color='orange', alpha=0.25, s=10, label='Pré-Controle (Aberto)')

mean_ab_x, mean_ab_y = df_ab['x_pos_real'].mean(), df_ab['y_pos_real'].mean()
mean_fe_x, mean_fe_y = df_fe['x_pos_real'].mean(), df_fe['y_pos_real'].mean()

ax_map.plot(0, 0, marker='+', color='black', markersize=14, markeredgewidth=2.5, linestyle='None', label='Alvo Central (0, 0)')
ax_map.plot(mean_ab_x, mean_ab_y, marker='X', color='darkorange', markeredgecolor='black', markersize=12, linestyle='None', label=f'Média Pré ({mean_ab_x:.4f}, {mean_ab_y:.4f})')
ax_map.plot(mean_fe_x, mean_fe_y, marker='o', color='darkgreen', markeredgecolor='black', markersize=10, linestyle='None', label=f'Média Pós ({mean_fe_x:.4f}, {mean_fe_y:.4f})')

circle_ab = plt.Circle((mean_ab_x, mean_ab_y), df_ab['r_pos_real'].std(), color='red', fill=False, linestyle='-', lw=2, label=f'1σ Raio Pré ({df_ab["r_pos_real"].std():.3f})')
circle_fe = plt.Circle((mean_fe_x, mean_fe_y), df_fe['r_pos_real'].std(), color='darkgreen', fill=False, linestyle='-', lw=2, label=f'1σ Raio Pós ({df_fe["r_pos_real"].std():.3f})')
ax_map.add_patch(circle_ab)
ax_map.add_patch(circle_fe)

ax_map.set_xlim(-0.6, 0.6)
ax_map.set_ylim(-0.6, 0.6)
ax_map.axhline(0, color='gray', linestyle=':', lw=1)
ax_map.axvline(0, color='gray', linestyle=':', lw=1)
ax_map.set_aspect('equal', adjustable='box')
ax_map.set_xlabel('Posição Real X')
ax_map.set_ylabel('Posição Real Y')
ax_map.set_title('Centralização do Feixe e Ponto Médio (Zoom [-0.6, 0.6])')
ax_map.legend(loc='upper right', fontsize=8.5)
ax_map.grid(True, alpha=0.3)

# 5. Comparação de Oscilação (Boxplot / Violino de Posição X, Y e Raio)
ax_osc = fig.add_subplot(gs[2, 1])
data_plot = pd.DataFrame({
    'Valor': np.concatenate([
        df_ab['x_pos_real'], df_fe['x_pos_real'],
        df_ab['y_pos_real'], df_fe['y_pos_real'],
        df_ab['r_pos_real'], df_fe['r_pos_real']
    ]),
    'Métrica': (['Pos X']*len(df_ab) + ['Pos X']*len(df_fe) +
                ['Pos Y']*len(df_ab) + ['Pos Y']*len(df_fe) +
                ['Erro Radial (R)']*len(df_ab) + ['Erro Radial (R)']*len(df_fe)),
    'Modo': (['Aberto (Pré)']*len(df_ab) + ['Fechado (Pós)']*len(df_fe)) * 3
})

sns.boxplot(data=data_plot, x='Métrica', y='Valor', hue='Modo', ax=ax_osc, showfliers=False, palette={'Aberto (Pré)': 'orange', 'Fechado (Pós)': 'mediumseagreen'})
ax_osc.axhline(0, color='black', linestyle=':', lw=1)
ax_osc.set_title('Comparação de Oscilação e Centralização (Sem Outliers Extremos)')
ax_osc.set_ylabel('Posição / Distância ao Centro')
ax_osc.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig(f'PolarisTest/analise_{filename}.png', dpi=150)
print(f"Saved analise_{filename}.png successfully.")

df_ab_end = df_ab.tail(1000)
print("Últimas 1000 amostras do modo Aberto (média x, y):", df_ab_end['x_pos_real'].mean(), df_ab_end['y_pos_real'].mean())

dist_centro_ab = np.sqrt(df_ab['x_pos_real'].mean()**2 + df_ab['y_pos_real'].mean()**2)
dist_centro_fe = np.sqrt(df_fe['x_pos_real'].mean()**2 + df_fe['y_pos_real'].mean()**2)
dist_centro_fe_nsat = np.sqrt(df_fe[df_fe['saturado']==0]['x_pos_real'].mean()**2 + df_fe[df_fe['saturado']==0]['y_pos_real'].mean()**2)

print(f"Distância do ponto médio à origem (0,0):")
print(f"  Pré-controle (Aberto): {dist_centro_ab:.5f}")
print(f"  Pós-controle (Fechado - Total): {dist_centro_fe:.5f} (redução de {(1 - dist_centro_fe/dist_centro_ab)*100:.1f}%)")
print(f"  Pós-controle (Fechado - Sem saturação): {dist_centro_fe_nsat:.5f} (redução de {(1 - dist_centro_fe_nsat/dist_centro_ab)*100:.1f}%)")

print("Razão erro_radial / sigma_turb médio:")
print("  Aberto:", df_ab['r_pos_real'].mean() / df_ab['sigma_turb'].mean())
print("  Fechado:", df_fe['r_pos_real'].mean() / df_fe['sigma_turb'].mean())