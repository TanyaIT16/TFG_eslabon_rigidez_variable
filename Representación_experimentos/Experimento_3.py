import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os

porcentajes = [25, 50, 75, 100]
base_path = "/Users/tanya/Desktop/datosDelTFG/datos/EXP3_comparacion/{}/0bar/a"
sync_data_list = []

def procesar_experimento(df_force, df_pos, pct_label):
    start_time = min(df_force['time'].min(), df_pos['time'].min())
    df_force['time'] -= start_time
    df_pos['time'] -= start_time
    end_time = min(df_force['time'].max(), df_pos['time'].max())

    fuerza_offset = df_force[df_force['time'] <= 2.0]['force_z'].mean()

    t_common = np.arange(0, end_time, 0.1)
    force_interp = np.interp(t_common, df_force['time'], df_force['force_z']) - fuerza_offset
    pos_interp = np.interp(t_common, df_pos['time'], df_pos['motor_position'])

    fuerza_umbral = 0.01
    inicio_idx = np.argmax(force_interp > fuerza_umbral)
    fin_idx = inicio_idx + np.argmax(pos_interp[inicio_idx:] == pos_interp[inicio_idx:].max())
    if inicio_idx >= fin_idx:
        fin_idx = len(t_common) - 1

    t_common = t_common[inicio_idx:fin_idx + 1]
    force_interp = force_interp[inicio_idx:fin_idx + 1]
    pos_interp = pos_interp[inicio_idx:fin_idx + 1]

    print(f"[{pct_label}%] Posición inicial para offset (grados): {pos_interp[0]}")

    tiempo_normalizado = (t_common - t_common[0]) / (t_common[-1] - t_common[0])
    pos_interp -= pos_interp[0]
    pos_rad = np.radians(pos_interp)

    df_sync = pd.DataFrame({
        'time_norm': tiempo_normalizado,
        'force_z': force_interp,
        'motor_position_rad': pos_rad,
        'pct': pct_label
    })

    return df_sync

# Procesar todos los experimentos
for pct in porcentajes:
    path = base_path.format(pct)
    df_force = pd.read_csv(os.path.join(path, "Force.csv"))
    df_position = pd.read_csv(os.path.join(path, "motor_position.csv"))

    df_force.rename(columns={'%time': 'time', 'field.data': 'force_z'}, inplace=True)
    df_position.rename(columns={'%time': 'time', 'field.data': 'motor_position'}, inplace=True)

    df_force['time'] = pd.to_numeric(df_force['time'], errors='coerce') / 1e9
    df_position['time'] = pd.to_numeric(df_position['time'], errors='coerce') / 1e9
    df_force.dropna(subset=['time', 'force_z'], inplace=True)
    df_position.dropna(subset=['time', 'motor_position'], inplace=True)

    df_sync = procesar_experimento(df_force, df_position, pct)
    sync_data_list.append(df_sync)

# Cortar en posición máxima del 25%
pos_max_25 = sync_data_list[0]['motor_position_rad'].max()

# Recorte por posición máxima común
for i in range(len(sync_data_list)):
    df = sync_data_list[i]
    mask = df['motor_position_rad'] <= pos_max_25
    df_cortado = df[mask].copy()
    t_norm = df_cortado['time_norm'].to_numpy()
    t_norm = (t_norm - t_norm[0]) / (t_norm[-1] - t_norm[0])
    df_cortado['time_norm'] = t_norm
    sync_data_list[i] = df_cortado

# Imprimir fuerza máxima dentro del rango útil
print("\n--- Fuerza máxima en el rango útil (hasta la posición del 25%) ---")
for df in sync_data_list:
    pct = df['pct'].iloc[0]
    fmax_recortado = df['force_z'].max()
    print(f"{pct}% blando: {fmax_recortado:.3f} N")

# Graficar
colors = {25: 'tab:blue', 50: 'tab:green', 75: 'tab:red', 100: 'tab:orange'}

plt.figure(figsize=(8, 5))
for df in sync_data_list:
    pct = df['pct'].iloc[0]
    plt.plot(df['time_norm'], df['motor_position_rad'], label=f'{pct}% blando', color=colors[pct])
plt.xlabel('Tiempo Normalizado (τ)')
plt.ylabel('Posición [rad]')
plt.title('Posición vs Tiempo Normalizado. Experimento 0 bar')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig("grafica_posicion_vs_tiempo_recortado.png")
plt.show()

plt.figure(figsize=(8, 5))
for df in sync_data_list:
    pct = df['pct'].iloc[0]
    plt.plot(df['time_norm'], df['force_z'], label=f'{pct}% blando', color=colors[pct])
plt.xlabel('Tiempo Normalizado (τ)')
plt.ylabel('Fuerza [N]')
plt.title('Fuerza vs Tiempo Normalizado. Experimento 0 bar')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig("grafica_fuerza_vs_tiempo_recortado.png")
plt.show()

plt.figure(figsize=(8, 5))
for df in sync_data_list:
    pct = df['pct'].iloc[0]
    plt.plot(df['motor_position_rad'], df['force_z'], label=f'{pct}% blando', color=colors[pct])
plt.xlabel('Posición [rad]')
plt.ylabel('Fuerza [N]')
plt.title('Fuerza vs Posición. Experimento 0 bar')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig("grafica_fuerza_vs_posicion_recortado.png")
plt.show()
