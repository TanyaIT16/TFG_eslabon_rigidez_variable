import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os

# Configuración
porcentajes = [25, 50, 75, 100]
base_path = "/Users/tanya/Desktop/datosDelTFG/datos/EXP3_comparacion/{}/08bar/a"
rigido_path = "/Users/tanya/Desktop/datosDelTFG/datos/EXP3_comparacion/rigido/a"

sync_data_list = []

def procesar_experimento(df_force, df_pos, etiqueta):
    # Tiempo relativo
    start_time = min(df_force['time'].min(), df_pos['time'].min())
    df_force['time'] -= start_time
    df_pos['time'] -= start_time
    end_time = min(df_force['time'].max(), df_pos['time'].max())

    # Calcular offset de fuerza
    fuerza_offset = df_force[df_force['time'] <= 2.0]['force_z'].mean()

    # Interpolación
    t_common = np.arange(0, end_time, 0.1)
    force_interp = np.interp(t_common, df_force['time'], df_force['force_z']) - fuerza_offset
    pos_interp = np.interp(t_common, df_pos['time'], df_pos['motor_position'])

    # Detectar inicio
    fuerza_umbral = 0.01
    inicio_idx = np.argmax(force_interp > fuerza_umbral)

    # Detectar fin
    fin_idx = inicio_idx + np.argmax(pos_interp[inicio_idx:] == pos_interp[inicio_idx:].max())
    if inicio_idx >= fin_idx:
        fin_idx = len(t_common) - 1

    # Recorte
    t_common = t_common[inicio_idx:fin_idx + 1]
    force_interp = force_interp[inicio_idx:fin_idx + 1]
    pos_interp = pos_interp[inicio_idx:fin_idx + 1]

    # Guardar posición original en grados para depuración
    print(f"[{etiqueta}] Posición inicial para offset (grados): {pos_interp[0]}")

    # Restar posición inicial y convertir a radianes
    pos_interp -= pos_interp[0]
    pos_rad = np.radians(pos_interp)

    # Tiempo normalizado
    tiempo_normalizado = (t_common - t_common[0]) / (t_common[-1] - t_common[0])

    # DataFrame final
    df_sync = pd.DataFrame({
        'time_norm': tiempo_normalizado,
        'force_z': force_interp,
        'motor_position_rad': pos_rad,
        'condicion': etiqueta
    })

    return df_sync

# --- Procesar porcentajes blandos ---
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

    df_sync = procesar_experimento(df_force, df_position, f'{pct}% blando')
    sync_data_list.append(df_sync)

# --- Procesar rígido ---
df_force_rig = pd.read_csv(os.path.join(rigido_path, "Force.csv"))
df_position_rig = pd.read_csv(os.path.join(rigido_path, "motor_position.csv"))

df_force_rig.rename(columns={'%time': 'time', 'field.data': 'force_z'}, inplace=True)
df_position_rig.rename(columns={'%time': 'time', 'field.data': 'motor_position'}, inplace=True)

df_force_rig['time'] = pd.to_numeric(df_force_rig['time'], errors='coerce') / 1e9
df_position_rig['time'] = pd.to_numeric(df_position_rig['time'], errors='coerce') / 1e9
df_force_rig.dropna(subset=['time', 'force_z'], inplace=True)
df_position_rig.dropna(subset=['time', 'motor_position'], inplace=True)

df_sync_rig = procesar_experimento(df_force_rig, df_position_rig, "rígido")
sync_data_list.append(df_sync_rig)

# --- Colores para gráficas ---
colors = {'25% blando': 'tab:blue', '50% blando': 'tab:green', '75% blando': 'tab:red', '100% blando': 'tab:orange', 'rígido': 'black'}

# --- 1. Posición vs Tiempo ---
plt.figure(figsize=(8, 5))
for df in sync_data_list:
    etiqueta = df['condicion'].iloc[0]
    plt.plot(df['time_norm'].to_numpy(), df['motor_position_rad'].to_numpy(), label=etiqueta, color=colors[etiqueta])
plt.xlabel('Tiempo Normalizado (τ)')
plt.ylabel('Posición [rad]')
plt.title('Posición vs Tiempo Normalizado. Experimento -0.9 bar')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig("grafica_posicion_vs_tiempo.png")
plt.show()

# --- 2. Fuerza vs Tiempo ---
plt.figure(figsize=(8, 5))
for df in sync_data_list:
    etiqueta = df['condicion'].iloc[0]
    plt.plot(df['time_norm'].to_numpy(), df['force_z'].to_numpy(), label=etiqueta, color=colors[etiqueta])
plt.xlabel('Tiempo Normalizado (τ)')
plt.ylabel('Fuerza [N]')
plt.title('Fuerza vs Tiempo Normalizado. Experimento -0.9 bar')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig("grafica_fuerza_vs_tiempo.png")
plt.show()

# --- 3. Fuerza vs Posición ---
plt.figure(figsize=(8, 5))
for df in sync_data_list:
    etiqueta = df['condicion'].iloc[0]
    plt.plot(df['motor_position_rad'].to_numpy(), df['force_z'].to_numpy(), label=etiqueta, color=colors[etiqueta])
plt.xlabel('Posición [rad]')
plt.ylabel('Fuerza [N]')
plt.title('Fuerza vs Posición. Experimento -0.9 bar')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig("grafica_fuerza_vs_posicion.png")
plt.show()