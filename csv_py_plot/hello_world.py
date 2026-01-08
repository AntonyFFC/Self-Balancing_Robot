import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('pid_data_share.csv')

df['time_s'] = df.index * 0.1

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

ax1.plot(df['time_s'], df['Pitch'], label='Pitch', linewidth=1.5)
ax1.plot(df['time_s'], df['SetPitch'], label='SetPitch', linewidth=1.5, linestyle='--')
ax1.set_ylabel('Pitch (deg)')
ax1.set_title('Pitch Tracking over Time')
ax1.legend()
ax1.grid(True, alpha=0.3)

ax2.plot(df['time_s'], df['ControlSignal'], label='Control Signal', linewidth=1.5, color='green')
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Control Signal')
ax2.set_title('Control Signal over Time')
ax2.legend()
ax2.grid(True, alpha=0.3)

plt.tight_layout()
plt.show()