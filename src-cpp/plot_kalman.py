import pandas as pd
import matplotlib.pyplot as plt

# Load the data
try:
    df = pd.read_csv("kalman_test_log.csv")
except FileNotFoundError:
    print("Error: kalman_test_log.csv not found!")
    exit(1)

# Create a figure with two subplots (Altitude and Velocity)
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10), sharex=True)

# --- Subplot 1: Altitude ---
# Plot True Altitude as a thick black line
ax1.plot(df['Time'], df['True_Alt'], label='True Altitude (OpenRocket)', color='black', linewidth=2, zorder=3)

# Scatter plot the noisy sensor readings (red dots showing the jumps/jitter)
ax1.scatter(df['Time'], df['Noisy_Alt'], label='Noisy Sensor Input', color='red', s=8, alpha=0.5, zorder=1)

# Plot the Kalman Filter Estimate as a smooth blue line
ax1.plot(df['Time'], df['KF_Alt'], label='Kalman Filter Estimate', color='dodgerblue', linewidth=2, zorder=2)

ax1.set_ylabel('Altitude (m)', fontsize=12)
ax1.set_title('Altitude Tracking: Kalman Filter vs Noisy Sensors', fontsize=14, fontweight='bold')
ax1.legend(loc='best')
ax1.grid(True, linestyle='--', alpha=0.7)

# --- Subplot 2: Velocity ---
# Plot True Velocity
ax2.plot(df['Time'], df['True_Vel'], label='True Velocity', color='black', linewidth=2)

# Plot Kalman Filter Velocity Estimate
ax2.plot(df['Time'], df['KF_Vel'], label='Kalman Filter Estimate', color='orange', linewidth=2)

ax2.set_xlabel('Time (seconds)', fontsize=12)
ax2.set_ylabel('Velocity (m/s)', fontsize=12)
ax2.set_title('Velocity Tracking', fontsize=14, fontweight='bold')
ax2.legend(loc='best')
ax2.grid(True, linestyle='--', alpha=0.7)

# Formatting and Saving
plt.tight_layout()
output_filename = "kalman_plot.png"
plt.savefig(output_filename, dpi=300)
print(f"[PYTHON] Data successfully plotted and saved to {output_filename}")