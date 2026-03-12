import pandas as pd
import matplotlib.pyplot as plt

TARGET_APOGEE = 1341.12
USE_FEET = True  # False = meters, True = feet
M_TO_FT = 3.28084


# For mini motor
TARGET_APOGEE = 220


# 1. Load the data
try:
    df = pd.read_csv('sitl_flight_log.csv')
except FileNotFoundError:
    print("Error: sitl_flight_log.csv not found!")
    exit()

# 2. Set up the figure
fig, ax1 = plt.subplots(figsize=(10, 6))

# 3. Plot Altitude and Prediction on the primary Y-axis
alt_scale = M_TO_FT if USE_FEET else 1.0
alt_unit = 'feet' if USE_FEET else 'meters'
alt_unit_short = 'ft' if USE_FEET else 'm'

altitude = df['Alt(m)'] * alt_scale
unbraked_prediction = df['Unbraked_Pred(m)'] * alt_scale
target_apogee = TARGET_APOGEE * alt_scale

ax1.plot(df['Time(s)'], altitude, label='Actual Altitude', color='blue', linewidth=2)
ax1.plot(df['Time(s)'], unbraked_prediction, label='Unbraked Prediction', color='orange', linestyle='--')
ax1.axhline(target_apogee, color='red', linestyle=':', label=f'Target Apogee ({target_apogee:.2f}{alt_unit_short})')

ax1.set_xlabel('Time (seconds)')
ax1.set_ylabel(f'Altitude ({alt_unit})', color='black')
ax1.tick_params(axis='y', labelcolor='black')
ax1.grid(True, linestyle='--', alpha=0.6)

# 4. Plot the Flap Angle on a secondary Y-axis (right side)
ax2 = ax1.twinx()  
ax2.plot(df['Time(s)'], df['Actual_Flap(deg)'], label='Flap Angle', color='green', alpha=0.6, linewidth=2)
ax2.set_ylabel('Flap Angle (Degrees)', color='green')
ax2.tick_params(axis='y', labelcolor='green')

# 5. Combine legends
lines_1, labels_1 = ax1.get_legend_handles_labels()
lines_2, labels_2 = ax2.get_legend_handles_labels()
ax1.legend(lines_1 + lines_2, labels_1 + labels_2, loc='center right')

plt.title('SITL Airbrake Simulation Results')
plt.tight_layout()

# 6. Save the plot as an image and (optionally) display it
plt.savefig('flight_plot.png', dpi=300)
print("Plot successfully saved as 'flight_plot.png'")

# Uncomment the line below if you want a window to pop up with the graph 
# (Requires a desktop interface, skip if you are using Pi over SSH without X11)
# plt.show()