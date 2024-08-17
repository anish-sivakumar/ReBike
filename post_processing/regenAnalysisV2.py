import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import csv

# Define constants
TIME_INCREMENT = 0.1012

# Load the CSV file
df = pd.read_csv('LOG29.csv')

# Print column names to check for correct names
print(df.columns)

# Scaling factors
SCALING_FACTORS = {
    'vDC': 0.00217529296,
    'iDC': 1/750,
    'iA': 1/750,
    'iB': 1/750,
    'vA': 0.00217529296,
    'vB': 0.00217529296,
    'iqRef': 1/750,
    'iqFdb': 1/750,
    'iBMS': 1/10,
    'vBMS': 1/100
}

# Apply scaling
for column, factor in SCALING_FACTORS.items():
    if column in df.columns:
        df[column] = df[column] * factor

# Rename columns
df.rename(columns={'iB': 'IAB', 'vB': 'VAB', 'iqRef': 'IqRef', 'iqFdb': 'IqFdb'}, inplace=True)

# Verify the column names again after renaming
print(df.columns)

# Initialize variables for calculations
regen_requests = 0
total_regen_time = 0
total_positive_energy = 0
total_positive_energy_efficiency = 0
total_regen_energy = 0
total_regen_energy_efficiency = 0
starting_voltage = df['vDC'].iloc[:3].mean()
ending_voltage = df['vDC'].iloc[-3:].mean()

# Loop through the data to perform calculations
for i in range(len(df)):
    # Check for regen request
    if df['IqFdb'].iloc[i] < 0:
        regen_requests += 1

    # Calculate total regen time
    if df['IqFdb'].iloc[i] < 0:
        total_regen_time += TIME_INCREMENT

    # Calculate positive energy and efficiency
    if df['throttle'].iloc[i] > 0:
        total_positive_energy += df['power'].iloc[i]
        total_positive_energy_efficiency += df['IqFdb'].iloc[i] / df['IqRef'].iloc[i]

    # Calculate regen energy and efficiency
    if df['IqFdb'].iloc[i] < 0:
        total_regen_energy += df['power'].iloc[i]
        total_regen_energy_efficiency += abs(df['IqFdb'].iloc[i]) / df['IqRef'].iloc[i]

# Calculate average efficiencies
positive_throttle_count = len(df[df['throttle'] > 0])
if positive_throttle_count > 0:
    total_positive_energy_efficiency /= positive_throttle_count

regen_count = len(df[df['IqFdb'] < 0])
if regen_count > 0:
    total_regen_energy_efficiency /= regen_count

# Calculate battery SoC change
battery_soc_change = starting_voltage - ending_voltage

# Estimate effect on range
estimated_range_effect = (total_positive_energy - total_regen_energy) / 15  # Wh/km

# Results dictionary
results = {
    "Regen Requests": regen_requests,
    "Total Regen Time (s)": total_regen_time,
    "Total Positive Energy (Wh)": total_positive_energy,
    "Total Positive Energy Efficiency (%)": total_positive_energy_efficiency * 100 if positive_throttle_count > 0 else "N/A",
    "Total Regen Energy (Wh)": total_regen_energy,
    "Total Regen Energy Efficiency (%)": total_regen_energy_efficiency * 100 if regen_count > 0 else "N/A",
    "Starting Battery SoC (V)": starting_voltage,
    "Ending Battery SoC (V)": ending_voltage,
    "Battery SoC Change (V)": battery_soc_change,
    "Estimated Effect on Range (km)": estimated_range_effect
}

# Write results to CSV
output_file = 'regen_analysis_results.csv'
with open(output_file, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["Metric", "Value"])
    for key, value in results.items():
        writer.writerow([key, value])

# Create plots
plt.figure(figsize=(14, 10))

# Plot 1: Power over Time
plt.subplot(2, 2, 1)
plt.plot(df.index * TIME_INCREMENT, df['power'], label='Power (W)')
plt.xlabel('Time (s)')
plt.ylabel('Power (W)')
plt.title('Power over Time')
plt.legend()

# Plot 2: Battery Voltage over Time
plt.subplot(2, 2, 2)
plt.plot(df.index * TIME_INCREMENT, df['vDC'], label='Battery Voltage (V)', color='orange')
plt.xlabel('Time (s)')
plt.ylabel('Voltage (V)')
plt.title('Battery Voltage over Time')
plt.legend()

# Plot 3: Current over Time
plt.subplot(2, 2, 3)
plt.plot(df.index * TIME_INCREMENT, df['iDC'], label='iDC (A)')
plt.plot(df.index * TIME_INCREMENT, df['iA'], label='iA (A)')
plt.plot(df.index * TIME_INCREMENT, df['IAB'], label='IAB (A)')
plt.xlabel('Time (s)')
plt.ylabel('Current (A)')
plt.title('Current over Time')
plt.legend()

# Plot 4: Throttle Input over Time
plt.subplot(2, 2, 4)
plt.plot(df.index * TIME_INCREMENT, df['throttle'], label='Throttle Input (%)', color='green')
plt.xlabel('Time (s)')
plt.ylabel('Throttle Input (%)')
plt.title('Throttle Input over Time')
plt.legend()

plt.tight_layout()
plt.show()

# Plot all signals over time in separate windows
# Window 1
plt.figure(figsize=(14, 10))
plt.subplot(3, 1, 1)
plt.plot(df.index * TIME_INCREMENT, df['vDC'], label='vDC (V)')
plt.plot(df.index * TIME_INCREMENT, df['vA'], label='vA (V)')
plt.plot(df.index * TIME_INCREMENT, df['VAB'], label='VAB (V)')
plt.xlabel('Time (s)')
plt.ylabel('Voltage (V)')
plt.title('Voltages over Time')
plt.legend()

# Window 2
plt.subplot(3, 1, 2)
plt.plot(df.index * TIME_INCREMENT, df['iDC'], label='iDC (A)')
plt.plot(df.index * TIME_INCREMENT, df['iA'], label='iA (A)')
plt.plot(df.index * TIME_INCREMENT, df['IAB'], label='IAB (A)')
plt.plot(df.index * TIME_INCREMENT, df['iBMS'], label='iBMS (A)')
plt.xlabel('Time (s)')
plt.ylabel('Current (A)')
plt.title('Currents over Time')
plt.legend()

# Window 3
plt.subplot(3, 1, 3)
plt.plot(df.index * TIME_INCREMENT, df['IqRef'], label='IqRef (A)')
plt.plot(df.index * TIME_INCREMENT, df['IqFdb'], label='IqFdb (A)')
plt.plot(df.index * TIME_INCREMENT, df['power'], label='Power (W)')
plt.plot(df.index * TIME_INCREMENT, df['speed'], label='Speed (RPM)')
plt.plot(df.index * TIME_INCREMENT, df['throttle'], label='Throttle Input (%)')
plt.xlabel('Time (s)')
plt.ylabel('Various Units')
plt.title('Other Signals over Time')
plt.legend()

plt.tight_layout()
plt.show()
